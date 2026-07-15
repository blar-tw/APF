"""Unit tests for the pure-python APF core (no ROS)."""

import numpy as np
import pytest

from apf_oa import apf_core
from apf_oa.apf_core import Config, StuckDetector


@pytest.fixture
def cfg():
    return Config()


# --- attractive force ---

def test_attractive_proportional_within_saturation(cfg):
    pos = np.array([0.0, 0.0, 0.0])
    goal = np.array([2.0, 0.0, 0.0])  # dist 2 < att_saturation 3
    f = apf_core.attractive_force(pos, goal, cfg)
    assert np.allclose(f, cfg.k_att * (goal - pos))


def test_attractive_saturates_far_away(cfg):
    pos = np.zeros(3)
    goal = np.array([100.0, 0.0, 0.0])
    f = apf_core.attractive_force(pos, goal, cfg)
    assert np.isclose(np.linalg.norm(f), cfg.k_att * cfg.att_saturation)
    assert f[0] > 0  # still pointing at the goal


def test_attractive_zero_at_goal(cfg):
    p = np.array([1.0, 2.0, -3.0])
    assert np.allclose(apf_core.attractive_force(p, p, cfg), 0.0)


def test_attractive_3d_direction(cfg):
    pos = np.array([0.0, 0.0, 0.0])
    goal = np.array([1.0, 1.0, -1.0])
    f = apf_core.attractive_force(pos, goal, cfg)
    assert np.allclose(f / np.linalg.norm(f), goal / np.linalg.norm(goal))


# --- repulsive force ---

def test_repulsive_zero_outside_influence(cfg):
    pos = np.zeros(3)
    pts = np.array([[cfg.influence_radius + 0.1, 0.0, 0.0]])
    assert np.allclose(apf_core.repulsive_force(pos, pts, cfg), 0.0)


def test_repulsive_zero_for_empty_set(cfg):
    assert np.allclose(apf_core.repulsive_force(np.zeros(3), np.zeros((0, 3)), cfg), 0.0)


def test_repulsive_pushes_away(cfg):
    pos = np.zeros(3)
    pts = np.array([[1.0, 0.0, 0.0]])  # obstacle to the +x side
    f = apf_core.repulsive_force(pos, pts, cfg)
    assert f[0] < 0  # pushed toward -x
    assert np.isclose(f[1], 0.0) and np.isclose(f[2], 0.0)


def test_repulsive_grows_when_closer(cfg):
    pos = np.zeros(3)
    far = np.linalg.norm(apf_core.repulsive_force(pos, np.array([[2.0, 0, 0]]), cfg))
    near = np.linalg.norm(apf_core.repulsive_force(pos, np.array([[0.5, 0, 0]]), cfg))
    assert near > far > 0


def test_repulsive_vertical_component(cfg):
    # NED: obstacle point 0.5 m below the drone -> force must push up (-D)
    pos = np.array([0.0, 0.0, -2.0])
    pts = np.array([[0.0, 0.0, -1.5]])
    f = apf_core.repulsive_force(pos, pts, cfg)
    assert f[2] < 0


def test_repulsive_clipped(cfg):
    pos = np.zeros(3)
    pts = np.array([[0.01, 0.0, 0.0]])  # extremely close -> would explode
    f = apf_core.repulsive_force(pos, pts, cfg)
    assert np.linalg.norm(f) <= cfg.max_rep_force + 1e-9


# --- tangential (swirl) force ---

FAR = 10.0  # nearest-obstacle distance beyond influence -> swirl taper fully on


def test_tangential_perpendicular_and_horizontal(cfg):
    f_rep = np.array([-1.0, 0.0, 0.0])          # repulsion pointing -N
    t = apf_core.tangential_force(f_rep, np.array([1.0, 0.0, 0.0]), FAR, cfg)
    assert t[2] == 0.0                          # never touches the vertical axis
    assert abs(np.dot(t[:2], f_rep[:2])) < 1e-9  # perpendicular to the repulsion
    assert np.isclose(np.linalg.norm(t), cfg.tangential_gain * 1.0)


def test_tangential_steers_toward_goal_side(cfg):
    # Obstacle straight ahead (-N repulsion), goal ahead and to +E:
    # the swirl must push toward +E so the drone rounds the correct way.
    f_rep = np.array([-1.0, 0.0, 0.0])
    t = apf_core.tangential_force(f_rep, np.array([1.0, 0.5, 0.0]), FAR, cfg)
    assert t[1] > 0


def test_tangential_scales_with_repulsion(cfg):
    g = np.array([1.0, 0.0, 0.0])
    small = np.linalg.norm(apf_core.tangential_force(np.array([-0.5, 0, 0]), g, FAR, cfg))
    big = np.linalg.norm(apf_core.tangential_force(np.array([-2.0, 0, 0]), g, FAR, cfg))
    assert big > small > 0


def test_tangential_zero_without_repulsion_or_when_disabled(cfg):
    g = np.array([1.0, 0.0, 0.0])
    assert np.allclose(apf_core.tangential_force(np.zeros(3), g, FAR, cfg), 0.0)
    off = Config(tangential_gain=0.0)
    assert np.allclose(apf_core.tangential_force(np.array([-1.0, 0, 0]), g, FAR, off), 0.0)


def test_swirl_tapers_off_close_to_obstacle(cfg):
    # Inside the safety radius the swirl must vanish so the radial push (which
    # creates clearance) takes over; well outside it is at full strength.
    f_rep, g = np.array([-1.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0])
    close = apf_core.tangential_force(f_rep, g, cfg.swirl_safe_radius * 0.5, cfg)
    full = apf_core.tangential_force(f_rep, g, cfg.swirl_safe_radius + cfg.swirl_taper_band, cfg)
    assert np.allclose(close, 0.0)
    assert np.isclose(np.linalg.norm(full), cfg.tangential_gain * 1.0)
    assert apf_core.swirl_taper(cfg.swirl_safe_radius, cfg) == 0.0
    assert apf_core.swirl_taper(1e9, cfg) == 1.0


# --- acceleration (slew-rate) limiting ---

def test_limit_acceleration_caps_the_step(cfg):
    dt = 0.05
    out = apf_core.limit_acceleration(np.zeros(3), np.array([1.5, 0.0, 0.0]), dt, cfg)
    assert np.isclose(np.linalg.norm(out), cfg.max_accel * dt)  # 4.0 * 0.05 = 0.2
    assert out[0] > 0  # moves toward the desired command


def test_limit_acceleration_passthrough_small_change(cfg):
    dt = 0.05
    v_prev = np.array([1.0, 0.0, 0.0])
    v_des = np.array([1.05, 0.0, 0.0])  # 0.05 < max step 0.2
    assert np.allclose(apf_core.limit_acceleration(v_prev, v_des, dt, cfg), v_des)


def test_limit_acceleration_disabled_when_infinite():
    off = Config(max_accel=float('inf'))
    v_des = np.array([10.0, -5.0, 2.0])
    assert np.allclose(apf_core.limit_acceleration(np.zeros(3), v_des, 0.05, off), v_des)


# --- velocity conversion ---

def test_velocity_clamped_to_v_max(cfg):
    v = apf_core.force_to_velocity(np.array([100.0, 0.0, 0.0]), cfg)
    assert np.isclose(np.linalg.norm(v), cfg.v_max)


def test_velocity_passthrough_when_small(cfg):
    f = np.array([0.3, -0.2, 0.1])
    assert np.allclose(apf_core.force_to_velocity(f, cfg), f)


# --- combined step ---

def test_apf_step_deflects_around_obstacle(cfg):
    # Goal straight ahead (+N), obstacle slightly right of the path (+E):
    # the command must keep forward progress and deflect left (-E)
    pos = np.zeros(3)
    goal = np.array([10.0, 0.0, 0.0])
    obstacle = np.array([[1.5, 0.3, 0.0]])
    v, info = apf_core.apf_step(pos, goal, obstacle, cfg)
    assert v[0] > 0
    assert v[1] < 0
    assert info["dist_goal"] == pytest.approx(10.0)


def test_apf_step_no_obstacles(cfg):
    v, info = apf_core.apf_step(np.zeros(3), np.array([5.0, 0, 0]), np.zeros((0, 3)), cfg)
    assert v[0] > 0
    assert np.allclose(info["f_rep"], 0.0)
    assert np.allclose(info["f_tan"], 0.0)


def test_apf_step_accel_limits_against_previous_command(cfg):
    # Same state, but with a previous command the jump must be bounded, while
    # the raw (no v_prev) call still returns the full field velocity.
    pos, goal, obs = np.zeros(3), np.array([10.0, 0, 0]), np.zeros((0, 3))
    v_raw, _ = apf_core.apf_step(pos, goal, obs, cfg)
    dt = 0.05
    v_lim, _ = apf_core.apf_step(pos, goal, obs, cfg, v_prev=np.zeros(3), dt=dt)
    assert np.linalg.norm(v_raw) > cfg.max_accel * dt          # field wants a big step
    assert np.linalg.norm(v_lim) <= cfg.max_accel * dt + 1e-9  # limiter caps it


def test_smoothed_apf_escapes_symmetric_gate(cfg):
    # Two obstacles straddling the straight path form a head-on local minimum -
    # the exact case where a plain APF chatters in place. Swirl + accel limit
    # must carry the drone through to the goal instead of stalling/oscillating.
    obstacles = np.array([[6.0, 0.9, 0.0], [6.0, -0.9, 0.0]])
    goal = np.array([12.0, 0.0, 0.0])
    pos, v_prev, dt = np.zeros(3), np.zeros(3), 0.05
    reached = False
    for _ in range(4000):  # 200 s of sim time, ample margin
        v, info = apf_core.apf_step(pos, goal, obstacles, cfg, v_prev=v_prev, dt=dt)
        v_prev, pos = v, pos + v * dt
        if info["dist_goal"] < cfg.goal_threshold:
            reached = True
            break
    assert reached and pos[0] > 6.0  # got past the gate at N=6, not stuck before it


# --- local minima detection ---

def test_stuck_after_n_slow_ticks(cfg):
    det = StuckDetector(cfg)
    for i in range(cfg.stuck_ticks - 1):
        assert det.update(speed=0.01, dist_to_goal=5.0) is False
    assert det.update(speed=0.01, dist_to_goal=5.0) is True
    assert det.episodes == 1


def test_not_stuck_when_moving(cfg):
    det = StuckDetector(cfg)
    for _ in range(cfg.stuck_ticks * 2):
        assert det.update(speed=1.0, dist_to_goal=5.0) is False


def test_not_stuck_near_goal(cfg):
    det = StuckDetector(cfg)
    for _ in range(cfg.stuck_ticks * 2):
        assert det.update(speed=0.0, dist_to_goal=cfg.goal_threshold * 0.5) is False


def test_stuck_counter_resets_on_motion(cfg):
    det = StuckDetector(cfg)
    for _ in range(cfg.stuck_ticks - 1):
        det.update(speed=0.01, dist_to_goal=5.0)
    det.update(speed=1.0, dist_to_goal=5.0)  # burst of motion resets
    assert det.update(speed=0.01, dist_to_goal=5.0) is False
    assert det.count == 1


def test_stuck_on_oscillation(cfg):
    # Fast but going nowhere: bouncing inside a 0.1 m box must count as stuck
    det = StuckDetector(cfg)
    stuck = False
    for i in range(cfg.stuck_ticks + 1):
        pos = np.array([0.05 * (i % 2), 0.0, 0.0])
        stuck = det.update(speed=1.0, dist_to_goal=5.0, pos=pos)
    assert stuck is True


def test_no_stuck_with_forward_progress(cfg):
    det = StuckDetector(cfg)
    for i in range(cfg.stuck_ticks * 2):
        pos = np.array([0.05 * i, 0.0, 0.0])  # 1 m/s at 20 Hz
        assert det.update(speed=1.0, dist_to_goal=50.0, pos=pos) is False


# --- frame helpers ---

def test_enu_ned_roundtrip():
    v = np.array([1.0, 2.0, 3.0])
    assert np.allclose(apf_core.ned_to_enu(apf_core.enu_to_ned(v)), v)
    # ENU (E=1, N=2, U=3) -> NED (N=2, E=1, D=-3)
    assert np.allclose(apf_core.enu_to_ned(v), [2.0, 1.0, -3.0])


def test_rotate_by_quat_identity():
    v = np.array([1.0, 2.0, 3.0])
    assert np.allclose(apf_core.rotate_by_quat((1, 0, 0, 0), v), v)


def test_rotate_by_quat_yaw90():
    # 90 deg yaw about +z: body +x maps to +y
    s = np.sqrt(0.5)
    out = apf_core.rotate_by_quat((s, 0, 0, s), np.array([1.0, 0.0, 0.0]))
    assert np.allclose(out, [0.0, 1.0, 0.0], atol=1e-9)


# --- world spec sanity ---

def test_world_spec_points():
    from apf_oa import world_spec
    pts = world_spec.obstacle_points_enu()
    assert pts.shape[1] == 3
    assert pts.shape[0] > 100
    # every sampled point stays within its cylinder's bounding volume
    assert pts[:, 2].min() >= 0.0
    assert pts[:, 2].max() <= max(h for *_, h in world_spec.CYLINDERS) + 1e-6
