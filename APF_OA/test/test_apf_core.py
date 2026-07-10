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
