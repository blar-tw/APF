# APF_OA — Artificial Potential Field Obstacle Avoidance

> 3D APF(人工勢場)避障:PX4 SITL + Gazebo 模擬的多旋翼,用深度相機
> (OakD-Lite on x500_depth)感知障礙,即時計算引力/斥力合力飛向目標。
> 姊妹專案:[HOLO-DWA](../HOLO-DWA)(同環境,DWA 演算法)。

## 需求

- WSL2 + Ubuntu 22.04(或原生)、ROS 2 Humble
- PX4-Autopilot v1.14.4 + Gazebo Garden(`x500_depth` 模型與 `4002` airframe 為內建)
- `ros_gz` bridge(需對應 Gazebo 版本,見 HOLO-DWA 的 [installation.md](../HOLO-DWA/docs/installation.md))
- Micro-XRCE-DDS-Agent、`px4_msgs`、`tmux`、Python 3.10 + `numpy`
- 首次啟動需能連網一次:Gazebo 會從 Fuel 下載 OakD-Lite 模型(之後走快取)

## 快速開始

**第一次跑之前**(每個 airframe 一次):`gz_x500_depth`(airframe 4002)預設
不允許無 RC 的 Offboard,會直接 failsafe。啟動後在 PX4 的 `pxh>` console 設:

```
param set NAV_DLL_ACT 0
param set COM_RCL_EXCEPT 4
param save
```

```bash
cd ~/ws
colcon build --packages-select apf_oa --symlink-install
cd src/APF
./run.sh                      # goal (12, 0), 高度 2 m,固定障礙列表,有 GUI 就開 RViz
./run.sh 8.0 -3.0             # 自訂 goal_x goal_y(Gazebo world 座標)
./run.sh 8.0 -3.0 2.5         # 再加目標高度
OBST_SOURCE=depth ./run.sh    # 障礙來源改用深度相機點雲
HEADLESS=1 ./run.sh           # 無 GUI(WSL2 下追蹤較穩)
./run.sh kill                 # 全部收掉
```

跑完看結果:

```bash
python3 APF_OA/tools/check_log.py     # 最新一筆飛行 CSV 的摘要
```

## 運作方式

```
Gazebo (x500_depth, world: apf_test)
   └─ OakD-Lite depth camera ──► /depth_camera/points (gz)
        ▼ ros_gz_bridge (→ /depth_points)
 obstacle_sensor_node ──► /apf/obstacle_points (PointCloud2, NED)
        │   source=fixed:  world_spec 圓柱表面取樣點(全知,驗證用)
        │   source=depth:  深度點雲 → FLU→FRD→NED 轉換 + 濾地面/降採樣
        ▼
 apf_planner_node (20 Hz)
        │   INIT → TAKEOFF → NAVIGATE → GOAL_REACHED
        │   NAVIGATE: apf_core.apf_step() → 3D 速度指令
        ▼
 /fmu/in/trajectory_setpoint ──► XRCE-DDS Agent ──► PX4 SITL
```

APF 核心([apf_core.py](APF_OA/apf_oa/apf_core.py),純 numpy、無 ROS):

- **引力**:`F_att = k_att · (goal − pos)`,距離超過 `att_saturation` 後飽和(遠距等速拉)。
- **斥力**(Khatib):影響半徑 `influence_radius` 內,`F = k_rep·(1/d − 1/d0)/d²` 推離障礙點;
  先做**扇區縮減**(方位 12 × 仰角 3,每扇區只取最近點)再加總——斥力大小與點雲密度無關,
  fixed / depth 兩種來源行為一致(沒有這步,密點雲的合計斥力會蓋過引力,無人機停在半路)。
- **合力 → 速度**:合力視為期望速度,模長截到 `v_max`,3 軸(vx, vy, vz)直接下給 PX4
  (per-axis NaN passthrough)。
- **Local minima 偵測**:距 goal 尚遠時,「速度連續近零」**或**「一段時間內位移極小(震盪)」
  即判定卡住,記進 log 與 CSV(尚未做脫困,見下方 Step 5)。

障礙物高度是刻意設計的:4 根 3.5 m 高柱(擋住 2 m 巡航高度 → 逼水平繞行)+
1 根 1.2 m 矮柱擋在路徑上(z 軸斥力 → 直接飛越),一次展示 3D APF 的兩種行為。

## 檔案

| 路徑 | 角色 | 怎麼測試 |
|------|------|---------|
| [`APF_OA/apf_oa/apf_core.py`](APF_OA/apf_oa/apf_core.py) | APF 演算法核心(引力/斥力/速度轉換/卡住偵測),純 numpy | `python3 -m pytest test/`(24 條) |
| [`APF_OA/apf_oa/world_spec.py`](APF_OA/apf_oa/world_spec.py) | 障礙物規格單一來源(SDF、fixed 列表、RViz marker 共用) | 改完跑 `tools/gen_world.py` 重生成 |
| [`APF_OA/apf_oa/obstacle_sensor_node.py`](APF_OA/apf_oa/obstacle_sensor_node.py) | 障礙來源 node:fixed(已知列表)/ depth(相機點雲→NED) | `ros2 topic echo /apf/obstacle_points --field width` |
| [`APF_OA/apf_oa/apf_planner_node.py`](APF_OA/apf_oa/apf_planner_node.py) | 飛行 node:offboard 狀態機 + APF 導航 + CSV logging + RViz 視覺化 | 跑 `./run.sh` 看 1 Hz 狀態列;`tools/check_log.py` 看 CSV |
| [`APF_OA/apf_oa/pc2_util.py`](APF_OA/apf_oa/pc2_util.py) | PointCloud2 ↔ numpy 輕量轉換 | 由整合測試覆蓋 |
| [`APF_OA/config/apf_params.yaml`](APF_OA/config/apf_params.yaml) | k_att / k_rep / influence_radius / goal / 卡住判定等所有參數 | 改完 `./run.sh kill` 再 `./run.sh`(symlink-install 免 rebuild) |
| [`APF_OA/launch/apf_nodes.launch.py`](APF_OA/launch/apf_nodes.launch.py) | ROS 端三件組(sensor + planner + RViz)launch | `ros2 launch apf_oa apf_nodes.launch.py rviz:=false` |
| [`APF_OA/worlds/apf_test.sdf`](APF_OA/worlds/apf_test.sdf) | 產生的 Gazebo world(5 圓柱 + goal 標記) | `gz sdf -k` 已驗證;run.sh 每次自動同步進 PX4 |
| [`APF_OA/tools/gen_world.py`](APF_OA/tools/gen_world.py) | 從 world_spec 產生上面的 SDF | `python3 tools/gen_world.py` |
| [`APF_OA/tools/sim_offline.py`](APF_OA/tools/sim_offline.py) | 離線動力學模擬(免 ROS/Gazebo,秒級),調參前先跑 | `python3 tools/sim_offline.py [--trap]` |
| [`APF_OA/tools/check_log.py`](APF_OA/tools/check_log.py) | 飛行 CSV 摘要(達標/淨空/卡住次數) | `python3 tools/check_log.py` |
| [`run.sh`](run.sh) | 一鍵 tmux 啟動全 stack(PX4+gz / agent / bridge / nodes) | `./run.sh`,收掉 `./run.sh kill` |
| [`WORKLOG.md`](WORKLOG.md) | 決策紀錄與待辦 | — |

CSV 欄位:`t, state, 位置(n,e,d), 速度, F_att, F_rep, F_total, v_cmd, dist_goal, 障礙點數, stuck, stuck_episodes` → `logs/apf_*.csv`。

## Step 5:local minima 測試(給你的提醒)

APF 的經典弱點:障礙物正對 goal 排成一直線時,引力與斥力抵銷,無人機卡住。
先用離線模擬看行為:

```bash
python3 APF_OA/tools/sim_offline.py --trap   # 在 x=6 加一道三圓柱牆 → 觀察 STUCK
```

Gazebo 實測:在 [`world_spec.py`](APF_OA/apf_oa/world_spec.py) 加一根 `("trap", 6.0, 0.0, 0.4, 3.5)`
(與既有 `cyl_tall_2` (6,−1.2) 形成攔截牆),跑 `gen_world.py` 後 `./run.sh`,
看 planner 印出 `LOCAL MINIMUM suspected`、CSV 的 `stuck=1`。

可能的改良(**尚未實作**,等你確認):切線繞行力(斥力旋轉 90°)、隨機擾動、
沿牆走(wall following)、或暫時抬高 goal 高度讓 3D 勢場自己找路。

## 參數調整

改 [`config/apf_params.yaml`](APF_OA/config/apf_params.yaml) 後重跑 `./run.sh` 即可(不用 rebuild)。
調參順序建議:先 `tools/sim_offline.py --k-rep X --influence Y` 離線掃,行為對了再上 Gazebo。
