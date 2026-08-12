# Pick-place 导航设计

导航链路只有四个职责：

```text
USD CollisionAPI 三角网格 → 二维碰撞切片 → 安全膨胀 → A* 路径
```

实现分别位于：

- `occupancy.py`：筛选启用的碰撞 Mesh，在世界坐标 Z 平面投影三角面，并构造 Isaac Sim MobilityGen `OccupancyMap`。
- `navigation.py`：不依赖 Kit 的纯 Python A*、端点吸附、折点压缩、等距采样和姿态生成。
- `navigation_config.py` / `navigation.yaml`：读取和校验地图与规划参数。
- `artifacts.py`：保存 ROS 地图、路径预览和 JSON。
- `scripts/plan_pick_place_path.py`：启动 Isaac Lab 场景并串联以上步骤。
- `navigator.py`：将地图生成、规划和产物保存包装为可复用的 `PickPlaceNavigator`。

## Lab 与 Isaac World 对齐

场景坐标以 `loco_manipulation/config/scene_coordinates.py` 为唯一来源。Lab USD
根节点放在 Isaac World `(0, 0, 0.20)`：碰撞网格的地面在 USD 局部坐标中低于
原点 `0.20 m`，因此这个平移会把实际地面放到世界 `Z=0`。USD 内部已经完成视觉
重建坐标到 Z-up 的轴向转换，场景根节点不应再附加 90° 旋转。

杯子、篮子、导航起终点均使用绝对 Isaac World 坐标，不会再叠加 Lab 的
`0.20 m` 偏移。配置姿态遵循 Isaac Lab 3.0 的 `xyzw`，导航函数输入仍遵循参数名
声明的 `wxyz`。

## 碰撞体与标注

障碍物由 `UsdPhysics.CollisionAPI` 区分。`physics:collisionEnabled = false` 的 Prim、机器人以及本任务中的杯子和篮子不会进入静态环境地图。Replicator semantics 只服务于合成数据标签，不参与物理或导航，所以不需要手工语义标注。

当前扫描场景的环境 collider 已经三角化。遇到非三角碰撞 Mesh 时程序会直接报错，而不是静默漏掉障碍物。

## YAML 参数

| 参数 | 含义 |
| --- | --- |
| `map.slice_world_z` | 世界坐标中的水平碰撞切片高度，穿过该平面的 collider 三角面会成为障碍 |
| `map.min_world_xy` / `max_world_xy` | 地图的世界坐标 XY 边界 |
| `map.resolution` | 每个栅格的米数 |
| `planning.safety_margin` | 机器人包围半径之外的附加安全距离 |
| `planning.waypoint_spacing` | 输出路径点的最大间距 |
| `planning.max_endpoint_snap_distance` | 起点或终点被占用时允许吸附到最近自由栅格的最大距离 |

起点和终点的位置、姿态属于每次调用的数据，因此由命令行参数传入，不放在 YAML 中。四元数采用 Isaac World `wxyz` 顺序。

## 规划安全性

机器人半径由运行时已启用 collider 的世界坐标 AABB 计算，不硬编码。地图按“外接圆半径 + safety margin”膨胀后再运行 8 邻接 A*；对角移动禁止穿过被占用的直角拐角。路径只压缩共线栅格并做线性重采样，不使用可能切入障碍物的样条平滑。

## 产物

默认目录为 `outputs/pick_place/navigation/`：

- `map.png` / `map.yaml`：ROS 格式原始占用图。
- `map_buffered.png`：按机器人安全距离膨胀后的地图。
- `path.png`：路径预览，绿色为起点，红色为终点。
- `path.json`：世界坐标路径点、端点姿态、地图元数据和吸附状态。
- `status.txt`：`complete` 表示本次运行完成；失败时 `error.txt` 保存完整 traceback。

## 为什么不直接使用 PhysX OMap Generator

Isaac Sim 6.0 的 OMap Generator 依赖已经进入 PhysX scene query 的 collision shape。本项目的重建场景虽然在 USD 上写有 `CollisionAPI`，但运行时 OMap 和 PhysX 查询都只看到了机器人，没有看到环境扫描网格。这里采用 NVIDIA 导航指南给出的 collider-driven direct projection：仍以 USD 碰撞 Schema 为唯一障碍来源，同时避免依赖该资产未建立的 PhysX shape。
