# `omni.replicator.core` 使用说明

本文面向 Isaac Sim 6.0，说明下面这条导入语句的作用，以及它在本项目中的适用范围：

```python
import omni.replicator.core as rep
```

`omni.replicator.core` 是 Omniverse Replicator 的核心 Python 接口，`rep` 只是模块别名。它主要用于构建或修改 USD 场景、执行场景随机化、创建渲染输出，以及采集合成训练数据（Synthetic Data Generation，SDG）。

它不是 A* 路径规划器，也不是占用图生成器。

## 导入顺序

Replicator 是 Isaac Sim Kit 扩展，不是一个可以脱离 Isaac Sim 单独使用的普通 Python 包。必须先启动 Isaac Sim，再导入 `omni.replicator.core`。

Standalone 程序：

```python
from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": True})

import omni.replicator.core as rep
```

Isaac Lab 程序应先通过 `AppLauncher` 启动应用：

```python
from isaaclab.app import AppLauncher

app_launcher = AppLauncher(headless=True)
simulation_app = app_launcher.app

import omni.replicator.core as rep
```

如果提前导入，可能出现模块找不到、扩展尚未加载或底层插件尚未初始化等问题。

## 主要功能

| 功能 | 常用接口 | 说明 |
| --- | --- | --- |
| 创建或引用 USD 对象 | `rep.create`、`rep.functional.create` | 创建相机、灯光、几何体，或者引用已有 USD |
| 配置物理属性 | `rep.functional.physics` | 创建 Physics Scene，为 Prim 应用碰撞体或刚体属性 |
| 修改场景 | `rep.modify` | 修改位姿、材质、可见性和语义信息 |
| 随机采样 | `rep.distribution` | 创建均匀分布、正态分布、离散选择等随机源 |
| 场景随机化 | `rep.randomizer` | 注册和执行自定义随机化逻辑 |
| 触发控制 | `rep.trigger` | 指定随机化或采集发生的时机 |
| 创建渲染输出 | `rep.create.render_product` | 将相机和输出分辨率组合成渲染通道 |
| 获取真值数据 | Annotator | 获取深度、语义分割、实例分割和包围框等数据 |
| 写入磁盘 | `rep.backends`、`rep.writers` | 保存 RGB、深度和标签数据 |
| 推进采集流程 | `rep.orchestrator` | 执行一帧采集、异步采集或等待写入完成 |

## Functional API

Isaac Sim 6.0 的官方 `amr_navigation.py` 使用 `rep.functional` 创建和配置场景。例如：

```python
rep.functional.physics.create_physics_scene(
    "/PhysicsScene",
    enableCCD=True,
    broadphaseType="MBP",
    enableGPUDynamics=False,
)

rep.functional.create.scope(name="Environment")

ground = rep.functional.create.plane(
    parent="/Environment",
    name="GroundPlane",
    scale=(100, 100, 1),
)

rep.functional.physics.apply_collider(ground)
```

这段代码依次完成：

1. 创建 PhysX Physics Scene。
2. 创建 `/Environment` Scope。
3. 创建地面 Prim。
4. 为地面应用碰撞属性。

`apply_collider()` 负责把碰撞相关的 USD/PhysX Schema 写到 Prim 上；实际碰撞检测和动力学计算仍由 PhysX 完成，并不是 Replicator 自己进行物理仿真。

### 引用 USD 资产

```python
robot = rep.functional.create.reference(
    position=(0, 0, 0),
    rotation=(0, 0, 0),
    usd_path=robot_usd_path,
    parent="/World",
    name="Robot",
)
```

这会在 `/World/Robot` 下引用一个已有 USD。返回值是对应的 USD Prim，后续可以读取子 Prim、属性或添加物理配置。

同一个 Prim 不应同时由 Isaac Lab Scene 配置和 Replicator 重复创建。本项目的机器人、环境和物体已经由 Isaac Lab 配置加载，因此不应再用 `rep.functional.create.reference()` 在相同路径加载一遍。

## Render Product 和 Writer

Render Product 表示“从哪台相机、以什么分辨率生成渲染数据”。下面是与 Isaac Sim 6.0 官方 AMR 示例一致的基本模式：

```python
render_product = rep.create.render_product(
    "/World/Robot/Camera",
    (1024, 1024),
    name="robot_camera",
    force_new=True,
)

backend = rep.backends.get("DiskBackend")
backend.initialize(output_dir="outputs/replicator")

writer = rep.writers.get("BasicWriter")
writer.initialize(backend=backend, rgb=True)
writer.attach([render_product])

rep.orchestrator.step(rt_subframes=16)
rep.orchestrator.wait_until_complete()
```

各步骤的作用是：

1. 从指定相机创建渲染输出。
2. 配置磁盘输出目录。
3. 获取并初始化 `BasicWriter`。
4. 将 Writer 绑定到 Render Product。
5. 渲染并采集一帧。
6. 等待后台数据写入完成。

如果程序位于异步环境中，可以使用：

```python
await rep.orchestrator.step_async(rt_subframes=16)
await rep.orchestrator.wait_until_complete_async()
```

使用完 Render Product 后，可以解绑 Writer 并销毁输出对象：

```python
writer.detach()
render_product.destroy()
```

## 随机化

Replicator 的经典接口适合描述按帧执行的随机化流程。例如随机物体位置：

```python
with rep.trigger.on_frame(num_frames=100):
    with objects:
        rep.modify.pose(
            position=rep.distribution.uniform(
                (-2.0, -2.0, 0.0),
                (2.0, 2.0, 0.0),
            )
        )
```

这表示在 100 个触发帧中，从给定范围的均匀分布重新采样物体位置。`rep.create`、`rep.modify` 和 `rep.trigger` 通常用于声明一套由 Replicator 执行的生成流程；`rep.functional` 则更适合按普通程序步骤直接构建和配置场景。

对于简单的 Python 随机逻辑，也可以像官方 AMR 示例一样直接使用 `random.uniform()`，然后修改 USD 属性。是否使用 `rep.distribution` 取决于随机化是否需要进入 Replicator 的触发图。

## 语义标签和碰撞体不是一回事

Replicator 可以为对象设置语义标签，例如：

```python
cube = rep.create.cube(semantics=[("class", "obstacle")])
```

语义标签用于生成语义分割、目标检测等训练标签，但不会让对象自动成为碰撞体。

| 属性 | 用途 | 是否参与占用图/物理碰撞 |
| --- | --- | --- |
| Replicator semantics | 训练数据分类与分割标注 | 否 |
| `UsdPhysics.CollisionAPI` / PhysX Collider | 物理碰撞和可通行性判断 | 是 |
| Rigid Body | 允许物体参与动力学运动 | 视碰撞配置而定 |

因此，为本项目生成占用图不要求手工添加 Replicator 语义标签。环境中真正影响占用结果的是碰撞体几何、碰撞启用状态、扫描高度和地图边界。

## 与本项目导航模块的关系

当前 pick-place 导航链路是：

```text
USD/PhysX 碰撞体
        ↓
CollisionAPI 三角网格的二维切片
        ↓
MobilityGen OccupancyMap 与安全膨胀
        ↓
A* 路径规划
        ↓
世界坐标路径点
```

对应实现位于：

- `loco_manipulation/task/pick_place/navigation.py`
- `loco_manipulation/task/pick_place/occupancy.py`
- `loco_manipulation/task/pick_place/artifacts.py`
- `loco_manipulation/task/pick_place/navigation.yaml`
- `scripts/plan_pick_place_path.py`

Replicator 在这条链路中不是必需组件。它适合在以下阶段使用：

- 给临时生成的地面或物体添加 collider；
- 随机障碍物、待抓取物体、灯光或相机；
- 为导航和抓取过程采集 RGB、深度、分割或检测标签；
- 将不同环境和物体组合成批量 SDG 数据集。

当前场景已经由 Isaac Lab 创建，因此占用图代码直接检查 `UsdPhysics.CollisionAPI`，将碰撞三角网格投影为 MobilityGen `OccupancyMap`。不要为了生成占用图而给所有物体添加语义标签，也不要仅根据视觉 Mesh 判断其是否可碰撞。

## `amr_navigation.py` 中各模块的职责

官方示例虽然名为 AMR Navigation，但导航并不是由 `omni.replicator.core` 计算的：

- Nova Carter USD 资产内部包含导航相关的 OmniGraph 和控制逻辑。
- 脚本修改 `targetXform`，为机器人设置导航目标。
- Isaac Sim 时间线驱动物理仿真和机器人运动。
- Replicator 创建/切换环境、配置部分物理属性、创建渲染输出并保存图像。
- 当机器人接近目标时，脚本暂停时间线并调用 `rep.orchestrator.step()` 采集数据。

因此不能从示例中推断 `rep` 自带 A* 或 Occupancy Map。它在该示例中的核心角色是 SDG 流程编排。

## 参考资料

- [Isaac Sim 官方 `amr_navigation.py`](https://github.com/isaac-sim/IsaacSim/blob/main/source/standalone_examples/replicator/amr_navigation.py)
- [Omniverse Replicator Core 文档](https://docs.omniverse.nvidia.com/py/replicator/1.5.0/source/extensions/omni.replicator.core/docs/README.html)
- [Isaac Sim 6.0 Python API](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/py/index.html)
