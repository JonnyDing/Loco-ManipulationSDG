# Loco-ManipulationSDG

## 运行环境

- Ubuntu Linux（x86_64）及受支持的 NVIDIA GPU
- Python 3.12
- Isaac Sim 6.0.1 系列（配置固定为 `6.0.1.0`）
- Isaac Lab 3.0.0 系列（配置固定为 `3.0.0`）

## 创建 Conda 环境

```bash
conda env create -f environment.yml
conda activate loco-manipulation-sdg
```

## Pick-place 导航路径

从当前 USD 中启用的碰撞 Mesh 生成占用图，并为指定的 pick-place 起点和终点姿态规划路径：

```bash
python scripts/plan_pick_place_path.py --device cuda:0
```

起终点 pose 通过命令行传入，旋转采用 Isaac World `wxyz` 四元数：

```bash
python scripts/plan_pick_place_path.py --device cuda:0 \
  --start_world_xyz 0.0 0.0 0.0 \
  --start_rot_wxyz 1.0 0.0 0.0 0.0 \
  --goal_world_xyz -0.1 -2.65 0.0 \
  --goal_rot_wxyz 0.7071 0.0 0.0 -0.7071
```

默认输出目录为 `outputs/pick_place/navigation/`，包含 ROS 格式的
`map.png`、`map.yaml`，带安全缓冲的 `map_buffered.png`，以及
`path.png`、`path.json`。地图范围、分辨率、安全距离和路径采样参数位于
`loco_manipulation/task/pick_place/navigation.yaml`；可通过
`--navigation_cfg path/to/navigation.yaml` 使用其他配置文件。

占用图只读取 `UsdPhysics.CollisionAPI`，不读取 Replicator 语义标签，因此场景不需要额外的人工语义标注。详细设计与参数说明见 [`docs/navigation.md`](docs/navigation.md)。

## 开发文档

- [`omni.replicator.core` 使用说明](docs/replicator_core.md)：介绍 Isaac Sim 6.0 中 Replicator 的场景构建、物理配置、随机化和数据采集接口，以及它与占用图和 A* 导航的职责边界。
- [Pick-place 导航设计](docs/navigation.md)：介绍碰撞体筛选、占用图切片、机器人安全半径、A* 和输出产物。
