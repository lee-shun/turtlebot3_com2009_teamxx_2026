# Task 2: Avoiding Obstacles - 任务需求文档

## 任务概述

开发 ROS node(s) 使真实的 TurtleBot3 Waffle 能够自主探索包含各种障碍物的环境。机器人必须在 **90 秒** 内尽可能多地探索环境，且不能碰撞到任何东西！

---

## 前置要求

- 完成 Assignment #1 的 **Part 3** 及之前内容
- 参考资源：
  - [Understanding the Waffles](../../../../waffles/essentials/) - Motion and Velocity Control, LiDAR Sensor
  - COM2009 Lecture 3: Cybernetic Control Principles, Braitenberg's "Vehicles" (特别是 Vehicle 3b)
  - Lecture 8: Local Guidance Strategies (Brownian Motion, Levy Walks)

---

## 环境详情

### 竞技场规格
- **尺寸**: 4x4m 正方形竞技场 (Diamond Computer Room 5 Robot Arena)
- **障碍物**: 短木墙和彩色圆柱体
- **探索区域**: 12 个 1x1m 的外围区域

### 重要说明
- 障碍物位置每次可能完全不同
- 木墙不一定与竞技场外边缘接触
- 彩色圆柱体可能在探索区域内
- 唯一不变的是：竞技场尺寸、外墙壁位置、地板布局（区域位置）

### 机器人起始位置
- 竞技场中心
- 垂直于四面外墙壁之一

---

## 任务要求

1. **探索时间**: 90 秒内探索环境
2. **不碰撞**: 不能触碰任何竞技场墙壁或内部障碍物
3. **计时规则**: 机器人开始在竞技场内移动时，90 秒计时器启动
4. **碰撞处理**: 如果在时间结束前碰撞到任何东西，尝试结束，记录碰撞时间用于评分
5. **探索区域**: 尽可能进入 12 个探索区域中的每一个
6. **持续移动**: 机器人必须在整个任务期间保持移动
   - 可以短暂停止并原地转向几秒钟
   - 如果机器人探索一段时间后停止并不再移动，Run Time 评分只计算到停止点

---

## 代码执行方式

教学团队将使用以下命令执行代码：

```bash
ros2 launch com2009_teamXX_2026 task2.launch.py
```

其中 `XX` 是团队编号。

### 要求
- 必须包含名为 `task2.launch.py` 的 launch 文件
- ROS 已在机器人上运行
- Zenoh Session 已在笔记本电脑上运行

---

## 依赖限制

- 可以使用现有的 Python 库或 ROS 2 包
- **禁止使用 Nav2**
- 详细依赖限制见 [Assessment Requirements](../../assessment/#dependencies)

---

## 评分标准 (总分 20 分)

### A. Run Time (运行时间) - 8 分

**要求**: 机器人必须离开中心区域 (1x1m 红色区域) 才能获得此部分分数。

**惩罚**: 如果机器人没有超出"部分探索区域"(橙色区域)，Run Time 分数将乘以 0.5 系数。

| 时间 (秒) | 分数 |
|---------|------|
| 0-9     | 0    |
| 10-19   | 1    |
| 20-29   | 2    |
| 30-39   | 3    |
| 40-49   | 4    |
| 50-59   | 5    |
| 60-89   | 6    |
| 完整的 90 秒 | 8    |

### B. Exploration (探索) - 12 分

- 每进入一个探索区域得 1 分
- 共 12 个区域，满分 12 分
- 每个区域只需进入一次
- **机器人整个车身必须完全进入区域标记内**才能获得该区域分数

---

## 仿真资源

### 仿真环境
- 包名：`tuos_task_sims`
- 启动命令：
```bash
ros2 launch tuos_task_sims obstacle_avoidance.launch.py
```

### 注意事项
- 仿真环境仅作为真实环境的示例
- **仿真中能工作不代表真实环境中也能工作**
- 务必在实验室的真实机器人上充分测试
- 确保使用最新版本的 Course Repo

---

## 技术建议

### 可选方法
1. **基于 LiDAR + Action Framework**: Assignment #1 Part 5 的方法
2. **Braitenberg Vehicle 3b**: 简单的避障行为
3. **搜索策略**: Brownian Motion, Levy Walks 等

### 关键传感器
- LiDAR 传感器：检测环境中物体的距离
- 参考 Assignment #1 Part 3 了解 LiDAR 使用方法

---

## 检查清单

- [ ] 创建 `task2.launch.py` launch 文件
- [ ] 实现避障功能
- [ ] 实现探索策略
- [ ] 确保机器人持续移动
- [ ] 在仿真中测试
- [ ] 在真实机器人上测试
- [ ] 确保不使用 Nav2
- [ ] 验证机器人能离开中心区域
- [ ] 验证机器人能进入多个探索区域
- [ ] 测试至少 90 秒不碰撞
