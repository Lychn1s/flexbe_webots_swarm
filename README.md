## Webots 群体机器人 FlexBE 行为包

本包为 Webots 群体机器人（swarm）项目提供 FlexBE 行为与状态，用于对不同机器人小队的运动进行统一调度控制。

当前仓库只保留了与实际任务相关的核心文件：

- `flexbe_webots_swarm_flexbe_states/team_move_state.py`
- `flexbe_webots_swarm_flexbe_states/team_wait_state.py`
- `flexbe_webots_swarm_flexbe_behaviors/swarmmoveexample1_sm.py`

### `team_move_state.py`：小队运动控制状态

**位置**：`flexbe_webots_swarm_flexbe_states/team_move_state.py`  
**作用**：向指定小队（例如 `scout`、`carrier`）发布速度控制指令，在给定时间内保持线速度和角速度，实现：

- 直线前进（`linear_x` > 0，`angular_z` = 0）
- 转弯 / 原地旋转（`linear_x` = 0，`angular_z` ≠ 0）
- 通过 `duration` 控制动作持续时间

在 FlexBE 行为中，可以通过设置：

- `team_name`：要控制的小队名称
- `linear_x` / `angular_z`：运动指令
- `duration`：本段动作持续时间  

正常完成会返回 `done`，异常时返回 `failed`。

### `team_wait_state.py`：小队等待 / 停留状态

**位置**：`flexbe_webots_swarm_flexbe_states/team_wait_state.py`  
**作用**：在行为流程中插入一段纯时间等待，用于：

- 让连续动作之间留出停顿，使行为节奏更清晰
- 等待上一个运动结束、姿态稳定或传感器数据更新

该状态只接收一个 `duration` 参数（秒），到时后返回 `done`，不发布任何运动指令。

### `swarmmoveexample1_sm.py`：Swarm Move Example 1 行为

**位置**：`flexbe_webots_swarm_flexbe_behaviors/swarmmoveexample1_sm.py`  
**行为名称**：`Swarm Move Example 1`（类 `SwarmMoveExample1SM`）

该行为使用 `TeamMoveState` 和 `TeamWaitState` 组合出了一个示例群体运动流程：

1. **Scout 小队前进 2 秒**  
   `team_name='scout'，linear_x=0.3，angular_z=0.0，duration=2.0`
2. **Scout 小队等待 2 秒**
3. **Scout 小队左转 3 秒**  
   `team_name='scout'，linear_x=0.0，angular_z=0.3，duration=3.0`
4. **Scout 小队再等待 1 秒**
5. **Carrier 小队前进 2 秒，行为结束**  
   `team_name='carrier'，linear_x=0.3，angular_z=0.0，duration=2.0`

通过这个示例可以看到：

- 使用同一套状态即可控制不同角色（如 `scout` / `carrier`）的小队
- 通过“运动状态 + 等待状态”的组合，可以拼装出清晰可读的群体任务序列

### 使用说明（简要）

1. 确保本包已在 ROS2 中成功构建并安装。  
2. 在 FlexBE App 中加载本行为包后，可以直接运行 `Swarm Move Example 1` 行为，观察 Webots 中各机器人小队的动作。  
3. 可以以当前三个核心文件为模板，继续添加更多小队控制状态与更复杂的 swarm 行为。

说明：原始仓库中与 `example_*` 相关的通用示例状态和行为文件已被删除，只保留与 Webots 群体机器人项目直接相关的实现，以避免示例代码污染仓库。