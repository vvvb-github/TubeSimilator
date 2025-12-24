# 试管流水线 tick 模拟器（Conveyor Simulator）

这是一个可运行的 Python 项目，用于模拟你描述的工业流水线：

- 轨道（闭环，两条环状轨道组成“回字形”）
- 被动基座（无动力，随轨道运动；可空或装载试管）
- 站台（S 检验站台、I 输入输出站台）
- 换向器/换向盘（C，占据两条相邻轨道的相邻格，默认堵住；通过旋转缺口搬运基座，实现放行或换道）

项目包含：

- `tick` 级离散仿真内核（可检查非法操作并告警）
- YAML 配置（轨道布局、站台属性、输入试管流、参数等）
- 一个简单调度算法（每 tick 观察系统状态并控制 C/S/I）
- 2D 可视化（**pygame 实时动画**：基座平滑移动、换向器指示线平滑转向、背景轨道“滚动点”效果；也保留 matplotlib 版本用于无 pygame 环境）

---

## 1. 运行方式

### 1.1 可视化运行（默认 pygame）

```bash
python run_demo.py --config example_config.yaml
```

pygame 窗口快捷键：

- `SPACE`：暂停/继续
- `N`：暂停状态下单步（走 1 tick）
- `ESC`：退出

你也可以选择不同调度算法（用于对比试验）：

```bash
# 原始算法（同道放行 + 站台堵塞）
python run_demo.py --config example_config_extended.yaml --scheduler simple

# 新算法（内环快车道 + 外环多 buffer + 换道/释放）
python run_demo.py --config example_config_extended.yaml --scheduler buffering
```

如果你更想用旧的 matplotlib 可视化：

```bash
python run_demo.py --config example_config.yaml --viz mpl
```

常用参数：

```bash
python run_demo.py --config example_config.yaml --max-ticks 800 --tick-ms 400 --fps 60
```

- `--max-ticks`: 模拟 tick 数
- `--tick-ms`: (pygame) 每个 tick 的真实时间长度（毫秒）
- `--fps`: (pygame) 渲染 FPS

matplotlib 参数（`--viz mpl` 时生效）：

- `--subframes`: 每个 tick 的动画插帧数量（越大越平滑）
- `--interval-ms`: 每帧间隔（毫秒）

### 1.2 无可视化运行（命令行）

```bash
python run_demo.py --config example_config.yaml --no-viz --max-ticks 600
```

---

## 2. 配置文件（YAML）

示例：`example_config.yaml`

### 2.1 grid.rows：轨道 ASCII 布局

- 支持两种写法：
  - **空格分隔 token**（如 `example_config.yaml`）：每行用空格分隔单格 token。
  - **紧凑字符画**（如 `example_config_extended.yaml`）：每行是一个等宽字符串；用 `.` 表示空白格（非轨道）。
- token：
  - `=`、`|`：轨道格
  - `C`：换向器格（**注意：每个 C 必须与一个相邻 C 成对出现**，代表一个换向器占据两条轨道）
  - `S`：检验站台格（站台的**下游必须紧跟一个 C**，用于堵住基座并交互）
  - `I`：输入输出站台格（同样要求下游紧跟 C）

### 2.2 simulation：参数

- `base_count`：基座数量（默认 10）
- `seed`：基座初始随机分布的随机种子
- `diverter_ticks_per_90`：换向器旋转 90 度所需 tick（默认 1）

### 2.3 stations：站台定义

- 按照 grid 中 `S` 的扫描顺序（从上到下、从左到右）依次绑定。
- `projects`：该站台可做的检验项目
- `interaction_ticks`：交互耗时

### 2.4 io：输入输出站台定义

- grid 中必须且只能有一个 `I`。

### 2.5 tube_arrivals：带时间戳的试管输入流

- `time`：到达 I 缓冲区的 tick
- `tube_id`：试管 ID
- `project`：需要检验的项目（示例中每个试管只做一个项目）

---

## 3. 核心仿真规则实现说明

### 3.1 轨道/基座运动

- 每 tick 轨道带动未堵住的基座前进 1 格。
- 堵住判定（实现版本）：
  - 基座的“前方格”是 `C`（换向器格）时，**永远堵住**（除非调度算法下发换向器搬运动作）。
  - 若前方是一个“被堵住的基座”，则本基座也堵住（堵塞向后传播）。

> 这使得连续紧贴的基座会一起移动；只有前车被 `C` 卡住时才会造成整段排队。

### 3.2 换向器 C

- C 以**成对**出现：两个相邻的 `C`（通常上下相邻）构成一个换向器，占据两条轨道。
- 默认堵住：基座不能靠轨道运动进入 `C` 格。
- 通过下发 `DiverterTransfer(diverter_id, src_side, dst_side)` 执行搬运：
  - `src_side`：从哪条轨道的上游抓取基座
  - `dst_side`：搬运到哪条轨道的下游释放基座
  - `src_side == dst_side`：直行放行
  - `src_side != dst_side`：换道
- 时间模型（与题述示例对齐的简化版）：
  - 直行：耗时 = `diverter_ticks_per_90`
  - 换道：耗时 = `4 * diverter_ticks_per_90`
- 搬运期间基座会进入换向器“货舱”（可视化显示在换向器中心），结束后落到目标下游格。

### 3.3 站台 S / I

- 站台必须位于其 stop-diverter（下游 C）**上游一格**，以便 C 堵住后交互。
- `StationStart(station_id, op)`：
  - `op="test"`：S 站台执行检验（消耗 `interaction_ticks`），完成后在 tube 状态里记录已完成项目
  - `op="load"`：I 站台把输入缓冲区的下一根试管装入基座
  - `op="unload"`：I 站台把已完成试管从基座取出（进入输出列表）

---

## 4. 调度算法（SimpleScheduler）

位于 `conveyor_sim/scheduler.py`

策略很简单：

- 对每个换向器：默认“直行放行”，让基座持续流动
- 需要交互时（I 装载/卸载、S 检验）：
  - 让基座停在站台格（不对 stop-diverter 下发 transfer）
  - 站台空闲则下发 `StationStart`
  - 完成后再放行

---

## 5. 告警与合法性校验

### 5.1 配置校验（启动时）

- 必须且只能有一个 `I`
- `S` 数量必须等于 `stations` 定义数量
- 每个 `S/I` 的下游格必须是 `C`
- 每个 `C` 必须恰好与一个相邻 `C` 配对（形成换向器）
- 轨道拓扑必须形成两条闭环（两个连通分量、每个节点度数为 2）

### 5.2 运行时告警（每 tick）

- 对 busy 的换向器重复下发 transfer
- 换向器抓取点无基座
- 换向器释放点被占用
- 对站台在无基座/基座空/项目不匹配等情况下发起交互
- 两个基座占据同一格（碰撞）

告警会显示在可视化窗口左上角，也会在 `--no-viz` 模式下打印。

---

## 6. 二次开发建议

你可以在以下方向继续增强：

- 更精细的换向器角度/误抓取模型（允许指定顺/逆时针，转动过程中检查经过的抓取点）
- 在配置中显式定义每个换向器的几何/抓取点顺序，而不是使用简化的 1/4 时间模型
- 更高级的调度算法（例如：使用内圈作为 bypass、考虑堵塞成本、优化吞吐）

