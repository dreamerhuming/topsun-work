# Pickup 轮廓侧向/航向约束设计（最小改动）

## Goal

在不动 `pickup3` 状态机、世界锚点、odom 外推的前提下，用车尾点云的
**后缘轮廓特征**补强现有 ICP 对 **侧向 y / 航向 yaw** 的弱可观性，降低
`charge_info` 世界系 y、yaw 抖动，同时保持纵向 x 仍由模板配准主导。

主验收包：`2026-07-21_10-42-46`（及既有 SIL 回归集）。成功判据相对当前
`pickup3` 基线：

1. tracking 段世界系 `pickup_wy` 相邻跳变均值 / P95 明显下降；
2. 世界航向散布下降，且不出现新的系统性偏航；
3. ICP 接受率不显著下降，错关联/创新门拒绝不显著上升；
4. 近距 `odom_only` 行为不变。

## Evidence and Root Cause

当前 `Pickup3Lidar::Refine` 用后脸模板做 SE(2) 网格 / FastGICP。后脸点云对
纵向距离敏感，对横向平移和小 yaw 的残差变化弱；SIL 上表现为局部/世界 **y
抖动大、yaw 抖**，世界跳变常顶到 `max_pickup_innovation_m`（0.15 m）。

轮廓/角点对左右缘更敏感，正好补这一侧。问题不是“再换一个匹配器”，而是
**观测分工**。

## Considered Approaches

### 1. 用轮廓完全替换 ICP

轮廓直接出 `(x,y,yaw)` 写 `charge_info`。侧向可能更好，但缺稠密一致性校验，
遮挡/假角点会直接污染锚点；近距碎轮廓风险更大。**不采用。**

### 2. 轮廓与 ICP 加权融合出最终位姿

每帧对两路结果做固定权重平均。实现简单，但错误相关时更飘，且破坏现有
“过门才刷新锚点”的语义。**不采用（首版）。**

### 3. 轮廓只提供 `(y, yaw)` 先验，ICP 负责 `x` 与一致性验收（采用）

在现有 Stage 2 裁剪之后、Stage 4 搜索之前，从 `icp_cloud` 提后缘轮廓，估出
相对 BEV 车尾系的 `d_lateral` / `dyaw`。把它当作与 `prev_hint` 同级的
**暖启动候选**参与现有误差竞争；最终仍走原有置信度/误差/修正量门控。
可选再加一条“轮廓与 ICP 侧向/航向一致性”软门，不一致则本帧不刷新锚点。

改动面小，可读性好，失败时行为退回今天的 ICP-only。

## Design Principles

1. **最小插入点**：只改 `pickup3_lidar` 配准链；`pickup3.cpp` FSM / 锚点逻辑不动。
2. **扁平可读**：轮廓逻辑放在独立文件，对外两个清晰步骤函数；`Refine()` 里只多
   一段顺序代码，不加深 lambda / 嵌套分支。
3. **失败静默降级**：轮廓无效时 `Refine` 行为与今天完全一致。
4. **不改 crop 闭环**：首版不用轮廓结果重裁点云（历史“修正位姿重裁”曾正反馈，
   已打回）。轮廓只参与修正量候选。
5. **开关默认关或保守开**：YAML 可关，便于 A/B。

## Architecture

```text
现有 Refine 流水线（插入点用 * 标出）

Stage1  前置检查
Stage2  BEV 框裁剪 → 高度 → 车尾半区 → icp_cloud
Stage3  BEV 初值位姿 + scan|init
*Stage3.5  轮廓观测：icp_cloud → (d_lat, dyaw) 候选   [新]
Stage4  hint / 粗网格 / GICP / 兜底网格 / polish
        * 把轮廓候选当作额外 warm-start，用同一 ScanToTemplateError 竞争
Stage5  置信度
Stage6  原有门控
        * 可选：轮廓与最终 corr 的 |Δy|/|Δyaw| 一致性门
输出    pickup / heading / applied_corr（接口不变）
```

`pickup3` 外层：仍只看 `Pickup3RefineResult.valid`；无需改 HandleControl。

## Contour Observation（算法）

坐标系：与现有 `PlanarRearCorrection` 一致——车体前向 / 左向 / yaw 逆时针；
修正量相对 **BEV 车尾参考点**（`T_rear_in_baselink`），不是模板原点。

### 输入

- `icp_cloud`（baselink，已裁车尾）
- `T_rear_in_baselink`（BEV 初值车尾位姿）
- 配置：高度带、BEV 投影分辨率、最小点数、角点质量门

### 步骤（顺序、无深嵌套）

1. **变到车尾局部系**  
   用 `T_rear_in_baselink.inverse()` 把 `icp_cloud` 变到 rear-local：
   `+x` 朝车头，`+y` 朝左，车尾附近 `x≈0`。

2. **取后缘带**  
   保留 `x ∈ [x_min, x_max]`（默认约 `[-0.15, 0.25]` m）且已在 ICP 高度带内的点。
   点数不足 → 轮廓无效。

3. **BEV 栅格占有**  
   将点投到 rear-local XY 栅格（分辨率默认 0.05 m），每格记最小 `x`（最近后缘）。

4. **提取后缘折线**  
   对每个有点的 `y` 列取最小 `x`，得到折线 `(x_edge[y], y)`。
   左右各丢掉过稀的列；列数不足 → 无效。

5. **估 yaw**  
   对折线做稳健直线拟合（简单实现：中段点最小二乘；可选去掉左右 10% 离群列）。
   `dyaw = atan2` 得到的后缘法向相对 `+x` 的偏角，并夹紧到 `max_yaw_correction_deg`。

6. **估侧向**  
   在拟合直线上取左右端点（或占有 y 的 min/max），中点相对 `y=0` 的偏移即为
   `d_lateral`。车尾宽度相对名义半宽偏差过大则降置信或判无效。

7. **纵向不估**  
   `d_longitudinal = 0`。轮廓首版不争 x，避免和 ICP 抢纵向可观性。

### 输出结构

```cpp
struct Pickup3ContourObs {
  bool valid = false;
  float d_lateral_m = 0.f;
  float dyaw_rad = 0.f;
  float quality = 0.f;   // 0~1，供日志；不过度设计成第二套置信度体系
  std::string reason;    // empty / too_few / edge_short / width_outlier ...
};
```

## Integration in Refine（可读编排）

新增文件（建议）：

- `pickup3/pickup3_contour.h`
- `pickup3/pickup3_contour.cpp`

对外只暴露两个函数，避免类状态机嵌套：

```cpp
// 1) 从车尾裁剪点云估侧向/航向观测（失败返回 valid=false）
Pickup3ContourObs EstimateRearContourObs(
    const pcl::PointCloud<pcl::PointXYZ>& icp_cloud_baselink,
    const Eigen::Matrix4f& T_rear_in_baselink,
    const PickupContourConfig& cfg);

// 2) 把轮廓观测写成可参与竞争的平面修正（longitudinal 固定 0）
PlanarRearCorrection ContourToCorrection(const Pickup3ContourObs& obs);
```

`Refine()` 中插入方式（示意，保持与现有 Stage 注释同风格）：

```cpp
// ===== Stage 3.5: 轮廓侧向/航向观测（可选）=====
Pickup3ContourObs contour_obs;
if (config_.contour_enable) {
  contour_obs = EstimateRearContourObs(*icp_cloud, T_rear_in_baselink, config_.contour);
}

// ===== Stage 4a 之后追加：轮廓候选与 hint/网格同一套误差竞争 =====
if (contour_obs.valid) {
  PlanarRearCorrection contour_corr = ContourToCorrection(contour_obs);
  contour_corr.dyaw_rad = std::clamp(contour_corr.dyaw_rad, -max_yaw_rad, max_yaw_rad);
  const float scan_err_contour = evaluator.ScanToTemplateError(contour_corr);
  if (scan_err_contour < best_err) {
    corr_used = contour_corr;
    best_err = scan_err_contour;
    used_contour_prior = true;
  }
}
```

要点：

- **不**把轮廓逻辑塞进 GICP / 网格函数内部。
- **不**为轮廓再开一套并行 Refine 分支；只多一个候选。
- 跟踪 hint 仍优先按现有 `ShouldUseTrackingHint` 判定；轮廓是额外竞争者，
  谁误差低用谁，规则与 coarse grid 一致，读起来不需要新状态机。

### 可选一致性门（Stage 6 末尾，默认开）

若本帧轮廓有效且最终 `corr_used` 被接受前：

- `|corr_used.d_lateral_m - contour_obs.d_lateral_m| > contour_max_lateral_disagree_m`
  或 `|Δyaw| > contour_max_yaw_disagree_deg` → `valid=false`，
  `reason=contour_icp_disagree`。

含义：ICP 与轮廓在弱可观自由度上打架时，宁可不刷新锚点（上层 odom 外推），
也不写入可疑侧向。该门可用 YAML 关掉，方便对比。

## Config

挂在现有 `pickup_lidar:` 下，解析进 `PickupLidarConfig`（或内嵌
`PickupContourConfig`）：

| 键 | 默认 | 含义 |
|----|------|------|
| `contour_enable` | `false` 首版 / 验证后可 `true` | 总开关 |
| `contour_x_min_m` | `-0.15` | rear-local 后缘带 |
| `contour_x_max_m` | `0.25` | |
| `contour_grid_res_m` | `0.05` | BEV 栅格分辨率 |
| `contour_min_columns` | `8` | 最少有效 y 列 |
| `contour_min_points` | `30` | 后缘带最少点数 |
| `contour_nominal_half_width_m` | `0.7` | 名义半宽，过宽/过窄判异常 |
| `contour_width_tol_m` | `0.25` | 半宽容差 |
| `contour_agree_enable` | `true` | 与 ICP 一致性门 |
| `contour_max_lateral_disagree_m` | `0.12` | |
| `contour_max_yaw_disagree_deg` | `8.0` | |

首版默认 `contour_enable: false`，合并进主流程前用同一 MCAP 开/关对比后再改默认。

## Diagnostics

在现有 `pickup_icp_match.txt` / 帧日志追加一行即可，不新开话题：

```text
contour_valid=1 d_lat=... dyaw_deg=... quality=... reason=...
used_contour_prior=0/1 contour_scan_err=...
contour_agree=1/0
```

`Pickup3RefineResult` 增加少量字段（可选，便于 SIL）：

- `bool contour_valid`
- `bool used_contour_prior`
- `float contour_d_lateral_m`
- `float contour_dyaw_rad`

不强制改 ROS 消息。

## File Change List（最小）

| 文件 | 改动 |
|------|------|
| `pickup3/pickup3_contour.h/.cpp` | **新增** 轮廓观测 |
| `pickup3/pickup3_lidar.cpp` | Stage 3.5 调用 + Stage 4 候选竞争 + 可选同意门 |
| `pickup3/pickup3_lidar.h` | result 诊断字段（可选） |
| `common/data_type_inner.h` | 配置字段 |
| `common/config.cpp` | YAML 解析 |
| `config/charge_detection_node.yaml` | 默认参数 |
| `CMakeLists.txt` / `fusion_sil.pro` | 编译进新 cpp |
| `tests/pickup_contour_test.cpp` | **新增** 合成后缘折线单测 |

**不改**：`pickup3.cpp`、世界锚点、创新门阈值、模板 PCD、crop 几何（首版）。

## Readability Rules for Implementation

1. `EstimateRearContourObs` 内部用线性步骤 1→7，每步一个局部变量块，禁止
   三层以上 `if` 嵌套；失败用早返回。
2. 不把轮廓估计算进 `RunFastGicpPrimary` / `RunPlanarGridSearch`。
3. `Refine` 里轮廓相关代码集中在两处注释块（3.5 与 4 候选），不要散落。
4. 不引入回调、策略多态、多状态 enum 机；一个 `bool valid` 足够。

## Test Plan

### 单测

1. 合成矩形后缘点云（已知 `d_lat=0.10`, `dyaw=5°`）→ 观测误差在栅格容差内。
2. 空云 / 单侧残缺 → `valid=false`，reason 明确。
3. 轮廓候选写入 `PlanarRearCorrection` 后，`ScanToTemplateError` 可调用不崩溃。

### SIL / 回灌

1. 同一二进制、同一 MCAP：`contour_enable=false` 作基线。
2. `contour_enable=true`，对比：
   - 世界 `wy` 相邻 `|dxy|` 均值/P95
   - 世界 yaw 散布
   - `valid` 率、`contour_icp_disagree` 占比
   - 创新门拒绝次数
3. 主包：`2026-07-21_10-42-46`；再抽 1–2 个既有回归包防回退。

### 接受标准

- 世界侧向稳定性明显优于基线（建议目标：tracking 段 `wy` P95 跳变降 ≥30%，
  且无新的持续偏置肉眼可见）。
- 接受率下降 &lt; 5 个百分点，或下降但世界稳定性收益足够大且无错车锁定。
- 关闭开关后数值路径与改前一致（或仅多空日志字段）。

## Non-Goals（本轮不做）

- 不用轮廓结果重裁 `icp_cloud`
- 不估轮廓纵向、不做 3D 面元分割 / RANSAC 多平面
- 不改 `pickup3` 三态 FSM，不恢复旧 stabilizer
- 不换模板、不改外参、不改 ROS `charge_info` 字段
- 不做 EKF 多传感器融合

## Rollout

1. 实现 + 单测，默认开关关。
2. SIL 开/关对比主包，调 `disagree` 门与后缘带。
3. 指标达标后默认改为 `true`，保留一键关闭。
4. 若轮廓噪声大：先关 `contour_agree`，只保留 warm-start；仍差则保持默认关，
   仅作实验分支。
