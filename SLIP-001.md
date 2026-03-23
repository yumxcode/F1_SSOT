# SLIP-001 | 双足跑步 | 探索线B：SLIP方案 | 2026.03.01

## 1. 实验假设

初次尝试 SLIP（Spring-Loaded Inverted Pendulum）方案实现腾空相。

假设：通过在 walk 框架上叠加 SLIP 启发的参考轨迹（含 duty_factor 控制的腾空窗口），配合显式腾空奖励，策略可以从头学出具有腾空相的跑步步态，无需走路 checkpoint。

核心设计逻辑：walk/run 在 `run_vel_threshold=1.4 m/s` 处软插值过渡，duty_factor 随速度从 0.45（v=1.4）下降到 0.36（v=2.0），腾空相占比约 20%。

## 2. 代码变更

### 2a. 参数调整（L1）

| 参数名 | 所在位置 | 旧值 | 新值 | 变更理由 |
|--------|---------|------|------|---------|
| `cycle_time` | `class rewards` | 0.7 | 0.65 | 统一 walk+run 步态周期 |
| `run_vel_threshold` | `class rewards` | 无 | 1.4 | m/s，超过此速度激活 run reference |
| `run_duty_factor_base` | `class rewards` | 无 | 0.45 | v=1.4 时 duty，轻微腾空 |
| `run_duty_factor_min` | `class rewards` | 无 | 0.36 | v=2.0 时 duty，明显腾空 |
| `run_stance_knee_scale` | `class rewards` | 无 | 0.14 | 支撑相轻屈膝 delta，rad |
| `run_stance_ankle_scale` | `class rewards` | 无 | 0.20 | 踝蹬伸幅度 delta，rad |
| `run_flight_knee_bend` | `class rewards` | 无 | 0.60 | 腾空相屈膝峰值 delta，rad |
| `run_base_height_stance` | `class rewards` | 无 | 0.60 | 支撑相身高目标，m |
| `run_base_height_flight` | `class rewards` | 无 | 0.63 | 腾空相身高目标，m |
| `ref_joint_pos` scale | `class scales` | 2.2 | 1.8 | run 轨迹是软约束，降低束缚 |
| `feet_air_time` scale | `class scales` | 1.2 | 2.5 | run 核心奖励，大幅提高 |
| `feet_contact_number` scale | `class scales` | 2.0 | 1.5 | 放开步态约束以允许腾空 |
| `torques` scale | `class scales` | -8e-9 | -4e-9 | run 需更多能量，放宽力矩惩罚 |
| `dof_acc` scale | `class scales` | -1e-7 | -5e-8 | 跑步关节加速度更大，放宽惩罚 |
| `action_smoothness` scale | `class scales` | -0.002 | -0.001 | 适应跑步更剧烈的动作变化 |
| `flight_phase` scale | `class scales` | 无 | 1.5 | 新增腾空相奖励 |
| `slip_base_height` scale | `class scales` | 无 | 0.3 | 新增动态身高奖励 |
| `num_single_obs` | `class env` | 47 | 49 | 新增 flight_flag 和 duty_norm 两维 |
| `lin_vel_x` range | `class commands` | [-0.4, 1.2] | [-0.4, 2.0] | 给跑步速度空间 |
| `gait` | `class commands` | 不含 run | 含 run | 新增 run gait 类型 |
| `gait_time_range["run"]` | `class commands` | 无 | [4, 8] | run 阶段持续时间 |

### 2b. Reward 函数修改（L2）

| 函数名 | 修改描述 | 变更理由 |
|--------|---------|---------|
| `_get_stance_mask` | 替换为支持 walk/run 双模式的统一版本，设置 `self.flight_phase_flag` | SLIP 相位框架需要独立的 duty_factor 驱动支撑/腾空时序 |
| `compute_ref_state` | 替换为 walk/run 软插值版本，在 threshold 附近平滑过渡 | 避免 reference 在 walk→run 切换时跳变 |
| `_reward_feet_contact_number` | 新增 run 腾空相分支：腾空时期望 0 接触 | 原版在腾空时会惩罚双脚离地，直接压制腾空相 |
| `_reward_feet_air_time` | run 模式 air_time 上限 0.5→0.65s | 跑步腾空时间比走路更长，原上限不够 |

### 2c. 新增 Reward（L3）

| 新函数名 | 初始 scale | 设计意图 |
|---------|-----------|---------|
| `_reward_flight_phase` | 1.5 | 显式奖励 run 模式下双脚同时腾空；期望腾空+实际腾空=+1，非腾空期乱跳=-0.5 |
| `_reward_slip_base_height` | 0.3 | 动态身高奖励，支撑相目标 0.60m，腾空相目标 0.63m，替代原固定目标 |
| `_get_slip_duty_factor` | 无（辅助函数） | 基于速度指令计算 duty_factor，供 `_get_stance_mask` 和 `_compute_slip_ref` 调用 |
| `_compute_slip_ref` | 无（辅助函数） | SLIP 跑步关节参考轨迹计算，含髋/膝/踝三关节分相轨迹 |
| `_resample_run_command` | 无（命令采样） | run gait 命令采样，前向速度 [1.4, 2.0] m/s |

**obs 新增两维：**
- `flight_flag`：当前是否处于腾空相（0/1）
- `duty_norm`：归一化 duty_factor，供 policy 感知跑步强度

## 3. 训练曲线摘要
<!-- 基于截图与测试表现综合得出 -->

- **总训练步数**：基于截图（约 10000 步+）
- **平均 episode 长度（收敛时）**：较长（未出现频繁早停）
- **是否收敛**：已收敛，但陷于局部最优（仅达成稳定步行，未完整学会跑步）
- **收敛步数（估算）**：前期已收敛

### 核心指标（A组）
| 指标 | 收敛值 | 趋势 | 异常 |
|------|-------|------|------|
| `rew_tracking_lin_vel` | 较低 | 稳定但有较大静态误差 | 严重限速：指令1.6仅0.92，指令2.3仅1.3 |
| `rew_tracking_ang_vel` | 正常 | 平稳 | |
| `rew_ref_joint_pos` | 正常 | 平稳 | 机器表现为从容步行 |
| `rew_orientation` | 正常 | 平稳 | 走得非常稳 |
| `rew_feet_contact_number` | 偏高 | 稳定 | 高速下依然呈双支撑相特征（步行步态） |
| `rew_stand_still` | 正常 | 平稳 | |

### 步态质量指标（B组）
| 指标 | 收敛值 | 趋势 | 异常 |
|------|-------|------|------|
| `rew_feet_clearance` | 正常 | 平稳 | |
| `rew_feet_air_time` | 偏低 | 未见突破 | 仅在2.3m/s指令时略有提升 |
| `rew_feet_distance` | 正常 | 平稳 | |
| `rew_knee_distance` | 正常 | 平稳 | |
| `rew_feet_rotation` | 正常 | 平稳 | |
| `rew_base_height` | 正常 | 平稳 | |

### 主要惩罚项（D组）
| 指标 | 均值（绝对值） | 是否异常 |
|------|-------------|---------|
| `rew_action_smoothness` | 正常 | |
| `rew_collision` | 正常 | |
| `rew_foot_slip` | 正常 | |
| `rew_dof_pos_limits` | 正常 | |

### 新增指标（L3）
| 指标 | 收敛值 | 量级是否合理 | 备注 |
|------|-------|------------|------|
| `rew_flight_phase` | 接近 0 | 偏低 | 关键指标：未能在 1.6m/s 以下激发腾空相 |
| `rew_slip_base_height` | 正常 | 合理 | 能稳定行走说明身高目标可达 |

### 异常区间记录
- 普遍限速：高速指令阶段，为避免腾空或剧烈动作惩罚，速度被钳制。

## 4. 姿态观察

- **截图路径**：`assets/Snipaste_2026-03-02_18-40-37.png` 等
- **对应训练步数**：训练收敛后测试
- **AI 姿态分析**：基于测试描述：低于 1.6m/s 时呈现完美的稳定步行步态。2.3m/s 时出现轻微腾空和双脚支撑过渡，说明高速度惩罚或时间限制在抑制自然步态生长。
- **人工口头描述补充**：0.8m/s指令达0.6m/s；1.4m/s指令达0.8m/s；1.6m/s指令达0.92m/s（受限于命令时间限制被要求降速）；2.3m/s指令时升至1.3m/s，略微有腾空相，也略微有双支撑相。

**姿态评分表：**
| 姿态维度 | 得分 | 备注 |
|---------|------|------|
| 躯干前倾程度（5=直立） | 5 | 完美行走，非常稳 |
| 步态对称性（5=完全对称） | 5 | 对称性良好 |
| 落脚稳定性（5=稳定无滑） | 5 | 非常稳 |
| 抬脚充分性（5=无拖步） | 4 | 未见异常，但高速步幅未打开 |
| 整体流畅度（5=自然流畅） | 4 | 行走流畅，但强制降速说明高速能力受压制 |
| **总分** | **23/25** | |

## 5. 问题与发现

- **[问题] 严重的速度受限 (Velocity Mismatch)** → `rew_tracking_lin_vel` 佐证。不仅 1.4m/s 跑不到，甚至要求 1.6 才达到 0.92。
- **[问题] 腾空相缺失** → `rew_flight_phase` 与 `rew_feet_air_time` 佐证。不到极端指令（2.3m/s），系统死守步行策略，具备较高的保守性。
- **[发现]** “时间限制被要求降速”由于 play.py 里的测速循环切分了时间，使得在某些速度下没完全加速就被切走。但是机器人能在2.3m/s下稳定跑到1.3m/s并涌现一丝腾空迹象，说明策略里确实包含了向更高速度和腾空相演化的潜力。

## 6. 结论

- **假设验证结果**：未完全成立。叠加 SLIP 启发的参考轨迹和腾空奖励，没能激发出显式的 1.4m/s 腾空奔跑步态。
- **原因分析**：
  1. 速度跟踪奖励可能不足以克服探索腾空相所带来的巨大潜在惩罚（跌倒、力矩惩罚、关节速度惩罚）。机器人发现“我不跑/我降速”受到的总惩罚，比“我去尽力满足高抬腿腾空”更小。
  2. 速度指令时间短，还没充分加速进入腾空相即发生阶段切换。
- **可复用的发现**：机器人在低速区间依然具备强鲁棒性，新增的 state (flight_flag) 和 SLIP reward 没有破坏原有的 walking 能力。
- **遗留问题**：速度为什么上不去？是因为 tracking reward scale 不够大，还是 play.py 指令驻留时间不足？如何破除1m/s左右的速度墙？

## 7. 下次实验方向

- 建议使用 rl-advisor 进一步深入分析。考虑增强追踪奖励宽度或放宽罚项以突破“速度墙”限制。
- 备注：方案A（人类重定向跑步数据）数据优化完成后，可与 SLIP-001 结果对比

---
_模板版本: v1 | 预填充时间: 2026-03-02 | 生成来源: 人工预填充（代码变更节完整，数据节待 data-pipeline 填充）_
