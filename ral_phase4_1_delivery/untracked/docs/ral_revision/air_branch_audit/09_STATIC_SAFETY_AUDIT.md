# 静态性能与安全审计

本报告只基于源码，不包含 build、WCET、stack high-water、SD throughput 或 flight replay。

## 频谱计算与内存

| 项目 | 静态结论 | 状态 |
|---|---|---|
| complexity | 每次 eval 对约 B 个 frequency bins 遍历 N samples，O(BN) Goertzel；default 0.5--4 Hz、0.25 Hz step，约 15 bins | `PARTIAL` |
| eval cadence | 不是每样本做完整频谱；default window 4 s，约每 2 s eval | `PASS` |
| 50 Hz/4 s | N=200，低于 `kMaxSamples=256` | `PASS` capacity |
| 50 Hz/10 s | N=500，超过 256，旧样本会丢，参数语义与实际 window不一致 | `FAIL` |
| persistent memory | 256 floats + 256 uint64 timestamps，约 3 KiB，另有 state | `PARTIAL` |
| stack | eval locals 再含最多 256 sample/timestamp arrays，约 3 KiB，加函数栈 | `NEEDS RUNTIME VERIFICATION` |
| dynamic allocation | estimator ring/locals为固定数组，未见 per-sample heap allocation | `PASS` |
| realtime context | EKF2 INS0 realtime work queue，eval尖峰可能影响 EKF deadline | `NEEDS RUNTIME VERIFICATION` |

保持“每样本 O(1)，每 0.25--0.5 s 或更慢做一次 O(BN)”是合理方向，但必须在目标 MCU 上测 WCET 和 stack；当前 default 2 s eval 对诊断状态可能过慢，不能只为更快日志而每样本重算。

## 数值与时间风险

1. **FAIL：错误频点而非显式 invalid。** `upper_limit_hz=min(4.0,Nyquist-0.1)`，高 flap center 被钳位到 4 Hz，继续产生可能“有效”的 ratio（`src/modules/ekf2/EKF2.cpp:376-400`）。
2. **FAIL：不规则混合采样。** raw sample timestamps 与 validated publication timestamps 可进入同一 ring；代码假定 insertion order，并由首末值计算平均 Fs。非单调时间可能造成错误 dt/Fs/reset。
3. **FAIL：无 anti-alias/resampling。** 10 Hz path 无法可靠覆盖 5--6.8 Hz，且 held validated samples可能扭曲 spectrum。
4. **PARTIAL：early-boot unsigned underflow。** `time_us - spec_win_us` 在 uptime 小于 window 时为 unsigned 下溢（`EKF2.cpp:317-318`）。正常飞行前通常超过 4 s，但应显式饱和为 0。
5. **PARTIAL：invalid dt。** estimator 对 invalid/gap dt 有 reset/skip 路径（`EKF2.cpp:173-174,213-215`），但 mixed timestamp semantics 仍可能触发不必要 reset或导数尖峰。
6. **PARTIAL：NaN。** 输入、frequency 和 state有有限性检查，invalid spectral ratio可退化为 dv-only；但 selector 对 invalid/stale q 可直接 disable output，fallback不完整。
7. **PARTIAL：除零。** frequency range、duration、variance normalization用 epsilon/constraints防护；需要单测覆盖 zero window, equal timestamps, empty/one-bin bands, zero energy。
8. **FAIL：参数容量不一致。** 允许 10 s window，而 50 Hz 下 ring仅 5.12 s。

## 传感器与 Hall 安全

- `MS4525DO::RunImpl()` 对每次 read 执行两次 transaction，要求 first normal/second stale；一致性检查只比较 pressure MSB 和 temperature，没有比较 pressure LSB（`MS4525DO.cpp:175-176`）。这可能让不同 pressure value 被误接受。
- `MS4525DO::probe()` 在 `MS4525DO.cpp:73` 存在 `data_1[2] == data_1[2]` 恒真比较，降低 probe 一致性验证有效性。状态 `FAIL/P1`，需 driver unit/bench test。
- Hall ISR coalescing 只记录“已有 pending interrupt” error，不能恢复丢失边沿；consumer 200 Hz 不能保证突发事件不丢。`hall_event` 又未配置队列/日志。
- `RPM_CAP_ENABLE=0` 为默认，未确认 RPM input function。未启动 Hall 时 wing phase在首个 Hall 前 invalid；这比伪造 phase安全，但缺显式原因/timeout日志。

## Fusion/selector fail behavior

- q gate关闭会阻止 airspeed fusion，属于保守行为；但 `EKF2_ARSP_THR=0` 默认已禁用 airspeed fusion，若实验参数未明确设置，论文 adaptive covariance path可能根本不生效。
- selector quality safeguard可将最终 airspeed置 NaN/disabled，且不会自动进入 blockage fallback。TECS/mode/controller随后依赖各自 invalid handling；该行为必须先在 mode-specific SITL/bench test 中验证，不能直接飞行。
- 未知 experiment mode 没有处理，因为 mode本身不存在。新增 mode后应 fail closed 到 BASELINE并记录错误。
- runtime 参数可改变 quality行为，未被锁定或记录为实验段边界，存在模式污染风险。

## 需要的静态与运行验证

在允许后依序执行：quality math unit tests；mode truth-table tests；selector fallback/reason tests；Hall burst/queue tests；MS4525 read consistency tests；target build；50 Hz synthetic spectral sweep；EKF INS0 WCET/stack；producer timestamp/rate bench；SD bandwidth/dropout soak；disarmed mode lock；最后才是 tethered/low-risk flight。当前任何运行结果均为 `UNKNOWN`。
