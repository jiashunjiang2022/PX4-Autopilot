# 在线频谱管线审计

总体状态：`FAIL`。当前实现不能可靠覆盖正式 4--5.5 Hz 或平台最大 6.8 Hz。

## 真实输入追踪

调用者是 `EKF2::UpdateAirspeedSample()`，被调用者是 `AirspeedQualityEstimator::update()`：

| Path | topic.field | timestamp | producer | 调用证据 |
|---|---|---|---|---|
| validated | `airspeed_validated.true_airspeed_m_s` | `airspeed_validated.timestamp`，publication time | `AirspeedSelector::Run()` 固定约 10 Hz | `src/modules/ekf2/EKF2.cpp:2755-2777`；selector `airspeed_selector_main.cpp:76,294` |
| raw while validated is present/asynchronous | `airspeed.true_airspeed_m_s` | `airspeed.timestamp_sample` | `Sensors::diff_pres_poll()` 约 20 Hz | `EKF2.cpp:2862-2884` |
| raw fallback | `airspeed.true_airspeed_m_s` | `airspeed.timestamp_sample` | validated 超过 3 s 不可用时约 20 Hz | `EKF2.cpp:2926-2951` |

同一个 EKF Run 中若两者都更新，validated 分支优先；异步到达时 raw 也可能进入同一 estimator。因此静态代码支持的实际流是约 20--30 Hz 的混合事件流，不是严格固定 10 Hz。准确频率、重复值比例、时间戳单调性均为 `NEEDS RUNTIME VERIFICATION`。

`EKF2_ASP_SFS` 默认 10 Hz（`src/modules/ekf2/params_airspeed.yaml:65-75`）没有执行 decimation，也没有决定 Goertzel 的实际采样率；它只用于 `ceil(expected_fs * window * 0.9)` 类的最小样本门限（`EKF2.cpp:336`）。实际 `fs_hz` 由窗口首末 timestamp 和样本数计算（`EKF2.cpp:366-367`）。

## 算法参数与实现

| 项目 | 当前值/行为 | 证据 | 状态 |
|---|---|---|---|
| signal | TAS，非 raw pressure/IAS | `EKF2.cpp:2755-2777,2862-2884` | `PARTIAL` |
| sample model | 将不规则样本当作平均 Fs 下的等间隔序列 | `EKF2.cpp:320-367` | `FAIL` |
| resampling/interpolation | `NOT IMPLEMENTED` | estimator ring/eval path | `NOT_IMPLEMENTED` |
| anti-aliasing | `NOT IMPLEMENTED` | estimator input path | `NOT_IMPLEMENTED` |
| method | 多频点 Goertzel，非 FFT/STFT/Welch | `EKF2.cpp:47-66,393-400` | `PASS` method identified |
| window duration | `EKF2_ASP_SWIN`, default 4 s, range 1--10 s | yaml `76-85` | `PARTIAL` |
| N | runtime count, max 256 | `src/modules/ekf2/EKF2.hpp:211` | `PARTIAL` |
| minimum N | default约 36 = 10*4*0.9 | `EKF2.cpp:336` | `PARTIAL` |
| window function | rectangular; mean detrend only | `EKF2.cpp:354-364` | `PARTIAL` |
| reference band | 0.5 Hz to `min(4.0, Nyquist-0.1)` | `EKF2.cpp:376-381` | `FAIL` |
| flap half-width | `EKF2_ASP_DF`, default 0.5 Hz | yaml `86-94`; `EKF2.cpp:388-391` | `PASS` parameterized |
| bin spacing | `max(0.1, 1/window)`，default 0.25 Hz | `EKF2.cpp:382` | `PARTIAL` |
| eval interval | half window，default 2 s | `EKF2.cpp:341-344` | `PARTIAL` |
| result hold/stale | held between evals; stale after `max(2*interval,5 s)`, default 5 s | `EKF2.cpp:414-436` | `PARTIAL` |
| epsilon | `FLT_EPSILON`; no parameter | normalization code | `PASS` |
| invalid window | insufficient samples/gap/invalid input resets or yields invalid ratio | `EKF2.cpp:173-174,213-215,317-354,414-436` | `PARTIAL` |
| flap timeout | `EKF2_FLAP_T_TO`, default 0.8 s | yaml `216-224` | `PASS` |
| CPU context | EKF2 `INS0` realtime work queue | `EKF2.cpp:3839` | `NEEDS RUNTIME VERIFICATION` |

## 10 Hz、Nyquist 与高频扑翼的答复

1. **当前是否仍以 10 Hz 输入运行？** `PARTIAL/UNKNOWN`。10 Hz 是默认样本数假设；validated producer 确为 10 Hz，但 raw producer 约 20 Hz 也可进入同一 ring。应记录实际 input source、timestamp_sample 和 measured Fs 后台架确认。
2. **如果只有 10 Hz，Nyquist 是否 5 Hz？** 是。但代码还将有效搜索上限降到最多 4 Hz。
3. **4--5.5 Hz 如何处理？** center 在 `EKF2.cpp:388-391` 被钳到 `upper_limit_hz`。4 Hz 以上不搜索真实中心，而继续测 4 Hz 附近能量；在 10 Hz 路径中 5--5.5 Hz 还处于 Nyquist 边界/以上并可能混叠。
4. **6.8 Hz 如何处理？** 同样钳到 4 Hz；若输入含 10 Hz 分量则 6.8 Hz 可混叠为约 3.2 Hz。混合 10/20 Hz 且无重采样时不能建立单一、可辩护的 alias 关系。
5. **高于 reference max 会发生什么？** 不是空频带，也不是显式 invalid，而是中心钳位后继续得到可能看似有效的错误 ratio，风险高于明确拒绝。
6. **当前是否仅适用于低于 4 Hz？** 代码频带上限最多 4 Hz；即使低于 4 Hz，非均匀混合采样仍需验证，因此只能说理论上限 4 Hz。
7. **提高 ULog 到 80 Hz 是否改变在线算法？** 否。logger 是 subscriber，只改变记录上限。

## 时域指标

论文符号 `delta_z`, `delta_t`, `D_t`, `lambda_t`, `D_t0`, `phi_t` 没有按该体系实现。当前 `AirspeedQualityEstimator::update()` 只计算 `abs(TAS-last_TAS)/dt`，用固定 alpha 0.2 平滑（`EKF2.cpp:178-186`），再由 `EKF2_ASP_DV0` 默认 5 m/s^2 归一化并与 spectral ratio 加权形成 `q_raw`（`EKF2.cpp:440-446`）。

q smoothing 使用 `alpha=dt/(tau+dt)`（`EKF2.cpp:465-470`；`EKF2_ASP_QTAU` 默认 0.25 s），属于物理时间常数；提高输入频率不需要按固定离散步长重算 tau，但必须处理 invalid/non-monotonic dt。日志只含 filtered `dv`，没有 raw delta、`D_t` 或 `phi_t`，无法完整重建 q_raw 的瞬态来源，状态为 `PARTIAL`。

## 改为 50 Hz 时必须同步审查的接口

1. 在 `EKF2::UpdateAirspeedSample()` 前建立唯一 input topic/field、source enum、严格单调 `timestamp_sample` 和 50 Hz resampler/decimator；validated held values不得混入。
2. 设计明确的抗混叠低通和缺样策略，日志记录 raw input、resampled input、actual Fs、gap/reset reason。
3. 将频带上限参数化并覆盖至少 0--6.8 Hz；50 Hz Nyquist 25 Hz 可留出滤波过渡带。
4. 同步选择 `N` 和 window duration。50 Hz*4 s=200 可装入 256；当前参数允许 10 s，会需要 500 samples，必须增大容量或限制参数。
5. 明确 window function、频率分辨率、flap half-width 与 update period。推荐先保持 4 s/0.25 Hz，并评估 0.25--0.5 s 的状态发布与较低频谱重算周期。
6. 给 estimator 添加 deterministic unit tests：0--6.8 Hz sweep、jitter/drop/duplicate、Nyquist、timeout、empty band、NaN、early boot、source switch。
7. 在 EKF2 INS0 work queue 上测 WCET、stack high-water 和 missed deadlines；Goertzel 不应每样本全量计算，当前只在 eval 时计算是可保留的方向。
