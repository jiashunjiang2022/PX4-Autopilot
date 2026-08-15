# Memory and real-time evidence

The spectral ring stores 256 floats and the preallocated scratch buffer stores
256 floats. No dynamic memory was added. Ordinary 50 Hz updates do not copy the
window; scratch is populated only when a spectral evaluation is due. The former
per-update local sample/timestamp arrays are absent, avoiding about 3 KiB of
temporary EKF2 stack use.

NuttX builds with `-Wframe-larger-than=2500` passed. Runtime perf counters record
quality estimator elapsed time, spectral evaluation elapsed time, and missed or
overwritten quality inputs. The missed counter saturates at `UINT32_MAX` rather
than wrapping.

Compared with the Phase 4 final artifact, FLASH changed from 1,828,392 B to
1,829,352 B, a net increase of 960 B. AXI SRAM remains 61,892 B. Static ELF,
object-size, and symbol reports are under
`artifacts/ral_revision_phase4_1/static_memory/`.

Static build evidence does not establish runtime margin. EKF2 stack high-water,
WCET distributions, CPU load, scheduling deadlines, and missed-input behavior
must be measured on the target bench.
