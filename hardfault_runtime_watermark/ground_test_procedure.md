# Ground diagnostic procedure

1. Install only on a ground-test controller. Keep propulsion removed or electrically inhibited; do not conduct flight.
2. Set and save the existing stack monitor parameter:
   param set SYS_STCK_EN 1
   param save
3. Start the logger and verify:
   listener task_stack_info
   listener work_queue_stack_watermark
   top once
4. Exercise disarmed STABILIZED callbacks first. Then, only if the bench safety case explicitly permits it, exercise AUTO_MISSION transitions, parameter updates, QGC reconnect, logger start/stop and software restart. Do not command thrust.
5. Record every new watermark event: timestamp, queue, WorkItem, previous and new stack_free. Repeated attribution to one WorkItem makes it the leading candidate, but does not prove exclusive stack ownership.
6. Stop on any stack below the observed safe bench margin, corrupted topic, scheduler error or fault. Preserve the ULog and fault console output, including BFAR and stacked registers if available.

This procedure is diagnostic only. It does not authorize actuator excitation or flight.
