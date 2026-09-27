# RUNTIME WATERMARK AUDIT

SOURCE_HEAD=0e3c15559cf9ae2264502b99849dc2e2240141e5
EXISTING_LOAD_MON_STACK_CHECK=YES
TASK_STACK_INFO_LOGGED_BY_DEFAULT=NO (default logged_topics.cpp did not contain task_stack_info)
STACK_MEASUREMENT_IS_WATERMARK=YES
CURRENT_TCB_LOOKUP_METHOD=nxsched_get_tcb(getpid())
STACK_REMAIN_API=up_check_tcbstack_remain(tcb)
WORKQUEUE_RUN_STACK_BASE=24
WORKQUEUE_RUN_STACK_DIAG=40
WORKQUEUE_RUN_STACK_DELTA=16
FIXEDWING_RATE_CONTROL_RUN_STACK_BASE=1392
FIXEDWING_RATE_CONTROL_RUN_STACK_DIAG=1392
NEW_DIAGNOSTIC_TOPIC=work_queue_stack_watermark
NEW_MIN_ONLY_PUBLICATION=YES
BFAR_CAPTURE_ADDED=NO
BUILD_PASS=YES

## Existing mechanism

LoadMon is enabled by SYS_STCK_EN (default parameter remains unchanged), calls up_check_tcbstack_remain() while scanning system_load TCB entries, and publishes task_stack_info with task_name, stack_free and timestamp. The normal logged topic profile did not already include task_stack_info. The new topic is added at event-driven logger rate 0 only; no controller-frequency subscription or file I/O is added.

NuttX stack checking is historical high-water accounting, not an instantaneous SP measurement. A new lower value after an execution interval is useful attribution for the interval that created that watermark, but does not prove the deepest callee or exclusive ownership of all consumed bytes.

## Instrumentation

Only the queue whose name is exactly wq:nav_and_controllers is attributed. Before Run(), the implementation copies WorkItem::ItemName() into a fixed member message while the object is valid, and samples the current task TCB. After Run(), it does not dereference WorkItem; it samples the same TCB and publishes only if the observed before/after minimum is lower than the queue's previous minimum. Publication contains timestamp, current and previous minimum, queue name and item name. No std::string, dynamic allocation, snprintf, filesystem access or actuator/control-path call is used.

The topic message is five small fields and the message object is a WorkQueue member, avoiding a large diagnostic local. The current TCB is obtained with nxsched_get_tcb(getpid()), matching the verified NuttX API. The instrumentation is NuttX-only; POSIX behavior is unchanged.

## Stack and safety evidence

The diagnostic FMUv6C build passed. Compiler .su measurement under production optimization reports WorkQueue::Run 24 B before and 40 B after (+16 B), LoadMon::stack_usage 72 B in both builds, and FixedwingRateControl::Run 1392 B in both builds. CONFIG_WQ_NAV_AND_CONTROLLERS_STACKSIZE remains 2240 B. The +16 B overhead is diagnostic-only but material in a near-limit task; no stack-size increase is made.

Ground-only procedure is in ground_test_procedure.md. Enable SYS_STCK_EN explicitly, inspect task_stack_info and work_queue_stack_watermark, and keep propulsion disabled/removed. Do not treat a positive watermark as flight clearance.

CONTROL_PATH_SEMANTICS_CHANGED=NO
NAV_STACK_SIZE_CHANGED=NO
FIXEDWING_RATE_CONTROL_FRAME_CHANGED=NO
GROUND_TEST_READY=YES
FLIGHT_READY=NO
NEXT_ACTION=Run disarmed bench diagnostics, correlate new minima with queue members, and capture BFAR plus stacked registers on any repeat fault before considering further code changes

RUNTIME_WATERMARK_INSTRUMENTATION_VALID=YES
NAV_AND_CONTROLLERS_ATTRIBUTION_AVAILABLE=YES
DIAGNOSTIC_STACK_OVERHEAD_BYTES=16
CONTROL_PATH_SEMANTICS_CHANGED=NO
NAV_STACK_SIZE_CHANGED=NO
FIXEDWING_RATE_CONTROL_FRAME_CHANGED=NO
GROUND_TEST_READY=YES
FLIGHT_READY=NO
NEXT_ACTION=Use the topic on a ground-only diagnostic firmware and retain the 2240-byte stack
