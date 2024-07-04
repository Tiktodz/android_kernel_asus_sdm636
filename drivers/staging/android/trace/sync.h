#undef TRACE_SYSTEM
#define TRACE_INCLUDE_PATH ../../drivers/staging/android/trace
#define TRACE_SYSTEM sync

#if !defined(_TRACE_SYNC_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_SYNC_H

#include "../sync.h"
#include <linux/tracepoint.h>

TRACE_EVENT(sync_timeline,
	TP_PROTO(struct sync_timeline *timeline),

	TP_ARGS(timeline),

	TP_STRUCT__entry(
			__string(name, timeline->name)
			__array(char, value, 32)
	),

	TP_fast_assign(
			__assign_str(name, timeline->name);
			if (timeline->ops->timeline_value_str) {
				timeline->ops->timeline_value_str(timeline,
							__entry->value,
							sizeof(__entry->value));
			} else {
				__entry->value[0] = '\0';
			}
	),

	TP_printk("name=%s value=%s", __get_str(name), __entry->value)
);

TRACE_EVENT(sync_wait,
	TP_PROTO(struct sync_fence *fence, int begin),

	TP_ARGS(fence, begin),

	TP_STRUCT__entry(
#ifdef CONFIG_SYNC_DEBUG
			__string(name, fence->name)
#endif
			__field(s32, status)
			__field(u32, begin)
	),

	TP_fast_assign(
#ifdef CONFIG_SYNC_DEBUG
			__assign_str(name, fence->name);
#endif
			__entry->status = atomic_read(&fence->status);
			__entry->begin = begin;
	),

	TP_printk("%s name=%s state=%d", __entry->begin ? "begin" : "end",
#ifdef CONFIG_SYNC_DEBUG
			__get_str(name),
#else
			"",
#endif
			__entry->status)
);

TRACE_EVENT(sync_pt,
	TP_PROTO(struct fence *pt),

	TP_ARGS(pt),

	TP_STRUCT__entry(
		__string(timeline, pt->ops->get_timeline_name(pt))
		__array(char, value, 32)
	),

	TP_fast_assign(
		__assign_str(timeline, pt->ops->get_timeline_name(pt));
		if (pt->ops->fence_value_str) {
			pt->ops->fence_value_str(pt, __entry->value,
							sizeof(__entry->value));
		} else {
			__entry->value[0] = '\0';
		}
	),

	TP_printk("name=%s value=%s", __get_str(timeline), __entry->value)
);

#endif /* if !defined(_TRACE_SYNC_H) || defined(TRACE_HEADER_MULTI_READ) */

/* This part must be outside protection */
#include <trace/define_trace.h>
