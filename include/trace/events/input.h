/* Copyright (c) 2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM input

#if !defined(_TRACE_EVENT_INPUT_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_EVENT_INPUT_H

#include <linux/tracepoint.h>
#include <linux/types.h>

TRACE_EVENT(input_user_press_down,

	TP_PROTO(int x,
		int y),

	TP_ARGS(x, y),

	TP_STRUCT__entry(
		__field(int, x)
		__field(int, y)
	),

	TP_fast_assign(
		__entry->x	= x;
		__entry->y	= y;
	),

	TP_printk("press down@ %d, %d",
			__entry->x, __entry->y)
);

TRACE_EVENT(input_user_press_up,

	TP_PROTO(int x,
		int y),

	TP_ARGS(x, y),

	TP_STRUCT__entry(
		__field(int, x)
		__field(int, y)
	),

	TP_fast_assign(
		__entry->x	= x;
		__entry->y	= y;
	),

	TP_printk("press up@ %d, %d",
			__entry->x, __entry->y)
);

#endif

#include <trace/define_trace.h>

