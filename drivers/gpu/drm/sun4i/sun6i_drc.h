/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef _SUN6I_DRC_H_
#define _SUN6I_DRC_H_

#include <linux/types.h>

struct device_node;

void sun6i_drc_engine_mode_set(struct device_node *engine_node,
			       u32 width, u32 height);

#endif /* _SUN6I_DRC_H_ */
