/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Interface to the Allwinner A80 AR100 (ARISC) power-management coprocessor.
 */
#ifndef _LINUX_SOC_SUNXI_ARISC_H
#define _LINUX_SOC_SUNXI_ARISC_H

#include <linux/types.h>

/**
 * struct sunxi_arisc_vf - one voltage/frequency operating point
 * @freq_hz:  operating frequency, in Hz (note: the DVFS *request* takes kHz)
 * @volt_mv:  core voltage for that point, in mV
 */
struct sunxi_arisc_vf {
	u32 freq_hz;
	u32 volt_mv;
};

#if IS_ENABLED(CONFIG_SUNXI_ARISC)

bool sunxi_arisc_is_ready(void);

/**
 * sunxi_arisc_dvfs_set_cpufreq() - ask the coprocessor to drive a CPU cluster
 * @freq_khz: target frequency in kHz. Zero powers the cluster *down*.
 * @pll:      1 selects cluster 0 (A7), 2 selects cluster 1 (A15).
 *
 * This single request performs both the cluster power-up and the frequency
 * and voltage change; there is no separate "power on" message.
 */
int sunxi_arisc_dvfs_set_cpufreq(u32 freq_khz, u32 pll);

/**
 * sunxi_arisc_dvfs_cfg_vf_table() - load a cluster voltage/frequency table
 * @cluster: 0 for the A7 cluster, 1 for the A15 cluster
 * @vf:      operating points, highest frequency first
 * @n:       number of valid entries; the rest are zero-filled
 *
 * Must be sent before any DVFS request: the firmware looks a voltage up by
 * frequency in this table, and with it empty it has nothing to raise the
 * rail to.
 */
int sunxi_arisc_dvfs_cfg_vf_table(unsigned int cluster,
				  const struct sunxi_arisc_vf *vf, int n);

/* Read or write one of the firmware voltage rails.
 * Type 29 is the external CPU-B DC-DC that supplies the A15 cluster.
 */
int sunxi_arisc_get_volt(u32 type);
int sunxi_arisc_set_volt(u32 type, u32 mv);

#else

static inline bool sunxi_arisc_is_ready(void) { return false; }

static inline int sunxi_arisc_dvfs_set_cpufreq(u32 freq_khz, u32 pll)
{
	return -ENODEV;
}

static inline int sunxi_arisc_dvfs_cfg_vf_table(unsigned int cluster,
						const struct sunxi_arisc_vf *vf,
						int n)
{
	return -ENODEV;
}

#endif /* CONFIG_SUNXI_ARISC */

#endif /* _LINUX_SOC_SUNXI_ARISC_H */
