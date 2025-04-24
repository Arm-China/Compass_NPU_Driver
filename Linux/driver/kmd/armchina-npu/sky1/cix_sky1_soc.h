// SPDX-License-Identifier: GPL-2.0
// Copyright 2024 Cix Technology Group Co., Ltd. All Rights Reserved.
#ifndef __CIX_SKY1_SOC_H__
#define __CIX_SKY1_SOC_H__

#include <linux/reset.h>
#include <linux/clk.h>
#include <linux/devfreq.h>
#include <linux/devfreq-event.h>
#include "aipu_priv.h"

#define CIX_NPU_PD_MAX_NUM				(3)
struct cix_aipu_priv {
	struct device *pd_core[CIX_NPU_PD_MAX_NUM];
	struct device_link *link;
	struct device *opp_pmdomain;
	struct device_link *opp_dl;
	struct devfreq_dev_profile devfreq_profile;
	struct devfreq *devfreq;
};
#endif /* __CIX_SKY1_SOC_H__ */
