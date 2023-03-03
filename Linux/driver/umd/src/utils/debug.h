// Copyright (C) 2022-2023 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  debug.h
 * @brief UMD debug usage macro header
 */

#ifndef _DEBUG_H_
#define _DEBUG_H_

#define __PRINT_MACRO(x) #x
#define PRINT_MACRO(x) #x " is defined to be " __PRINT_MACRO(x)

#define RTDEBUG_TRACKING_MEM_OPERATION  0
#define DUMP_ALL_MEM_OP_MASK 0x3f
#if RTDEBUG_TRACKING_MEM_OPERATION
#define DUMP_MEM_OP_MASK (DUMP_ALL_MEM_OP_MASK)
#else
#define DUMP_MEM_OP_MASK (0)
#endif

#if ((defined RTDEBUG) && (RTDEBUG==1))

#define LOG_ENABLE 1
#define RTDEBUG_SIMULATOR_LOG_LEVEL     3

#else /* ! RTDEBUG */

#define LOG_ENABLE 0
#define RTDEBUG_SIMULATOR_LOG_LEVEL     0

#endif /* #ifdef RTDEBUG */

#endif /* _DEBUG_H_ */
