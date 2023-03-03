// Copyright (C) 2022-2023 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


/**
 * @file  type.h
 */

#ifndef _TYPE_H_
#define _TYPE_H_

namespace aipudrv
{
typedef uint32_t DEV_PA_32;
typedef uint64_t DEV_PA_64;

inline DEV_PA_32 get_high_32(DEV_PA_64 pa)
{
    return pa >> 32;
}

inline DEV_PA_32 get_low_32(DEV_PA_64 pa)
{
    return pa & 0xFFFFFFFF;
}

typedef uint64_t GRAPH_ID;
typedef uint64_t JOB_ID;

inline GRAPH_ID job_id2graph_id(JOB_ID id)
{
    return (id & (0xFFFFFFFFUL << 32));
}

inline JOB_ID create_full_job_id(GRAPH_ID g, JOB_ID id)
{
    return g + id;
}

inline GRAPH_ID get_graph_id(uint64_t id)
{
    return (id & (0xFFFFFFFFUL << 32));
}

inline bool valid_graph_id(GRAPH_ID id)
{
    if (id >= (1UL << 32))
    {
        return true;
    } else {
        return false;
    }
}

inline bool valid_job_id(JOB_ID id)
{
    if ((id >= (1UL << 32)) && ((id & 0xFFFFFFFF) > 0))
    {
        return true;
    } else {
        return false;
    }
}

}

#endif /* _TYPE_H_ */