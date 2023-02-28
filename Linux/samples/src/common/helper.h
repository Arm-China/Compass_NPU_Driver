// Copyright (C) 2022 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0


#ifndef _HELPER_H_
#define _HELPER_H_

#include <iostream>
#include <vector>
#include <memory>
#include <sys/sem.h>
#include <unistd.h>
#include <sys/syscall.h>
#include "standard_api.h"

#define gettid() syscall(SYS_gettid)

union semun
{
    int val;
    struct semid_ds *buf;
    unsigned short *arry;
};

/**
 * @class SemOp
 * @brief semophore for multi process sync
 */
class SemOp
{
    public:
    int m_sem_id;

    SemOp()
    {
        m_sem_id = semget((key_t)1234, 1, 0666 | IPC_CREAT);
        if(!set_semvalue()) {
            fprintf(stderr,  "Failed to initialize semaphore\n");
            exit(EXIT_FAILURE);
        }
    }

    ~SemOp()
    {
        /* don't delete sem immediately from the system */
        // del_semvalue();
    }

    int set_semvalue()
    {
        union semun sem_union;

        sem_union.val = 1;
        if(semctl(m_sem_id, 0, SETVAL, sem_union) == -1)
            return 0;
        return 1;
    }

    void del_semvalue()
    {
        union semun sem_union;

        if(semctl(m_sem_id, 0, IPC_RMID, sem_union) == -1) {
            fprintf(stderr,  "Failed to delete semaphore, %d\n", errno);
        } else {
           fprintf(stdout,  "delete sem: %d\n", m_sem_id);
        }

    }

    /* sem down Op = P() */
    int semaphore_p()
    {
        struct sembuf sem_b;
        sem_b.sem_num = 0;
        sem_b.sem_op = -1;
        sem_b.sem_flg = SEM_UNDO;
        if(semop(m_sem_id, &sem_b, 1) == -1) {
            fprintf(stderr,  "semaphore_p failed\n");
            return 0;
        }
        return 1;
    }

    /* sem up op = V() */
    int semaphore_v()
    {
        struct sembuf sem_b;
        sem_b.sem_num = 0;
        sem_b.sem_op = 1;
        sem_b.sem_flg = SEM_UNDO;
        if(semop(m_sem_id, &sem_b, 1) == -1) {
            fprintf(stderr,  "semaphore_v failed\n");
            return 0;
        }
        return 1;
    }
};

int dump_file_helper(const char* fname, void* src, unsigned int size);
int load_file_helper(const char* fname, char** dest, uint32_t* size);
int unload_file_helper(char* data);
int check_result_helper(const std::vector<char*>& outputs,
    const std::vector<aipu_tensor_desc_t>& descs, char* gt, uint32_t gt_size);
int check_result(std::vector< std::shared_ptr<char> >outputs,
    const std::vector<aipu_tensor_desc_t>& descs, char* gt, uint32_t gt_size);

extern std::shared_ptr<SemOp> semOp_sp;
int help_create_dir(const char *path);

#endif /* _HELPER_H_ */