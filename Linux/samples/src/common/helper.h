// Copyright (C) 2023-2024 Arm Technology (China) Co. Ltd.
//
// SPDX-License-Identifier: Apache-2.0

#ifndef _HELPER_H_
#define _HELPER_H_

#include <sys/sem.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <iostream>
#include <memory>
#include <vector>

#include "standard_api.h"

#define gettid() syscall(SYS_gettid)

#ifdef __ANDROID__
class SemOp {
public:
  explicit SemOp(int32_t count = 1) : m_count(count) {}

  void semaphore_v() {
    std::unique_lock<std::mutex> lock(m_mutex);
    ++m_count;
    m_cv.notify_one();
  }

  void semaphore_p() {
    std::unique_lock<std::mutex> lock(m_mutex);
    m_cv.wait(lock, [=] { return m_count > 0; });
    --m_count;
  }

private:
  std::mutex m_mutex;
  std::condition_variable m_cv;
  int32_t m_count;
};

#else

union semun {
  int val;
  struct semid_ds *buf;
  unsigned short *arry;
};

/**
 * @class SemOp
 * @brief semophore for multi process sync
 */
class SemOp {
public:
  int m_sem_id;

  SemOp() {
    m_sem_id = semget((key_t)1234, 1, 0666 | IPC_CREAT);
    if (!set_semvalue()) {
      fprintf(stderr, "Failed to initialize semaphore\n");
      exit(EXIT_FAILURE);
    }
  }

  ~SemOp() {
    /* don't delete sem immediately from the system */
    // del_semvalue();
  }

  int set_semvalue() {
    union semun sem_union;

    sem_union.val = 1;
    if (semctl(m_sem_id, 0, SETVAL, sem_union) == -1)
      return 0;
    return 1;
  }

  void del_semvalue() {
    union semun sem_union;

    if (semctl(m_sem_id, 0, IPC_RMID, sem_union) == -1) {
      fprintf(stderr, "Failed to delete semaphore, %d\n", errno);
    } else {
      fprintf(stdout, "delete sem: %d\n", m_sem_id);
    }
  }

  /* sem down Op = P() */
  int semaphore_p() {
    struct sembuf sem_b;
    sem_b.sem_num = 0;
    sem_b.sem_op = -1;
    sem_b.sem_flg = SEM_UNDO;
    if (semop(m_sem_id, &sem_b, 1) == -1) {
      fprintf(stderr, "semaphore_p failed\n");
      return 0;
    }
    return 1;
  }

  /* sem up op = V() */
  int semaphore_v() {
    struct sembuf sem_b;
    sem_b.sem_num = 0;
    sem_b.sem_op = 1;
    sem_b.sem_flg = SEM_UNDO;
    if (semop(m_sem_id, &sem_b, 1) == -1) {
      fprintf(stderr, "semaphore_v failed\n");
      return 0;
    }
    return 1;
  }
};

#endif

int dump_file_helper(const char *fname, void *src, unsigned int size);
int load_file_helper(const char *fname, char **dest, uint32_t *size);
int unload_file_helper(char *data);
int check_result_helper(const std::vector<char *> &outputs,
                        const std::vector<aipu_tensor_desc_t> &descs,
                        const std::vector<char *> &gt,
                        const std::vector<uint32_t> &gt_size);
int check_result(const std::vector<std::shared_ptr<char>> &outputs,
                 const std::vector<aipu_tensor_desc_t> &descs,
                 const std::vector<char *> &gt,
                 const std::vector<uint32_t> &gt_size);

extern std::shared_ptr<SemOp> semOp_sp;
int help_create_dir(const char *path);
std::string timestamp_helper(int time_stamp_type = 0);
std::vector<std::string> split_string(const std::string &s,
                                      const std::string &splitter,
                                      const int keep_spliter = 0);

#endif /* _HELPER_H_ */