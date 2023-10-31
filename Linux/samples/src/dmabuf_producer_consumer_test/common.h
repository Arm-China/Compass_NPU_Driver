#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <sys/un.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/epoll.h>
#include <fcntl.h>
#include "standard_api.h"
#include "common/cmd_line_parsing.h"
#include "common/helper.h"
#include "common/dbg.hpp"
#include "kmd/armchina_aipu.h"

#define PAGE_ALIGN_SIZE(size) ((size + 4095) & (~(1 << 12)))

#define PRODUCER_SERVER_PATH "foo.producer"
#define CONSUMER_SERVER_PATH "foo.consumer"

int sender(int filefd, const char *server_path);
int recver(const char *server_path);
void *map_file_data(char *file, int &size);
void unmap_file_data(void *data, int size);
int dmabuf_malloc(uint64_t size);
int dmabuf_free(int _fd);
int dmabuf_fill(int fd, char *data, uint32_t size);
bool is_output_correct(volatile char* src1, char* src2, uint32_t cnt);
int dmabuf_dump_file(const char *name, int dmabuf_fd, int size);