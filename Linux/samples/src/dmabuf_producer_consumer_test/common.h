#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/epoll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/un.h>
#include <unistd.h>

#include "common/cmd_line_parsing.h"
#include "common/dbg.hpp"
#include "common/helper.h"

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
bool is_output_correct(const char *src1, const char *src2, uint32_t cnt);
int dmabuf_dump_file(const char *name, int dmabuf_fd, int size);