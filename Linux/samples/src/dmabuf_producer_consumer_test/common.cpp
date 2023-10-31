#include <iostream>
#include <fstream>
#include "common.h"

#define DEV_EXPORTER "/dev/aipu"

/**
 * data container for file descriptor
 */
union {
    struct cmsghdr cm;
    char control[CMSG_SPACE(sizeof(int))];
} control_un;

/**
 * transfer file fd to local server
 */
int sender(int filefd, const char *server_path)
{
    int clifd = -1;
    int ret = -1;
    char buf[100] = {0};
    struct sockaddr_un servaddr;
    struct msghdr msg;
    struct iovec iov[1];
    struct cmsghdr *pcmsg;

    clifd  = socket(AF_UNIX, SOCK_STREAM, 0) ;
    if (clifd < 0)
    {
        AIPU_ERR()("socket failed.\n");
        return  -1 ;
    }

    bzero (&servaddr, sizeof(servaddr));
    servaddr.sun_family = AF_UNIX;
    strcpy ( servaddr.sun_path, server_path);

    ret = connect(clifd, (struct sockaddr*)&servaddr, sizeof(servaddr));
    if(ret < 0)
    {
        AIPU_ERR()("connect failed.\n");
        return -1;
    }

    msg.msg_name = NULL;
    msg.msg_namelen = 0;
    iov[0].iov_base = buf;
    iov[0].iov_len = 100;
    msg.msg_iov = iov;
    msg.msg_iovlen = 1;

    msg.msg_control = control_un.control;
    msg.msg_controllen = sizeof(control_un.control);

    pcmsg = CMSG_FIRSTHDR(&msg);
    pcmsg->cmsg_len = CMSG_LEN(sizeof(int));
    pcmsg->cmsg_level = SOL_SOCKET;
    pcmsg->cmsg_type = SCM_RIGHTS;       // send file handle
    *((int*)CMSG_DATA(pcmsg)) = filefd;  // fill file handle to auxiliary buffer

    ret = sendmsg(clifd, &msg, 0);
    AIPU_INFO()("send filefd=%d\n", filefd);
    return ret ;
}

/**
 * receive file fd from local server
 */
int recver(const char *server_path)
{
    int clifd = -1, listenfd = -1;
    int ret = -1;
    int recvfd = -1;
    char buf[100] = {0};
    struct sockaddr_un servaddr, cliaddr;
    struct msghdr msg;
    struct iovec iov[1];
    struct cmsghdr *pcmsg;
    socklen_t clilen;

    listenfd = socket (AF_UNIX, SOCK_STREAM, 0);
    if(listenfd < 0)
    {
        AIPU_ERR()("socket failed.\n");
        return -1;
    }

    unlink(server_path);
    bzero(&servaddr, sizeof(servaddr));
    servaddr.sun_family = AF_UNIX;
    strcpy(servaddr.sun_path, server_path);

    ret = bind (listenfd, (struct sockaddr*)&servaddr, sizeof(servaddr));
    if(ret < 0)
    {
        AIPU_ERR()("bind failed. errno = %d\n",  errno) ;
        close(listenfd);
        return -1 ;
    }
    listen(listenfd, 2);

    while(1)
    {
        clilen = sizeof(cliaddr);
        clifd = accept(listenfd, (struct sockaddr*)&cliaddr, &clilen);
        if (clifd < 0)
        {
            AIPU_ERR()("accept failed.\n");
            continue;
        }

        msg.msg_name = NULL;
        msg.msg_namelen = 0;
        iov[0].iov_base = buf;
        iov[0].iov_len = 100;
        msg.msg_iov = iov;
        msg.msg_iovlen = 1;

        msg.msg_control = control_un.control;
        msg.msg_controllen = sizeof(control_un.control) ;

        ret = recvmsg(clifd, &msg, 0);
        if( ret <= 0 )
        {
            AIPU_ERR()("recvmsg failed.\n");
            return ret;
        }

        if((pcmsg = CMSG_FIRSTHDR(&msg) ) != NULL && ( pcmsg->cmsg_len == CMSG_LEN(sizeof(int))))
        {
            if (pcmsg->cmsg_level != SOL_SOCKET)
            {
                AIPU_ERR()("cmsg_leval is not SOL_SOCKET\n");
                continue;
            }

            if (pcmsg->cmsg_type != SCM_RIGHTS)
            {
                AIPU_ERR()("cmsg_type is not SCM_RIGHTS\n");
                continue;
            }

            recvfd = *((int*)CMSG_DATA(pcmsg));
            AIPU_INFO()("recv fd = %d\n", recvfd);
            break;
        }
    }

    return recvfd ;
}

/**
 * mmap a file
 */
void *map_file_data(char *fname, int &size)
{
    int fd = 0;
    void *data = nullptr;
    struct stat finfo;

    if(fname == nullptr)
    {
        AIPU_ERR()("file name is null\n");
        goto finish;
    }

    if(stat(fname, &finfo) != 0)
    {
        AIPU_ERR()("no such a file: %s\n", fname);
        goto finish;
    }

    fd = open(fname, O_RDWR);
    if(fd <= 0)
    {
        AIPU_ERR()("open file failed: %s!(errno = %d)\n", fname);
        goto finish;
    }

    data = mmap(nullptr, finfo.st_size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
    if(MAP_FAILED == data)
    {
        AIPU_ERR()("mmap failed\n");
        goto finish;
    }

    size = finfo.st_size;

finish:
    if(fd > 0)
        close(fd);

    return data;
}

/**
 * unmap a file
 */
void unmap_file_data(void *data, int size)
{
    if(data != nullptr)
        munmap(data, size);
}

/**
 * check the inferrence output with referrence data
 */
bool is_output_correct(volatile char* src1, char* src2, uint32_t cnt)
{
    for(uint32_t out_chr = 0; out_chr < cnt; out_chr++)
    {
        if(src1[out_chr] != src2[out_chr])
            return false;
    }
    return true;
}


/**
 * request one dma_buf from dma_buf exporter(NUP driver module),
 * record its fd to 'dmabuf_fd'.
 */
int dmabuf_malloc(uint64_t size)
{
    int ret = 0;
    int fd = 0;
    struct aipu_dma_buf_request dma_buf_req = {0};

    fd = open(DEV_EXPORTER, O_RDWR);
    if(fd < 0)
    {
        ret = -1;
        AIPU_ERR() << "open " << DEV_EXPORTER << " [fail]\n";
        goto out;
    }

    dma_buf_req.bytes = size;
    ret = ioctl(fd, AIPU_IOCTL_ALLOC_DMA_BUF, &dma_buf_req);
    if(ret < 0)
    {
        AIPU_ERR() << "ioctl " << DEV_EXPORTER << " [fail]\n";
        goto out;
    }

    ret = dma_buf_req.fd;

out:
    close(fd);
    return ret;
}

/**
 * free allocated dma_buf
 */
int dmabuf_free(int _fd)
{
    int ret = 0;
    int fd = 0;

    fd = open(DEV_EXPORTER, O_RDWR);
    if(fd < 0)
    {
        ret = -1;
        AIPU_ERR() << "open " << DEV_EXPORTER << " [fail]\n";
        goto out;
    }

    ret = ioctl(fd, AIPU_IOCTL_FREE_DMA_BUF, &_fd);
    if(ret < 0)
    {
        AIPU_ERR() << "ioctl " << DEV_EXPORTER << " [fail]\n";
        goto out;
    }

out:
    close(fd);
    return ret;
}

/**
 * map physical pages of requested dma_buf to user mode,
 * then fill its with input data which is taken as model's
 * input data.
 */
int dmabuf_fill(int fd, char *data, uint32_t size)
{
    int ret = 0;
    char *va = nullptr;

    va =(char *)mmap(NULL, size, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
    if(va == MAP_FAILED)
    {
        ret = -1;
        AIPU_ERR() << "mmap dmabuf [fail]\n";
        goto out;
    }

    memcpy(va, data, size);
    munmap(va, size);

out:
    return ret;
}

/**
 * dump data in one dma-buf to file
 */
int dmabuf_dump_file(const char *name, int dmabuf_fd, int size)
{
    int ret = -1;
    char *data = nullptr;

    std::fstream file(name, std::ios::out | std::ios::binary);
    if (!file)
    {
        AIPU_ERR() << "Error opening file.";
        goto out;
    }

    data =(char *)mmap(NULL, size, PROT_READ|PROT_WRITE, MAP_SHARED, dmabuf_fd, 0);
    if(data == MAP_FAILED)
    {
        ret = -1;
        AIPU_ERR() << "mmap dmabuf [fail]\n";
        goto out;
    }

    file.write(data, size);
    file.close();
    munmap(data, size);
    ret = 0;

out:
    return ret;
}