#include "tcpservice.h"
#include <stdlib.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <iostream>
#include <unistd.h>
#include <string.h>
#include <QDebug>

using namespace std;

TcpService::TcpService()
{
    cout<<"tcp service init"<<endl;
    int ret;
    int backLog = 10;

    enable = 1;

    fd_socket_motor= -1;

    addr_len = sizeof(struct sockaddr);

    fd_socket_local = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

    setsockopt(fd_socket_local, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));

    if (fd_socket_local == -1)
    {
        cout<<"ERROR: local sock fail"<<endl;
    }

    addr_in.sin_family = AF_INET;
    addr_in.sin_addr.s_addr = INADDR_ANY;
    addr_in.sin_port = htons(1234);

    ret = bind(fd_socket_local, (struct sockaddr *)&addr_in, sizeof(addr_in));
    if (ret != 0)
    {
        cout<<"ERROR: local Bind failed"<<endl;
    }

    ret = listen(fd_socket_local, backLog);

    cout<<"tcp socke finish->"<<endl;
}

TcpService::~TcpService()
{
    close(fd_socket_local);
}

void TcpService::TcpAccept()
{
    int keepalive = 1;
    int keepidle = 10;
    int keepinterval = 1;
    int keepcount = 10;
    int enable = 1;

    fd_socket_client = accept(fd_socket_local, (struct sockaddr *) &addr, &addr_len);
    setsockopt(fd_socket_client, SOL_SOCKET, SO_KEEPALIVE, (void *)&keepalive, sizeof(keepalive));
    setsockopt(fd_socket_client, IPPROTO_TCP, TCP_KEEPIDLE, (void *)&keepidle, sizeof(keepidle));
    setsockopt(fd_socket_client, IPPROTO_TCP, TCP_KEEPINTVL, (void *)&keepinterval, sizeof(keepinterval));
    setsockopt(fd_socket_client, IPPROTO_TCP, TCP_KEEPCNT, (void *)&keepcount, sizeof(keepcount));
    setsockopt(fd_socket_client, IPPROTO_TCP, TCP_NODELAY, &enable, sizeof(enable));
    cout<<"from "<<inet_ntoa(addr.sin_addr)<<endl;
}

ssize_t TcpService::readn(int fd, char *buffer, size_t size)
{
    ssize_t numRead;
    size_t totRead;
    char *buf;

    buf = buffer;
    for (totRead = 0; totRead < size; )
    {

        numRead = read(fd, buf, size - totRead);

        if (numRead == 0)
            return totRead;
        if (numRead == -1) {
            if (errno == EINTR)
                continue;
            else
                return -1;
        }

        totRead += numRead;
        buf += numRead;
    }
    return totRead;
}

int TcpService::TcpRecv(char *buffer, size_t size)
{
   return readn(fd_socket_client, buffer, size);
}

int TcpService::TcpRecvMotor(char *data, int size)
{
    return readn(fd_socket_motor, data, size);
}

int TcpService::TcpSendMotor(char *data, int size)
{
    int ret = 0;
//    char *buffer = (char *) malloc(sizeof(char) * (size+6));

    if (fd_socket_motor == -1)
    {
        cout<<"create motor connect"<<endl;
        struct sockaddr_in addr;
        struct timeval timeout;

        timeout.tv_sec = 0;
        timeout.tv_usec = 100000;

        fd_socket_motor = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

        setsockopt(fd_socket_motor, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));
        setsockopt(fd_socket_motor, IPPROTO_TCP, TCP_NODELAY, &enable, sizeof(enable));

        if (setsockopt (fd_socket_motor, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout,
                        sizeof(timeout)) < 0)
            cout<<"setsockopt failed"<<endl;

        if (fd_socket_motor == -1)
        {
            cout<<"ERROR: motor socket fail"<<endl;
            goto error;
        }

        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = inet_addr((const char *) "192.168.123.1");
        addr.sin_port = htons(80);

        ret = connect(fd_socket_motor, (struct sockaddr *) &addr,
                sizeof(struct sockaddr));

        if (ret != 0)
        {
            cout<<"ERROR: motor connect fail"<<endl;
            goto error;
        }
    }

//    buffer[0] = 0x5a;
//    buffer[1] = 0x5a;
//    memcpy(&buffer[2], &size, sizeof(int));
//    memcpy(&buffer[6], data, size);
//    ret = send(fd_socket_motor, data, size + 6, MSG_NOSIGNAL);
    ret = send(fd_socket_motor, data, size, MSG_NOSIGNAL);

    if(ret <= 0)
    {
        cout<<"ERROR: send motor data socket fail"<<endl;
        goto error;
    }
//    free(buffer);
    return 1;

error:
    close(fd_socket_motor);
    fd_socket_motor = -1;
//    free(buffer);
    return -1;

}
