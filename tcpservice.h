#ifndef TCPSERVICE_H
#define TCPSERVICE_H

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>


class TcpService
{
public:
    TcpService();
    ~TcpService();

    int fd_socket_local;
    int fd_socket_client;
    int fd_socket_camera;
    int fd_socket_motor;

    void TcpAccept();
    int TcpRecv(char *buffer, size_t size);
    int TcpSendMotor(char *data, int size);
    int TcpRecvMotor(char *data, int size);

private:
    int enable;
    socklen_t addr_len;
    struct sockaddr_in addr_in;
    struct sockaddr_in addr;
    char ipstr[INET6_ADDRSTRLEN];

    ssize_t readn(int fd, char *buffer, size_t size);

};

#endif // TCPSERVICE_H
