#ifndef NETWORK_H
#define NETWORK_H

#include <QTcpSocket>

class Network
{
public:
    Network(QObject *parent = Q_NULLPTR);
    ~Network();

    void connect(bool isConnect, QString ip_address);
    void sendCmd(char *cmd, uint8_t len);

private:
    QTcpSocket *m_infoSock, *m_cmdSock;
};

#endif // NETWORK_H
