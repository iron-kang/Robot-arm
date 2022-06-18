#include "network.h"

Network::Network(QObject *parent)
{
    m_infoSock = new QTcpSocket(parent);
//    connect(m_infoSock,SIGNAL(readyRead()),this,SLOT(readyRead()));
//    connect(m_infoSock,SIGNAL(connected()),this,SLOT(connected()));
//    connect(m_infoSock,SIGNAL(disconnected()),this,SLOT(disConnected()));

    m_cmdSock = new QTcpSocket(parent);
    m_cmdSock->setSocketOption(QAbstractSocket::LowDelayOption, 1);
}

Network::~Network()
{
    m_infoSock->disconnectFromHost();
    m_infoSock->close();
    m_cmdSock->disconnectFromHost();
    m_cmdSock->close();

    delete m_infoSock;
    delete m_cmdSock;
}

void Network::connect(bool isConnect, QString ip_address)
{
    if (isConnect)
    {
        m_cmdSock->connectToHost(ip_address, 80);
    }
    else {
        m_cmdSock->disconnected();
        m_cmdSock->close();
    }
}

void Network::sendCmd(char *cmd, uint8_t len)
{
    m_cmdSock->write(cmd, len);
    m_cmdSock->waitForBytesWritten();
}
