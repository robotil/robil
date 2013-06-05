#include <QTcpSocket>
#include <QHostAddress>
#include <QChar>
#include "tcpConnection.h"
#include "iostream"

using namespace std;

CTcpConnection::CTcpConnection(QString ipAddress,int port)
{
  IsSendingImage = false;
  IsSendingGrid = false;
  IsSendingPath = false;
  WaitingForResponse = false;
  IsSendingExecuterStatus = false;
  pITcpConnectionInterface = NULL;
  ImgSize = 0;
  Counter = 0;
  IpAddress = new  QHostAddress(ipAddress);
  pConnection = new QTcpSocket(this);
  connect(pConnection, SIGNAL(readyRead()), SLOT(SltReadyRead()));
  connect(pConnection, SIGNAL(error(QAbstractSocket::SocketError)),SLOT(SltError(QAbstractSocket::SocketError)));
  std::cout<<"TCP: trying to connect!\n";
  pConnection->connectToHost(*IpAddress,port);
  pTimer = new QTimer();
  connect(pTimer, SIGNAL(timeout()), SLOT(SltOnTimer()));
//  pTimer->start(1000);
}

CTcpConnection::~CTcpConnection()
{
//  if(pConnection != NULL)
//    {
//      delete pConnection;
//      pConnection =  NULL;
//    }
  if(IpAddress != NULL)
    {
      delete IpAddress;
    }
}

void CTcpConnection::SetSubscriber(ITcpConnectionInterface* pitcpConnectionInterface)
{
  pITcpConnectionInterface = pitcpConnectionInterface;
}

void CTcpConnection::SltReadyRead()
{
  std::cout<<"TCP: Data received!\n";
  int bufSize;
  short msgId;
  unsigned int dataSize;
  unsigned int counter;
  QDataStream in(pConnection);
  in.setByteOrder(QDataStream::LittleEndian);

  while(pConnection->bytesAvailable() > 0)
    {
      std::cout<<"TCP: Bytes avaliable: " << pConnection->bytesAvailable() << "\n";
 //     pConnection->waitForReadyRead();
      bufSize = pConnection->bytesAvailable();
      if(IsSendingImage)
        {
          if(bufSize < ImgSize)
            {
              std::cout<<"TCP: Size not big enough!\n";
              std::cout<<"TCP: expected = " << ImgSize << " received = " << bufSize << "\n";
              return;
            }
          else
          {
              IsSendingImage = false;
          }
          if(!IsSendingImage)
           {
              QImage img;
             in >> img;
  //           if(!img.save("received.jpg"))
  //             {
  //               std::cout<<"TCP: Image save failed!\n";
  //             }
             std::cout<<"TCP: Image receive completed!\n";
             if(pITcpConnectionInterface != NULL)
             {
               pITcpConnectionInterface->OnImgReceived(img);
             }
           }
        }
      else if(IsSendingGrid)
        {
          if(bufSize < ImgSize)
            {
              std::cout<<"TCP: Size not big enough!\n";
              std::cout<<"TCP: expected = " << ImgSize << " received = " << bufSize << "\n";
              return;
            }
            else
            {
                IsSendingGrid = false;
            }
            if(!IsSendingGrid)
              {
                StructGridData grid;
                in >> grid.RobotPos.x;
                in >> grid.RobotPos.y;
                in >> grid.RobolOrientation;
                in >> grid.XOffset;
                in >> grid.YOffset;
                for(int i=0; i<100; i++)
                {
                  for(int j=0; j<100;j++)
                  {
                      in>>(int &)(grid.Grid[i][j]);
                  }
                }
                std::cout<<"TCP: Grid receive completed!\n";
                std::cout<<"Robot pos: "<<grid.RobotPos.x<<","<<grid.RobotPos.y<<"\n";
                std::cout<<"Robot orientation: "<<grid.RobolOrientation<<"\n";
                std::cout<<"xOffset: "<<grid.XOffset<<"\n";
                std::cout<<"yOffset: "<<grid.YOffset<<"\n";
 //               emit SigOnGridReceived(grid.Grid,grid.RobotPos, grid.XOffset, grid.YOffset, grid.RobolOrientation);
                if(pITcpConnectionInterface != NULL)
                  {
                    pITcpConnectionInterface->OnOccupancyGridReceived(grid.Grid,grid.RobotPos, grid.XOffset, grid.YOffset, grid.RobolOrientation);
                  }
              }
        }
      else if(IsSendingPath)
        {
          if(bufSize < ImgSize)
          {
            std::cout<<"TCP: Size not big enough!\n";
            std::cout<<"TCP: expected = " << ImgSize << " received = " << bufSize << "\n";
            return;
          }
          else
          {
              IsSendingPath = false;
          }
          if(!IsSendingGrid)
            {
              short pathLength;
              std::vector<StructPoint> points;
              in >> pathLength;
              std::cout<<"TCP: Path received:\n";
              for(int i=0; i<pathLength; i++)
                {
                  StructPoint point;
                  in >> point.x;
                  in >> point.y;
                  std::cout<<"TCP: Point "<<i<<" is: "<<point.x<<" "<< point.y<<":\n";
                  points.push_back(point);
                }
              if(pITcpConnectionInterface != NULL)
                {
                  pITcpConnectionInterface->OnPathReceived(points);
                }
            }
        }
//      else if(IsSendingExecuterStatus)
//      {
//    	  if(bufSize < ImgSize*sizeof(char))
//			{
//			  std::cout<<"TCP: Size not big enough!\n";
//			  std::cout<<"TCP: expected = " << ImgSize << " received = " << bufSize << "\n";
//			  return;
//			}
//			else
//			{
//				IsSendingExecuterStatus = false;
//			}
//    	  if(!IsSendingExecuterStatus)
//    	  {
//    		  QString str;
//    		  in>>str;
//    		  std::cout<<"ExecuterStatus received: "<<str.toStdString()<<std::endl;
////			char* str = new char(ImgSize);
////			pConnection->read(str,ImgSize);
////			QString strQString(str);
////			std::cout<<"ExecuterStatus received: "<<strQString.toStdString()<<"\n";
////			pITcpConnectionInterface->OnExecuterStackUpdate(strQString);
//    	  }
//      }
      else
      { if (pConnection->bytesAvailable() < (int)sizeof(short))
        {
                return;
        }
        in >> msgId;
        if(msgId == 45)
          {
        	char* s = new char(pConnection->bytesAvailable());
        	pConnection->read(s,pConnection->bytesAvailable());
        	std::cout<<"The Hello msg is: " << s << std::endl;
            std::cout<<"TCP: Hello Received!\n";
            return;
          }
        else
          {
   //         std::cout<<"TCP: msg id: " << msgId << "\n";
          }
        bufSize = pConnection->bytesAvailable();
        if (pConnection->bytesAvailable() < (int)sizeof(unsigned int))
        {
                return;
        }
        in >> dataSize;
        std::cout<<"TCP: image coming size = " << dataSize << "\n";
        bufSize = pConnection->bytesAvailable();
        if (pConnection->bytesAvailable() < (int)sizeof(unsigned int))
        {
                return;
        }
        in >> counter;
        if(1 == msgId)
          {
            IsSendingImage = true;
            ImgSize = dataSize;
            std::cout<<"TCP: image coming!\n";
          }
        else if(2 == msgId)
          {
            IsSendingGrid = true;
            ImgSize = dataSize;
            std::cout<<"TCP: grid coming!\n";
          }
        else if(3 == msgId)
          {
            IsSendingPath = true;
            ImgSize = dataSize;
            std::cout<<"TCP: path coming!\n";
          }
        else if(4 == msgId)
          {
            if(pITcpConnectionInterface != NULL)
            {
                WaitingForResponse = true;
              pITcpConnectionInterface->OnHMIResponseReceived();
            }
          }
        else if(5 == msgId)
          {
            int status;
            in >> status;
            pITcpConnectionInterface->OnExecutionStatusUpdate(status);
          }
        else if(31 == msgId)
		  {
 //       	IsSendingExecuterStatus = true;
 //       	ImgSize = dataSize;
 //       	std::cout<<"TCP: ExecuterStack coming!\n";
        	QString str;
		    in>>str;
		    std::cout<<"ExecuterStatus received: "<<str.toStdString()<<std::endl;
		    pITcpConnectionInterface->OnExecuterStackUpdate(str);
		  }
      }
    }
}

void CTcpConnection::SltError(QAbstractSocket::SocketError socketError)
{
  switch (socketError) {
  case QAbstractSocket::RemoteHostClosedError:
    std::cout<<"TCP: Connection to Agent been closed!\n";
    break;
  case QAbstractSocket::HostNotFoundError:
    std::cout<<"TCP: Agent host not found!\n";
    break;
  case QAbstractSocket::ConnectionRefusedError:
    std::cout<<"TCP: Connection refused!\n";
    break;
  default:
    std::cout<<"TCP: Connection error!\n";
    break;

  }
}

void CTcpConnection::SltOnTimer()
{
//  std::cout<<"TCP: Bytes avaliable: "<< pConnection->bytesAvailable() <<"\n";
//  std::cout<<"TCP: The socket is: "<< pConnection->state() <<"\n";
}

void CTcpConnection::LoadMission(int index)
{
  StructHeader header;
  header.MessageID = 12;
  header.DataSize = 0;
  header.Counter = Counter;
  Counter++;
  QByteArray block;
  QDataStream out(&block, QIODevice::WriteOnly);
  out.setByteOrder(QDataStream::LittleEndian);
  out << header.MessageID;
  out << header.DataSize;
  out << header.Counter;
  out << index;
  pConnection->write(block);
  pConnection->flush();
  pConnection->waitForBytesWritten();
  std::cout<<"TCP: LoadMission sent\n";
}

void CTcpConnection::Pause()
{
  StructHeader header;
  header.MessageID = 13;
  header.DataSize = 0;
  header.Counter = Counter;
  Counter++;
  QByteArray block;
  QDataStream out(&block, QIODevice::WriteOnly);
  out.setByteOrder(QDataStream::LittleEndian);
  out << header.MessageID;
  out << header.DataSize;
  out << header.Counter;
  pConnection->write(block);
  pConnection->flush();
  pConnection->waitForBytesWritten();
  std::cout<<"TCP: Pause sent\n";
}

void CTcpConnection::Resume()
{
  if(WaitingForResponse)
    {
      WaitingForResponse = false;
      StructHeader header;
      header.MessageID = 14;
      header.DataSize = 0;
      header.Counter = Counter;
      Counter++;
      QByteArray block;
      QDataStream out(&block, QIODevice::WriteOnly);
      out.setByteOrder(QDataStream::LittleEndian);
      out << header.MessageID;
      out << header.DataSize;
      out << header.Counter;
      pConnection->write(block);
      pConnection->flush();
      pConnection->waitForBytesWritten();
      std::cout<<"TCP: HMIResponse sent\n";
    }
  else
    {
      StructHeader header;
      header.MessageID = 11;
      header.DataSize = 0;
      header.Counter = Counter;
      Counter++;
      QByteArray block;
      QDataStream out(&block, QIODevice::WriteOnly);
      out.setByteOrder(QDataStream::LittleEndian);
      out << header.MessageID;
      out << header.DataSize;
      out << header.Counter;
      pConnection->write(block);
      pConnection->flush();
      pConnection->waitForBytesWritten();
      std::cout<<"TCP: Resume sent\n";
    }
}

void CTcpConnection::SendPathUpdate(std::vector<StructPoint> points)
{
  StructHeader header;
  header.MessageID = 15;
  header.DataSize = points.size()*sizeof(StructPoint) + sizeof(short);
  header.Counter = Counter;
  Counter++;
  QByteArray block;
  QDataStream out(&block, QIODevice::WriteOnly);
  out.setByteOrder(QDataStream::LittleEndian);
  out << header.MessageID;
  out << header.DataSize;
  out << header.Counter;
  short pathLength = points.size();
  out << pathLength;
  for(int i=0; i<pathLength; i++)
  {
      out << points[i].x;
      out << points[i].y;
  }
  pConnection->write(block);
  pConnection->flush();
  pConnection->waitForBytesWritten();
  std::cout<<"TCP: Pause sent\n";
}

void CTcpConnection::SendImageRequest()
{
  WaitingForResponse = false;
  StructHeader header;
  header.MessageID = 16;
  header.DataSize = 0;
  header.Counter = Counter;
  Counter++;
  QByteArray block;
  QDataStream out(&block, QIODevice::WriteOnly);
  out.setByteOrder(QDataStream::LittleEndian);
  out << header.MessageID;
  out << header.DataSize;
  out << header.Counter;
  pConnection->write(block);
  pConnection->flush();
  pConnection->waitForBytesWritten();
  std::cout<<"TCP: SendImageRequest sent\n";
}

void CTcpConnection::SendGridRequest()
{
  WaitingForResponse = false;
  StructHeader header;
  header.MessageID = 17;
  header.DataSize = 0;
  header.Counter = Counter;
  Counter++;
  QByteArray block;
  QDataStream out(&block, QIODevice::WriteOnly);
  out.setByteOrder(QDataStream::LittleEndian);
  out << header.MessageID;
  out << header.DataSize;
  out << header.Counter;
  pConnection->write(block);
  pConnection->flush();
  pConnection->waitForBytesWritten();
  std::cout<<"TCP: SendGridRequest sent\n";
}

void CTcpConnection::SendPathRequest()
{
  WaitingForResponse = false;
  StructHeader header;
  header.MessageID = 18;
  header.DataSize = 0;
  header.Counter = Counter;
  Counter++;
  QByteArray block;
  QDataStream out(&block, QIODevice::WriteOnly);
  out.setByteOrder(QDataStream::LittleEndian);
  out << header.MessageID;
  out << header.DataSize;
  out << header.Counter;
  pConnection->write(block);
  pConnection->flush();
  pConnection->waitForBytesWritten();
  std::cout<<"TCP: SendPathRequest sent\n";
}
