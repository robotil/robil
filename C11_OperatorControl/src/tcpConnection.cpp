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
  ResetImgReceive = false;
  ExecuterString = "";
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
              if(ResetImgReceive)
                {
                  ResetImgReceive = false;
                  ImgSize = 0;
                  QByteArray bfr;
                  IsSendingImage = false;
                  bfr = pConnection->read(bufSize);
                }
              return;
            }
          else
          {
              IsSendingImage = false;
          }
          if(!IsSendingImage)
           {
        	  ImgSize = 0;
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
            	ImgSize = 0;
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
//                for(int i=99; i>=0; i--)
//                  {
//                          for(int j=0; j<100; j++)
//                          {
//                              switch ( grid.Grid[i][j] ){
//                              case 0:
//                                  std::cout<<'.'; break;
//                              case 1:
//                                  std::cout<<'*'; break;
//                              case 2:
//                                  std::cout<<'.'; break;
//                              default:
//                                  std::cout<<'?'; break;
//                              }
//                          }
//                          std::cout<<std::endl;
//                  }

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
        	  ImgSize = 0;
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
      else if(IsSendingExecuterStatus)
      {
    	  QString str;
    	  in>>str;
    	  ExecuterString.append(str);
    	  if(!ExecuterString.length()<=ImgSize)
    	  {
    		  ImgSize = 0;
    		  IsSendingExecuterStatus = false;
    		  pITcpConnectionInterface->OnExecuterStackUpdate(ExecuterString);
    		  ExecuterString = "";
    	  }
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
      }
      else
      { if (pConnection->bytesAvailable() < (int)sizeof(short))
        {
                return;
        }
        in >> msgId;

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
 //           pTimer->start(5000);
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
              ImgSize = 0;
            }
          }
        else if(5 == msgId)
          {
            int status;
            in >> status;
            pITcpConnectionInterface->OnExecutionStatusUpdate(status);
            ImgSize = 0;
          }
        else if(6 == msgId)
          {
            double timeSec;
            int completionScore;
            int falls;
            QString message;
            in >> timeSec;
            in >> completionScore;
            in >> falls;
            in >> message;
            pITcpConnectionInterface->OnVRCScoreData(timeSec,completionScore,falls,message);
            ImgSize = 0;
          }
        else if(7 == msgId)
        {
          QString down;
          in >> down;
          pITcpConnectionInterface->OnDownlinkUpdate(down);
          ImgSize = 0;
        }
        else if(8 == msgId)
        {
          QString up;
          in >> up;
          pITcpConnectionInterface->OnUplinkUpdate(up);
          ImgSize = 0;
        }
        else if(9 == msgId)
		{
          StructPoint pos;
          StructOrientation orient;
		  in >> pos.x >> pos.y >> orient.yaw >> orient.pitch >> orient.roll;
		  std::cout<<"TCP: Robot data -> "<<pos.x<<pos.y<<orient.yaw<<orient.pitch<<orient.roll<<"\n";
		  pITcpConnectionInterface->OnRobotData(pos,orient);
		  ImgSize = 0;
		}
        else if(31 == msgId)
		  {
 //       	IsSendingExecuterStatus = true;
 //       	ImgSize = dataSize;
 //       	std::cout<<"TCP: ExecuterStack coming!\n";
        	ImgSize = dataSize;
        	QString str;

		    in>>str;
		    if(str.length() < ImgSize)
		    {
		    	IsSendingExecuterStatus = true;
		    }
		    else
		    {
	//		    std::cout<<"ExecuterStatus received: "<<str.toStdString()<<std::endl;
				pITcpConnectionInterface->OnExecuterStackUpdate(str);
				ImgSize = 0;
		    }
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
  std::cout<<"TCP: Image receive timeout"<<std::endl;
  ResetImgReceive = true;
  pTimer->stop();
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

void CTcpConnection::Stop()
{
  StructHeader header;
  header.MessageID = 20;
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
  std::cout<<"TCP: Stop sent\n";
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

void CTcpConnection::SendAllRequest()
{
  WaitingForResponse = false;
  StructHeader header;
  header.MessageID = 19;
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
  std::cout<<"TCP: SendAllRequest sent\n";
}

void CTcpConnection::SendGridAndPathRequest()
{
	WaitingForResponse = false;
	StructHeader header;
	header.MessageID = 23;
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
	std::cout<<"TCP: SendAllRequest sent\n";
}


void CTcpConnection::SendNewGoal(StructPoint goal)
{
  StructHeader header;
  header.MessageID = 21;
  header.DataSize = 0;
  header.Counter = Counter;
  Counter++;
  QByteArray block;
  QDataStream out(&block, QIODevice::WriteOnly);
  out.setByteOrder(QDataStream::LittleEndian);
  out << header.MessageID;
  out << header.DataSize;
  out << header.Counter;
  out << goal.x;
  out << goal.y;
  pConnection->write(block);
  pConnection->flush();
  pConnection->waitForBytesWritten();
  std::cout<<"TCP: SendNewGoal sent\n";
}

void CTcpConnection::SendReset()
{
  StructHeader header;
  header.MessageID = 22;
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
  std::cout<<"TCP: SendReset sent\n";
}
