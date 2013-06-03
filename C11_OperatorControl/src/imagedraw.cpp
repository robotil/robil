#include <QGraphicsItem>
#include <QFileDialog>
#include <QDateTime>
#include <QGraphicsTextItem>
#include <QFile>
//#include "cnode.h"
//#include "figure.h"
#include "imagedraw.h"

ImageDraw::ImageDraw(int argc, char** argv, QWidget *parent, Qt::WFlags flags)
	: QMainWindow(parent, flags)
//        , C11node(argc,argv,this)
{
	ui.setupUi(this);

	ui.mainToolBar->hide();
	ui.menuBar->hide();
	ui.statusBar->hide();
	ui.btnCreate->setEnabled(false);

	ImageAreaCount = 0;
	IsUpdateCurrentImg = false;
	ERunStatus = STOPPED_ENUM;

	WaitTimer = new QTimer(this);

	connect(this,SIGNAL(SigOnNewImg(QImage)),this,SLOT(SltOnNewImg(QImage)),Qt::QueuedConnection);
	connect(ui.btnPlayPause,SIGNAL(clicked(bool)),this,SLOT(SltOnPlayPauseClick(bool)));
	connect(ui.btnAllow,SIGNAL(clicked()),this,SLOT(SltOnAllowClick()));
	connect(ui.btnCreate,SIGNAL(clicked(bool)),this,SLOT(SltOnCreateClick(bool)));
	connect(ui.btnPath,SIGNAL(clicked(bool)),this,SLOT(SltOnPathClick(bool)));
	connect(WaitTimer,SIGNAL(timeout()),this,SLOT(SltOnWaitTimeout()));
	connect(ui.mapWidget,SIGNAL(SigOperatorAction()),this,SLOT(SltOperatorAction()));
//	C11node.init();

	QFile file("C11Config.txt");
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
	  {
	    std::cout << "Can't open config file!!! Restart the application" << std::endl;
	    std::cout << "Can't open config file!!! Restart the application" << std::endl;
	    std::cout << "Can't open config file!!! Restart the application" << std::endl;
	    std::cout << "Can't open config file!!! Restart the application" << std::endl;
	    std::cout << "Can't open config file!!! Restart the application" << std::endl;
	    std::cout << "Can't open config file!!! Restart the application" << std::endl;
	    std::cout << "Can't open config file!!! Restart the application" << std::endl;
	    std::cout << "Can't open config file!!! Restart the application" << std::endl;
	    std::cout << "Can't open config file!!! Restart the application" << std::endl;
	    std::cout << "Can't open config file!!! Restart the application" << std::endl;
	  }

	else
	  {
	    QTextStream in(&file);
            QString line = in.readLine();
//	    pCTcpConnection = new CTcpConnection(QString("172.23.1.130"),45671);
            pCTcpConnection = new CTcpConnection(line,45677);


            connect(pCTcpConnection,SIGNAL(SigOnImgReceived(QImage)),this,SLOT(SltOnNewImg(QImage)));
            connect(pCTcpConnection,SIGNAL(SigOnGridReceived(int[100][100],StructPoint,int,int,double)),this,SLOT(SltOnGridReceived(int[100][100],StructPoint,int,int,double)));

            pCTcpConnection->SetSubscriber(this);
	  }

	QFile missfile("Missions.txt");
	if (!missfile.open(QIODevice::ReadOnly | QIODevice::Text))
          {
            std::cout << "Can't open Missions config file!!! Restart the application" << std::endl;
          }
	else
	  {
	    while (!missfile.atEnd())
	     {
	        QString line = missfile.readLine();
	        MissionsList.append(line);
             }
	    ui.cmbMissions->addItems(MissionsList);
	  }

//	QString fileName = QFileDialog::getOpenFileName(this,
//	     tr("Open Image"), "", tr("Image Files (*.png *.jpg *.bmp)"));
//
//	QImage image;
//	image.load(fileName);
//	SltOnNewImg(image);
}

ImageDraw::~ImageDraw()
{
  if(WaitTimer!=NULL)
    {
      delete WaitTimer;
      WaitTimer = NULL;
    }
  if(pCTcpConnection != NULL)
    {
      delete pCTcpConnection;
      pCTcpConnection = NULL;
    }
}

void ImageDraw::CreateNewImageArea(QString imageName)
{
	CloseOpenedImages();

	QRectF rect(0,0,520,428);
	QGraphicsScene* pScene = new QGraphicsScene(rect,this);

	QDateTime dateTime = QDateTime::currentDateTime();
	QString dateStr = dateTime.toString("dd.MM.yyyy");
	QString timeStr = dateTime.toString("hh:mm:ss");
	QString DateTimeStr = dateStr + " " + timeStr;

	CGraphicsView* pCGraphicsView = new CGraphicsView(ImageAreaCount,imageName,DateTimeStr,this);
	ImageAreaCount++;

	pCGraphicsView->setScene(pScene);
	pCGraphicsView->setSceneRect(rect);

	ui.layImages->addWidget(pCGraphicsView);

	ImageAreas.insert(pCGraphicsView->GetId(),pCGraphicsView);

	for(int i=ImageAreaCount-1; i>=0; i--)
	{
		if(i != pCGraphicsView->GetId())
		{
			ui.layImages->removeWidget(ImageAreas[i]);
			ui.layImages->addWidget(ImageAreas[i]);
		}
	}

	connect(pCGraphicsView,SIGNAL(SigOpened(int)),this,SLOT(SltImageAreaOpened(int)));
}

void ImageDraw::CloseOpenedImages()
{
	for(int i=0; i<ImageAreaCount; i++)
	{
		if(ImageAreas[i]->IsOpen())
		{
			ImageAreas[i]->MinimizeView();
		}
	}
}

void ImageDraw::SltImageAreaOpened(int id)
{
	for(int i=0; i<ImageAreaCount; i++)
	{
		if(ImageAreas[i]->GetId() != id)
		{
			ImageAreas[i]->MinimizeView();
		}
	}
}

void ImageDraw::OnImgReceived(QImage image)
{
	std::cout << "Step2" << std::endl;
	QImage myImage(image);
	emit SigOnNewImg(myImage);
	std::cout << "Step3" << std::endl;

}

void ImageDraw::OnImgReceived(std::string fileName)
{
	QImage myImage;
	QString qfileName = QString::fromStdString(fileName);
	myImage.load(qfileName);
	emit SigOnNewImg(myImage);
}

void ImageDraw::SltOnGridReceived(int grid[100][100],StructPoint robotPos,int xOffset,int yOffset,double orient)
{
  OnOccupancyGridReceived(grid,robotPos,xOffset,yOffset,orient);
}

void ImageDraw::OnOccupancyGridReceived(int grid[100][100], StructPoint robotPos, int xOffset, int yOffset, double orient)
{
	ui.mapWidget->UpdateGrid(grid,robotPos,xOffset,yOffset,orient);
}

void ImageDraw::OnPathReceived(std::vector<StructPoint> points)
{
	ui.mapWidget->AddPath(points);
}

void ImageDraw::OnHMIResponseReceived()
{
              ui.btnPlayPause->setChecked(false);
              ERunStatus = PAUSED_ENUM;
              ui.btnCreate->setEnabled(true);
              ui.mapWidget->SetEditable(true);
              WaitTimer->setSingleShot(true);
              WaitTimer->start(10000);
}

void ImageDraw::SltOnWaitTimeout()
{
  std::cout << "SltOnWaitTimeout" << std::endl;
  OnWaitResponseFinished();
//  C11node.Resume();
  pCTcpConnection->Resume();
}

void ImageDraw::OnWaitResponseFinished()
{
              ui.btnPlayPause->setChecked(true);
              ERunStatus = RUNNING_ENUM;
              ui.btnCreate->setEnabled(false);
              SltOnCreateClick(false);
              ui.mapWidget->SetEditable(false);
}

void ImageDraw::OnExecutionStatusUpdate(int status)
{
        if(0 == status)
          {
            ui.btnPlayPause->setChecked(true);
            ERunStatus = RUNNING_ENUM;
            ui.btnCreate->setEnabled(false);
            SltOnCreateClick(false);
            ui.mapWidget->SetEditable(false);
          }
        else if (1 == status)
          {
            ui.btnPlayPause->setChecked(false);
            ERunStatus = PAUSED_ENUM;
            ui.btnCreate->setEnabled(true);
            ui.mapWidget->SetEditable(true);
          }
        else if( 2 == status)
          {
            ui.btnPlayPause->setChecked(false);
            ERunStatus = STOPPED_ENUM;
            ui.btnCreate->setEnabled(false);
            SltOnCreateClick(false);
            ui.mapWidget->SetEditable(false);
          }
}

void ImageDraw::SltOnNewImg(QImage image)
{
//	QLabel* lbl = new QLabel(this);
//	lbl->setPixmap(QPixmap::fromImage(image));
//	ui.layImages->addWidget(lbl);
	if(!IsUpdateCurrentImg)
	{
//		IsUpdateCurrentImg = true;
		std::cout << "Step4" << std::endl;
		CloseOpenedImages();

		std::cout << "Step5" << std::endl;
		QRectF rect(0,0,image.size().width(),image.size().height());
		QGraphicsScene* pScene = new QGraphicsScene(rect,this);

		QDateTime dateTime = QDateTime::currentDateTime();
		QString dateStr = dateTime.toString("dd.MM.yyyy");
		QString timeStr = dateTime.toString("hh:mm:ss");
		QString DateTimeStr = dateStr + " " + timeStr;

		std::cout << "Step6" << std::endl;
		CGraphicsView* pCGraphicsView = new CGraphicsView(ImageAreaCount,image,DateTimeStr,this);
		ImageAreaCount++;
		std::cout << "Step7" << std::endl;

		pCGraphicsView->setScene(pScene);
		pCGraphicsView->setSceneRect(rect);

		ui.layImages->addWidget(pCGraphicsView);

		ImageAreas.insert(pCGraphicsView->GetId(),pCGraphicsView);

		for(int i=ImageAreaCount-1; i>=0; i--)
		{
			if(i != pCGraphicsView->GetId())
			{
				ui.layImages->removeWidget(ImageAreas[i]);
				ui.layImages->addWidget(ImageAreas[i]);
			}
		}

		connect(pCGraphicsView,SIGNAL(SigOpened(int)),this,SLOT(SltImageAreaOpened(int)));
		std::cout << "Step8" << std::endl;
	}
	else
	{
//		QDateTime dateTime = QDateTime::currentDateTime();
//		QString dateStr = dateTime.toString("dd.MM.yyyy");
//		QString timeStr = dateTime.toString("hh:mm:ss");
//		QString DateTimeStr = dateStr + " " + timeStr;
//
//		CGraphicsView* pCGraphicsView = ImageAreas[ImageAreaCount-1];
//		pCGraphicsView->UpdateImage(image,DateTimeStr);
	}
}

void ImageDraw::SltOnRectClick()
{
/*	CNode* node1 = new CNode(275,290,ui.graphicsView->scene());
	ui.graphicsView->scene()->addItem(node1);
	CNode* node2 = new CNode(435,290,ui.graphicsView->scene());
	ui.graphicsView->scene()->addItem(node2);
	CNode* node3 = new CNode(435,460,ui.graphicsView->scene());
	ui.graphicsView->scene()->addItem(node3);
	CNode* node4 = new CNode(275,460,ui.graphicsView->scene());
	ui.graphicsView->scene()->addItem(node4);

	CFigure* figure = new CFigure(ui.graphicsView->scene());
	figure->AddNode(node1);
	figure->AddNode(node2);
	figure->AddNode(node3);
	figure->AddNode(node4);*/
}

void ImageDraw::SltOnOpenUImgClick()
{
/*	QString fileName = QFileDialog::getOpenFileName(this,
     tr("Open Image"), "", tr("Image Files (*.png *.jpg *.bmp)"));

	image.load(fileName);
	image = image.scaled(696,529);
	ui.graphicsView->setBackgroundBrush(image);
	update();*/
}

void ImageDraw::SltOnPlayPauseClick(bool checked)
{
	if(checked)
	{
	        if(ERunStatus==STOPPED_ENUM)
                {
	            QString curMission = ui.cmbMissions->currentText();
	            std::cout << "Selected mission is: "<< curMission.toStdString() << std::endl;

                  if(!curMission.isEmpty())
                  {
                          int index=0;
                          index = MissionsList.indexOf(curMission);
                          //C11node.LoadMission(index);
                          pCTcpConnection->LoadMission(index);
                          ERunStatus = RUNNING_ENUM;
                          ui.btnCreate->setEnabled(false);
                          SltOnCreateClick(false);
                          ui.mapWidget->SetEditable(false);
                  }
                }
	        else
                {
	            std::vector<StructPoint> points = ui.mapWidget->GetUpdatedRoute();
	            if(!points.empty())
	              {
	                //C11node.SendPathUpdate(points);
	                pCTcpConnection->SendPathUpdate(points);
	              }
//	            C11node.Resume();
	            pCTcpConnection->Resume();
	            ERunStatus = RUNNING_ENUM;
	            ui.btnCreate->setEnabled(false);
	            SltOnCreateClick(false);
                }
	}
	else
        {
	    //C11node.Pause();
	    pCTcpConnection->Pause();
	    ERunStatus = PAUSED_ENUM;
	    ui.btnCreate->setEnabled(true);
	    ui.mapWidget->SetEditable(true);
        }
}

void ImageDraw::SltOnCreateClick(bool checked)
{
	if(checked)
	{
		ui.btnPath->setEnabled(true);
		ui.btnNoGo->setEnabled(true);
		ui.btnDoor->setEnabled(true);
		ui.btnCarInt->setEnabled(true);
		ui.btnRotate->setEnabled(true);
		ui.btnSteps->setEnabled(true);
	}
	else
	{
		ui.btnPath->setEnabled(false);
		ui.btnNoGo->setEnabled(false);
		ui.btnDoor->setEnabled(false);
		ui.btnCarInt->setEnabled(false);
		ui.btnRotate->setEnabled(false);
		ui.btnSteps->setEnabled(false);
	}
}

void ImageDraw::SltOnPathClick(bool checked)
{
	if(checked)
	{
		ui.mapWidget->setMode(E_PATH_MODE);
	}
}

void ImageDraw::SltOnAllowClick()
{
  int reqType = ui.cmbRequest->currentIndex();
  switch(reqType)
  {
    case 1:
      pCTcpConnection->SendImageRequest();
      break;
    case 2:
      pCTcpConnection->SendGridRequest();
      break;
    case 3:
      pCTcpConnection->SendPathRequest();
      break;
    default:
      break;
  }
}


void ImageDraw::SltOperatorAction()
{
        if(WaitTimer->isActive())
          {
            WaitTimer->stop();
          }
}
