#include <QGraphicsItem>
#include <QFileDialog>
#include <QDateTime>
#include <QGraphicsTextItem>
#include "cnode.h"
#include "figure.h"
#include "imagedraw.h"

ImageDraw::ImageDraw(int argc, char** argv, QWidget *parent, Qt::WFlags flags)
	: QMainWindow(parent, flags)
        , C11node(argc,argv)
{
	ui.setupUi(this);
	ImageAreaCount = 0;
	
	/*connect(ui.btnRect,SIGNAL(clicked()),this,SLOT(SltOnRectClick()));
	connect(ui.btnOpenImg,SIGNAL(clicked()),this,SLOT(SltOnOpenUImgClick()));*/

	CreateNewImageArea("RobilDemoPic02.jpg");
	CreateNewImageArea("niagara_falls.jpg");
	CreateNewImageArea("RobilDemoPic02.jpg");

	C11node.init();
}

ImageDraw::~ImageDraw()
{

}

void ImageDraw::CreateNewImageArea(QString imageName)
{
	CloseOpenedImages();

	QRectF rect(0,0,800,600);
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
