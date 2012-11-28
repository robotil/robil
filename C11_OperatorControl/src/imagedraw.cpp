#include <QGraphicsItem>
#include <QFileDialog>
#include "cnode.h"
#include "figure.h"
#include "imagedraw.h"

ImageDraw::ImageDraw(int argc, char** argv, QWidget *parent, Qt::WFlags flags)
	: QMainWindow(parent, flags)
        , C11node(argc,argv)
{
	ui.setupUi(this);
	/*image.load("..\RobilDemoPic02.jpg");
	image = image.scaled(696,529);*/
	QRectF rect(0,0,696,529);
	scene = new QGraphicsScene(rect,this);
//	scene->setSceneRect(ui.graphicsView->geometry());
	ui.graphicsView->setScene(scene);
	ui.graphicsView->setSceneRect(rect);
	ui.graphicsView->setRenderHint(QPainter::Antialiasing);
	ui.graphicsView->setBackgroundBrush(image);
	ui.graphicsView->setCacheMode(QGraphicsView::CacheBackground);
    ui.graphicsView->setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
	
	/*connect(ui.btnRect,SIGNAL(clicked()),this,SLOT(SltOnRectClick()));
	connect(ui.btnOpenImg,SIGNAL(clicked()),this,SLOT(SltOnOpenUImgClick()));*/

	C11node.init();
}

ImageDraw::~ImageDraw()
{

}

void ImageDraw::SltOnRectClick()
{
	CNode* node1 = new CNode(275,290,ui.graphicsView->scene());
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
	figure->AddNode(node4);
}

void ImageDraw::SltOnOpenUImgClick()
{
	QString fileName = QFileDialog::getOpenFileName(this,
     tr("Open Image"), "", tr("Image Files (*.png *.jpg *.bmp)"));

	image.load(fileName);
	image = image.scaled(696,529);
	ui.graphicsView->setBackgroundBrush(image);
	update();
}
