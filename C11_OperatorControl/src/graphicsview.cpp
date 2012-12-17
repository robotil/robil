#include <QMouseEvent>
#include <iostream>
#include "cline.h"
#include "graphicsview.h"

CGraphicsView::CGraphicsView(int id, QString imageName, QString dateTimeStr, QWidget *parent)
	: QGraphicsView(parent)
{
	IsOpened = true;
	Id = id;
	DateTimeStr = dateTimeStr;
	Image.load(imageName);
	Image = Image.scaled(520,420);
	resize(520,420);
	setBackgroundBrush(Image);
	setRenderHint(QPainter::Antialiasing);
	setCacheMode(QGraphicsView::CacheBackground);
	setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
}

CGraphicsView::CGraphicsView(int id, QImage image, QString dateTimeStr, QWidget *parent)
{
	std::cout << "Step61" << std::endl;
	IsOpened = true;
	Id = id;
	DateTimeStr = dateTimeStr;
//	Image = image;
//	Image = Image.scaled(800,600);
	if(image.isNull())
	{
		std::cout << "image.isNull()" << std::endl;
		return;
	}
	std::cout << "Step611" << std::endl;
	if(image.scaled(520,420,Qt::KeepAspectRatio).isNull())
	{
		std::cout << "image.scaled(520,420).isNull()" << std::endl;
		return;
	}
	std::cout << "Step614" << std::endl;
	Image = image.scaled(520,420,Qt::KeepAspectRatio);
	std::cout << "Step62" << std::endl;
	resize(520,420);
	std::cout << "Step63" << std::endl;
	setBackgroundBrush(Image);
	std::cout << "Step64" << std::endl;
	setRenderHint(QPainter::Antialiasing);
	setCacheMode(QGraphicsView::CacheBackground);
	setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
	std::cout << "Step65" << std::endl;
}

CGraphicsView::~CGraphicsView()
{

}

void CGraphicsView::setScene(QGraphicsScene * theScene)
{
	QGraphicsView::setScene(theScene);
	DateTimeItem = new QGraphicsTextItem(DateTimeStr,NULL);
	DateTimeItem->setPos(320,390);
	DateTimeItem->setDefaultTextColor(Qt::white);
	scene()->addItem(DateTimeItem);
	scene()->setSceneRect(geometry());
}

void CGraphicsView::mouseDoubleClickEvent ( QMouseEvent * event )
{
	if(IsOpened)
	{
		MinimizeView();
	}
	else
	{
		emit SigOpened(Id);
		OpenView();
	}
}

int CGraphicsView::GetId()
{
	return Id;
}

void CGraphicsView::SetOpen(bool isOpen)
{
	IsOpened = isOpen;
}

bool CGraphicsView::IsOpen()
{
	return IsOpened;
}

void CGraphicsView::OpenView()
{
	if(!IsOpened)
	{
		Image.scaled(520,420);
		QRectF rect(0,0,520,420);
		IsOpened = true;
		setSceneRect(rect);
		resize(520,420);
		DateTimeItem->setPos(320,390);
	}
}

void CGraphicsView::MinimizeView()
{
	if(IsOpened)
	{
		Image.scaled(520,100);
		QRectF rect(0,0,520,100);
		IsOpened = false;
		setSceneRect(rect);
		resize(520,100);
		DateTimeItem->setPos(320,70);
	}
}

