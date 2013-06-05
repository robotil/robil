#include <QMouseEvent>
#include <iostream>
#include <QGraphicsTextItem>
//#include "cline.h"
#include "graphicsview.h"

CGraphicsView::CGraphicsView(int id, QString imageName, QString dateTimeStr, QWidget *parent)
	: QGraphicsView(parent)
{
	IsOpened = true;
	Id = id;
	DateTimeStr = dateTimeStr;
	Image.load(imageName);
	Image = Image.scaled(520,428,Qt::KeepAspectRatio);
	resize(520,428);
	setMaximumHeight(428);
	setMinimumHeight(428);
	setBackgroundBrush(Image);
	setRenderHint(QPainter::Antialiasing);
	setCacheMode(QGraphicsView::CacheBackground);
	setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
}

CGraphicsView::CGraphicsView(int id, QImage image, QString dateTimeStr, QWidget *parent)
{
	std::cout << "Step61" << std::endl;
//	QRectF rect(0,0,520,428);
	IsOpened = true;
//	setSceneRect(rect);
	Id = id;
	DateTimeStr = dateTimeStr;
	if(image.isNull())
	{
		std::cout << "image.isNull()" << std::endl;
		return;
	}
	std::cout << "Step611" << std::endl;
	try
	{
//		Image = QPixmap::fromImage(image).scaled( 520,428, Qt::KeepAspectRatio ).toImage();//image.scaled(520,428,Qt::KeepAspectRatio);
		Image = image;
	}
	catch(...)
	{
		std::cout << "can't scale Image" << std::endl;
		return;
	}
	if(Image.isNull())
	{
		std::cout << "image.scaled(520,428).isNull()" << std::endl;
		return;
	}
	std::cout << "Step62" << std::endl;
	resize(image.size().width(),image.size().height());
//	setMaximumHeight(1088);
//	setMinimumHeight(1088);
	std::cout << "Step63" << std::endl;
	setBackgroundBrush(Image);
	std::cout << "Step64" << std::endl;
	setRenderHint(QPainter::Antialiasing);
	setCacheMode(QGraphicsView::CacheBackground);
	setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
	setMaximumHeight(428);
	setMinimumHeight(428);
	std::cout << "Step65" << std::endl;
}

CGraphicsView::~CGraphicsView()
{

}

void CGraphicsView::setScene(QGraphicsScene * theScene)
{
	QGraphicsView::setScene(theScene);
	DateTimeItem = new QGraphicsTextItem(DateTimeStr,NULL);
	QRectF rect = sceneRect();
	DateTimeItem->setPos(rect.width()-200,rect.height()-30);
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
//		Image.scaled(520,320);
//		QRectF rect(0,0,520,428);
		IsOpened = true;
//		setSceneRect(rect);
//		resize(520,428);
		QRectF rect = sceneRect();
		setMaximumHeight(428);
		setMinimumHeight(428);
		DateTimeItem->setPos(rect.width()-200,rect.height()-30);
	}
}

void CGraphicsView::MinimizeView()
{
	if(IsOpened)
	{
//		Image.scaled(520,320);
//		QRectF rect(0,0,520,100);
		IsOpened = false;
//		setSceneRect(rect);
//		resize(520,100);
		QRectF rect = sceneRect();
		setMaximumHeight(100);
		setMinimumHeight(100);
		DateTimeItem->setPos(rect.width()-200,rect.height()-30);
	}
}

void CGraphicsView::UpdateImage(QImage image, QString dateTimeStr)
{
		std::cout << "Step61" << std::endl;
		DateTimeStr = dateTimeStr;
		DateTimeItem->setPlainText(DateTimeStr);
	//	Image = image;
	//	Image = Image.scaled(800,600);
		if(image.isNull())
		{
			std::cout << "image.isNull()" << std::endl;
			return;
		}
		std::cout << "Step611" << std::endl;
//		if(image.scaled(726,428,Qt::KeepAspectRatio).isNull())
//		{
//			std::cout << "image.scaled(726,428).isNull()" << std::endl;
//			return;
//		}
//		std::cout << "Step614" << std::endl;
		try
		{
			Image = QPixmap::fromImage(image).scaled( 520,428, Qt::KeepAspectRatio ).toImage();//image.scaled(520,428,Qt::KeepAspectRatio);
		}
		catch(...)
		{
			std::cout << "can't scale Image" << std::endl;
			return;
		}
		if(Image.isNull())
		{
			std::cout << "image.scaled(520,428).isNull()" << std::endl;
			return;
		}
		std::cout << "Step62" << std::endl;
//		resize(726,428);
//		std::cout << "Step63" << std::endl;
		setBackgroundBrush(Image);
		std::cout << "Step64" << std::endl;
		setRenderHint(QPainter::Antialiasing);
		setCacheMode(QGraphicsView::CacheBackground);
		setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
		std::cout << "Step65" << std::endl;
}
