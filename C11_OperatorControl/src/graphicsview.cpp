#include <QMouseEvent>
#include "cline.h"
#include "graphicsview.h"

CGraphicsView::CGraphicsView(int id, QString imageName, QString dateTimeStr, QWidget *parent)
	: QGraphicsView(parent)
{
	IsOpened = true;
	Id = id;
	DateTimeStr = dateTimeStr;
	Image.load(imageName);
	Image = Image.scaled(800,600);
	resize(800,600);
	setBackgroundBrush(Image);
	setRenderHint(QPainter::Antialiasing);
	setCacheMode(QGraphicsView::CacheBackground);
	setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
}

CGraphicsView::~CGraphicsView()
{

}

void CGraphicsView::setScene(QGraphicsScene * theScene)
{
	QGraphicsView::setScene(theScene);
	DateTimeItem = new QGraphicsTextItem(DateTimeStr,NULL);
	DateTimeItem->setPos(600,570);
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
		Image.scaled(800,600);
		QRectF rect(0,0,800,600);
		IsOpened = true;
		setSceneRect(rect);
		resize(800,600);
		DateTimeItem->setPos(600,570);
	}
}

void CGraphicsView::MinimizeView()
{
	if(IsOpened)
	{
		Image.scaled(800,100);
		QRectF rect(0,0,800,100);
		IsOpened = false;
		setSceneRect(rect);
		resize(800,100);
		DateTimeItem->setPos(600,70);
	}
}

