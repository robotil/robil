#include "pixitem.h"
#include <QPainter>
//#include <QStyleOptionGraphicsItem>
#include <QGraphicsScene>

int first;
QPointF lastPoint;

CPixItem::CPixItem(int color,QGraphicsScene* scene,int px,int py)
	//: QGraphicsRectItem()
{
	pScene = scene;
	posX = px;
	posY = py;
	setPos(posX,posY);
	PixColor = color;
	first = 0;
}
CPixItem::~CPixItem()
{

}
QRectF CPixItem::boundingRect()  const
{
    return QRectF(0, 0,12, 12);
}

void CPixItem::SetColor(int color)
{
	PixColor = color;
}

QBrush CPixItem::TransEnumToBrush()
{
	/*clr_gray_20,
	clr_gray_30,
	clr_gray_40,
	clr_red,
	clr_black*/
	QBrush br;
	switch(PixColor)
		{
		case 0:
			{
				br=QBrush(Qt::white/*QColor(248,248,248)*/);//clr_white
				break;
			}
		case 3:
			{
				br=QBrush(QColor(50,50,50));//clr_gray_10
				break;
			}
		case 4:
			{
				br=br=QBrush(QColor(80,80,80));//clr_gray_20
				break;
			}
		case 5:
			{
				br=QBrush(QColor(141,141,141));//clr_gray_30
				break;
			}
		case 6:
			{
				br=QBrush(QColor(200,200,200));//clr_gray_40
				break;
			}
		case 1:
			{
				br=QBrush(Qt::red/*QColor(50,50,50)*/);
				break;
			}
		case 2:
			{
				br=QBrush(Qt::black);//black
				break;
			}
		/*case 7:
			{
				br=QBrush(QColor(100,100,100));
				break;
			}*/
		}
	return br;
}

//void CPixItem::ChangColor()
//{
//	switch(PixColor)
//		{
//		case 0:
//			{
//				PixColor++;
//				break;
//			}
//		case 1:
//			{
//				PixColor++;
//				break;
//			}
//		case 2:
//			{
//				PixColor++;
//				break;
//			}
//		case 3:
//			{
//				PixColor++;
//				break;
//			}
//		case 4:
//			{
//				PixColor++;
//				break;
//			}
//		case 5:
//			{
//				PixColor++;
//				break;
//			}
//		case 6:
//			{
//				PixColor++;
//				break;
//			}
//		case 7:
//			{
//				PixColor=0;
//				break;
//			}
//		}
//}
void CPixItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
	painter->setPen(QPen(QColor(230,230,230)));
	painter->setBrush(TransEnumToBrush());
	painter->drawRect(boundingRect());
}
//void CPixItem::mousePress(QPointF p)
//{
//	QPointF tempPoint;
//	tempPoint.setX(this->pos().x()-p.x());
//	tempPoint.setY(this->pos().y()-p.y());
//	QPainter pPaint;
//	QPointF lastPoint;
//	pPaint.setPen(QPen(Qt::black, 0));
//	//if(this->contains(p))
//	//{
//		lastPoint = tempPoint;
//		ChangColor();
//		TransEnumToBrush();
//	//}
//	pPaint.drawRect(boundingRect());
//	update();
//}

//bool CPixItem::isContain(QPointF p)
//{
//	QPointF tempPoint;
//	tempPoint.setX(posX-p.x());
//	tempPoint.setY(posY-p.y());
//
//	if((p.x()>boundingRect().left())&&(p.x()<boundingRect().right()))
//		if((p.y()>boundingRect().bottom())&&(p.y()<boundingRect().top()))
//			return true;
//		else
//			return false;
//	else
//		return false;
//}
//void CPixItem::mouseMoveEvent ( QGraphicsSceneMouseEvent * event )
//{
//	if ((event->buttons() & Qt::LeftButton))
//         drawLineTo(event->pos());
//
//	QGraphicsItem::mouseMoveEvent(event);
//
//}
//void CPixItem::mousePressEvent(QGraphicsSceneMouseEvent *event)
//{
//	PressPoint.setX(event->pos().x());
//	PressPoint.setY(event->pos().y());
//
//	QPainter pPaint;
//	QPointF lastPoint;
//	pPaint.setPen(QPen(Qt::black, 0));
//	if((this->contains(event->pos()))&&(event->button() == Qt::LeftButton))
//	{
//		lastPoint = event->pos();
//		ChangColor();
//		TransEnumToBrush();
//	}
//	pPaint.drawRect(boundingRect());
//	//update();
//}

//void CPixItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
//{
//	if (event->button() == Qt::LeftButton) 
//	{
//         //drawLineTo(event->pos());
//		QGraphicsItem::mouseReleaseEvent(event);
//    }
//	update();
//}
void CPixItem::drawLineTo(const QPointF &endPoint)
{
     /*QPainter painter;
	 lastPoint.setX(0.0);
	 lastPoint.setY(0.0);
     painter.setPen(QPen(Qt::white));
     painter.drawLine(lastPoint, endPoint);
                                   
     lastPoint = endPoint;*/
 }
