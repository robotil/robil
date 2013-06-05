#include "pixitem.h"
#include <QPainter>
#include <QGraphicsScene>

static bool first;
QPointF lastPoint;

CPixItem::CPixItem(int color,QGraphicsScene* scene,int px,int py)
	: QGraphicsItem()
{
	posX = px;
	posY = py;
	setPos(posX,posY);
	PixColor = color;
	/*if(!first)
	{
		traingle = new CtraingleItem(scene,QPointF(posX,posY));
		scene->addItem(traingle);
		first = 1;
	}*/
}
CPixItem::~CPixItem()
{
	//delete pScene;
}
QRectF CPixItem::boundingRect()  const
{
    return QRectF(0, 0,12, 12);
}

QBrush CPixItem::TransEnumToBrush()
{
	QBrush br;
	switch(PixColor)
		{
		case 0:
			{
				br=QBrush(Qt::white);//clr_white
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
				br=QBrush(Qt::red);//red
				break;
			}
		case 2:
			{
				br=QBrush(Qt::black);//black
				break;
			}
		default:
			break;
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
//		default:
//			break;
//		}
//}

void CPixItem::SetColor(int color)
{
  PixColor = color;
}

void CPixItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
	painter->setPen(QPen(QColor(230,230,230)));
	painter->setBrush(TransEnumToBrush());
	painter->drawRect(boundingRect());
}
