#include <QPainter>
#include <QGraphicsScene>
#include <QLine>
#include "scaleitem.h"

CScaleItem::CScaleItem(QGraphicsScene* scene): QGraphicsItem()
{

}

CScaleItem::~CScaleItem()
{

}

QRectF CScaleItem::boundingRect() const
{
  return QRectF(0, 0,12, 800);
}

void CScaleItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
  painter->setPen(QPen(QColor(230,0,0)));
  QBrush br;
  br=QBrush(Qt::white);
  painter->setBrush(br);
  painter->drawLine(51,942,51,140);
  painter->drawLine(62,130,862,130);
  int startYPos = -5;
  int startXPos = -12;
  for(int i=0; i<26; i++)
    {
      painter->drawLine(46,942-(i*8*4),56,942-(i*8*4));
      painter->drawText(30,945-(i*8*4),QString::number(startYPos));
      if(i<25)
      {
          painter->drawLine(74+(i*8*4),125,74+(i*8*4),135);
          painter->drawText(71+(i*8*4),120,QString::number(startXPos));
      }
      startYPos++;
      startXPos++;
    }
//  painter->drawRect(boundingRect());
}
