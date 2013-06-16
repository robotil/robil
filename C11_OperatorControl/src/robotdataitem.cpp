#include <QPainter>
#include <QGraphicsScene>
#include <QLine>
#include "robotdataitem.h"

CRobotDataItem::CRobotDataItem(QGraphicsScene* scene, QString str): QGraphicsItem(), Text(str)
{

}

CRobotDataItem::~CRobotDataItem()
{

}

QRectF CRobotDataItem::boundingRect() const
{
  return QRectF(0, 0,500, 12);
}

void CRobotDataItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
  painter->setPen(QPen(QColor(255,255,255)));
  QBrush br;
  br=QBrush(Qt::white);
  painter->setBrush(br);

  painter->drawText(30,100,Text);
}

void CRobotDataItem::SetText(QString str)
{
	Text = str;
	update();
}
