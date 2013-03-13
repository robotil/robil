#include "arcitem.h"
#include <QPainter>
#include <QGraphicsScene>
#include <math.h>

CArcItem::CArcItem(QPointF p,QGraphicsScene* scene)
: QGraphicsEllipseItem()
{
	centerPos = p;
	radius = 160;
	setStartAngle(60 * 16);
	setSpanAngle(60 * 16);

	RightPoint = new CPointItem(GetRightPoint(),scene);
	scene->addItem(RightPoint);

	LeftPoint = new CPointItem(GetLeftPoint(),scene);
	scene->addItem(LeftPoint);
	
	PressedPoint = NULL;
}

CArcItem::~CArcItem()
{
	
}
void CArcItem::updatePie_LeftPressed(QPointF p)
{
	double pi = 3.1415926535;
	int spanAngl;
	double c,span;
	int lastSpanAngle = spanAngle()/16;
	int lastStartAngle = startAngle()/16;

	if(p.y() < centerPos.y())
	{
		c = (p.x()-centerPos.x())/radius;
		span = acos(c);
		spanAngl = (180*span)/pi;
		setSpanAngle((spanAngl-lastStartAngle)*16);
		LeftPoint->setPointParam(GetLeftPoint());
	}
	else
	{
		c = (p.x()-centerPos.x())/radius;
		span = acos(-c);
		spanAngl = 180+(180*span)/pi;
		setSpanAngle((spanAngl-lastStartAngle)*16);
		LeftPoint->setPointParam(GetLeftPoint());
	}
}
void CArcItem::updatePie_RightPressed(QPointF p)
{
	double pi = 3.1415926535;
	int startAngl,spanAngl;
	double c,start,span;
	int lastStartAngle = startAngle()/16;
	int lastSpanAngle = spanAngle()/16;
	
	QPointF LastRightPoint = GetRightPoint();
	QPointF LastLeftPoint = GetLeftPoint();
	
	if(p.y() <= centerPos.y())
	{
		c = (p.x()-centerPos.x())/radius;
		start = acos(c);
		startAngl = (180*start)/pi;
		setStartAngle(startAngl*16);
		RightPoint->setPointParam(p);

		c = (LastLeftPoint.x()-centerPos.x())/radius;
		span = acos(c);
		spanAngl = (180*span)/pi;
		setSpanAngle((spanAngl-startAngl)*16);
		LeftPoint->setPointParam(GetLeftPoint());
	}
	else
	{
		c = (p.x()-centerPos.x())/radius;
		start = acos(-c);
		startAngl = 180+(180*start)/pi;
		setStartAngle(startAngl*16);
		RightPoint->setPointParam(p);

		c = (LastLeftPoint.x()-centerPos.x())/radius;
		span = acos(-c);
		spanAngl = 180+(180*span)/pi;
		setSpanAngle(spanAngl*16);
		LeftPoint->setPointParam(GetLeftPoint());
	}
}
QPointF CArcItem::GetRightPoint()
{
	double pi = 3.1415926535;
	
	QPointF R_temp;
	double startAngl = startAngle()/16;
	double start = pi / (180/startAngl);
	double s = sin((double)(start));
	double c = cos((double)(start));

	R_temp.setY(centerPos.y()-(s*radius));
	R_temp.setX(centerPos.x()+(c*radius));
	
	return R_temp;
}
QPointF CArcItem::GetLeftPoint()
{
	double pi = 3.1415926535;
	
	QPointF L_temp;
	double startAngl =( spanAngle()/16)+(startAngle()/16);
	double start = pi / (180/startAngl);
	double s = sin((double)(start));
	double c = cos((double)(start));

	L_temp.setY(centerPos.y()-(s*radius));
	L_temp.setX(centerPos.x()+(c*radius));
	
	return L_temp;
}
QPainterPath CArcItem::shape()  const 
{
	return QGraphicsEllipseItem::shape();
}
QRectF CArcItem::boundingRect()  const
{
	return QRectF(centerPos.x()-radius,centerPos.y()-radius,radius*2,radius*2);
}
void CArcItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
	QRectF rec;
	painter->setPen(QPen(Qt::green));
	rec = QRectF(QPointF(centerPos.x()-radius,centerPos.y()-radius),QPointF(centerPos.x()+radius,centerPos.y()+radius));
	
	painter->drawPie(rec, startAngle(), spanAngle());
}
bool CArcItem::IsArcContains(QPointF p)
{
	bool b;
	b = RightPoint->isContainPoint(p);
	if(b)
		PressedPoint = RightPoint;
	else
	{
		b = LeftPoint->isContainPoint(p);
		if(b)
			PressedPoint = LeftPoint;
	}
	
	return b;
}
void CArcItem::MoveArcPoint(QPointF p)
{
	checkMousePos(p);
	scene()->update();
	update();
}
void CArcItem::checkMousePos(QPointF new_point)
{
	float x1,y1;
	double m;
	int Xnew,Ynew;
	double pi = 3.1415926535;

	x1=new_point.x()-centerPos.x();
	y1=new_point.y()-centerPos.y();

	if(x1!=0)
		m=y1/x1;

	double b,a;

	a = radius;
	b = radius;

	if(new_point.x() > centerPos.x())
		Xnew=(a*b)/(sqrt(pow(b,2)+(pow(a,2)*pow(m,2))))+centerPos.x();
	else
    {
        if(new_point.x() < centerPos.x())
            Xnew=-(a*b)/(sqrt(pow(b,2)+(pow(a,2)*pow(m,2))))+centerPos.x();
        else
            Xnew = centerPos.x();
    }
	
	if(Xnew != centerPos.x())
		Ynew=m*(Xnew-centerPos.x())+centerPos.y(); 
	else
    {
        if(new_point.y() > centerPos.y())
            Ynew=b+centerPos.y();
        else
        {
            if(new_point.y() < centerPos.y())
               Ynew=-b+centerPos.y();
            else
               Ynew = centerPos.y();
        }
    }

	QPointF ppp;
	ppp.setX(Xnew);
	ppp.setY(Ynew);
	//PressedPoint->setPointParam(ppp);

	if(PressedPoint == RightPoint)
		updatePie_RightPressed(QPointF(Xnew,Ynew));
	else
		if(PressedPoint == LeftPoint)
			updatePie_LeftPressed(QPointF(Xnew,Ynew));

}
