#include "mapmain.h"
#include <QMouseEvent>
#include <QPainter>

bool Drawing;

CMapMain::CMapMain(QWidget *parent, Qt::WFlags flags)
	: QWidget(parent, flags)
{
	ui.setupUi(this);

	setMouseTracking(true);

	Drawing = false;
	PixPressed.i = 0;
	PixPressed.j = 0;
	for(int i=0; i<48; i++)
	{
		for(int j=0; j<48; j++)
		{
			PixColor[i][j] = 0;
		}
	}
	startX = 175;
	startY = 400;
	pos[2];
	p_i=startX;
	p_j=startY;
	pos[0] = p_i;
	pos[1] = p_j;

	/*QRectF rect(0,0,950,1000);
	GridMainScene = new QGraphicsScene(rect,this);
	ui.graphicsView->setScene(GridMainScene);
	ui.graphicsView->setSceneRect(rect);
	ui.graphicsView->setRenderHint(QPainter::Antialiasing);
	ui.graphicsView->setCacheMode(QGraphicsView::CacheBackground);
    ui.graphicsView->setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
	ui.graphicsView->setBackgroundBrush(QBrush(Qt::black));*/

	AddPix();

	//pGridItem = new CGridItem(ui.graphicsView->scene());
	//ui.graphicsView->scene()->addItem(pGridItem);
}

CMapMain::CMapMain(int arr[48][48],QWidget *parent, Qt::WFlags flags)
	: QWidget(parent, flags)
{
	ui.setupUi(this);

	setMouseTracking(true);
	
	Drawing = false;
	PixPressed.i = 0;
	PixPressed.j = 0;
	for(int i=0; i<48; i++)
	{
		for(int j=0; j<48; j++)
		{
			PixColor[i][j] = arr[i][j];
		}
	}
	startX = 175;
	startY = 400;
	pos[2];
	p_i=startX;
	p_j=startY;
	pos[0] = p_i;
	pos[1] = p_j;

	/*QRectF rect(0,0,950,1000);
	GridMainScene = new QGraphicsScene(rect,this);	
	ui.graphicsView->setScene(GridMainScene);
	ui.graphicsView->setSceneRect(rect);
	ui.graphicsView->setRenderHint(QPainter::Antialiasing);
	ui.graphicsView->setCacheMode(QGraphicsView::CacheBackground);
    ui.graphicsView->setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
	ui.graphicsView->setBackgroundBrush(QBrush(Qt::black));*/
	
	AddPix();
	
	//pGridItem = new CGridItem(ui.graphicsView->scene());
	//ui.graphicsView->scene()->addItem(pGridItem);
}
CMapMain::~CMapMain()
{

}
void CMapMain::AddPix()
{
	startX = 175;
	startY = 400;
	pos[2];
	p_i=startX;
	p_j=startY;
	
	for(int i=0;i<48;i++)
	{
		pos[0] = p_i;
		for(int j=0;j<48;j++)
		{
			pos[1] = p_j;
			pPixItem[i][j] = new CPixItem(PixColor[i][j],ui.graphicsView->scene(),pos[0],pos[1]);
			ui.graphicsView->scene()->addItem(pPixItem[i][j]);
			p_j=p_j+12.0;
		}
		p_j = startY;
		p_i=p_i+12.0;
	}
}

//void CMapMain::mousePressEvent(QMouseEvent *event )
//{
//	QPointF p;
//	p=event->posF();
//	/*p.setX(event->posF().x());
//	p.setY(event->posF().y());
//
//	if ((event->buttons() & Qt::LeftButton))
//	{
//		for(int i=0;i<48;i++)
//		{
//			for(int j=0;j<48;j++)
//			{
//				if(pPixItem[i][j]->isContain(event->posF()))
//				{
//					pPixItem[i][j]->mousePress(event->globalPos());
//					Drawing=true;
//					PixPressed.i = i;
//					PixPressed.j = j;
//					i=48;
//					j=48;
//				}
//			}
//		}
//	}*/
//	QWidget::mousePressEvent(event);
//}
//void CMapMain::mouseReleaseEvent(QMouseEvent *event)
//{
//	QPainter pPaint(this);
//	static const QPointF points[3] = 
//	{
//		 QPointF(175.0, 400.0),
//		 QPointF(187.0, 412.0),
//		 QPointF(199.0, 424.0),
//    };
//
//	pPaint.drawPolyline(points, 3);
//
//	pPaint.setPen(QPen(Qt::red));
//	QRect rec(175.0,400.0,100,100);
//	
//	pPaint.drawRect(rec);
//	Drawing=false;
//}
//
//void CMapMain::mouseMoveEvent(QMouseEvent *event )
//{
//	if ((event->buttons() & Qt::LeftButton)&&(Drawing))
//         pPixItem[PixPressed.i][PixPressed.j]->drawLineTo(event->pos());
//
//}
void CMapMain::drawLine()
{
	/*QPainter pPaint;
	static const QPointF points[3] = 
	{
		 QPointF(175.0, 400.0),
		 QPointF(187.0, 412.0),
		 QPointF(199.0, 424.0),
    };

	pPaint.drawPolyline(points, 3);

	pPaint.setPen(QPen(Qt::red));
	QRect rec(175.0,400.0,100,100);
	
	CPixItem::drawLineTo(QPointF(199.0, 424.0));

	pPaint.drawRect(rec);
	Drawing=false;*/
}
