#include "mapwidget.h"
#include <QPainter>
#include <QGraphicsSceneMouseEvent>
#include "mapmain.h"

CMapWidget::CMapWidget(QWidget *parent)
	: QGraphicsView(parent)
{
	QRectF rect(0,0,950,1000);
	QGraphicsScene *GridMainScene;
	GridMainScene = new QGraphicsScene(rect,this);
	GridMainScene->setItemIndexMethod(QGraphicsScene::NoIndex);

	setScene(GridMainScene);
	setSceneRect(rect);
	setRenderHint(QPainter::Antialiasing);
	setTransformationAnchor(AnchorUnderMouse);
	setCacheMode(QGraphicsView::CacheBackground);
    setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
	setBackgroundBrush(QBrush(Qt::black));
	setWindowTitle(tr("Map Widget"));
	 
}
CMapWidget::~CMapWidget()
{

}
void CMapWidget::mouseReleaseEvent (QMouseEvent * event)
{
	 //QPainter pPaint;
	 //pPaint.begin(this);
  //   pPaint.drawLine(0.0,0.0,100.0,0.0);
	 //pPaint.drawLine(100.0,0.0,100.0,100.0);
	 //pPaint.drawLine(100.0,100.0,0.0,0.0);
		// // drawing code
  //   pPaint.end();
}
