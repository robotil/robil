#include "mapwidget.h"
#include <QtGui>

CMapWidget::CMapWidget(QWidget *parent)
	:QGraphicsView(parent)
{
	setTransformationAnchor(AnchorUnderMouse);
}
CMapWidget::~CMapWidget()
{

}
