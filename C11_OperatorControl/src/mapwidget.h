#ifndef MAPWIDGET_H
#define MAPWIDGET_H

#include <QGraphicsView>
#include <QMouseEvent>

class CMapWidget : public QGraphicsView
{
	Q_OBJECT

public:
	CMapWidget(QWidget *parent);
	~CMapWidget();

private:

protected:
	void mouseReleaseEvent ( QMouseEvent * event );
};

#endif // MAPWIDGET_H
