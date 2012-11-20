#ifndef CNODE_H
#define CNODE_H

#include <QPoint>
#include <QGraphicsItem>

class CNode : public QGraphicsItem
{

public:
	CNode(QGraphicsScene* scene);
	CNode(int x, int y, QGraphicsScene* scene);
	~CNode();

	QRectF boundingRect() const;
    QPainterPath shape() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

	QPointF GetPos();

protected:

     void mousePressEvent(QGraphicsSceneMouseEvent *event);
     void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
	 void mouseMoveEvent( QGraphicsSceneMouseEvent * event );

private:
	bool IsMoving;
	QPointF StartPos;

	QGraphicsScene* pScene;
};

#endif // CNODE_H
