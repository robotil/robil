#ifndef CLINE_H
#define CLINE_H

#include <QGraphicsLineItem>

class CNode;
class IFigure;

class CLine : public QGraphicsLineItem
{
//
//signals:
//	void SigOnLineMove(CLine* line, QPointF point);

public:
	CLine(IFigure* ifigure);
	CLine(IFigure* ifigure, CNode *node1, CNode *node2);
	~CLine();
	
	CNode* GetNode1() {return pNode1;}
	CNode* GetNode2() {return pNode2;}


	QRectF boundingRect() const;
    QPainterPath shape() const;
	void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

protected:
	void mousePressEvent(QGraphicsSceneMouseEvent *event);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
	void mouseMoveEvent( QGraphicsSceneMouseEvent * event );

private:
	CNode* pNode1;
	CNode* pNode2;
	bool IsMoving;
	bool IsFirstMove;
	QPointF StartPos;

	IFigure* pIFigure;
};

#endif // CLINE_H
