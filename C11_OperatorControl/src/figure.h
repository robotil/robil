#ifndef FIGURE_H
#define FIGURE_H

class CNode;
class CLine;
#include <QVector>
#include <QGraphicsItem>
#include "IFigure.h"

class CFigure : public QGraphicsItem, public IFigure
{

public:
	CFigure(QGraphicsScene* scene);
	~CFigure();

	void AddNode(CNode* node);
	void InsertNode(CNode* node,int pos);

	QRectF boundingRect() const;
    QPainterPath shape() const;
	void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

	virtual void OnLineMove(CLine* line, QPointF point);

private:
	QVector<CNode*> Nodes;
	QVector<CLine*> Lines;
	QGraphicsScene* pScene;
};

#endif // FIGURE_H
