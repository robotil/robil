#include <QGraphicsScene>
#include "cnode.h"
#include "cline.h"
#include "figure.h"

CFigure::CFigure(QGraphicsScene* scene)
{
	pScene = scene;
}

CFigure::~CFigure()
{

}

void CFigure::AddNode(CNode* node)
{
	if(!Nodes.empty())
	{
		CLine* newLine = new CLine(this,Nodes.last(),node);
		if(Nodes.size()>1)
		{
			if(Lines.size() > 1)
			{
				pScene->removeItem(Lines.last());
				Lines.remove(Lines.size()-1);
			}
			CLine* lastLine = new CLine(this,node,Nodes.first());
			Lines.push_back(newLine);
			pScene->addItem(newLine);
			Lines.push_back(lastLine);
			pScene->addItem(lastLine);
		}
		else
		{
			Lines.push_back(newLine);
			pScene->addItem(newLine);
		}
	}
	Nodes.push_back(node);
}

void CFigure::InsertNode(CNode* node,int pos)
{
	Nodes.insert(pos,node);
}

void CFigure::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
}

QRectF CFigure::boundingRect()  const
{
	qreal adjust = 2;
    return QRectF( -10 - adjust, -10 - adjust,23 + adjust, 23 + adjust);
}
    
QPainterPath CFigure::shape()  const 
{
	QPainterPath path;
	if(!Nodes.empty())
	{
		path.moveTo(Nodes[0]->GetPos());
		if(Nodes.size() > 1)
		{
			for(int i=1; i<Nodes.size(); i++)
			{
				path.lineTo(Nodes[i]->GetPos());
			}
			path.lineTo(Nodes[0]->GetPos());
		}

	}
	return path;
}

void CFigure::OnLineMove(CLine* line, QPointF point)
{
	CNode* newNode = new CNode(point.x(),point.y(),pScene);
	pScene->addItem(newNode);
	CNode* node1 = line->GetNode1();
	CNode* node2 = line->GetNode2();

	Nodes.insert(Nodes.indexOf(node2),newNode);
	CLine* newLine1 = new CLine(this,node1,newNode);
	pScene->addItem(newLine1);
	CLine* newLine2 = new CLine(this,newNode,node2);
	pScene->addItem(newLine2);

	pScene->removeItem(line);
	Lines.insert(Lines.indexOf(line),newLine1);
	Lines.insert(Lines.indexOf(line)+1,newLine2);
	Lines.remove(Lines.indexOf(line));
}