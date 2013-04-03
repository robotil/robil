#include "routeitem.h"
#include <QPainter>
#include <QGraphicsScene>

//QPointF points[40];
//int numOfPointsInPoly;
	
CRouteItem::CRouteItem(QGraphicsScene* scene, QColor c)
{
	pFirstLineItemList = NULL;
	pScene = scene;
	numOfPoints = 0;
	numOfPointsInPoly = 0;
	pPressedPointInLineList = NULL;
	isPoly = false;
	penColor = c;
}
CRouteItem::~CRouteItem()
{
	if(isPoly)
	{
		for(int i=0;i<numOfPointsInPoly; i++)
			points[i] = QPoint(0.0,0.0);
	}
	
	delete pPressedPointInLineList;
	
	CLineItemList *prevLineItem = NULL;
	if(pFirstLineItemList != NULL)
	{
		CLineItemList *curLineItem = pFirstLineItemList;
		for(int i=0; i<numOfPoints; i++)
		{
			prevLineItem = curLineItem;

			if(curLineItem->getNextLineItem()!=NULL)
			{
				curLineItem = curLineItem->getNextLineItem();
				delete prevLineItem;
			}
		}
		if(curLineItem->getNextLineItem()!=NULL)
		{
			curLineItem=NULL;
		}
		delete curLineItem;	
	}
}

QVector<QPointF> CRouteItem::getRoutePoints()
{
	QPointF p;
	QVector<QPointF> pointVec;	
	CLineItemList *prevLineItem = NULL;

	if(pFirstLineItemList != NULL)
	{
		CLineItemList *curLineItem = pFirstLineItemList;
		for(int i=0; i<numOfPoints; i++)
		{
			prevLineItem = curLineItem;

			if(curLineItem->getNextLineItem()!=NULL)
			{
				curLineItem = curLineItem->getNextLineItem();
				p = prevLineItem->getLineItem()->getPointItem1()->getPoint();
				pointVec.insert(i,p);
			}
			else
			{
				p = curLineItem->getLineItem()->getPointItem1()->getPoint();
				pointVec.insert(i,p);
			}
		}	
	}
	return pointVec;
}

QRectF CRouteItem::boundingRect()  const
{
    return QRectF(0, 0, 950, 1000);
}
QPainterPath CRouteItem::shape()  const 
{
	QPainterPath path;
	path.moveTo(points[0]);
	path.lineTo(points[1]);
	path.lineTo(points[2]);
	path.lineTo(points[3]);
	return path;
}
void CRouteItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
	painter->setPen(QPen(/*Qt::darkYellow*/penColor, 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));	
	if(isPoly)
	{
		painter->setBrush(QBrush(QColor(255,255,0,100)));	
	}
	painter->drawPolygon(points,numOfPointsInPoly);
}

void CRouteItem::addPointToLine(QPointF p,bool ShowLines)
{
	CLineItemList *currentItemList;
	currentItemList = pFirstLineItemList;
			
	if(pFirstLineItemList==NULL)
	{
		pFirstLineItemList = new CLineItemList(p,ShowLines,pScene,penColor);
		pScene->addItem(pFirstLineItemList);
		pScene->update();
		update();
	}
	else
	{	
		if(numOfPoints%2)
		{
			for(int i=0; i<(numOfPoints-1); i++)
			{
				if(currentItemList->getNextLineItem()!=NULL)
				{
					currentItemList = currentItemList->getNextLineItem();
				}
				else
					return;
			}
			currentItemList->addPointItem1(p);
		}
		else
		{
			for(int i=0; i<(numOfPoints-2); i++)
			{
				if(currentItemList->getNextLineItem()!=NULL)
					currentItemList = currentItemList->getNextLineItem();
				else
					return;
			}
			currentItemList->getNextLineItem()->addPointItem1(p);
		}
	}
	numOfPoints++;
}
bool CRouteItem::isPointOnEdge(CLineItemList *ItmList,QPointF p)
{
	return ItmList->isPointOnEdge(p);
}
bool CRouteItem::isContain(CLineItemList *ItmList,QPointF p)
{
	return ItmList->isContain(p);	
}
void CRouteItem::drawReadyPolygon(QVector<QPointF> vec_p, bool b)
{
	CLineItemList *ItemList;
	QPointF pp;
	QVector<QPointF>::iterator iter;
    for(iter = vec_p.begin(); iter!= vec_p.end(); iter++)
    {
        pp=*iter;
		addPointToLine(pp,b);
		if(iter == vec_p.begin())
		{
			ItemList = pFirstLineItemList;
			points[vec_p.indexOf(*iter)] = ItemList->getLineItem()->getPoint1();
		}
		else
		{
			ItemList = ItemList->getNextLineItem();
			points[vec_p.indexOf(*iter)] = ItemList->getLineItem()->getPoint1();
		}
	}

	ItemList->LastPoint(pFirstLineItemList->getLineItem()->getPoint1());//update: ItemList->point2 = ItemList->point1
	ItemList->setNextLineItem(pFirstLineItemList);
	ItemList->getLineItem()->setEndLine(true);
	numOfPointsInPoly = numOfPoints;
	isPoly = true;	
}
void CRouteItem::drawReadyPath(QVector<QPointF> vec_p, bool b)
{
  if(vec_p.empty())
    {
      return;
    }
	CLineItemList *ItemList;
	QPointF pp;

	QVector<QPointF>::iterator iter;

    for(iter = vec_p.begin(); iter!= vec_p.end(); iter++)
    {
        pp=*iter;
		addPointToLine(pp,b);
		if(iter == vec_p.begin())
		{
			ItemList = pFirstLineItemList;
		}
		else
			ItemList = ItemList->getNextLineItem();
    }
    ItemList->LastPoint(ItemList->getLineItem()->getPoint1());
}
void CRouteItem::ReleasePoint()
{
	pPressedPointInLineList->getLineItem()->setRadius(10);
	pPressedPointInLineList = NULL;
}
bool CRouteItem::selectEdge(QPointF p)
{
	bool b = false;
	if(pFirstLineItemList != NULL)
	{
		CLineItemList *ItemList;
		CLineItemList *AddedLineItem;
		ItemList = pFirstLineItemList;

		for(int i=0; i<numOfPoints; i++)
		{
			b = isPointOnEdge(ItemList,p);
			if(b)
			{
				pPressedPointInLineList = ItemList;

				pPressedPointInLineList->getLineItem()->setPointItem2(p);
				
				if(pPressedPointInLineList->getNextLineItem() !=NULL)
				{
					AddedLineItem = new CLineItemList(p,true,pScene,penColor);
					pScene->addItem(AddedLineItem);
					pScene->update();
					
					AddedLineItem->getLineItem()->setPointItem2(pPressedPointInLineList->getNextLineItem()->getLineItem()->getPoint1());
					AddedLineItem->getLineItem()->setPointItem1(pPressedPointInLineList->getLineItem()->getPoint2());
					AddedLineItem->setNextLineItem(pPressedPointInLineList->getNextLineItem());
					pPressedPointInLineList->setNextLineItem(AddedLineItem);
					
					pPressedPointInLineList = AddedLineItem;
					numOfPoints++;
					if(isPoly)
					{
						if(pPressedPointInLineList->getLineItem()->getEndLine())
						{
							AddedLineItem->getLineItem()->setEndLine(true);
							pPressedPointInLineList->getLineItem()->setEndLine(false);						
						}
						updatePoints();
					}
					update();
					
					return b;
				}
				else
					return b;
			}
			else
			{
				if(ItemList->getNextLineItem()!=NULL)
					ItemList = ItemList->getNextLineItem();
			}
		}
	}
	return b;
}
void CRouteItem::UpdatetLastPoint()
{
	CLineItemList *tempLineItem = GoToLastLine();
	tempLineItem->getLineItem()->setPointItem2(pFirstLineItemList->getLineItem()->getPoint1());
}
CLineItemList* CRouteItem::GoToLastLine()
{
	CLineItemList *curLineItem = pFirstLineItemList;

	while(curLineItem->getNextLineItem()!=NULL)
	{
		curLineItem = curLineItem->getNextLineItem();
	}
	return curLineItem;
}
void CRouteItem::updatePoints()
{
//	int i=0;
	CLineItemList *curLineItem = pFirstLineItemList;

	//while(curLineItem->getNextLineItem()!=NULL)
	for(int i=0; i<numOfPoints; i++)
	{
		points[i] = curLineItem->getLineItem()->getPoint1();
		curLineItem = curLineItem->getNextLineItem();
	}
	//points[i] = curLineItem->getLineItem()->getPoint1();
	numOfPointsInPoly = numOfPoints;
}
bool CRouteItem::selectPoint(QPointF p)
{
	bool b = false;
	if(pFirstLineItemList != NULL)
	{
		CLineItemList *ItemList;
		ItemList = pFirstLineItemList;

		for(int i=0; i<numOfPoints; i++)
		{
			b = isContain(ItemList,p);
			if(b)
			{
				ItemList->getLineItem()->setRadius(20);
				pPressedPointInLineList = ItemList;
				return b;
			}
			else
			{
				if(ItemList->getNextLineItem()!=NULL)
					ItemList = ItemList->getNextLineItem();
			}
		}
	}
	return b;
}
void CRouteItem::endPath(QPointF p)
{
	CLineItemList *ItemList;
	ItemList = pFirstLineItemList;
	bool b = false;

	for(int i=0; i<=(numOfPoints-1); i++)
	{
		b = isContain(ItemList,p);
		if(b)
		{
			ItemList->removeLastPoint();
			ItemList->LastPoint(ItemList->getLineItem()->getPoint1());
			numOfPoints--;
		}
		else
		{
			if(ItemList->getNextLineItem()!=NULL)
				ItemList = ItemList->getNextLineItem();
		}
	}
}
void CRouteItem::ConnectLastPoint(QPointF p)
{
	CLineItemList *ItemList;
	ItemList = pFirstLineItemList;
	bool b = false;

	for(int i=0; i<=(numOfPoints-1); i++)
	{
		b = isContain(ItemList,p);
		if(b)
		{
			ItemList->removeLastPoint();
			ItemList->LastPoint(pFirstLineItemList->getLineItem()->getPoint1());//update: ItemList->point2 = ItemList->point1
			ItemList->setNextLineItem(pFirstLineItemList);
			ItemList->getLineItem()->setEndLine(true);
			points[i] = ItemList->getLineItem()->getPoint1();
			numOfPoints--;
			numOfPointsInPoly = numOfPoints;
			isPoly = true;
			
		}
		else
		{
			points[i] = ItemList->getLineItem()->getPoint1();
			if(ItemList->getNextLineItem()!=NULL)
			{
				ItemList = ItemList->getNextLineItem();
			}
		}
	}
}
void CRouteItem::MoveLineTo(QPointF p)
{
	MovePointTo(p);
}
void CRouteItem::MovePointTo(QPointF p)
{
	CLineItemList *prevLineItem = NULL;
	CLineItemList *curLineItem = pFirstLineItemList;

	while(curLineItem->getLineItem()->getPoint2()!= pPressedPointInLineList->getLineItem()->getPoint1())
	{
		if(!isLastPointInPath(curLineItem)/*curLineItem->getNextLineItem()!=NULL*/)
		{
			prevLineItem = curLineItem;
			curLineItem = curLineItem->getNextLineItem();
		}
		else
		{
			updatePolygonPoints(pPressedPointInLineList->getLineItem()->getPoint1(),p);
			pPressedPointInLineList->getLineItem()->MovePoint1To(p);
			return;
		}
	}

	curLineItem->getLineItem()->MovePoint2To(p);
	updatePolygonPoints(pPressedPointInLineList->getLineItem()->getPoint1(),p);
	pPressedPointInLineList->getLineItem()->MovePoint1To(p);

	if(isLastPointInPath(pPressedPointInLineList))//&&(pPressedPointInLineList->getLineItem()->getPoint2()!=pFirstLineItemList->getLineItem()->getPoint1()))
	{
		pPressedPointInLineList->getLineItem()->MovePoint2To(p);
	}
}
bool CRouteItem::isLastPointInPath(CLineItemList *LnLst)
{
	if((LnLst->getNextLineItem()==NULL)&&(LnLst->getLineItem()->getPoint2()!=pFirstLineItemList->getLineItem()->getPoint1()))
		return true;
	else
		return false;

}
void CRouteItem::updatePolygonPoints(QPointF p,QPointF pNew)
{
	for(int i=0; i<numOfPointsInPoly; i++)
	{
		if(points[i] == p)
			points[i] = pNew;
	}
}

QPointF CRouteItem::GetPoint(int index)
{
	if(index < numOfPoints)
	{
		return points[index];
	}
	else
	{
		QPointF p(0,0);
		return p;
	}
}

int CRouteItem::GetNumOfPoints()
{
	return numOfPoints;
}
