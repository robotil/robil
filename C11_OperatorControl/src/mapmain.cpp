#include "mapmain.h"
#include <QMouseEvent>
#include <QPainter>
#include "routeitem.h"
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsPathItem>
#include <iostream>
#include "math.h"

bool Drawing;
bool Moving;
bool LineMoving;
bool ArcMoving;

CMapMain::CMapMain(QWidget *parent, Qt::WFlags flags)
	: QWidget(parent, flags)
{
	ui.setupUi(this);
	QRectF rect(0,0,950,1000);
	QGraphicsScene *GridMainScene;
	GridMainScene = new QGraphicsScene(rect,this);
	GridMainScene->setItemIndexMethod(QGraphicsScene::NoIndex);
	ui.graphicsView->setScene(GridMainScene);
	
	ui.graphicsView->setSceneRect(rect);

	ui.graphicsView->setRenderHint(QPainter::Antialiasing);
	ui.graphicsView->setCacheMode(QGraphicsView::CacheBackground);
	ui.graphicsView->setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
	ui.graphicsView->setBackgroundBrush(QBrush(Qt::black));

	GridMainScene->installEventFilter(this);

	Drawing = false;
	Moving = false;
	LineMoving = false;
	ArcMoving = false;
	PixPressed.i = 0;
	PixPressed.j = 0;
	for(int i=0; i<100; i++)
	{
		for(int j=0; j<100; j++)
		{
			PixColor[i][j] = 0;
		}
	}
	startX = 175;
	startY = 930;
	p_i=startX;
	p_j=startY;
	pos[0] = p_i;
	pos[1] = p_j;

	AddPix();
	setMode(E_NULL_MODE);

	routePathReady = NULL;;
	routeStepsReady = NULL;;
	routePolygonReady = NULL;;

	routePath = NULL;
	routeSteps = NULL;
	routePolygon = NULL;

	RobotPos.x = 0;
	RobotPos.y = 0;
	GridXOffset = 12;
	GridYOffset = 2;
	CalculateCornerPos();
}

CMapMain::CMapMain(int arr[100][100],QWidget *parent, Qt::WFlags flags)
	: QWidget(parent, flags)
{
	ui.setupUi(this);
		ui.graphicsView->scene()->setItemIndexMethod(QGraphicsScene::NoIndex);
		ui.graphicsView->setRenderHint(QPainter::Antialiasing);
		ui.graphicsView->setCacheMode(QGraphicsView::CacheBackground);
		ui.graphicsView->setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
		ui.graphicsView->setBackgroundBrush(QBrush(Qt::black));
		ui.graphicsView->scene()->installEventFilter(this);

		setMouseTracking(true);

		Drawing = false;
		Moving = false;
		LineMoving = false;
		PixPressed.i = 0;
		PixPressed.j = 0;
		for(int i=0; i<100; i++)
		{
			for(int j=0; j<100; j++)
			{
				PixColor[i][j] = 2;
			}
		}
		startX = 175;
		startY = 930;
		p_i=startX;
		p_j=startY;
		pos[0] = p_i;
		pos[1] = p_j;

		AddPix();
		setMode(E_NULL_MODE);

		routePathReady = NULL;;
		routeStepsReady = NULL;;
		routePolygonReady = NULL;;

		routePath = NULL;
		routeSteps = NULL;
		routePolygon = NULL;
}
CMapMain::~CMapMain()
{
	delete traingle;
	if(routePath != NULL)
		delete routePath;
	if(routePolygon != NULL)
		delete routePolygon;
	if(routeSteps != NULL)
		delete routeSteps;
	if(routePathReady != NULL)
		delete routePathReady;
	if(routePolygonReady != NULL)
		delete routePolygonReady;
	if(routeStepsReady != NULL)
		delete routeStepsReady;
}


ModeDraw CMapMain::getMode()
{
	return mode;
}
void CMapMain::setMode(ModeDraw m)
{
	QVector<QPointF> vecP;
	setMode(m,vecP);
}

void CMapMain::deleteReadyPath()
{
	if(routePathReady!=NULL)
	{
		delete routePathReady;
		routePathReady=NULL;
	}
}
void CMapMain::deletePath()
{
	if(routePath!=NULL)
	{
		delete routePath;
		routePath=NULL;
	}
}
void CMapMain::deleteRoute(ModeDraw m)
{
	switch(m)
	{
	case E_READY_POLYGON_MODE:
		{
			if(routePolygonReady!=NULL)
			{
				delete routePolygonReady;
				routePolygonReady=NULL;
			}
			break;
		}
	case E_READY_PATH_MODE:
		{
			if(routePathReady!=NULL)
			{
				delete routePathReady;
				routePathReady=NULL;
			}
			break;
		}
		case E_READY_STEPS_MODE:
		{
			if(routeStepsReady!=NULL)
			{
				delete routeStepsReady;
				routeStepsReady=NULL;
			}
			break;
		}
		case E_PATH_MODE:
		{
			if(routePath!=NULL)
			{
				delete routePath;
				routePath=NULL;
			}
			break;
		}
	case E_STEPS_MODE:
		{
			if(routeSteps!=NULL)
			{
				delete routeSteps;
				routeSteps=NULL;
			}
			break;
		}
	case E_POLYGON_MODE:
		{
			if(routePolygon!=NULL)
			{
				delete routePolygon;
				routePolygon=NULL;
			}
			break;
		}
	default:
		{
			break;
		}
	}
}

void CMapMain::setMode(ModeDraw m,QVector<QPointF> vecPoints)
{
	mode = m;
	switch(mode)
	{
	case E_NULL_MODE:
		{
			break;
		}
	case E_READY_POLYGON_MODE:
		{
			routePolygonReady = new CRouteItem(ui.graphicsView->scene(),Qt::darkYellow);
			ui.graphicsView->scene()->addItem(routePolygonReady);
			ui.graphicsView->scene()->update();

			setReadyPolygon(vecPoints);
			break;
		}
	case E_READY_PATH_MODE:
		{
		        if(routePathReady != NULL)
		          {
		            deleteReadyPath();
		          }
		        if(routePath != NULL)
		          {
		            deletePath();
		          }
			routePathReady = new CRouteItem(ui.graphicsView->scene());
			ui.graphicsView->scene()->addItem(routePathReady);
			ui.graphicsView->scene()->update();

			setReadyPath(vecPoints);
			break;
		}
	case E_READY_STEPS_MODE:
		{
			routeStepsReady = new CRouteItem(ui.graphicsView->scene());
			ui.graphicsView->scene()->addItem(routeStepsReady);
			ui.graphicsView->scene()->update();

			setReadySteps(vecPoints);

			break;
		}
	case E_PATH_MODE:
		{
                      if(routePathReady != NULL)
                      {
                        deleteReadyPath();
                      }
                    if(routePath != NULL)
                      {
                        deletePath();
                      }
			routePath = new CRouteItem(ui.graphicsView->scene());
			ui.graphicsView->scene()->addItem(routePath);
			break;
		}
	case E_STEPS_MODE:
		{
			routeSteps = new CRouteItem(ui.graphicsView->scene());
			ui.graphicsView->scene()->addItem(routeSteps);
			break;
		}
	case E_POLYGON_MODE:
		{
			routePolygon = new CRouteItem(ui.graphicsView->scene(),Qt::darkYellow);
			ui.graphicsView->scene()->addItem(routePolygon);
			break;
		}
		
	default:
		{
			mode = E_NULL_MODE;
			break;
		}
	}
}

void CMapMain::UpdateGrid(int grid[100][100], StructPoint robotPos, int xOffset, int yOffset, double orient)
{
        RobotPos.x = robotPos.x;
        RobotPos.y = robotPos.y;
        RobotOrientation = orient;
        GridXOffset = xOffset;
        GridYOffset = yOffset;
        CalculateCornerPos();
        StructIntPoint oldPoint;
        StructIntPoint newPoint;
        int i,j;
        WorldToRobotOrientation = (90.0f/RAD2DEG + orient);
//        WorldToRobotOrientation = 90.0f/RAD2DEG;
	for(i=0; i<100; i++)
	{
		for(j=0; j<100; j++)
		{
//			PixColor[i][j] = grid[i][j];
//			pPixItem[i][j]->SetColor(grid[i][j]);
		    PixColor[i][j] = 2;
		    pPixItem[i][j]->SetColor(2);
		}
	}
	for(i=0; i<100; i++)
        {
                for(j=0; j<100; j++)
                {
//                      PixColor[i][j] = grid[i][j];
//                      pPixItem[i][j]->SetColor(grid[i][j]);
                    oldPoint.x = i;
                    oldPoint.y = j;
                    newPoint = CalculateGridPoint(oldPoint);
//                    std::cout<<"newPoint.x="<<newPoint.x<<" newPoint.y="<<newPoint.y<<"\n";
                    if(newPoint.x>=0 && newPoint.x<=99 && newPoint.y>=0 && newPoint.y<=99)
                    {
                        PixColor[newPoint.x][newPoint.y] = grid[i][j];
                        pPixItem[newPoint.x][newPoint.y]->SetColor(grid[i][j]);
                        //std::cout<<"Bingo!\n";
                    }
                }
        }
//	std::cout<<"RobotGridPos.x="<<RobotGridPos.x<<"RobotGridPos.y="<<RobotGridPos.y<<"!\n";
	update();
}

void CMapMain::AddPix()
{
	startX = 50;//175;
	startY = 930;//400;
	p_i=startX;//line
	p_j=startY;//column
	
	for(int i=0;i<100;i++)
	{
		pos[1] = p_j;
		for(int j=0;j<100;j++)
		{
			pos[0] = p_i;
			pPixItem[i][j] = new CPixItem(PixColor[i][j],ui.graphicsView->scene(),pos[0],pos[1]);
			pPixItem[i][j]->setPos(pos[0],pos[1]);
			ui.graphicsView->scene()->addItem(pPixItem[i][j]);
			p_i=p_i+8.0;
		}
		p_i = startX;
		p_j=p_j-8.0;
	}
	traingle = new CtraingleItem(ui.graphicsView->scene(),QPointF(pPixItem[19][49]->pos()),QPointF(pPixItem[19][49]->pos()));
	ui.graphicsView->scene()->addItem(traingle);
}

bool CMapMain::eventFilter(QObject *o, QEvent* e)
{
	switch(e->type())
	{
		case QEvent::GraphicsSceneMouseDoubleClick:
		{
			QGraphicsSceneMouseEvent* event = static_cast<QGraphicsSceneMouseEvent*>(e);
			stopDrawing();
			return true;
			break;
		}
		case QEvent::GraphicsSceneMousePress:
		{
			QGraphicsSceneMouseEvent* event = static_cast<QGraphicsSceneMouseEvent*>(e);
			PressPoint = event->scenePos();

			if((Moving==false)||(LineMoving==false))
			{
				drawing();
			}
			return true;

			break;
		}
		case QEvent::GraphicsSceneMouseRelease:
		{
			QGraphicsSceneMouseEvent* event = static_cast<QGraphicsSceneMouseEvent*>(e);
			releasePoint();
			return true;
			break;
		}
		case QEvent::GraphicsSceneMouseMove:
		{
			QGraphicsSceneMouseEvent* event = static_cast<QGraphicsSceneMouseEvent*>(e);
			if(Moving)
			{
				MovePoint(event->scenePos());
			}
			else
			{
				if(LineMoving)
				{
					MoveLine(event->scenePos());
				}
				else
				{
					MovePoint(event->scenePos());
				}
			}
			return true;
			break;
		}
		default:
			return QWidget::eventFilter(o, e);
	}
return false;
}

void CMapMain::drawLines(CRouteItem *route,bool ShowLines)
{
	route->addPointToLine(PressPoint,ShowLines);
}
void CMapMain::MovePoint(QPointF p)
{
	if((mode == E_NULL_MODE)&&(Moving == true))
	{
		routeSelected->MovePointTo(p);
	}
	else
	{
		if((LineMoving==false)&&(Moving == false)&&(ArcMoving == true))
		{
//			traingle->MoveArc(p);
//			update();
//			ui.graphicsView->update();
		}		
	}
}
void CMapMain::MoveLine(QPointF p)
{
	if((mode == E_NULL_MODE)&&(LineMoving == true))
	{	
		routeSelected->MoveLineTo(p);
	}
}
void CMapMain::drawing()
{
	switch(mode)
	{
	case E_NULL_MODE:
		{
			selectPoint();
			break;
		}
	case E_PATH_MODE:
		{
			routeSelected = routePath;
			drawLines(routePath,true);
			break;
		}
	case E_STEPS_MODE:
		{
			routeSelected = routeSteps;
			drawLines(routeSteps,false);
			break;
		}
	case E_POLYGON_MODE:
		{
			routeSelected = routePolygon;
			drawLines(routePolygon,true);
			break;
		}
	default:
		break;
	}
}
void CMapMain::stopDrawing()
{
	switch(mode)
	{
	case E_NULL_MODE:
		{
			break;
		}
	case E_PATH_MODE:
		{
			routePath->endPath(PressPoint);
			break;
		}
	case E_STEPS_MODE:
		{
			routeSteps->endPath(PressPoint);
			
			break;
		}
	case E_POLYGON_MODE:
		{
			routePolygon->ConnectLastPoint(PressPoint);
			break;
		}
	default:
		break;
	}
	setMode(E_NULL_MODE);
}
void CMapMain::setReadySteps(QVector<QPointF> vecPoints)
{
        //////////////Ready Steps/////////////////////////
        routeStepsReady->drawReadyPath(vecPoints,false);
        setMode(E_NULL_MODE);
}

void CMapMain::setReadyPath(QVector<QPointF> vecPoints)
{
        //////////////Ready Path/////////////////////////
        routePathReady->drawReadyPath(vecPoints,true);
        setMode(E_NULL_MODE);
}
void CMapMain::setReadyPolygon(QVector<QPointF> vecPoints)
{
        //////////////Ready Polygon/////////////////////////
        routePolygonReady->drawReadyPolygon(vecPoints,true);

        setMode(E_NULL_MODE);
}
void CMapMain::selectPoint()
{
	bool b = false;
	if(mode == E_NULL_MODE)
	{//checking Points
		b = checkSelectedPoint(routePath);
		if(!b)
		{
			b = checkSelectedPoint(routePathReady);
			if(!b)
			{
				b = checkSelectedPoint(routePolygon);
				if(!b)
				{
					b = checkSelectedPoint(routePolygonReady);
					if(!b)
					{
						b = checkSelectedPoint(routeSteps);
						if(!b)
						{
							b = checkSelectedPoint(routeStepsReady);
							if(!b)//checking edges
							{
								b = checkSelectedEdge(routePath);
								if(!b)
								{
									b = checkSelectedEdge(routePathReady);
									if(!b)
									{
										b = checkSelectedEdge(routePolygon);
										if(!b)
										{
											b = checkSelectedEdge(routePolygonReady);
											if(!b)
											{
												b = checkSelectedArc();
												int y=0;
											}
										}
									}
								}
							}
						}
					}
				}
			}
		}			
	}
}
bool CMapMain::checkSelectedArc()
{
	bool b;
	b = traingle->selectArc(PressPoint); 
	if(b)
		ArcMoving = true;

	return b;
}
bool CMapMain::checkSelectedEdge(CRouteItem *Route)
{
	if(Route != NULL)
	{
		if(Route->selectEdge(PressPoint))
		{
			routeSelected = Route;
			LineMoving = true;
			return true;
		}
		else
			return false;
	}
	else
		return false;
}
bool CMapMain::checkSelectedPoint(CRouteItem *Route)
{
	if(Route != NULL)
	{
		if(Route->selectPoint(PressPoint))
		{
			routeSelected = Route;
			Moving = true;
			return true;
		}
		else
			return false;
	}
	else
		return false;
}
void CMapMain::releasePoint()
{
	if((Moving)||(LineMoving)||(ArcMoving))
	{
		if(!ArcMoving)
		{
			switch(mode)
			{
			case E_NULL_MODE:
				{
					if(routeSelected != NULL)
						routeSelected->ReleasePoint();
					break;
				}
			}
		}
		Moving = false;
		LineMoving = false;
		ArcMoving = false;
	}
}

void CMapMain::AddPath(std::vector<StructPoint> points)
{
	setMode(E_READY_PATH_MODE);
	std::cout<<"Path size: "<<points.size()<<"\n";
	for(int i=0; i<points.size(); i++)
	{
//		QPointF point = PointToPix(points[i]);
		if(i<points.size()-1)
		{
			routePathReady->addPointToLine(PointToPix(points[i]),true);
		}
		else
		{
			routePathReady->addPointToLine(PointToPix(points[i]),true);
			routePathReady->endPath(QPointF(0,0));
		}
	}
	setMode(E_NULL_MODE);
}

QPointF CMapMain::PointToPix(StructPoint point)
{
	QPointF GPoint;
//	GPoint.setX(50+((point.x - CornerPos.y)*32) - CornerPos.y*32 + 12.5*32);
	GPoint.setX(50+((point.y - RobotPos.y)*(-32)) + 400);
	GPoint.setY(930-((point.x - RobotPos.x)*32) - 160);
	std::cout<<"PointX: "<<GPoint.x()<<" PointY: "<<GPoint.y()<<"\n";
	return GPoint;
}

void CMapMain::CalculateCornerPos()
{
	CornerPos.x = GridXOffset;
	CornerPos.y = GridYOffset;
	RobotGridPos.x = (RobotPos.x-GridXOffset)*4;
	RobotGridPos.y = (RobotPos.y-GridYOffset)*4;
	std::cout<<"CornerPos.x: "<<CornerPos.x<<" CornerPos.y: "<<CornerPos.y<<"\n";
	std::cout<<"RobotGridPos.x: "<<RobotGridPos.x<<" RobotGridPos.y: "<<RobotGridPos.y<<"\n";
}

StructIntPoint CMapMain::CalculateGridPoint(StructIntPoint pointFromRos)
{
  StructIntPoint gridPoint;
  gridPoint.x = (pointFromRos.x-RobotGridPos.x)*sin(WorldToRobotOrientation) - (pointFromRos.y-RobotGridPos.y)*cos(WorldToRobotOrientation) + 20;
  gridPoint.y = (pointFromRos.y-RobotGridPos.y)*sin(WorldToRobotOrientation) + (pointFromRos.x-RobotGridPos.x)*cos(WorldToRobotOrientation) + 50;
//  std::cout<<"PointX="<<pointFromRos.x<<" PointY="<<pointFromRos.y<<" gridPoint.x = "<<gridPoint.x<<" gridPoint.y = "<<gridPoint.y<<" Orientation = "<<WorldToRobotOrientation<<"\n";
  return gridPoint;
}
