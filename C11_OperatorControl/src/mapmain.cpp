#include "mapmain.h"
#include <QMouseEvent>
#include <QPainter>
#include "routeitem.h"
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsPathItem>

bool Drawing;
bool Moving;
bool LineMoving;

CMapMain::CMapMain(QWidget *parent, Qt::WFlags flags)
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

	AddPix();
	setMode(E_NULL_MODE);

	routePathReady = NULL;;
	routeStepsReady = NULL;;
	routePolygonReady = NULL;;

	routePath = NULL;
	routeSteps = NULL;
	routePolygon = NULL;
}

CMapMain::CMapMain(int arr[48][48],QWidget *parent, Qt::WFlags flags)
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

}

ModeDraw CMapMain::getMode()
{
	return mode;
}

void CMapMain::setMode(ModeDraw m)
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
			routePolygonReady = new CRouteItem(ui.graphicsView->scene());
			ui.graphicsView->scene()->addItem(routePolygonReady);
			ui.graphicsView->scene()->update();
			setReadyPolygon();
			break;
		}
	case E_READY_PATH_MODE:
		{
			routePathReady = new CRouteItem(ui.graphicsView->scene());
			ui.graphicsView->scene()->addItem(routePathReady);
			ui.graphicsView->scene()->update();

			setReadyPath();
			break;
		}
	case E_READY_STEPS_MODE:
		{
			routeStepsReady = new CRouteItem(ui.graphicsView->scene());
			ui.graphicsView->scene()->addItem(routeStepsReady);
			ui.graphicsView->scene()->update();

			break;
		}
	case E_PATH_MODE:
		{
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
			routePolygon = new CRouteItem(ui.graphicsView->scene());
			ui.graphicsView->scene()->addItem(routePolygon);
			break;
		}
	}
}

void CMapMain::UpdateGrid(int grid[48][48])
{
	for(int i=0; i<48; i++)
	{
		for(int j=0; j<48; j++)
		{
			PixColor[i][j] = grid[i][j];
			pPixItem[i][j]->SetColor(grid[i][j]);
		}
	}
	update();
}

void CMapMain::AddPix()
{
	startX = 175;
	startY = 400;
	pos[2];//line=[0],column= [1]
	p_i=startX;//line
	p_j=startY;//column
	
	for(int i=0;i<48;i++)
	{
		pos[0] = p_i;
		for(int j=0;j<48;j++)
		{
			pos[1] = p_j;
			pPixItem[i][j] = new CPixItem(PixColor[i][j],ui.graphicsView->scene(),pos[0],pos[1]);
			pPixItem[i][j]->setPos(pos[0],pos[1]);
			ui.graphicsView->scene()->addItem(pPixItem[i][j]);
			p_j=p_j+12.0;
		}
		p_j = startY;
		p_i=p_i+12.0;
	}
}

bool CMapMain::eventFilter(QObject *o, QEvent* e)
{
	switch(e->type())
	{
		case QEvent::GraphicsSceneMouseDoubleClick:
		{
			QGraphicsSceneMouseEvent* event = static_cast<QGraphicsSceneMouseEvent*>(e);
			stopDrawing();
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

			break;
		}
		case QEvent::GraphicsSceneMouseRelease:
		{
			QGraphicsSceneMouseEvent* event = static_cast<QGraphicsSceneMouseEvent*>(e);
			releasePoint();
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
			}
			break;
		}
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
			setMode(E_POLYGON_MODE);
			break;
		}
	case E_STEPS_MODE:
		{
			routeSteps->endPath(PressPoint);
			setMode(E_NULL_MODE);
			break;
		}
	case E_POLYGON_MODE:
		{
			routePolygon->ConnectLastPoint(PressPoint);
			setMode(E_STEPS_MODE);
			break;
		}
	}
	//setMode(E_NULL_MODE);
}
void CMapMain::setReadyPath()
{
	//////////////Ready Path/////////////////////////
	//setMode(E_READY_PATH_MODE);
	routePathReady->addPointToLine(QPointF(175,400),true);
	routePathReady->addPointToLine(QPointF(200,450),true);
	routePathReady->addPointToLine(QPointF(300,550),true);
	routePathReady->addPointToLine(QPointF(175,600),true);
	routePathReady->addPointToLine(QPointF(300,700),true);
	routePathReady->endPath(QPointF(300,700));
	setMode(E_NULL_MODE);
	/////////////////////////////////////////////////
}
void CMapMain::setReadyPolygon()
{
	//////////////Ready Path/////////////////////////
	//setMode(E_READY_POLYGON_MODE);
	routePolygonReady->addPointToLine(QPointF(280,400),true);
	routePolygonReady->addPointToLine(QPointF(200,450),true);
	routePolygonReady->addPointToLine(QPointF(300,550),true);
	routePolygonReady->addPointToLine(QPointF(175,600),true);
	routePolygonReady->addPointToLine(QPointF(300,700),true);
	routePolygonReady->ConnectLastPoint(QPointF(300,700));
	setMode(E_NULL_MODE);
	/////////////////////////////////////////////////
}
void CMapMain::selectPoint()
{
	bool b = false;
	if(mode == E_NULL_MODE)
	{
		b = checkSelectedPoint(routePath);
		if(!b)
		{
			b = checkSelectedPoint(routePolygon);
			if(!b)
			{
				b = checkSelectedPoint(routeSteps);
				if(!b)
				{
					b = checkSelectedEdge(routePath);
					if(!b)
					{
						checkSelectedEdge(routePolygon);
					}
				}
			}
		}
	}
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
	if((Moving)||(LineMoving))
	{
		Moving = false;
		LineMoving = false;
	
		switch(mode)
		{
		case E_NULL_MODE:
			{
				routeSelected->ReleasePoint();
				break;
			}
		}
	}
}
