#ifndef MAPMAIN_H
#define MAPMAIN_H

#include <QtGui/QWidget>
#include "ui_mapmain.h"
#include "pixitem.h"
//#include <QMainWindow>
#include <QMouseEvent>

typedef struct
{
	int i;
	int j;
}sItemPressed;

typedef enum
{
	E_NULL_MODE,
	E_READY_PATH_MODE,
	E_READY_STEPS_MODE,
	E_READY_POLYGON_MODE,
	E_PATH_MODE,
	E_STEPS_MODE,
	E_POLYGON_MODE
}ModeDraw;

class CRouteItem;

class CMapMain : public QWidget
{
	Q_OBJECT

public:
	CMapMain(QWidget *parent = 0, Qt::WFlags flags = 0);
	CMapMain(int arr[48][48],QWidget *parent = 0, Qt::WFlags flags = 0);
	~CMapMain();
	void UpdateGrid(int grid[48][48]);
	void setReadyPath();
	void setReadyPolygon();
	void setMode(ModeDraw m);
	ModeDraw getMode();

private:
	Ui::CMapMainClass ui;
	CPixItem *pPixItem[48][48];
	ModeDraw mode;
	int PixColor[48][48];int pos[2];
	int startX,startY;
	float p_i;
	float p_j;
	sItemPressed PixPressed;
	QPointF PressPoint;
	QPointF ReleasePoint;

	CRouteItem *routeSelected;

	CRouteItem *routePathReady;
	CRouteItem *routeStepsReady;
	CRouteItem *routePolygonReady;

	CRouteItem *routePath;
	CRouteItem *routeSteps;
	CRouteItem *routePolygon;

	void drawLines(CRouteItem *route,bool ShowLines);
	void AddPix();
	void drawing();
	void stopDrawing();
	void selectPoint();
	void releasePoint();
	void MovePoint(QPointF p);
	void MoveLine(QPointF p);
	bool checkSelectedPoint(CRouteItem *Route);
	bool checkSelectedEdge(CRouteItem *Route);

protected:
	bool eventFilter(QObject *o, QEvent* e);
	
	
};

#endif // MAPMAIN_H
