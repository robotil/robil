#ifndef MAPMAIN_H
#define MAPMAIN_H

#include <QtGui/QWidget>
#include "ui_mapmain.h"
#include "griditem.h"
#include "pixitem.h"

//#include <QMouseEvent>

typedef struct
{
	int i;
	int j;
}sItemPressed;

class CMapMain : public QWidget
{
	Q_OBJECT

public:
	CMapMain(QWidget *parent = 0, Qt::WFlags flags = 0);
	CMapMain(int arr[48][48],QWidget *parent = 0, Qt::WFlags flags = 0);
	~CMapMain();
	void UpdateGrid(int grid[48][48]);
	static void drawLine();

private:
	Ui::CMapMainClass ui;
	QGraphicsScene *GridMainScene;
	CPixItem *pPixItem[48][48];
	int PixColor[48][48];
	CGridItem *pGridItem;
	int pos[2];
	int startX,startY;
	float p_i;
	float p_j;
	sItemPressed PixPressed;

	void AddPix();
	bool isContain(int i, int j,QPointF p);

//protected:
	//void mouseReleaseEvent ( QMouseEvent * event );
	//void mousePressEvent(QMouseEvent *event );
	//void mouseMoveEvent(QMouseEvent *event );
	
	
};

#endif // MAPMAIN_H
