#ifndef IMAGEDRAW_H
#define IMAGEDRAW_H

#include <QtGui/QMainWindow>
#include <QMap>
#include "graphicsview.h"
#include "C11_Node.h"
#include "ui_imagedraw.h"

class ImageDraw : public QMainWindow
{
	Q_OBJECT

public slots:
	void SltOnRectClick();
	void SltOnOpenUImgClick();
	void SltImageAreaOpened(int);

public:
	ImageDraw(int argc, char** argv, QWidget *parent = 0, Qt::WFlags flags = 0);
	~ImageDraw();

	void CreateNewImageArea(QString image);

protected:
	void CloseOpenedImages();

private:
	Ui::ImageDrawClass ui;
	C11_Node C11node;
	int ImageAreaCount;
	QMap<int,CGraphicsView*> ImageAreas;
};

#endif // IMAGEDRAW_H
