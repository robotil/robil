#ifndef IMAGEDRAW_H
#define IMAGEDRAW_H

#include <QtGui/QMainWindow>
#include "C11_Node.h"
#include "ui_imagedraw.h"

class ImageDraw : public QMainWindow
{
	Q_OBJECT

public slots:
	void SltOnRectClick();
	void SltOnOpenUImgClick();

public:
	ImageDraw(int argc, char** argv, QWidget *parent = 0, Qt::WFlags flags = 0);
	~ImageDraw();

private:
	Ui::ImageDrawClass ui;
	QGraphicsScene* scene;
	QImage image;
	C11_Node C11node;
};

#endif // IMAGEDRAW_H
