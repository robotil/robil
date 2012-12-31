#ifndef IMAGEDRAW_H
#define IMAGEDRAW_H

#include <QtGui/QMainWindow>
#include <QMap>
#include "graphicsview.h"
#include "C11_Node.h"
#include "C11_Node_Subscriber.h"
#include "ui_imagedraw.h"

class ImageDraw : public QMainWindow, public IC11_Node_Subscriber
{
	Q_OBJECT

public slots:
	void SltOnRectClick();
	void SltOnOpenUImgClick();
	void SltImageAreaOpened(int);
	void SltOnNewImg(QImage img);

signals:
	void SigOnNewImg(QImage img);

public:
	ImageDraw(int argc, char** argv, QWidget *parent = 0, Qt::WFlags flags = 0);
	~ImageDraw();

	void CreateNewImageArea(QString image);

	//C11_Node_Subscriber implementation
	virtual void OnImgReceived(QImage image);

protected:
	void CloseOpenedImages();

private:
	Ui::ImageDrawClass ui;
	C11_Node C11node;
	int ImageAreaCount;
	QMap<int,CGraphicsView*> ImageAreas;
	bool IsUpdateCurrentImg;	//don't create new image, update the current
};

#endif // IMAGEDRAW_H
