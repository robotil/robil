#ifndef GRAPHICSVIEW_H
#define GRAPHICSVIEW_H

#include <QGraphicsView>

class CGraphicsView : public QGraphicsView
{
	Q_OBJECT

signals:
	void SigOpened(int id);

public:
	CGraphicsView(int id, QString imageName, QString dateTimeStr, QWidget *parent = 0);
	CGraphicsView(int id, QImage image, QString dateTimeStr, QWidget *parent = 0);
	~CGraphicsView();

	void setScene(QGraphicsScene * scene);
	void UpdateImage(QImage image, QString dateTimeStr);

	void SetOpen(bool isOpen);
	bool IsOpen();
	int GetId();

	void OpenView();
	void MinimizeView();

protected:

	virtual void 	mouseDoubleClickEvent ( QMouseEvent * event );


private:
	int Id;
	bool IsOpened;
	QString DateTimeStr;
	QImage Image;
	QGraphicsTextItem* DateTimeItem;
};

#endif // GRAPHICSVIEW_H
