#include "imagedraw.h"
#include <QtGui/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	ImageDraw w(argc, argv);
	w.show();
	return a.exec();
}
