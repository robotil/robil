#include "imagedraw.h"
#include <QtGui/QApplication>
#include <QFile>
#include <QLatin1String>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	QFile styleSheetFile("src/C11.qss");
        if (styleSheetFile.open(QIODevice::ReadOnly))
          {
            QString ss = QLatin1String(styleSheetFile.readAll());
            a.setStyleSheet(ss);
          }
	ImageDraw w(argc, argv);
	w.show();
	return a.exec();
}
