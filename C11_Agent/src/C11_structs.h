#ifndef C11_STRUCTS_H
#define C11_STRUCTS_H

#include <QDataStream>
#include <QImage>
#include <QMetaType>
#include <sensor_msgs/Image.h>

typedef struct
{
        double x;
        double y;
} StructPoint;

typedef struct
{
    short MessageID;
    unsigned int DataSize;
    unsigned int Counter;
} StructHeader;

typedef struct
{
  StructHeader Header;
  QImage Img;
}StructImage;

typedef struct
{
  StructPoint   RobotPos;
  double       RobolOrientation;
  int           XOffset;
  int           YOffset;
  int           Grid[100][100];
}StructGridData;

Q_DECLARE_METATYPE(StructGridData)

typedef struct
{
  StructHeader  Header;
  StructGridData Data;
}StructGrid;

QDataStream & operator<< ( QDataStream & out,const StructHeader & data );
QDataStream & operator<< ( QDataStream & out,const StructImage & data );
QDataStream & operator<< ( QDataStream & out,const StructPoint & data );
QDataStream & operator<< ( QDataStream & out,const StructGridData & data );
QDataStream & operator<< ( QDataStream & out,const StructGrid & data );

#endif // STRUCTS_H
