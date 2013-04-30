#ifndef C11_STRUCTS_H
#define C11_STRUCTS_H

#include <QDataStream>
#include <QImage>
#include <sensor_msgs/Image.h>

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

QDataStream & operator<< ( QDataStream & out,const StructHeader & data );
QDataStream & operator<< ( QDataStream & out,const StructImage & data );

#endif // STRUCTS_H
