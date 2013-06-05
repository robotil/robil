#ifndef STRUCTS_H
#define STRUCTS_H

#include <QDataStream>

#define RAD2DEG 57.2957795

typedef enum
{
  RUNNING_ENUM=0,
  PAUSED_ENUM,
  STOPPED_ENUM
}EnumRunStatus;

typedef struct
{
	double x;
	double y;
} StructPoint;

typedef struct
{
        int x;
        int y;
} StructIntPoint;

typedef struct
{
    short MessageID;
    unsigned int DataSize;
    unsigned int Counter;
} StructHeader;

typedef struct
{
  StructPoint   RobotPos;
  double       RobolOrientation;
  int           XOffset;
  int           YOffset;
  int           Grid[100][100];
} StructGridData;

//QDataStream & operator>> ( QDataStream & in,const StructPoint & data);
//QDataStream & operator>> ( QDataStream & in,const StructGridData & data);

#endif // STRUCTS_H
