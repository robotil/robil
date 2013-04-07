#ifndef STRUCTS_H
#define STRUCTS_H

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

#endif // STRUCTS_H
