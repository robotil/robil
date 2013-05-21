#include "C11_structs.h"

QDataStream & operator<< ( QDataStream & out,const StructPoint & data )
{
        out<<data.x<<data.y;
        return out;
}

QDataStream & operator<< ( QDataStream & out,const StructHeader & data )
{
        out<<data.MessageID<<data.DataSize<<data.Counter;
        return out;
}

QDataStream & operator<< ( QDataStream & out,const StructImage & data )
{
        out<<data.Header<<data.Img;
        return out;
}

QDataStream & operator<< ( QDataStream & out,const StructGridData & data )
{
  out<<data.RobotPos;
  out<<data.RobolOrientation;
  out<<data.XOffset;
  out<<data.YOffset;
          for(int i=0; i<100; i++)
          {
            for(int j=0; j<100;j++)
            {
                out<<data.Grid[i][j];
            }
          }
          return out;
}

QDataStream & operator<< ( QDataStream & out,const StructGrid & data )
{
        out<<data.Header<<data.Data;
        return out;
}
