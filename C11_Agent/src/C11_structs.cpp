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

//          std::cout<<"MAP ON TCP SENDING:"<<std::endl;
//            for(int i=99; i>=0; i--)
//            {
//                    for(int j=0; j<100; j++)
//                    {
//                        switch ( data.Grid[i][j] ){
//                        case 0:
//                            std::cout<<'.'; break;
//                        case 1:
//                            std::cout<<'*'; break;
//                        case 2:
//                            std::cout<<'.'; break;
//                        default:
//                            std::cout<<'?'; break;
//                        }
//                    }
//                    std::cout<<std::endl;
//            }
          return out;
}

QDataStream & operator<< ( QDataStream & out,const StructGrid & data )
{
        out<<data.Header<<data.Data;
        return out;
}
