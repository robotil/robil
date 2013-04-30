#include "C11_structs.h"

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
