/*
 * Node.cpp
 *
 *  Created on: Aug 20, 2012
 *      Author: dan
 */

#include "Node.h"

#include "Seq.h"
#include "Sel.h"
#include "Task.h"
#include "Dec.h"
#include "Par.h"
#include "UnknownNode.h"

Node::Ref static_Node_createNode(BT& bt){
	if(bt.getRootType()=="seq"){
		return Node::Ref(new Seq(bt));
	}else
	if(bt.getRootType()=="sel"){
		return Node::Ref(new Sel(bt));
	}else
	if(bt.getRootType()=="dec"){
		return Node::Ref(new Dec(bt));
	}else
	if(bt.getRootType()=="tsk"){
		return Node::Ref(new Task(bt));
	}else
	if(bt.getRootType()=="par"){
		return Node::Ref(new Par(bt));
	}
	return Node::Ref(new UnknownNode(bt));
}
Node::Ref static_Node_createNode(Lookup::Ref lookup,BT& bt){
	BT nbt = lookup->createNode(bt);
	lookup->analize(nbt);
	return static_Node_createNode(nbt);
}

