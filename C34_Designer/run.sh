#!/bin/bash
CWD=$PWD
pushd `dirname $0` > /dev/null
SCRIPTPATH=`pwd`
popd > /dev/null
cd $SCRIPTPATH
java -jar bin-jar/BTDesigner.jar
cd $CWD
