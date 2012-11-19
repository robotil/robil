#!/bin/bash

echo "connect BTDesigner project to git source"
echo "Syntax: $0 <PATH_TO_BTDesigner_PROJECT> <PATH_TO_robil_git_FOLDER>"

function getDirName { 
	pushd
	cd $1
	pwd
	popd
}

PROJ=$( getDirName $1 )
GIT=$( getDirName $2 ) 
GNAME="C34_Designer"

echo "PROJECT	 : $PROJ"
echo "GIT	 : $GIT"


ERROR=$(tempfile)
function echoERROR { echo ERROR; cat $ERROR; }

echo "Remove folders and files"
cd $PROJ

echo -n "src 					"
rm -rf src 2>&1 > $ERROR && echo OK || echoERROR

echo -n "plans 					"
rm -rf plans 2>&1 > $ERROR && echo OK || echoERROR

echo -n "bin-jar				"
rm -rf bin-jar 2>&1 > $ERROR && echo OK || echoERROR

echo -n "BTDesigner.xml 			"
rm BTDesigner.xml 2>&1 > $ERROR && echo OK || echoERROR

echo "BTExecuter"
for f  in BTExecuter* ;do
	echo "... $f				"
	rm BTDesigner.xml 2>&1 > $ERROR && echo OK || echoERROR
done

echo "Link"

echo -n "src 					"
ln -s $GIT/$GNAME/src 2>&1 > $ERROR && echo OK || echoERROR

echo -n "plans 					"
ln -s $GIT/$GNAME/plans 2>&1 > $ERROR && echo OK || echoERROR

echo -n "bin-jar				"
ln -s $GIT/$GNAME/bin-jar 2>&1 > $ERROR && echo OK || echoERROR

echo -n "BTDesigner.xml 			"
ln -s $GIT/$GNAME/BTDesigner.xml 2>&1 > $ERROR && echo OK || echoERROR

echo "BTExecuter"
for f  in $GIT/$GNAME/BTExecuter* ;do
	echo "... $f				"
	ln -s $f  2>&1 > $ERROR && echo OK || echoERROR
done

rm $ERROR

echo "Done"
