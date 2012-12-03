#!/bin/bash

function uid { date +%y%m%d%H%M; }
mkdir -p tmp/lib tmp/includes/BTExecuter
cp bin_so/*.so tmp/lib
cp src/*.h tmp/includes/BTExecuter
cd tmp
F=BTExecuter-dev-$(uid).tgz
tar czfv $F * && cp $F ..
cd ..
rm -rf tmp
