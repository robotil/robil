#!/bin/sh
cat tasks_comp3 | sed 's/    */\n/g' | sed 's/^/#/g' | sed 's/^# *[0-9]\+\t*/1. /g' | sed 's/^#[\t ]*/  1. /g' | sed 's/^\(1\.\)\( .*\)$/\1###\2###/g' | egrep -v '^  1. $' | tee tasks_comp3_forWiki
