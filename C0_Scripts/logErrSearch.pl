#!/usr/bin/perl

use strict;
use warnings;

my $errCount = 0;
my $strNum = 0;
my $find = "Error";

open FILE, "<./log2.txt";
my @line = <FILE>;
for (@line) {
    $strNum++;
    if ($_ =~ /$find/) {
        $errCount++;
        print "string #$strNum: $_\n";
    }
}

if ($errCount > 0) {
   print "errors: $errCount\n";
}
else {
   print "The Build was successfull\n";
}
