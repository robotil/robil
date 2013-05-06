#!/usr/bin/perl

use strict;
use warnings;

my $errCount = 0;
my $strNum = 0;
my $find = "Error";

#===========================
my $except1 = "C23_dFind";
my $except2 = "C25_ekf";
my $except3 = "C41_QuasiStaticWalking";
my $except4 = "C66_Grasp";

open FILE, "<./log2.txt";
my @line = <FILE>;
for (@line) {
    $strNum++;
#===================================
    next if ($_ =~ /$except1/ | 
             $_ =~ /$except2/ | 
             $_ =~ /$except3/ | 
             $_ =~ /$except4/);  
#====================================

    if ($_ =~ /$find/) {
        $errCount++;
        print "string #$strNum: $_\n";
    }

}

if ($errCount > 0) {
   print "errors: $errCount\n";
   exit $errCount;
}

else {
   print "The Build was successfull\n";
   exit 0;
}


