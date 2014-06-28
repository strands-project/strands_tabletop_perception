#!/bin/sh

PL_PORT=$1
PL_PASSWORD=$2
PL_LOGFILE=$3

swipl -f `rospack find qsr_kb`/scripts/swipl-server.pl -g "start_server($PL_PORT, $PL_PASSWORD, '$PL_LOGFILE')" 
