#!/bin/bash
rm -f commands.txt
touch commands.txt
cat omnetpp.ini | grep -P '\[Config ' | sed 's/\[Config \(.*\)\]/ projeto omnetpp.ini -u Cmdenv -c "\1"/' > commands.txt.tmp
cat commands.txt.tmp | while read line; do echo "${line} && rm -f results/*.vec && rm -f results/*.vci">>commands.txt; done
rm commands.txt.tmp

