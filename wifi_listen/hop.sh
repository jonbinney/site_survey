#!/bin/bash

while true
do
  for c in 01 03 05 07 09 11 13 149 153 159 165 38 44 48 02 04 06 08 10 12 14 151 157 161 36 40 46 
  do
    iwconfig wlan1 channel $c
    echo $c
    sleep 3
  done
done
