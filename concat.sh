#!/bin/bash

if [ "$#" -ne 1 ]
then
  echo "Usage: $0 filename"
  echo
  echo "Files following will be concatenated if output filename specified:"
  ls -l *.wav
  exit
fi

echo
echo "Files following were concatenated:"
ls -l *.wav
SIZES=( `ls -l *.wav | awk '{ print $5; }'` )

echo
echo "Positions:"
result=0
for s in ${SIZES[@]}
do
  echo "$result,"
  result=`expr $result + $s`
done

cat *.wav > $1
