#!/bin/bash

if [ "$#" -ne 1 ]
then
  echo "Usage: $0 filename"
  exit
fi

SIZES=( `ls -l *.wav | awk '{ print $5; }'` )
FILE_LIST=( `ls -1 *.wav` )
NUM_FILES=`ls -1 *.wav | wc -l | awk '{ print $1; }'`

OFFSET=`expr 2 \* \( 1 + $NUM_FILES \)`
POS_LIST=`printf "%04x" $OFFSET`
result=$OFFSET
i=0
n=`expr ${#SIZES[@]}`
echo "Positions: (${NUM_FILES[@]} files)"
echo -e "[0x0000] 0($OFFSET bytes):\t\t[metadata]"
while :
do
  echo -e "[0x`printf "%04x" $result`] $result(${SIZES[$i]} bytes):\t${FILE_LIST[$i]}"
  result=`expr $result + ${SIZES[$i]}`

  i=`expr $i + 1`
  if [ $i -ge $n ]; then
    break
  fi

  POS_LIST="$POS_LIST `printf "%04x" $result`"
done

echo -e "[0x`printf "%04x" $result`] $result:\t\t\t(end)"

echo `printf "%04x" ${NUM_FILES}` $POS_LIST | xxd -r -p - | dd bs=2 conv=swab of=$1
cat *.wav >> $1
