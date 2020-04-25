#!/bin/bash

convert $1 tmp.rgb
W=`identify $1 | sed 's/.* \(...\)x... .*/\1/'`
H=`identify $1 | sed 's/.* ...x\(...\) .*/\1/'`
echo "Image $1 $W x $H"
echo -n > $1.raw
echo -ne \\\x`printf "%x" $((W/256))` >> $1.raw
echo -ne \\\x`printf "%x" $((W%256))` >> $1.raw
echo -ne \\\x`printf "%x" $((H/256))` >> $1.raw
echo -ne \\\x`printf "%x" $((H%256))` >> $1.raw
cat tmp.rgb >> $1.raw
