#!/bin/sh

if test -z "$ACLOCAL" ; then
 ACLOCAL=aclocal
fi

ACDIR=`which $ACLOCAL`
ACDIR=`dirname $ACDIR`
ACDIR=`dirname $ACDIR`/share/aclocal

for each in $ACDIR ; do
  if test -d "$each"  ; then 
    AFLAGS="-I $each $AFLAGS"
  fi
done

echo $ACLOCAL $AFLAGS $@
$ACLOCAL $AFLAGS $@
