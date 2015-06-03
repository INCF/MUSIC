#!/bin/sh

if test -z "$ACLOCAL" ; then
 for each in aclocal-1.10 aclocal-1.9 aclocal-1.8 aclocal-1.7 aclocal-1.6 aclocal ; do
   ACLOCAL=$each
   if test -n "`which $each 2>/dev/null`" ; then break ; fi
 done
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
