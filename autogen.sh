#!/bin/sh

[ -f autogen.sh ] || {
  echo "autogen.sh: run this command only at the top of the source tree."
  exit 1
}

if test -z "$AUTOMAKE" ; then
 for each in automake-1.10 automake-1.9 automake-1.8 automake-1.7 automake-1.6 automake ; do
   AUTOMAKE=$each
   if test -n "`which $each 2>/dev/null`" ; then break ; fi
 done
fi

./aclocal.sh &&
echo libtoolize --copy --automake &&
libtoolize --copy --automake &&
echo autoheader &&
autoheader &&
echo autoconf &&
autoconf &&
echo $AUTOMAKE --copy --add-missing &&
$AUTOMAKE --copy --add-missing &&
echo Now run configure and make.
