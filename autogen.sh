#!/bin/sh

[ -f autogen.sh ] || {
  echo "autogen.sh: run this command only at the top of the source tree."
  exit 1
}

if test -z "$AUTOMAKE" ; then
 AUTOMAKE=automake
fi

if [ `uname -s` = Darwin ] ; then
# libtoolize is glibtoolize on OSX
  LIBTOOLIZE=glibtoolize
else
  LIBTOOLIZE=libtoolize
fi

./aclocal.sh &&
echo $LIBTOOLIZE --copy --automake &&
$LIBTOOLIZE --copy --automake &&
echo autoheader &&
autoheader &&
echo autoconf &&
autoconf &&
echo $AUTOMAKE --copy --add-missing &&
$AUTOMAKE --copy --add-missing &&
echo Now run configure and make.
