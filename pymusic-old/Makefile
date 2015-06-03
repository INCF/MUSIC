

LIB=-lpython2.5 -lmpi_cxx -lmpi -lopen-rte -lopen-pal
INCLUDE=-I/usr/include/python2.5

python4openmpi: python_main.c
	g++ -fPIC -O2 -c python_main.c -o python_main.o ${INCLUDE}
	g++ python_main.o -o python4openmpi ${LIB}
