#include <Python.h>

int
main(int argc, char *argv[])
{
  Py_Initialize();
  //PyRun_SimpleString("from time import time,ctime\n"
  //                   "print 'Today is',ctime(time())\n");
  Py_Main(argc, argv);
  Py_Finalize();
  return 0;
}
