// Länkas ståendes i katalogin music/utils:
//
// g++ -o parsetest -I .. parsetest.cc -L ../rudeconfig/.libs -l rudeconfig 
//
// Användning:
// Behvovs inte,... typ
// export LD_LIBRARY_PATH=/home/hjorth/music/rudeconfig/.libs
//
// LD_LIBRARY_PATH=/home/hjorth/music/rudeconfig/.libs ./parsetest ../test/launchtest.music

#include "rudeconfig/src/config.h"
#include <iostream>

int
main (int argc, char** argv)
{
  rude::Config* cfile = new rude::Config ();
  cfile->load (argv[1]);
  int n_sections = cfile->getNumSections ();
  for (int s = 0; s < n_sections; ++s)
    {
      const char* name = cfile->getSectionNameAt (s);
      cfile->setSection (name);
      std::cout << "Section \"" << name << "\":" << std::endl;

      int n_members = cfile->getNumDataMembers ();
      for (int m = 0; m < n_members; ++m)
	{
	  const char* name = cfile->getDataNameAt (m);
	  std::cout << name << '\t' << cfile->getStringValue (name) << std::endl;
	}

      int n_sd = cfile->getNumSourceDestMembers();

      for (int i = 0; i < n_sd; i++)
        {
          const char* srcApp = cfile->getSrcAppAt(i);
          const char* srcObj = cfile->getSrcObjAt(i);
          const char* destApp = cfile->getDestAppAt(i);
          const char* destObj = cfile->getDestObjAt(i);
          const char* width = cfile->getWidthAt(i);

          std::cout << srcApp << "." << srcObj 
                    << " -> " << destApp << "." << destObj
                    << " [" << width << "]\n";
        }
    }
}
