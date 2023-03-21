# compile time python tests

# Test mpi4py version
import mpi4py
from looseversion import LooseVersion
is_v2 = LooseVersion(mpi4py.__version__) > LooseVersion("1.3.1")

# Write configuration file 
with open("pyconfig.pxi", "w") as f: 
	f.write("DEF MPI4V2 = {0}".format(is_v2))
