#include <map>
#include <sstream>

#include <music/port.hh>

namespace MUSIC
{
	class PortManager
	{
		public:
			// createPort
			// disconnect
			// getPort
			//
		private:
			std::map<std::string, Port*> persistent_ports_;
			std::map<std::string, Port*> non_persistent_ports_;
	}
}
