#ifndef MUSIC_MISC_HH

#include <type_traits>
#include <iostream>
#include <vector>
#include <memory>

namespace MUSIC
{

	template<typename T>
	std::ostream& operator<<(typename std::enable_if<std::is_enum<T>::value, std::ostream>::type& stream, const T& e)
	{
			return stream << static_cast<typename std::underlying_type<T>::type>(e);
	}

	template<typename T>
	using SPVec = std::vector<std::shared_ptr<T>>;

}
#define MUSIC_MISC_HH
#endif
