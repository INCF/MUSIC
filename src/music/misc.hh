#ifndef MUSIC_MISC_HH

#include <type_traits>
#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <map>

namespace MUSIC
{

	template <typename T>
	std::ostream& operator<<(typename std::enable_if<std::is_enum<T>::value, std::ostream>::type& stream, const T& e)
	{
			return stream << static_cast<typename std::underlying_type<T>::type>(e);
	}

	template <typename T>
	using SPVec = std::vector<std::shared_ptr<T>>;



	/* std::string getEnvVar(std::string const& key) */
	/* { */
	/* 	char const* val = getenv(key.c_str()); */
	/* 		return val == NULL ? std::string() : std::string(val); */
	/* } */

}
#define MUSIC_MISC_HH
#endif
