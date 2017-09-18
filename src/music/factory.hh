#ifndef MUSIC_FACTORY_HH

#include <type_traits>
#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <map>
#include <exception>

namespace MUSIC
{

	// Inspired by Alexandrescu's 'Modern C++' and enriched with C++11 features :)

	template <typename TKey, class TBase>
	class DefaultFactoryError
	{
		public:
			class Exception : public std::exception
			{
				Exception (const TKey& unknownKey)
					: unknownKey_ (unknownKey)
				{}
				virtual const char* what ()
				{
					return "Unknown object type passed to Factory.";
				}
				const TKey GetKey ()
				{
					return unknownKey_;
				}
				private:
					TKey unknownKey_;
			};
		protected:
			std::unique_ptr<TBase> OnUnknownType (const TKey& key)
			{
				throw Exception (key);
			}
	};

	template <
			class TBase,
			typename TKey = std::string,
			template <typename, class>
				class FactoryErrorPolicy = DefaultFactoryError,
			typename... Args
			>
	class Factory : FactoryErrorPolicy<TKey, TBase>
	{
		public:
			bool Register (const TKey& key, TBase base)
			{
				return map_.insert (FactoryMap::value_type (key, base)).second;
			}

			std::unique_ptr<TBase> Create (const TKey key, Args&&... args)
			{
				auto it = map_.find (key);
				if (it != map_.end ())
				{
					return it->second (std::forward<Args>(args)...);
				}
				return OnUnknownType (key);
			}
		private:
			using FactoryMap = std::map<TKey, TBase>;
			FactoryMap map_;
	};
}

#define MUSIC_FACTORY_HH
#endif
