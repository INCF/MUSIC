#ifndef MUSIC_CONTEXT_HH

#include "music/music-config.hh"
#if MUSIC_USE_MPI
#include <mpi.h>
#include <string>
#include <vector>
#include <memory>
#include <functional>

#include "music/configuration.hh"
#include "music/error.hh"

namespace MUSIC
{

	// Note: We need a better solution for all of this launcher business.
	struct OptionConstants
	{
		static const char* const opConfigFileName;
		static const char* const opAppLabel;
	};

	struct OptionHelpers
	{
	  static bool
	  getOption (int argc, char** argv, std::string option, std::string& result)
	  {
		result.assign("");
		for (int i = 1; i < argc; ++i)
		   if (option.compare(argv[i]) == 0 && argc > i){
			   result.assign(argv[i+1]);  // skip options
				 return true;
		   }
		 return false;
	  }
	};

	struct EnvHelpers
	{
	  static char* read_env(std::string envVarName)
	  {
		  return std::getenv(envVarName.c_str());
	  }
	};


	class MusicContext
	{
		public:
			MusicContext () = default;
			MusicContext (int argc, char** argv, bool launchedByMusic)
				: argc_ (argc)
				, argv_ (argv)
				, launchedByMusic_ (launchedByMusic)
			{}
			/* virtual bool isResponsible()  = 0; */
			virtual std::unique_ptr<Configuration> getConfiguration() = 0;
			virtual MPI::Intracomm getComm()  = 0;
			// TODO do we need finalize or just a destructor?
			virtual void finalize () {};
			virtual ~MusicContext() {};
			bool launchedByMusic() const { return launchedByMusic_; }

		protected:
			int argc_ {0};
			char** argv_ {nullptr};
			bool launchedByMusic_ {false};
	};


	class MPMDContext : public MusicContext
	{

		public:
			MPMDContext (int argc, char** argv)
				: MusicContext (argc, argv, true)
				, config_ (assembleConfigFromFile ())
			{}
			std::unique_ptr<Configuration> getConfiguration() override;
			MPI::Intracomm getComm()  override;

		private:
			std::unique_ptr<Configuration> config_;
			std::unique_ptr<Configuration> assembleConfigFromFile ();

			void loadConfigFile(std::string filename, std::string& result);
	};

	class ExecContext : public MusicContext
	{
		public:
			ExecContext (int argc, char** argv)
				: MusicContext (argc, argv, true)
				, config_ (assembleConfigFromEnv())
			{}
			std::unique_ptr<Configuration> getConfiguration()  override;
			MPI::Intracomm getComm()  override;

		private:
			std::unique_ptr<Configuration> config_;
			std::unique_ptr<Configuration> assembleConfigFromEnv();
	};

	class DefaultContext : public MusicContext
	{
		public:
			std::unique_ptr<Configuration> getConfiguration()  override;
			MPI::Intracomm getComm()  override;
	};

	// Too much individual stuff, this needs to be done differently.
	using ContextFactoryfunction = std::function<MusicContext*(int argc, char** argv)>;

	struct FactoryFunctions
	{
		static ContextFactoryfunction MPMDContextFactory;
		static ContextFactoryfunction ExecContextFactory;
		static ContextFactoryfunction DefaultContextFactory;
	};

	class MusicContextFactory
	{
		public:

			MusicContextFactory ()
				: fac_functions_ {FactoryFunctions::MPMDContextFactory,  FactoryFunctions::ExecContextFactory, FactoryFunctions::DefaultContextFactory }
			{}
			// TODO add default parameter for MPI Communicator
			std::unique_ptr<MusicContext> createContext (int& argc, char**& argv);
			std::unique_ptr<MusicContext> createContext (int& argc, char**& argv, int required, int* provided);
			void addContextFactoryfunction (ContextFactoryfunction fac_function);

		private:
			std::vector<ContextFactoryfunction> fac_functions_;
			std::unique_ptr<MusicContext> createContextImpl (int argc, char** argv) const;
	};

}

#endif
#define MUSIC_CONTEXT_HH
#endif
