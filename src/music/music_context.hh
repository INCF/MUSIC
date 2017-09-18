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
			explicit MusicContext ();
			explicit MusicContext (int argc, char** argv, bool launchedByMusic, MPI_Comm comm)
				: argc_ (argc)
				, argv_ (argv)
				, launchedByMusic_ (launchedByMusic)
				, comm_ (comm)
			{}
			/* virtual bool isResponsible()  = 0; */
			virtual std::unique_ptr<Configuration> getConfiguration() = 0;
			virtual MPI_Comm getComm()  = 0;
			// TODO do we need finalize or just a destructor?
			virtual void finalize () {};
			virtual ~MusicContext();
			bool launchedByMusic() const { return launchedByMusic_; }

		protected:
			int argc_ {0};
			char** argv_ {nullptr};
			bool launchedByMusic_ {false};
			MPI_Comm comm_ {MPI::COMM_WORLD};
	};


	class MPMDContext : public MusicContext
	{

		public:
			MPMDContext (int argc, char** argv, MPI_Comm comm = MPI::COMM_WORLD)
				: MusicContext (argc, argv, true, comm)
				, config_ (assembleConfiguration ())
			{}
			std::unique_ptr<Configuration> getConfiguration() override;
			MPI_Comm getComm()  override;

		private:
			std::unique_ptr<Configuration> assembleConfiguration ();
			void loadConfigFile(std::string filename, std::string& result);
			std::unique_ptr<Configuration> config_;
	};

	class ExecContext : public MusicContext
	{
		public:
			ExecContext (int argc, char** argv, MPI_Comm comm = MPI::COMM_WORLD)
				: MusicContext (argc, argv, true, comm)
			{}
			std::unique_ptr<Configuration> getConfiguration()  override;
			MPI_Comm getComm()  override;
	};

	class DefaultContext : public MusicContext
	{
		public:
			std::unique_ptr<Configuration> getConfiguration()  override;
			MPI_Comm getComm()  override;
	};

	// Too much individual stuff, this needs to be done differently.
	using ContextFactoryMethod = std::function<MusicContext*(int argc, char** argv)>;

	ContextFactoryMethod MPMDContextFactory = [] (int argc, char** argv) -> MusicContext*
	{
			std::string config = "";
			if (!OptionHelpers::getOption(argc, argv, OptionConstants::opConfigFileName, config) )
			{
				return nullptr;
			}

			return new MPMDContext (argc, argv);
	};

	ContextFactoryMethod ExecContextFactory = [] (int argc, char** argv) -> MusicContext*
	{
		if (EnvHelpers::read_env(Configuration::configEnvVarName) != NULL)
			return new ExecContext (argc, argv);
		else
			return nullptr;
	};

	ContextFactoryMethod DefaultContextFactory = [] (int argc, char** argv) -> MusicContext*
	{
		return new DefaultContext ();
	};


	class MusicContextFactory
	{
		public:

			MusicContextFactory ()
				: fac_methods_ {MPMDContextFactory,  ExecContextFactory, DefaultContextFactory }
			{}
			// TODO add default parameter for MPI Communicator
			std::unique_ptr<MusicContext> createContext (int& argc, char**& argv) const;
			std::unique_ptr<MusicContext> createContext (int& argc, char**& argv, int required, int* provided) const;
			void addContextFactoryMethod (ContextFactoryMethod fac_method);

		private:
			std::vector<ContextFactoryMethod> fac_methods_;
			std::unique_ptr<MusicContext> createContextImpl (int argc, char** argv);
	};

}

#endif
#define MUSIC_CONTEXT_HH
#endif
