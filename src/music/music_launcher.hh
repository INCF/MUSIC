#ifndef MUSIC_LAUNCHER_HH

#include "music/music-config.hh"
#if MUSIC_USE_MPI
#include <mpi.h>

#include <string>
#include <vector>
#include <memory>

#include "music/configuration.hh"
#include "music/error.hh"

namespace MUSIC
{

	class MusicLauncher
	{
		public:
			virtual bool isResponsible()  = 0;
			virtual std::unique_ptr<Configuration> getConfiguration()  = 0;
			virtual MPI::MPI_Comm getComm()  = 0;
			virtual ~MusicLauncher();
			// Rename that?
			void init (int argc, char** argv, MPI::MPI_Comm comm) { argc_ = argc; argv_ = argv, comm_ = comm };
			bool launchedByMusic() { return launchedByMusic_; }
		private:
			bool launchedByMusic_ {false};
			int argc_ {0};
			char** argv_ {nullptr};
			MPI::MPI_Comm comm_ {MPI::COMM_WORLD};
			/* static std::vector<MusicLauncher> classes_ {}; */
		/* protected: */
		/* 	static registerSubClass() */
	};


	class MPMDLauncher : public MusicLauncher
	{

		public:

			MPMDLauncher (): launchedByMusic_ (true) {};
			bool isResponsible () override;
			std::unique_ptr<Configuration> getConfiguration()  override;
			MPI::MPI_Comm getComm()  override;

		private:
			void loadConfigFile(std::string filename, std::string& result);
  			bool getOption (int argc, char** argv, std::string option, std::string& result);
	};

	class ExecLauncher : public MusicLauncher
	{
		public:
			ExecLauncher (): {};
			bool isResponsible()  override;
			std::unique_ptr<Configuration> getConfiguration()  override;
			MPI::MPI_Comm getComm()  override;

		private:
			char* read_env(std::string envVarName);
	};

	class DefaultLauncher : public MusicLauncher
	{
		DefaultLauncher (): {};
		bool isResponsible()  override;
		std::unique_ptr<Configuration> getConfiguration()  override;
		MPI::MPI_Comm getComm()  override;
	};


	class MUSICLauncherFactory
	{
		public:
			MUSICLauncherFactory ():
				launchers_ { MPMDLauncher(), ExecLauncher() } {};
			MusicLauncher create(int argc, char** argv, MPI::MPI_Comm comm = MPI::COMM_WORLD);
			void addLauncher(MusicLauncher s);

		private:
			std::vector<MusicLauncher> launchers_;
	};

}

#endif
#define MUSIC_LAUNCHER_HH
