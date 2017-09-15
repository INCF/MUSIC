#ifndef MUSIC_LAUNCHER_HH

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

	class MusicLauncher
	{
		public:
			MusicLauncher ();
			MusicLauncher (int argc, char** argv, bool launchedByMusic, MPI_Comm comm)
				: argc_ (argc)
				, argv_ (argv)
				, launchedByMusic_ (launchedByMusic)
				, comm_ (comm)
			{}
			/* virtual bool isResponsible()  = 0; */
			virtual std::unique_ptr<Configuration> getConfiguration() = 0;
			virtual MPI_Comm getComm()  = 0;
			virtual ~MusicLauncher();
			// Rename that?
			bool launchedByMusic() { return launchedByMusic_; }
			static bool isResponsible (int argc, char** argv) {return true;}

		protected:
			int argc_ {0};
			char** argv_ {nullptr};
			bool launchedByMusic_ {false};
			MPI_Comm comm_ {MPI::COMM_WORLD};
	};


	class MPMDLauncher : public MusicLauncher
	{

		public:
			MPMDLauncher (int argc, char** argv, MPI_Comm comm = MPI::COMM_WORLD)
				: MusicLauncher (argc, argv, true, comm)
				, config_ (assembleConfiguration ())
			{}
			/* bool isResponsible () override; */
			std::unique_ptr<Configuration> getConfiguration() override;
			MPI_Comm getComm()  override;
			static bool isResponsible (int argc, char** argv);

		private:
			std::unique_ptr<Configuration> assembleConfiguration ();
			void loadConfigFile(std::string filename, std::string& result);
  			static bool getOption (int argc, char** argv, std::string option, std::string& result);
			std::unique_ptr<Configuration> config_;
	};

	class ExecLauncher : public MusicLauncher
	{
		public:
			ExecLauncher (int argc, char** argv, MPI_Comm comm = MPI::COMM_WORLD)
				: MusicLauncher (argc, argv, true, comm)
			{}
			/* bool isResponsible()  override; */
			std::unique_ptr<Configuration> getConfiguration()  override;
			MPI_Comm getComm()  override;
			static bool isResponsible ();
	};

	class DefaultLauncher : public MusicLauncher
	{
		public:
			DefaultLauncher (int argc, char** argv, MPI_Comm comm = MPI::COMM_WORLD)
				: MusicLauncher (argc, argv, false, comm)
			{}
			/* bool isResponsible()  override; */
			std::unique_ptr<Configuration> getConfiguration()  override;
			MPI_Comm getComm()  override;
	};

	using LauncherSelectionHandler = std::function<std::unique_ptr<MusicLauncher>(int argc, char** argv)>;

	LauncherSelectionHandler MPMDHandler = [] (int argc, char** argv)
	{
		if (MPMDLauncher::isResponsible (argc, argv))
			return std::unique_ptr<MusicLauncher> {new MPMDLauncher (argc, argv)};
		else
			return std::unique_ptr<MusicLauncher> {nullptr};
	};

	LauncherSelectionHandler ExecHandler = [] (int argc, char** argv)
	{
		if (ExecLauncher::isResponsible ())
			return std::unique_ptr<MusicLauncher> {new ExecLauncher (argc, argv)};
		else
			return std::unique_ptr<MusicLauncher> {nullptr};
	};

	LauncherSelectionHandler DefaultHandler = [] (int argc, char** argv)
	{
		return std::unique_ptr<MusicLauncher> {new DefaultLauncher (argc, argv)};
	};


	class MusicLauncherFactory
	{
		public:

			MusicLauncherFactory ()
				: handler_ {MPMDHandler, ExecHandler, DefaultHandler}
			{}
			std::unique_ptr<MusicLauncher> create (int argc, char** argv) const;
			void addHandler (LauncherSelectionHandler handler);

		private:
			std::vector<LauncherSelectionHandler> handler_;
	};

}

#endif
#define MUSIC_LAUNCHER_HH
#endif
