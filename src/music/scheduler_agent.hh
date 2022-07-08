#ifndef MUSIC_SCHEDULER_AGENT_HH

#include "music/music-config.hh"
#include "music/scheduler.hh"
#include "music/multibuffer.hh"
#if MUSIC_USE_MPI

namespace MUSIC
{
  class Scheduler;
  typedef std::pair<Scheduler::SConnection, Scheduler::SConnData> SConnectionP;
  typedef std::vector< SConnectionP > SConnectionPV;


  class SchedulerAgent
  {
  protected:
    Scheduler *scheduler_;
    SchedulerAgent (Scheduler *scheduler);


    class CommObject
    {
    public:
      CommObject ()
      {
        time.set(ClockState(-1.0));
      }
      ;


      CommObject (Clock time_) :
        time (time_)
      {
      }
      ;


      virtual
      ~CommObject ()
      {
      }


      Clock time;


      void
      reset ()
      {
        time.set(ClockState(-1.0));
      }


      bool
      empty ()
      {
        return time.integerTime() < 0;
      }
    };


    virtual bool
    fillSchedule () = 0;

  public:
    virtual
    ~SchedulerAgent ()
    {
    }
    ;


    virtual void
    initialize (std::vector<Connector*>& connectors)=0;


    virtual bool
    tick (Clock& localTime)=0;

#if 0
    virtual void
    preFinalize (std::set<int> &cnn_ports)
    {
    }
    ;
#endif

    virtual void
    finalize (std::set<int> &cnn_ports) = 0;
  };


  class MulticommAgent : public virtual SchedulerAgent
  {
    static const int N_PLANNING_CYCLES = 100;

    std::map<int, Clock> commTimes;
    int rNodes;
    Clock time_;
    MultiBuffer* multiBuffer_;
    std::vector<MultiConnector*> multiConnectors;


    class Filter1
    {
      MulticommAgent &multCommObj_;
    public:

      Filter1 (MulticommAgent &multCommObj);

      bool
      operator() (SConnectionP &conn);
    };


    class Filter2
    {
      MulticommAgent &multCommObj_;

    public:
      Filter2 (MulticommAgent &multCommObj);

      bool
      operator() (SConnectionP &conn);
    };


    class MultiCommObject : public CommObject
    {
    private:
      unsigned int multiId_;
      unsigned int proxyId_;

    public:
      MultiCommObject (Clock time_, unsigned int multiId, unsigned int proxyId) :
          CommObject (time_), multiId_ (multiId), proxyId_ (proxyId)
      {
      }
      ;

      unsigned int
      multiId () const
      {
        return multiId_;
      }


      unsigned int
      proxyId () const
      {
        return proxyId_;
      }
    };


    std::vector<MultiCommObject> schedule;

  public:

    MulticommAgent (Scheduler *scheduler);

    ~MulticommAgent ();

    void initialize (std::vector<Connector*>& connectors);

#if 0
    void
    createMultiConnectors (Clock& localTime, MPI_Comm comm, int leader,
        std::vector<Connector*>& connectors);


    bool create (Clock& localTime);
#endif

    bool tick (Clock& localTime);

#if 0
    void preFinalize (std::set<int> &cnn_ports);
#endif
    void finalize (std::set<int> &cnn_ports);

  private:
    std::vector<bool>* multiProxies;

    std::vector<Connector*>
    connectorsFromMultiId (unsigned int multiId);

    bool
    fillSchedule ();

    void
    NextMultiConnection (SConnectionPV &candidates);

    void
    scheduleMulticonn (Clock &time, SConnectionPV::iterator first,
        SConnectionPV::iterator last);

    friend class Filter1;
    friend class Filter2;
    Filter1 *filter1;
    Filter2 *filter2;
  };


  class UnicommAgent : public virtual SchedulerAgent
  {
    class UniCommObject : public CommObject
    {
    public:
      Connector *connector;
      UniCommObject () :
          CommObject ()
      {
      }
      ;


      UniCommObject (Clock time_, Connector *connector_) :
          CommObject (time_), connector (connector_)
      {
      }
      ;
    };


    UniCommObject schedule;

    bool
    fillSchedule ();

  public:
    UnicommAgent (Scheduler *scheduler);

    ~UnicommAgent ()
    {
    }
    ;


    void
    initialize (std::vector<Connector*>& connectors)
    {
    }
    ;

    bool
    tick (Clock& localTime);

    void
    finalize (std::set<int> &cnn_ports);
  };


  class DummyAgent : public SchedulerAgent
  {

  protected:
    bool
    fillSchedule ()
    {
      return true;
    }

  public:
    DummyAgent (Scheduler* scheduler) :
        SchedulerAgent (scheduler)
    {
    }


    void
    initialize (std::vector<Connector*>& connectors)
    {
    }


    bool
    tick (Clock& localTime)
    {
      return true;
    }


    void
    finalize (std::set<int> &cnn_ports)
    {
    }
  };
}
#endif

#define MUSIC_SCHEDULER_AGENT_HH
#endif
