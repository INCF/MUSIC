/* -*- C++ -*-
 *  rtclock.h
 *
 *  Realtime clock
 *
 *  Copyright (C) 2015 Mikael Djurfeldt
 *
 *  rtclock is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  libneurosim is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef RTCLOCK_H
#define RTCLOCK_H

#include <time.h>
#include <sys/time.h>

class RTClock {
public:
  /**
   * Create a new RTClock and note the starting wallclock time which
   * time () and sleepUntil (t) will use as a reference.
   *
   * If the optional interval is specified, this gives the sleeping
   * time for the method sleepNext ().
   */
  RTClock (double interval);
  
  /**
   * Reset starting time.
   *
   * This resets the reference wallclock time for time () and
   * sleepUntil (t).
   */
  void reset ();
  
  /**
   * Return the time in seconds since starting time.
   */
  double time () const;

  /**
   * Sleep t seconds.
   */
  void sleep (double t) const;
  
  /**
   * Sleep until next interval counting from start time.
   *
   * If wallclock time has passed since last interval, sleep shorter
   * time such that we will wake up on a multiple of intervals from
   * start time.
   */
  void sleepNext ();
  
  /**
   * Sleep until time t counting from start time.
   */
  void sleepUntil (double t) const;
  
  /**
   * Convert a (relative) time t in seconds to a timeval
   */
  struct timeval timevalFromSeconds (double t) const;
  
  /**
   * Convert a (relative) time tv to seconds
   */
  double secondsFromTimeval (const struct timeval& tv) const;

  /**
   * Convert a (relative) time t in seconds to a timespec
   */
  struct timespec timespecFromSeconds (double s) const;
  
  /**
   * Convert tv into a timespec struct
   */
  struct timespec timespecFromTimeval (const struct timeval& tv) const;
  
private:
  struct timeval start_;
  struct timeval last_;
  struct timeval interval_;
};

inline struct timeval
RTClock::timevalFromSeconds (double t) const
{
  struct timeval tv;
  tv.tv_sec = t;
  tv.tv_usec = 1000000 * (t - tv.tv_sec);
  return tv;
}

inline double
RTClock::secondsFromTimeval (const struct timeval& tv) const
{
  return tv.tv_sec + 1e-6 * tv.tv_usec;
}

inline struct timespec
RTClock::timespecFromSeconds (double t) const
{
  struct timespec ts;
  ts.tv_sec = t;
  ts.tv_nsec = 1e9 * (t - ts.tv_sec);
  return ts;
}

inline struct timespec
RTClock::timespecFromTimeval (const struct timeval& tv) const
{
  struct timespec ts;
  ts.tv_sec = tv.tv_sec;
  ts.tv_nsec = 1000 * tv.tv_usec;
}

#endif /* RTCLOCK_H */
