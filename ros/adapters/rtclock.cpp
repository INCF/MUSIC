/*
 *  rtclock.cpp
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

#include "rtclock.h"

RTClock::RTClock (double interval = 0.)
{
  interval_ = timevalFromSeconds (interval);
  reset ();
}

void
RTClock::reset ()
{
  gettimeofday (&start_, 0);
  last_ = start_;
}

double
RTClock::time () const
{
  struct timeval now;
  gettimeofday (&now, 0);
  timersub (&now, &start_, &now);
  return secondsFromTimeval (now);
}

void
RTClock::sleep (double t) const
{
  struct timespec req = timespecFromSeconds (t);
  nanosleep (&req, NULL);
}

void
RTClock::sleepNext ()
{
  timeradd (&last_, &interval_, &last_);
  struct timespec req = timespecFromTimeval (last_);
  req.tv_sec = last_.tv_sec;
  req.tv_nsec = 1000 * last_.tv_usec;
  clock_nanosleep (CLOCK_REALTIME, TIMER_ABSTIME, &req, NULL);
}

void
RTClock::sleepUntil (double t) const
{
  struct timeval goal = timevalFromSeconds (t);
  timeradd (&start_, &goal, &goal);
  struct timespec req = timespecFromTimeval (goal);
  clock_nanosleep (CLOCK_REALTIME, TIMER_ABSTIME, &req, NULL);
}
