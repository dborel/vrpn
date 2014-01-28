// vrpn_Tracker_Leap.h
//	This file contains the class header for a Leap Motion.
// This file is based on the vrpn_Tracker_Fastrak.h file.
//	This version was written in the Winter of 2013 by Dave Borel.

#ifndef VRPN_TRACKER_LEAP_H
#define VRPN_TRACKER_LEAP_H

#include <stdio.h>                      // for NULL

#include "vrpn_Configure.h"             // for VRPN_API
#include "vrpn_Shared.h"                // for timeval
#include "vrpn_Tracker.h"               // for vrpn_Tracker_Serial
#include "vrpn_Types.h"                 // for vrpn_uint32

class VRPN_API vrpn_Clipping_Analog_Server;
class VRPN_API vrpn_Connection;

class VRPN_API vrpn_Tracker_Leap : public vrpn_Tracker {

 public:

  /// The constructor is given the name of the tracker (the name of
  /// the sender it should use) and the connection on which it is to
  /// send its messages.
  vrpn_Tracker_Leap(const char *name, vrpn_Connection *c);

  ~vrpn_Tracker_Leap();

  virtual void mainloop();
    
 protected:

  virtual int get_report(void);
  virtual void reset();

  struct timeval d_reset_time;
};

#endif
