// vrpn_Leap.C
//	This file contains the code to operate a Leap Motion.
//	This version was written in Jan 2014 by Dave Borel.

#include "vrpn_Leap.h"

#if defined(VRPN_USE_LEAP)

#include <ctype.h>                      // for isprint, isalpha
#include <stdio.h>                      // for fprintf, sprintf, stderr, etc
#include <stdlib.h>                     // for atoi
#include <string.h>                     // for strlen, strncpy, strtok

#include "quat.h"                       // for Q_W, Q_X, Q_Y, Q_Z
#include "vrpn_Analog.h"                // for vrpn_Clipping_Analog_Server
#include "vrpn_BaseClass.h"             // for ::vrpn_TEXT_ERROR, etc
#include "vrpn_Button.h"                // for vrpn_Button_Server
#include "vrpn_Connection.h"            // for vrpn_Connection
#include "vrpn_MessageMacros.h"         // for VRPN_MSG_INFO, VRPN_MSG_WARNING, VRPN_MSG_ERROR
#include "vrpn_Serial.h"                // for vrpn_write_characters, etc
#include "vrpn_Shared.h"                // for vrpn_SleepMsecs, timeval, etc
#include "vrpn_Tracker.h"               // for vrpn_TRACKER_FAIL, etc

#include VRPN_LEAP_HEADER

#define	INCHES_TO_METERS	(2.54/100.0)
#define POINTABLE_COUNT     16

class vrpn_Leap_Device{
    public:
        vrpn_Leap_Device() : d_timestamp(0) { memset(d_ids, 0, POINTABLE_COUNT); }
        Leap::Controller d_controller;
        int64_t d_timestamp;
        int32_t d_ids[POINTABLE_COUNT];
};

vrpn_Leap::vrpn_Leap(const char *name, vrpn_Connection *c)
    : vrpn_Analog(name, c)
{
    d_device = new vrpn_Leap_Device();
}

vrpn_Leap::~vrpn_Leap()
{
    delete(d_device);
}

void vrpn_Leap::mainloop()
{
    Leap::Frame frame = d_device->d_controller.frame();

    int64_t timestamp = frame.timestamp();
    if (timestamp == d_device->d_timestamp)
        return;
    d_device->d_timestamp = timestamp;

    for (int i = 0; i < POINTABLE_COUNT; ++i)
    {
        const Leap::Pointable& p = frame.pointable(d_device->d_ids[i]);
        if (!p.isValid())
            continue;

        Leap::Vector position = p.tipPosition();
        Leap::Vector rotation = p.direction(); //TODO: Lookrotation.

        for (int j = 0; j < 3; ++j)
        {
            channel[6*i + j] = position[j];
            channel[6*i + j + 3] = rotation[j];
        }

        //TODO: Save each pointable's pose into floats readable by AnalogFly.
        //TODO: Enforce consistent finger/hand/tool ordering in channels.
    }
}

#endif
