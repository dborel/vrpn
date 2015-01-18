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

const int MAX_HANDS = 2;
const int MAX_FINGERS = 10;
const int MAX_TOOLS = 4;
const int MAX_POINTABLES = MAX_HANDS + MAX_FINGERS + MAX_TOOLS;

class vrpn_Leap_Device{
    public:
        vrpn_Leap_Device()
            : d_timestamp(0)
        {
            memset(d_ids, -1, MAX_POINTABLES * sizeof(int32_t));
            d_controller.setPolicyFlags(Leap::Controller::POLICY_BACKGROUND_FRAMES);
        }

        void ComputeIds()
        {
            memset(d_ids, -1, MAX_POINTABLES * sizeof(int32_t));

            Leap::Frame frame = d_controller.frame();

            //TODO: Enforce consistent finger/hand/tool ordering in channels.

            Leap::HandList hands = frame.hands();
            for (int i = 0; i < hands.count(); ++i)
            {
                if (i >= MAX_HANDS)
                    break;

                d_ids[i] = hands[i].id();
            }

            Leap::FingerList fingers = frame.fingers();
            for (int i = 0; i < fingers.count(); ++i)
            {
                if (i >= MAX_FINGERS)
                    break;

                d_ids[MAX_HANDS + i] = fingers[i].id();
            }

            Leap::ToolList tools = frame.tools();
            for (int i = 0; i < tools.count(); ++i)
            {
                if (i >= MAX_TOOLS)
                    break;

                d_ids[MAX_HANDS + MAX_FINGERS + i] = tools[i].id();
            }
        }

        Leap::Controller d_controller;
        int64_t d_timestamp;
        int32_t d_ids[MAX_POINTABLES];
};

vrpn_Leap::vrpn_Leap(const char *name, vrpn_Connection *c)
    : vrpn_Analog(name, c)
{
    vrpn_Analog::num_channel = 6 * MAX_POINTABLES + 3 * MAX_HANDS;
    memset(channel, 0, sizeof(channel));
    memset(last, 0, sizeof(last));

    d_device = new vrpn_Leap_Device();
}

vrpn_Leap::~vrpn_Leap()
{
    delete(d_device);
}

void vrpn_Leap::mainloop()
{
    server_mainloop();

    d_device->ComputeIds();

    Leap::Frame frame = d_device->d_controller.frame();

    int64_t timestamp = frame.timestamp();
    if (timestamp == d_device->d_timestamp)
        return;
    d_device->d_timestamp = timestamp;

    for (int i = 0; i < MAX_POINTABLES; ++i)
    {
        if (d_device->d_ids[i] == -1)
            continue;

        // Get the Leap data structures.
        
        const Leap::Pointable& p = frame.pointable(d_device->d_ids[i]);
        const Leap::Hand& h = frame.hand(d_device->d_ids[i]);
        if (!p.isValid() && !h.isValid())
            continue;

        // Get the current finger tracking state.

        Leap::Vector position = (h.isValid()) ? h.palmPosition() : p.tipPosition();
        Leap::Vector d = (h.isValid()) ? h.direction() : p.direction();
        Leap::Vector rotation(d.pitch(), d.yaw(), 0);

        // Pack the data into analog channels.

        for (int j = 0; j < 3; ++j)
        {
            channel[6*i + j] = position[j];
            channel[6*i + j + 3] = rotation[j];
        }

        // Save the grab, pinch, and confidence values in the last 3 analog channels.

        if (i < MAX_HANDS)
        {
            channel[6 * MAX_POINTABLES + i * MAX_HANDS    ] = h.grabStrength();
            channel[6 * MAX_POINTABLES + i * MAX_HANDS + 1] = h.pinchStrength();
            channel[6 * MAX_POINTABLES + i * MAX_HANDS + 2] = h.confidence();
        }
    }

    vrpn_gettimeofday(&_timestamp, NULL);

    report_changes();
}

void vrpn_Leap::report_changes(vrpn_uint32 class_of_service)
{
    vrpn_Analog::timestamp = _timestamp;

    vrpn_Analog::report_changes(class_of_service);
}

void vrpn_Leap::report(vrpn_uint32 class_of_service)
{
    vrpn_Analog::timestamp = _timestamp;

    vrpn_Analog::report(class_of_service);
}

#endif
