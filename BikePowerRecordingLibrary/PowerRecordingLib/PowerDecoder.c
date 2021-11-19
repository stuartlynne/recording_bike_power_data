////////////////////////////////////////////////////////////////////////////////
// Dynastream Innovations Inc.
// Cochrane, AB, CANADA
//
// Copyright © 2014 Dynastream Innovations Inc.
// All rights reserved. This software may not be reproduced by
// any means without express written approval of Dynastream
// Innovations Inc.
// The software is being provided on an "as-is" basis and as an accommodation,
// and therefore all warranties, representations, or guarantees of any kind
// (whether express, implied or statutory) including, without limitation,
// warranties of merchantability, non-infringement, or fitness for a particular
// purpose, are specifically disclaimed.
//
////////////////////////////////////////////////////////////////////////////////

#include "string.h"
#include "stdbool.h"
#include "stdint.h"
#include "stdlib.h"
#include "math.h"

#include "DecodeCrankTorque.h"
#include "DecodeCrankTorqueFrequency.h"
#include "DecodePowerOnly.h"
#include "DecodeWheelTorque.h"
#include "PowerDecoder.h"

static PowerRecordReceiver prrPtr;
static unsigned char ucPowerMeterChannel = 255;
static unsigned char ucPowerMeterType = 255;
static bool bResyncPowerChannel = true;
static bool bResyncPowerOnlyChannel = true;
static unsigned char ucPowerOnlyEventCount = 255;
static double dPowerOnlyBundleRxTime = -1;
static unsigned char ucNewPowerOnlyEventCount;

void InitPowerDecoder(double dRecordInterval_, double dTimeBase_, double dReSyncInterval_, PowerRecordReceiver powerRecordReceiverPtr_)
{
    prrPtr = powerRecordReceiverPtr_;
    DecodePowerOnly_Init(dRecordInterval_, dTimeBase_, dReSyncInterval_, prrPtr);
    DecodeCrankTorque_Init(dRecordInterval_, dTimeBase_, dReSyncInterval_, prrPtr);
    DecodeCrankTorqueFreq_Init(dRecordInterval_, dTimeBase_, dReSyncInterval_, prrPtr);
    DecodeWheelTorque_Init(dRecordInterval_, dTimeBase_, dReSyncInterval_, prrPtr);
}

// 16 = Power Only, 17 = Wheel Torque, 18 = Crank Torque, 32 = Crank Torque Frequency, 255 = Unknown
void SetPowerMeterType(unsigned char ucPowerMeterType_)
{
    ucPowerMeterType = ucPowerMeterType_;
}

void DecodePowerMessage(double dRxTime_, unsigned char messagePayload_[8])
{
    // Initialize the received time for power only event count bundled messages or
    // if the received times differ greatly (we may have missed messages beyond the event count rollover)
    if (dPowerOnlyBundleRxTime < 0 || (dRxTime_ - dPowerOnlyBundleRxTime) > 30)
        dPowerOnlyBundleRxTime = dRxTime_;

    // do page decoding against the expected power pages.
    switch (messagePayload_[0])
    {
        case ANT_POWERONLY:
            ucNewPowerOnlyEventCount = messagePayload_[1];

            if (ucNewPowerOnlyEventCount != ucPowerOnlyEventCount)
            {
                ucPowerOnlyEventCount = ucNewPowerOnlyEventCount;
                dPowerOnlyBundleRxTime = dRxTime_;
            }

            // Don't grab the power decoding unless we're the
            // only power message type we've received so far.
            if (ucPowerMeterType == 255)
            {
                ucPowerMeterType = messagePayload_[0];
                DecodePowerOnly_Resync(dPowerOnlyBundleRxTime, messagePayload_);
            }

            if (bResyncPowerOnlyChannel)
            {
                DecodePowerOnly_Resync(dPowerOnlyBundleRxTime, messagePayload_);
                bResyncPowerOnlyChannel = false;
            }

            // For now we will only decode the power only page if it is the only bike power page we receive
            if (ucPowerMeterType == ANT_POWERONLY)
                DecodePowerOnly_Message(dPowerOnlyBundleRxTime, messagePayload_);
            break;

        case ANT_WHEELTORQUE:
            if (ucPowerMeterType != messagePayload_[0])
            {
                // set up the power only message in addition
                // to the crank torque data stream.
                DecodePowerOnly_Resync(dRxTime_, messagePayload_);
                bResyncPowerOnlyChannel = false;

                DecodeWheelTorque_Resync(dRxTime_, messagePayload_);
                bResyncPowerChannel = false;

                ucPowerMeterType = messagePayload_[0];
            }

            // This is resolved here in order to handle decoder specific
            // resync requirements when a new message is available.
            if (bResyncPowerChannel)
            {
                DecodeWheelTorque_Resync(dRxTime_, messagePayload_);
                bResyncPowerChannel = false;
            }

            DecodeWheelTorque_Message(dRxTime_, messagePayload_);
            break;

        case ANT_CRANKTORQUE:
            if (ucPowerMeterType != messagePayload_[0])
            {
                // set up the power only message in addition
                // to the crank torque data stream.
                DecodePowerOnly_Resync(dRxTime_, messagePayload_);
                bResyncPowerOnlyChannel = false;

                DecodeCrankTorque_Resync(dRxTime_, messagePayload_);
                bResyncPowerChannel = false;

                ucPowerMeterType = messagePayload_[0];
            }

            // This is resolved here in order to handle decoder specific
            // resync requirements when a new message is available.
            if (bResyncPowerChannel)
            {
                DecodeCrankTorque_Resync(dRxTime_, messagePayload_);
                bResyncPowerChannel = false;
            }

            DecodeCrankTorque_Message(dRxTime_, messagePayload_);
            break;
        case ANT_CRANKFREQ:
            if (ucPowerMeterType != messagePayload_[0])
            {
                // set up the power only message in addition
                // to the crank torque data stream.
                DecodePowerOnly_Resync(dRxTime_, messagePayload_);
                bResyncPowerOnlyChannel = false;

                DecodeCrankTorqueFreq_Resync(dRxTime_, messagePayload_);
                bResyncPowerChannel = false;

                ucPowerMeterType = messagePayload_[0];
            }

            // This is resolved here in order to handle decoder specific
            // resync requirements when a new message is available.
            if (bResyncPowerChannel)
            {
                DecodeCrankTorqueFreq_Resync(dRxTime_, messagePayload_);
                bResyncPowerChannel = false;
            }

            DecodeCrankTorqueFreq_Message(dRxTime_, messagePayload_);
            break;

        case ANT_TEPS:
            // This is an auxiliary message, not valid unless there is an actual power message
            // to match it to.
            // We still need to correct for Rx Time because we do not know which power only event count shared message comes first.
            ucNewPowerOnlyEventCount = messagePayload_[1];

            if (ucNewPowerOnlyEventCount != ucPowerOnlyEventCount)
            {
                ucPowerOnlyEventCount = ucNewPowerOnlyEventCount;
                dPowerOnlyBundleRxTime = dRxTime_;
            }
            break;

        case ANT_CALIBRATION_MESSAGE:
            switch (ucPowerMeterType)
            {
            case ANT_CRANKFREQ:
                // The only one that really matters is the crank torque frequency meter.
                DecodeCrankTorqueFreq_Calibration(dRxTime_, messagePayload_);
                break;
            default:
                break;
            }
            break;
        default:
            // Other pages are ignored in this example.
            break;
    }
}
