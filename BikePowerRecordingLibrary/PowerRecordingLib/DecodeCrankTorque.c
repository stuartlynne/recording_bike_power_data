/*
This software is subject to the license described in the License.txt file
included with this software distribution. You may not use this file except in compliance
with this license.

Copyright (c) Dynastream Innovations Inc. 2014
All rights reserved.
*/


#include "string.h"
#include "stdlib.h"
#define _USE_MATH_DEFINES
#include "math.h"

#include "RecordOutput.h"
#include "PowerDecoder.h"
#include "DecodeCrankTorque.h"

static BPSAMPLER stState;
static PowerRecordReceiver prrPtr;

static double dRecordInterval;
static double dReSyncInterval;

#define UPDATE_EVENT_BYTE  1
#define CRANK_TICKS_BYTE 2
#define INST_CADENCE_BYTE  3
#define ACCUM_PERIOD_LSB 4
#define ACCUM_PERIOD_MSB 5
#define ACCUM_TORQUE_LSB  6
#define ACCUM_TORQUE_MSB  7

///////////////////////////////////////////////////////////////////////////////
// void DecodeCrankTorque_Init(double dRecordInterval_, double dTimeBase_)
///////////////////////////////////////////////////////////////////////////////
//
// Call this to initialize the decoder.
// dTimeBase_ is set to zero to initialize event based decoding; otherwise
// the timebase value is assumed to be the sensor message update rate.
//
///////////////////////////////////////////////////////////////////////////////
void DecodeCrankTorque_Init(double dRecordInterval_, double dTimeBase_, double dReSyncInterval_, PowerRecordReceiver powerRecordReceiverPtr_)
{
    ResamplerOutput_Init(&stState, (int)(dRecordInterval_ * CT_TIME_QUANTIZATION), dRecordInterval_, (int)(dTimeBase_ * CT_TIME_QUANTIZATION));
    dRecordInterval = dRecordInterval_;
    prrPtr = powerRecordReceiverPtr_;
    dReSyncInterval = dReSyncInterval_;
}

///////////////////////////////////////////////////////////////////////////////
// void DecodeCrankTorque_Message(double dTime_, unsigned char aucByte_[])
///////////////////////////////////////////////////////////////////////////////
//
// Message event handler interface.
// This is intended to abstract away the top-level messiness of having to
// detect data gaps or duplicates, etc.
//
///////////////////////////////////////////////////////////////////////////////
void DecodeCrankTorque_Message(double dTime_, unsigned char aucByte_[])
{
    // see if the message is new.
    if (stState.ucLastEventCount != aucByte_[UPDATE_EVENT_BYTE])
    {
        if ((dTime_ - stState.dLastMessageTime) > dReSyncInterval)
        {
            DecodeCrankTorque_Resync(dTime_, aucByte_);
        }
        else
        {
            DecodeCrankTorque(dTime_, aucByte_);
        }
        stState.dLastMessageTime = dTime_;
    }
}


///////////////////////////////////////////////////////////////////////////////
// void DecodeCrankTorque_Resync(double dCurrentTime_, unsigned char aucByte_[])
///////////////////////////////////////////////////////////////////////////////
//
// Re-establish data baseline.
///////////////////////////////////////////////////////////////////////////////
void DecodeCrankTorque_Resync(double dCurrentTime_, unsigned char aucByte_[])
{
    unsigned short usCurrentAccumTorque;
    unsigned short usCurrentAccumPeriod;
    // CurrentRecordEpoch is the last time that we should have had a data record.
    double dCurrentRecordEpoch = (floor(dCurrentTime_ / dRecordInterval)) * dRecordInterval;

    if ((stState.dLastRecordTime != 0) && (dCurrentRecordEpoch - stState.dLastRecordTime < MAXIMUM_TIME_GAP))
    {
        // Figure out how many records we missed based on the receive timestamps.
        stState.ucRecordGapCount = (unsigned char)((dCurrentRecordEpoch - stState.dLastRecordTime + 0.5 * dRecordInterval)
            / (dRecordInterval));      // We need to fill in the gap with records.
        // Transfer the accumulated data to the gap.
        stState.fGapEnergy = stState.fAccumEnergy;
        stState.fGapRotation = stState.fAccumRotation;

        RecordOutput_FillGap(prrPtr, &stState);
    }

    usCurrentAccumPeriod = aucByte_[ACCUM_PERIOD_LSB];
    usCurrentAccumPeriod += ((unsigned short)aucByte_[ACCUM_PERIOD_MSB]) << 8;

    usCurrentAccumTorque = aucByte_[ACCUM_TORQUE_LSB];
    usCurrentAccumTorque += ((unsigned short)aucByte_[ACCUM_TORQUE_MSB]) << 8;

    stState.ucCadence = aucByte_[INST_CADENCE_BYTE];

    stState.fAccumEnergy = 0;
    stState.fPendingEnergy = 0;
    stState.fGapEnergy = 0;

    stState.fAccumRotation = 0;
    stState.fPendingRotation = 0;
    stState.fGapRotation = 0;
    stState.ucRecordGapCount = 0;

    stState.ulEventTime = 0;
    stState.ulLastRecordTime = 0;

    stState.dLastMessageTime = dCurrentTime_;

    // Update our saved state.
    stState.dLastRecordTime = dCurrentRecordEpoch;

    stState.usLastAccumTorque = usCurrentAccumTorque;
    stState.usLastAccumPeriod = usCurrentAccumPeriod;
    stState.ucLastRotationTicks = aucByte_[CRANK_TICKS_BYTE];
    stState.ucLastEventCount = aucByte_[UPDATE_EVENT_BYTE];
}

///////////////////////////////////////////////////////////////////////////////
//
//
///////////////////////////////////////////////////////////////////////////////
void DecodeCrankTorque(double dTime_, unsigned char aucByte_[])
{
    unsigned long ulNewEventTime;
    unsigned long ulEventCadence;
    unsigned long ulEventPower;
    unsigned short usCurrentAccumTorque;
    unsigned short usCurrentAccumPeriod;
    unsigned short usDeltaTorque;
    unsigned short usDeltaPeriod;
    unsigned char ucDeltaEventCount;
    unsigned char ucDeltaTicks;
    float fEventEnergy;

    usCurrentAccumPeriod = aucByte_[ACCUM_PERIOD_LSB];
    usCurrentAccumPeriod += ((unsigned short)aucByte_[ACCUM_PERIOD_MSB]) << 8;

    usCurrentAccumTorque = aucByte_[ACCUM_TORQUE_LSB];
    usCurrentAccumTorque += ((unsigned short)aucByte_[ACCUM_TORQUE_MSB]) << 8;

    usDeltaTorque = usCurrentAccumTorque - stState.usLastAccumTorque; // make sure this is done in 16 bit word width!
    usDeltaPeriod = usCurrentAccumPeriod - stState.usLastAccumPeriod; // make sure this is done in 16 bit word width!
    ucDeltaEventCount = aucByte_[UPDATE_EVENT_BYTE] - stState.ucLastEventCount;
    ucDeltaTicks = aucByte_[CRANK_TICKS_BYTE] - stState.ucLastRotationTicks;
    stState.ucCadence = aucByte_[INST_CADENCE_BYTE];

    // 65535 is an invalid value.
    if (usDeltaTorque == 65535)
    {
        usDeltaTorque = 0;
    }

    if (usDeltaPeriod && (usDeltaPeriod != 0xFFFF))
    {
        ulNewEventTime = stState.ulEventTime + (unsigned long)usDeltaPeriod;

        ulEventPower = ((long)(M_PI*2048.0 + 0.5) * usDeltaTorque / usDeltaPeriod + 8) >> 4;
        ulEventCadence = ((long)ucDeltaTicks * 60L * CT_TIME_QUANTIZATION + (usDeltaPeriod >> 1)) / usDeltaPeriod;
        fEventEnergy = (float)(M_PI * (float)usDeltaTorque / 16.0);
    }
    else
    {
        // This is basically a non-event.
        ulEventPower = 0;
        ulEventCadence = 0;
        fEventEnergy = 0;
        ulNewEventTime = stState.ulEventTime;
    }

    if (((unsigned short)(ulNewEventTime - stState.ulLastRecordTime)) >= stState.usRecordInterval)
    {
        // The event occurred after the end of the current record epoch.
        // First, figure out the number of records in a gap if it exists. This calculation uses
        // implicit truncation in the division so the subtraction can't be done first.
        stState.ucRecordGapCount = (unsigned char)((ulNewEventTime / stState.usRecordInterval) - (stState.ulLastRecordTime / stState.usRecordInterval) - 1);

        // Pending energy goes towards the partial accumulated record we currently have.
        stState.fPendingEnergy = stState.fAccumEnergy + fEventEnergy * ((float)(stState.usRecordInterval - (stState.ulEventTime % stState.usRecordInterval))) / ((float)usDeltaPeriod);

        // accumulated energy goes towards the *next* event.
        stState.fAccumEnergy = fEventEnergy * ((float)(ulNewEventTime % stState.usRecordInterval)) / ((float)usDeltaPeriod);

        // Gap energy fills the remainder.
        stState.fGapEnergy = fEventEnergy * ((unsigned short)stState.ucRecordGapCount * stState.usRecordInterval) / ((float)usDeltaPeriod);

        //Same for rotation.
        stState.fPendingRotation = stState.fAccumRotation + (float)ucDeltaTicks * ((float)(stState.usRecordInterval - (stState.ulEventTime % stState.usRecordInterval))) / ((float)usDeltaPeriod);
        stState.fAccumRotation = (float)ucDeltaTicks * ((float)(ulNewEventTime % stState.usRecordInterval)) / ((float)usDeltaPeriod);
        stState.fGapRotation = (float)((float)ucDeltaTicks * ((unsigned short)stState.ucRecordGapCount * stState.usRecordInterval) / ((float)usDeltaPeriod));
    }
    else
    {
        // This event came in before the next record epoch started - this
        // will happen when the event period is less than the recording period.
        stState.fAccumEnergy += fEventEnergy;
        stState.fAccumRotation += (float)ucDeltaTicks;
        stState.fPendingEnergy = 0;
        stState.fPendingRotation = 0;
        stState.ucRecordGapCount = 0;
    }
    stState.ulEventTime += (unsigned long)usDeltaPeriod;

    if (((unsigned short)(stState.ulEventTime - stState.ulLastRecordTime)) >= stState.usRecordInterval)
    {
        RecordOutput(prrPtr, &stState);
    }
    else
    {
        // We've had an event that either didn't have a rotation associated
        // with it (no event time increment) or else it was within the
        // recording interval.
        if ((dTime_ - stState.dLastRecordTime) > dRecordInterval)
        {
            while ((dTime_ - stState.dLastRecordTime) > dRecordInterval)
            {
                stState.dLastRecordTime += dRecordInterval;
                (prrPtr)(stState.dLastRecordTime, stState.dTotalRotation, stState.dTotalEnergy, 0.0, 0.0);
            }
        }
    }

    // Propagate the message state information.
    stState.ucLastEventCount = aucByte_[UPDATE_EVENT_BYTE];
    stState.ucLastRotationTicks = aucByte_[CRANK_TICKS_BYTE];
    stState.usLastAccumPeriod = usCurrentAccumPeriod;
    stState.usLastAccumTorque = usCurrentAccumTorque;
}

