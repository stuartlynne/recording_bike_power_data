/*
This software is subject to the license described in the License.txt file
included with this software distribution. You may not use this file except in compliance
with this license.

Copyright (c) Dynastream Innovations Inc. 2014
All rights reserved.
*/

#include "string.h"
#include "stdbool.h"
#include "stdlib.h"
#define _USE_MATH_DEFINES
#include "math.h"

#include "PowerDecoder.h"
#include "RecordOutput.h"
#include "DecodePowerOnly.h"

static BPSAMPLER stState;
static PowerRecordReceiver prrPtr;

static double dRecordInterval;
static double dReSyncInterval;

#define UPDATE_EVENT_BYTE  1
#define PEDAL_BALANCE_BYTE 2
#define INST_CADENCE_BYTE  3
#define ACCUM_POWER_LSB 4
#define ACCUM_POWER_MSB 5
#define INST_POWER_LSB  6
#define INST_POWER_MSB  7


void DecodePowerOnly_Init(double dRecordInterval_, double dTimeBase_, double dReSyncInterval_, PowerRecordReceiver powerRecordReceiverPtr_)
{
    ResamplerOutput_Init(&stState, (int)(dRecordInterval_* PO_TIME_QUANTIZATION), dRecordInterval_, (int)(dTimeBase_ * PO_TIME_QUANTIZATION));
    prrPtr = powerRecordReceiverPtr_;
    dRecordInterval = dRecordInterval_;
    dReSyncInterval = dReSyncInterval_;
}

//
// Message event handler interface.
// This is intended to abstract away the top-level messiness of having to detect data gaps or duplicates, etc.
//
void DecodePowerOnly_Message(double dTime_, unsigned char messagePayload_[])
{
    // see if the message is new.
    if (stState.ucLastEventCount != messagePayload_[1])
    {
        if ((dTime_ - stState.dLastMessageTime) > dReSyncInterval)
        {
            DecodePowerOnly_Resync(dTime_, messagePayload_);
        }
        else
        {
            DecodePowerOnly(dTime_, messagePayload_);
        }
        stState.dLastMessageTime = dTime_;
        stState.ucLastEventCount = messagePayload_[1];
    }
}

///////////////////////////////////////////////////////////////////////
//
// If the power-only messages are associated with torque based messages
// then we need to process them differently in order to make sure the
// total energy is properly calculated.
//
///////////////////////////////////////////////////////////////////////
void DecodePowerOnly_SetTimeBase(double dTimeBase_)
{
    // reset the timebase
    stState.usTimeBase = (int)(dTimeBase_ * PO_TIME_QUANTIZATION);
}

///////////////////////////////////////////////////////////////////////
//
// Re-establish data baseline.
///////////////////////////////////////////////////////////////////////
void DecodePowerOnly_Resync(double dCurrentTime_, unsigned char messagePayload_[])
{
    unsigned short usCurrentAccumPower;
    unsigned char ucCurrentEventCount = messagePayload_[UPDATE_EVENT_BYTE];

    double dCurrentRecordEpoch = (floor(dCurrentTime_ / dRecordInterval)) * dRecordInterval;

    if ((stState.dLastRecordTime != 0)
        && (dCurrentRecordEpoch - stState.dLastRecordTime > 0)
        && (dCurrentRecordEpoch - stState.dLastRecordTime < MAXIMUM_TIME_GAP))
    {
        stState.ucRecordGapCount = (unsigned char)((dCurrentRecordEpoch - stState.dLastRecordTime + dRecordInterval * 0.5)
            / dRecordInterval);

        // Transfer the accumulated data to the gap.
        stState.fGapEnergy = stState.fAccumEnergy;
        stState.fGapRotation = stState.fAccumRotation;

        // We need to fill in the gap with records.
        RecordOutput_FillGap(prrPtr, &stState);
    }

    usCurrentAccumPower = messagePayload_[ACCUM_POWER_LSB];
    usCurrentAccumPower += ((unsigned short)messagePayload_[ACCUM_POWER_MSB]) << 8;

    stState.ucCadence = messagePayload_[INST_CADENCE_BYTE];

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

    stState.usLastAccumPeriod = 0;
    stState.ucLastRotationTicks = messagePayload_[UPDATE_EVENT_BYTE];
    stState.ucLastEventCount = messagePayload_[UPDATE_EVENT_BYTE];

    stState.usLastAccumTorque = usCurrentAccumPower; // use the accumtorque field to store the accum power data
}

///////////////////////////////////////////////////////////////////////////////
//
//
///////////////////////////////////////////////////////////////////////////////
void DecodePowerOnly(double dTime_, unsigned char messagePayload_[])
{
    unsigned long ulNewEventTime;
    unsigned short usCurrentAccumPower;
    unsigned short usDeltaPeriod;
    unsigned short usDeltaPowerPeriod;
    unsigned short usDeltaPower;
    unsigned short usInstPower;
    unsigned char ucEventBalance = messagePayload_[PEDAL_BALANCE_BYTE];
    unsigned char ucCurrentEventCount = messagePayload_[UPDATE_EVENT_BYTE];
    unsigned char ucDeltaTicks;
    float fEventEnergy;


    usCurrentAccumPower = messagePayload_[ACCUM_POWER_LSB];
    usCurrentAccumPower += ((unsigned short)messagePayload_[ACCUM_POWER_MSB]) << 8;

    usInstPower = messagePayload_[INST_POWER_LSB];
    usInstPower += ((unsigned short)messagePayload_[INST_POWER_MSB]) << 8;

    usDeltaPower = usCurrentAccumPower - stState.usLastAccumTorque; // make sure this is done in 16 bit word width!
    ucDeltaTicks = messagePayload_[UPDATE_EVENT_BYTE] - stState.ucLastEventCount;
    stState.ucCadence = messagePayload_[INST_CADENCE_BYTE];

    // Sanity check on delta power vs. instantaneous.
    if ((usInstPower > 0) && (usDeltaPower > 100 * usInstPower))
    {
        usDeltaPower = usInstPower;
    }

    if (stState.ucCadence > 0)
    {
        usDeltaPeriod = (unsigned short)(((unsigned long)ucDeltaTicks * PO_TIME_QUANTIZATION * 60L + (stState.ucCadence >> 1)) / stState.ucCadence);
    }
    else
    {
        usDeltaPeriod = 0xFFFF;
        usDeltaPower = 0;
    }

    if (stState.usTimeBase != 0)
    {
        // time based messages.
        ulNewEventTime = stState.ulEventTime + (unsigned long)stState.usTimeBase*ucDeltaTicks;

#if defined (TIMEBASE_DRIFT_CORRECTION)
        // This is a correction for cases where the sensor timebase is fast compared to the
        // receiver timebase.
        if ((dTime_ - stState.dLastRecordTime) > (RECORD_INTERVAL * 2))
        {
            //create a gap to fill.
            ulNewEventTime += stState.usRecordInterval;
        }
#endif

        // Maybe we want to up the resolution on the power to energy
        // conversion. We round the power to the nearest watt so we
        // should be ok in the long term.
        usDeltaPowerPeriod = stState.usTimeBase*ucDeltaTicks;
        fEventEnergy = (float)usDeltaPower;
    }
    else
    {
        // event based messages
        usDeltaPowerPeriod = usDeltaPeriod;
        ulNewEventTime = stState.ulEventTime + (unsigned long)usDeltaPeriod;
        fEventEnergy = (float)usDeltaPower*usDeltaPeriod / PO_TIME_QUANTIZATION / ucDeltaTicks;
    }

    if (((unsigned short)(ulNewEventTime - stState.ulLastRecordTime)) >= stState.usRecordInterval)
    {
        // The event occurred after the end of the current record epoch.
        // First, figure out the number of records in a gap if it exists. This calculation uses
        // implicit truncation in the division so the subtraction can't be done first.
        stState.ucRecordGapCount = (unsigned char)((ulNewEventTime / stState.usRecordInterval) - (stState.ulLastRecordTime / stState.usRecordInterval) - 1);

        // Pending energy goes towards the partial accumulated record we currently have.
        stState.fPendingEnergy = stState.fAccumEnergy + fEventEnergy * ((float)(stState.usRecordInterval - (stState.ulEventTime % stState.usRecordInterval))) / ((float)usDeltaPowerPeriod);

        // accumulated energy goes towards the *next* event.
        stState.fAccumEnergy = fEventEnergy * ((float)(ulNewEventTime % stState.usRecordInterval)) / ((float)usDeltaPowerPeriod);

        // Gap energy fills the remainder.
        stState.fGapEnergy = fEventEnergy * ((unsigned short)stState.ucRecordGapCount * stState.usRecordInterval) / ((float)usDeltaPowerPeriod);

        //Same for rotation.
        stState.fPendingRotation = stState.fAccumRotation + (float)ucDeltaTicks * ((float)(stState.usRecordInterval - (stState.ulEventTime % stState.usRecordInterval))) / ((float)usDeltaPeriod);
        stState.fAccumRotation = (float)ucDeltaTicks * ((float)(ulNewEventTime % stState.usRecordInterval)) / ((float)usDeltaPeriod);
        stState.fGapRotation = (float)ucDeltaTicks * ((unsigned short)stState.ucRecordGapCount * stState.usRecordInterval) / ((float)usDeltaPeriod);
    }
    else
    {
        // This event came in before the next record epoch started - this
        // will happen when the event period is less than the recording period.
        stState.fAccumEnergy += fEventEnergy;
        if (stState.usTimeBase != 0)
        {
            stState.fAccumRotation += (float)ucDeltaTicks * (float)(stState.ucCadence) / 60.0f;
        }
        else
        {
            stState.fAccumRotation += (float)ucDeltaTicks;
        }

        stState.fPendingEnergy = 0;
        stState.fPendingRotation = 0;
        stState.ucRecordGapCount = 0;
    }

    stState.ulEventTime = ulNewEventTime;

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
                stState.dLastRecordTime += ((double)stState.usRecordInterval) / PO_TIME_QUANTIZATION;
                (prrPtr)(stState.dLastRecordTime, stState.dTotalRotation, stState.dTotalEnergy, 0.0, 0.0);
            }
        }
    }

    // Propagate the message state information.
    stState.ucLastRotationTicks = messagePayload_[UPDATE_EVENT_BYTE];
    stState.ucLastEventCount = messagePayload_[UPDATE_EVENT_BYTE];
    stState.usLastAccumTorque = usCurrentAccumPower;
}
