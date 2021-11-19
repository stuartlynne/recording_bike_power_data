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

#include "PowerDecoder.h"
#include "RecordOutput.h"
#include "DecodeCrankTorqueFrequency.h"

static BPSAMPLER stState;
static PowerRecordReceiver prrPtr;

static double dRecordInterval;
static double dReSyncInterval;

#define UPDATE_EVENT_BYTE  1
#define SLOPE_MSB 2
#define SLOPE_LSB  3
#define TIME_STAMP_MSB 4
#define TIME_STAMP_LSB 5
#define TORQUE_TICKS_MSB  6
#define TORQUE_TICKS_LSB  7

///////////////////////////////////////////////////////////////////////////////
// void DecodeCrankTorqueFreq_Init(double dRecordInterval_, double dTimeBase_)
///////////////////////////////////////////////////////////////////////////////
//
// Call this to initialize the decoder.
// dTimeBase_ is set to zero to initialize event based decoding; otherwise
// the timebase value is assumed to be the sensor message update rate.
//
///////////////////////////////////////////////////////////////////////////////
void DecodeCrankTorqueFreq_Init(double dRecordInterval_, double dTimeBase_, double dReSyncInterval_, PowerRecordReceiver powerRecordReceiverPtr_)
{
    ResamplerOutput_Init(&stState, (int)(dRecordInterval_ * CTF_TIME_QUANTIZATION), dRecordInterval_, (int)(dTimeBase_ * CTF_TIME_QUANTIZATION));
    stState.usTorqueOffset = 500; // This is a nominal cal point for the SRM's we've seen.
    prrPtr = powerRecordReceiverPtr_;
    dRecordInterval = dRecordInterval_;
    dReSyncInterval = dReSyncInterval_;
}

///////////////////////////////////////////////////////////////////////////////
// void DecodeCrankTorqueFreq_Message(double dTime_, unsigned char messagePayload_[])
///////////////////////////////////////////////////////////////////////////////
//
// Message event handler interface.
// This is intended to abstract away the top-level messiness of having to
// detect data gaps or duplicates, etc.
//
///////////////////////////////////////////////////////////////////////////////
void DecodeCrankTorqueFreq_Message(double dTime_, unsigned char messagePayload_[])
{
    // see if the message is new.
    if (stState.ucLastEventCount != messagePayload_[UPDATE_EVENT_BYTE])
    {
        if ((dTime_ - stState.dLastMessageTime) > dReSyncInterval)
        {
            DecodeCrankTorqueFreq_Resync(dTime_, messagePayload_);
        }
        else
        {
            DecodeCrankTorqueFreq(dTime_, messagePayload_);
        }
        stState.dLastMessageTime = dTime_;
    }
}


///////////////////////////////////////////////////////////////////////////////
// void DecodeCrankTorqueFreq_Resync(double dCurrentTime_, unsigned char messagePayload_[])
///////////////////////////////////////////////////////////////////////////////
//
// Re-establish data baseline.
///////////////////////////////////////////////////////////////////////////////
void DecodeCrankTorqueFreq_Resync(double dCurrentTime_, unsigned char messagePayload_[])
{
    unsigned short usCurrentTorqueTicks;
    unsigned short usCurrentTimeStamp;
    // CurrentRecordEpoch is the last time that we should have had a data record.
    double dCurrentRecordEpoch = (floor(dCurrentTime_ / dRecordInterval)) * dRecordInterval;

    if ((stState.dLastRecordTime != 0) && (dCurrentRecordEpoch - stState.dLastRecordTime < MAXIMUM_TIME_GAP))
    {
        // Figure out how many records we missed.
        stState.ucRecordGapCount = (unsigned char)((dCurrentRecordEpoch - stState.dLastRecordTime + dRecordInterval * 0.5)
            / dRecordInterval);

        // Transfer the accumulated data to the gap.
        stState.fGapEnergy = stState.fAccumEnergy;
        stState.fGapRotation = stState.fAccumRotation;

        // We need to fill in the gap with records.
        RecordOutput_FillGap(prrPtr, &stState);
    }

    usCurrentTimeStamp = messagePayload_[TIME_STAMP_LSB];
    usCurrentTimeStamp += ((unsigned short)messagePayload_[TIME_STAMP_MSB]) << 8;

    usCurrentTorqueTicks = messagePayload_[TORQUE_TICKS_LSB];
    usCurrentTorqueTicks += ((unsigned short)messagePayload_[TORQUE_TICKS_MSB]) << 8;

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

    stState.usLastAccumTorque = usCurrentTorqueTicks;
    stState.usLastAccumPeriod = usCurrentTimeStamp;
    stState.ucLastRotationTicks = messagePayload_[UPDATE_EVENT_BYTE];
    stState.ucLastEventCount = messagePayload_[UPDATE_EVENT_BYTE];
}

///////////////////////////////////////////////////////////////////////////////
//
//
///////////////////////////////////////////////////////////////////////////////
void DecodeCrankTorqueFreq(double dTime_, unsigned char messagePayload_[])
{
    unsigned long ulNewEventTime;
    unsigned long ulEventCadence;
    unsigned long ulEventPower;
    unsigned short usCurrentTorqueTicks;
    unsigned short usCurrentTimeStamp;
    unsigned short usDeltaTorque;
    unsigned short usDeltaPeriod;
    unsigned char ucCurrentEventCount = messagePayload_[UPDATE_EVENT_BYTE];
    unsigned char ucDeltaEventCount;
    unsigned short usTorqueSlope;
    float fEventEnergy;

    usTorqueSlope = messagePayload_[SLOPE_LSB];
    usTorqueSlope += ((unsigned short)messagePayload_[SLOPE_MSB]) << 8;

    usCurrentTimeStamp = messagePayload_[TIME_STAMP_LSB];
    usCurrentTimeStamp += ((unsigned short)messagePayload_[TIME_STAMP_MSB]) << 8;

    usCurrentTorqueTicks = messagePayload_[TORQUE_TICKS_LSB];
    usCurrentTorqueTicks += ((unsigned short)messagePayload_[TORQUE_TICKS_MSB]) << 8;

    usDeltaTorque = usCurrentTorqueTicks - stState.usLastAccumTorque;    // make sure this is done in 16 bit word width!
    usDeltaPeriod = usCurrentTimeStamp - stState.usLastAccumPeriod;       // make sure this is done in 16 bit word width!
    ucDeltaEventCount = ucCurrentEventCount - stState.ucLastEventCount;

    // 65535 is an invalid value.
    if (usDeltaTorque == 65535)
    {
        usDeltaTorque = 0;
    }

    if (usDeltaPeriod && (usDeltaPeriod != 0xFFFF))
    {
        unsigned long ulTempTorque;
        ulNewEventTime = stState.ulEventTime + (unsigned long)usDeltaPeriod;

#if defined (TIMEBASE_DRIFT_CORRECTION)
        // This is a correction for cases where the sensor timebase is fast compared to the
        // receiver timebase.
        if ((dTime_ - (dLastRecordTime + (double)usDeltaPeriod/CTF_TIME_QUANTIZATION)) > (RECORD_INTERVAL))
        {
            //create a gap to fill.
            ulNewEventTime += usRecordInterval;
        }
#endif

        // We multiply this up by 32 so that we end up with the torque quantized to 1/32 N*m
        // like it is for the other crank-torque sensors.
        ulTempTorque = ((unsigned long)usDeltaTorque * CTF_TIME_QUANTIZATION * 32) / usDeltaPeriod;
        if (ulTempTorque > ((unsigned long)stState.usTorqueOffset * 32))
        {
            ulTempTorque -= (unsigned long)stState.usTorqueOffset * 32;
        }
        else
        {
            ulTempTorque = 0;
        }
        ulTempTorque *= 10;
        ulTempTorque /= usTorqueSlope;

        ulEventPower = ((long)(M_PI*2000.0 + 0.5) * ulTempTorque / usDeltaPeriod + 8) >> 4;
        ulEventCadence = ((long)ucDeltaEventCount * 60L * CTF_TIME_QUANTIZATION + (usDeltaPeriod >> 1)) / usDeltaPeriod;
        fEventEnergy = (float)(M_PI * (float)ulTempTorque / 16.0f);
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
        stState.fPendingRotation = stState.fAccumRotation + (float)ucDeltaEventCount * ((float)(stState.usRecordInterval - (stState.ulEventTime % stState.usRecordInterval))) / ((float)usDeltaPeriod);
        stState.fAccumRotation = (float)ucDeltaEventCount * ((float)(ulNewEventTime % stState.usRecordInterval)) / ((float)usDeltaPeriod);
        stState.fGapRotation = (float)ucDeltaEventCount * ((unsigned short)stState.ucRecordGapCount * stState.usRecordInterval) / ((float)usDeltaPeriod);
    }
    else
    {
        // This event came in before the next record epoch started - this
        // will happen when the event period is less than the recording period.
        stState.fAccumEnergy += fEventEnergy;
        stState.fAccumRotation += (float)ucDeltaEventCount;
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
                stState.dLastRecordTime += ((double)stState.usRecordInterval) / CTF_TIME_QUANTIZATION;
                (prrPtr)(stState.dLastRecordTime, stState.dTotalRotation, stState.dTotalEnergy, 0.0, 0.0);
            }
        }
    }

    // Propagate the message state information.
    stState.ucLastEventCount = ucCurrentEventCount;
    stState.ucLastRotationTicks = ucCurrentEventCount;
    stState.usLastAccumPeriod = usCurrentTimeStamp;
    stState.usLastAccumTorque = usCurrentTorqueTicks;
}


void DecodeCrankTorqueFreq_Calibration(double dTime_, unsigned char messagePayload_[])
{
    if (messagePayload_[CALIBRATION_ID_BYTE] != ANT_CTF_CALIBRATION_ID)
    {
        // bad message
        return;
    }

    switch (messagePayload_[ANT_CTF_CAL_TYPE_BYTE])
    {
    case ANT_CTF_CAL_ZERO:
        // Tricky part here is that we don't have a good way to qualify this
        // offset with respect to user actions, unless the input record were to
        // also capture head unit requests to the PM.
        stState.usTorqueOffset = messagePayload_[ANT_CTF_CAL_ZERO_LSB_BYTE];
        stState.usTorqueOffset += ((unsigned short)messagePayload_[ANT_CTF_CAL_ZERO_MSB_BYTE]) << 8;
        break;
    case ANT_CTF_CAL_SLOPE:
        break;
    case ANT_CTF_CAL_ESN:
        break;
    case ANT_CTF_CAL_ACK:
        break;
    default:
        break;
    }
}
