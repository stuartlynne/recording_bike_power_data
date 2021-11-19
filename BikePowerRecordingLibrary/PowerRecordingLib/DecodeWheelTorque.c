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
#include "DecodeWheelTorque.h"

#define PROPAGATE_CADENCE

static BPSAMPLER stState;
static PowerRecordReceiver prrPtr;

static double dRecordInterval;
static double dReSyncInterval;

#define UPDATE_EVENT_BYTE  1
#define WHEEL_TICKS_BYTE 2
#define INST_CADENCE_BYTE  3
#define ACCUM_PERIOD_LSB 4
#define ACCUM_PERIOD_MSB 5
#define ACCUM_TORQUE_LSB  6
#define ACCUM_TORQUE_MSB  7


void DecodeWheelTorque_Init(double dRecordInterval_, double dTimeBasedPeriod_, double dReSyncInterval_, PowerRecordReceiver powerRecordReceiverPtr_)
{
    ResamplerOutput_Init(&stState, (int)(dRecordInterval_ * WT_TIME_QUANTIZATION), dRecordInterval_, (int)(dTimeBasedPeriod_ * WT_TIME_QUANTIZATION));
    prrPtr = powerRecordReceiverPtr_;
    dRecordInterval = dRecordInterval_;
    dReSyncInterval = dReSyncInterval_;
}

///////////////////////////////////////////////////////////////////////////////
// void DecodeCrankTorque_Message(double dTime_, unsigned char messagePayload_[])
///////////////////////////////////////////////////////////////////////////////
//
// Message event handler interface.
// This is intended to abstract away the top-level messiness of having to
// detect data gaps or duplicates, etc.
//
///////////////////////////////////////////////////////////////////////////////
void DecodeWheelTorque_Message(double dTime_, unsigned char messagePayload_[])
{
    // see if the message is new.
    if (stState.ucLastEventCount != messagePayload_[UPDATE_EVENT_BYTE])
    {
        if ((dTime_ - stState.dLastMessageTime) > dReSyncInterval)
        {
            DecodeWheelTorque_Resync(dTime_, messagePayload_);
        }
        else
        {
            DecodeWheelTorque(dTime_, messagePayload_);
        }
        stState.dLastMessageTime = dTime_;
        stState.ucLastEventCount = messagePayload_[UPDATE_EVENT_BYTE];
    }
}


///////////////////////////////////////////////////////////////////////////////
// void DecodeCrankTorque_Resync(double dCurrentTime_, unsigned char messagePayload_[])
///////////////////////////////////////////////////////////////////////////////
//
// Re-establish data baseline.
///////////////////////////////////////////////////////////////////////////////
void DecodeWheelTorque_Resync(double dCurrentTime_, unsigned char messagePayload_[])
{
    unsigned short usCurrentAccumTorque;
    unsigned short usCurrentAccumPeriod;
    // CurrentRecordEpoch is the last time that we should have had a data record.
    double dCurrentRecordEpoch = (floor(dCurrentTime_ / dRecordInterval)) * dRecordInterval;

    if ((stState.dLastRecordTime != 0) &&
        (dCurrentRecordEpoch - stState.dLastRecordTime > 0) &&
        (dCurrentRecordEpoch - stState.dLastRecordTime < MAXIMUM_TIME_GAP))
    {
        stState.ucRecordGapCount = (unsigned char)(dCurrentRecordEpoch - stState.dLastRecordTime + dRecordInterval * 0.5)
            / dRecordInterval;

        // Transfer the accumulated data to the gap.
        stState.fGapEnergy = stState.fAccumEnergy;
        stState.fGapRotation = stState.fAccumRotation;

        // We need to fill in the gap with records.
        RecordOutput_FillGap(prrPtr, &stState);
    }

    usCurrentAccumPeriod = messagePayload_[ACCUM_PERIOD_LSB];
    usCurrentAccumPeriod += ((unsigned short)messagePayload_[ACCUM_PERIOD_MSB]) << 8;

    usCurrentAccumTorque = messagePayload_[ACCUM_TORQUE_LSB];
    usCurrentAccumTorque += ((unsigned short)messagePayload_[ACCUM_TORQUE_MSB]) << 8;

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

    stState.usLastAccumTorque = usCurrentAccumTorque;
    stState.usLastAccumPeriod = usCurrentAccumPeriod;
    stState.ucLastRotationTicks = messagePayload_[WHEEL_TICKS_BYTE];
    stState.ucLastEventCount = messagePayload_[UPDATE_EVENT_BYTE];
}

///////////////////////////////////////////////////////////////////////////////
// void DecodeWheelTorque(double dTime_, unsigned char messagePayload_[])
///////////////////////////////////////////////////////////////////////////////
// This is the main decoding function for wheel torque messages.
// Emphasis is placed at this point on handling the specific data
// that comes from the Powertap system since it's the only commercial
// wheel torque power meter.
//
// For time based systems we split the wheel period and the power period,
// since the wheel rate is the reciprocal of the wheel period, but the
// power period is the timebase value.
//
// There is a further (compile-time) split in here to enable the output
// of the averaged cadence OR the wheel rotation rate. Compatibility with
// the other power meter outputs suggests that cadence output is preferable.
// The wheel output can be handled by a separate decoder or eventually
// as a special case... code is left here to illustrate the general method.
///////////////////////////////////////////////////////////////////////////////
void DecodeWheelTorque(double dTime_, unsigned char messagePayload_[])
{
    unsigned long ulNewEventTime;
    unsigned long ulEventWheelRPM;
    unsigned long ulEventPower;
    unsigned short usCurrentAccumTorque;
    unsigned short usCurrentAccumPeriod;
    unsigned short usDeltaTorque;
    unsigned short usDeltaPeriod;
    unsigned short usDeltaPowerPeriod;
    unsigned char ucDeltaEventCount;
    unsigned char ucDeltaTicks;
    float fEventEnergy;

    usCurrentAccumPeriod = messagePayload_[ACCUM_PERIOD_LSB];
    usCurrentAccumPeriod += ((unsigned short)messagePayload_[ACCUM_PERIOD_MSB]) << 8;

    usCurrentAccumTorque = messagePayload_[ACCUM_TORQUE_LSB];
    usCurrentAccumTorque += ((unsigned short)messagePayload_[ACCUM_TORQUE_MSB]) << 8;

    usDeltaTorque = usCurrentAccumTorque - stState.usLastAccumTorque; // make sure this is done in 16 bit word width!
    usDeltaPeriod = usCurrentAccumPeriod - stState.usLastAccumPeriod; // make sure this is done in 16 bit word width!
    usDeltaPowerPeriod = usDeltaPeriod;

    stState.ucCadence = messagePayload_[INST_CADENCE_BYTE];

    ucDeltaEventCount = messagePayload_[UPDATE_EVENT_BYTE] - stState.ucLastEventCount;
    ucDeltaTicks = messagePayload_[WHEEL_TICKS_BYTE] - stState.ucLastRotationTicks;
    if (ucDeltaTicks > 200)
    {
        // Unlikely to be right...
        ucDeltaTicks = 0;
    }

    // 65535 is an invalid value.
    if (usDeltaTorque == 65535)
    {
        usDeltaTorque = 0;
    }

    if (usDeltaPeriod && (usDeltaPeriod != 0xFFFF))
    {
        ulEventPower = ((long)(M_PI*2048.0 + 0.5) * usDeltaTorque / usDeltaPeriod + 8) >> 4;

        if (stState.usTimeBase != 0)
        {
            // time based messages.
            ulNewEventTime = stState.ulEventTime + (unsigned long)stState.usTimeBase * ucDeltaEventCount;

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
            usDeltaPowerPeriod = stState.usTimeBase;
            fEventEnergy = (float)ulEventPower;
            // the reported data reflects one revolution for each message update.
#if defined (PROPAGATE_CADENCE)
            if (stState.ucCadence)
            {
                ucDeltaTicks = ucDeltaEventCount;
            }
            else
            {
                ucDeltaTicks = 0;
            }
#else
            ucDeltaTicks = ucDeltaEventCount;
#endif
        }
        else
        {
            // event based messages
            ulNewEventTime = stState.ulEventTime + (unsigned long)usDeltaPeriod;
            fEventEnergy = (float)(M_PI * (float)usDeltaTorque / 16.0);
        }

        // This is actually the wheel rotation speed.
        ulEventWheelRPM = ((long)ucDeltaTicks * 60L * WT_TIME_QUANTIZATION + (usDeltaPeriod >> 1)) / usDeltaPeriod;
    }
    else
    {
        // This is basically a non-event.
        ulEventPower = 0;
        ulEventWheelRPM = 0;
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
        stState.fPendingEnergy = stState.fAccumEnergy + fEventEnergy * ((float)(stState.usRecordInterval - (stState.ulEventTime % stState.usRecordInterval))) / ((float)usDeltaPowerPeriod);

        // accumulated energy goes towards the *next* event.
        stState.fAccumEnergy = fEventEnergy * ((float)(ulNewEventTime % stState.usRecordInterval)) / ((float)usDeltaPowerPeriod);

        // Gap energy fills the remainder.
        stState.fGapEnergy = fEventEnergy * ((unsigned short)stState.ucRecordGapCount * stState.usRecordInterval) / ((float)usDeltaPowerPeriod);

        //Same for rotation. Within this framework we can propagate either the wheel speed or the cycling cadence...
#if defined (PROPAGATE_CADENCE)
        stState.fPendingRotation = stState.fAccumRotation + (float)ucDeltaTicks * (float)(stState.ucCadence) / 60.0f * ((float)(stState.usRecordInterval - (stState.ulEventTime % stState.usRecordInterval))) / ((float)WT_TIME_QUANTIZATION);
        stState.fAccumRotation = (float)ucDeltaTicks * (float)(stState.ucCadence) / 60.0f * ((float)(ulNewEventTime % stState.usRecordInterval)) / ((float)WT_TIME_QUANTIZATION);
        stState.fGapRotation = (float)ucDeltaTicks * (float)(stState.ucCadence) / 60.0f * ((unsigned short)stState.ucRecordGapCount * stState.usRecordInterval) / ((float)WT_TIME_QUANTIZATION);
#else
        stState.fPendingRotation = stState.fAccumRotation + (float)ucDeltaTicks * ((float)(stState.usRecordInterval - (stState.ulEventTime % stState.usRecordInterval)))/((float)usDeltaPeriod);
        stState.fAccumRotation = (float)ucDeltaTicks * ((float)(ulNewEventTime % stState.usRecordInterval))/((float)usDeltaPeriod);
        stState.fGapRotation = (float)ucDeltaTicks * ((unsigned short)stState.ucRecordGapCount * stState.usRecordInterval) / ((float)usDeltaPeriod);
#endif
    }
    else
    {
        // This event came in before the next record epoch started - this
        // will happen when the event period is less than the recording period.
        stState.fAccumEnergy += fEventEnergy;
#if defined (PROPAGATE_CADENCE)
        stState.fAccumRotation += (float)ucDeltaTicks * (float)(stState.ucCadence) / 60.0f;
#else
        stState.fAccumRotation += (float)ucDeltaTicks;
#endif

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
                stState.dLastRecordTime += ((double)stState.usRecordInterval) / WT_TIME_QUANTIZATION;
                (prrPtr)(stState.dLastRecordTime, stState.dTotalRotation, stState.dTotalEnergy, 0.0, 0.0);
            }
        }
    }

    // Propagate the message state information.
    stState.ucLastEventCount = messagePayload_[UPDATE_EVENT_BYTE];
    stState.ucLastRotationTicks = messagePayload_[WHEEL_TICKS_BYTE];
    stState.usLastAccumPeriod = usCurrentAccumPeriod;
    stState.usLastAccumTorque = usCurrentAccumTorque;
}