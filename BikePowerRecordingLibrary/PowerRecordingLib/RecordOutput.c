/*
This software is subject to the license described in the License.txt file
included with this software distribution. You may not use this file except in compliance
with this license.

Copyright (c) Dynastream Innovations Inc. 2014
All rights reserved.
*/

#include "string.h"
#include "stdlib.h"
#include "math.h"

#include "PowerDecoder.h"
#include "RecordOutput.h"

static double dRecordInterval;

void ResamplerOutput_Init(BPSAMPLER *pstDecoder_, unsigned short usRecordInterval_, double dRecordInterval_, unsigned short usTimeBase_)
{
    pstDecoder_->ucCadence = 0;
    pstDecoder_->dTotalEnergy = 0;
    pstDecoder_->fAccumEnergy = 0;
    pstDecoder_->fPendingEnergy = 0;
    pstDecoder_->fGapEnergy = 0;

    pstDecoder_->dTotalRotation = 0;
    pstDecoder_->fAccumRotation = 0;
    pstDecoder_->fPendingRotation = 0;
    pstDecoder_->fGapRotation = 0;
    pstDecoder_->ulEventTime = 0;
    pstDecoder_->ucRecordGapCount = 0;
    pstDecoder_->dLastRecordTime = 0;
    pstDecoder_->dLastMessageTime = 0;

    pstDecoder_->usRecordInterval = usRecordInterval_;
    pstDecoder_->usTimeBase = usTimeBase_;

    dRecordInterval = dRecordInterval_;
}

///////////////////////////////////////////////////////////////////////
// void RecordOutput(FILE *pfOut_, BPRESAMPLER *pstDecoder_)
///////////////////////////////////////////////////////////////////////
//
// This function pushes output records to catch up to the latest event.
// It also updates the state as required.
//
///////////////////////////////////////////////////////////////////////
void RecordOutput(PowerRecordReceiver powerRecordReceiverPtr_, BPSAMPLER *pstDecoder_)
{
    // Calculate average power and cadence over the recording interval.
    float fAveragePower = (float)(pstDecoder_->fPendingEnergy / dRecordInterval);
    float fAverageCadence = (float)(pstDecoder_->fPendingRotation * 60.0 / dRecordInterval);

    pstDecoder_->dTotalEnergy += pstDecoder_->fPendingEnergy;
    pstDecoder_->dTotalRotation += pstDecoder_->fPendingRotation;
    pstDecoder_->dLastRecordTime += dRecordInterval;
    pstDecoder_->ulLastRecordTime = (pstDecoder_->ulEventTime / pstDecoder_->usRecordInterval)*pstDecoder_->usRecordInterval;

    (*powerRecordReceiverPtr_)(pstDecoder_->dLastRecordTime, pstDecoder_->dTotalRotation, pstDecoder_->dTotalEnergy, fAverageCadence, fAveragePower);

    // If there was any recovered message outage, fill in here.
    RecordOutput_FillGap(powerRecordReceiverPtr_, pstDecoder_);
}

///////////////////////////////////////////////////////////////////////
// double RecordOutput_FillGap(FILE *pfOut_, BPRESAMPLER *pstDecoder_)
///////////////////////////////////////////////////////////////////////
//
// This function is called to fill the data record with energy/
// rotation that's evidently occurred during a message gap.
// If the gap is _too_ long then we shouldn't do this because
// otherwise it could cause some pretty huge files to be generated.
//
///////////////////////////////////////////////////////////////////////
void RecordOutput_FillGap(PowerRecordReceiver powerRecordReceiverPtr_, BPSAMPLER *pstDecoder_)
{
    int i;
    float fIncEnergy;
    float fIncRotation;
    float fAveragePower;
    float fAverageCadence;

    if (pstDecoder_->ucRecordGapCount > 0)
    {
        fIncEnergy = (float)(pstDecoder_->fGapEnergy / pstDecoder_->ucRecordGapCount);
        fIncRotation = (float)(pstDecoder_->fGapRotation / pstDecoder_->ucRecordGapCount);
        // These two things are broken out here for clarity.
        // With respect to the output generation they can simply be combined.
        fAveragePower = fIncEnergy / dRecordInterval;
        fAverageCadence = fIncRotation * 60.0f / dRecordInterval;

        for (i = 0; i < pstDecoder_->ucRecordGapCount; i++)
        {
            pstDecoder_->dTotalEnergy += fIncEnergy;
            pstDecoder_->dTotalRotation += fIncRotation;
            pstDecoder_->dLastRecordTime += dRecordInterval;
            (*powerRecordReceiverPtr_)(pstDecoder_->dLastRecordTime, pstDecoder_->dTotalRotation, pstDecoder_->dTotalEnergy, fAverageCadence, fAveragePower);
        }

        pstDecoder_->ucRecordGapCount = 0;
    }
}
