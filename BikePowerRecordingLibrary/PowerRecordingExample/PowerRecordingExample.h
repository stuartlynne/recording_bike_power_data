/*
This software is subject to the license described in the License.txt file
included with this software distribution. You may not use this file except in compliance
with this license.

Copyright (c) Dynastream Innovations Inc. 2014
All rights reserved.
*/

#ifndef _EXAMPLE_H_
#define _EXAMPLE_H_

#include "types.h"
#include "dsi_framer_ant.hpp"
#include "dsi_thread.h"
#include "dsi_serial_generic.hpp"

#define CHANNEL_TYPE_MASTER   (0)
#define CHANNEL_TYPE_SLAVE		(1)
#define CHANNEL_TYPE_INVALID	(2)



class Example {
public:
    Example();
    virtual ~Example();
    BOOL Init(UCHAR ucDeviceNumber_, UCHAR ucChannelType_, USHORT usAntDeviceNumber_, DOUBLE dRecordInterval_, DOUBLE dTimeBase_, UCHAR ucPowerMeterType_, DOUBLE dReSyncInterval_);
    void Start();
    void Close();

private:
    BOOL InitANT();

    //Starts the Message thread.
    static DSI_THREAD_RETURN RunMessageThread(void *pvParameter_);

    //Listens for a response from the module
    void MessageThread();
    //Decodes the received message
    void ProcessMessage(ANT_MESSAGE stMessage, USHORT usSize_);

    //Receiver for the power records from the power decoder
    static void RecordReceiver(double dLastRecordTime_, double dTotalRotation_, double dTotalEnergy_, float fAverageCadence_, float fAveragePower_);

    //Receiver for Torque Effectiveness/Pedal Smoothness data
    void Example::TePsReceiver(double dRxTime_, float fLeftTorqEff_, float fRightTorqEff_, float fLeftOrCPedSmth_, float fRightPedSmth_);

    //Receiver for Power Balance data
    void Example::PowerBalanceReceiver(double dRxTime_, float fPowerBalance_, bool bPowerBalanceRightPedalIndicator_);

    // print user menu
    void PrintMenu();

    BOOL bBursting; //holds whether the bursting phase of the test has started
    BOOL bBroadcasting;
    BOOL bMyDone;
    BOOL bDone;
    BOOL bPowerDecoderInitialized;
    UCHAR ucChannelType;
    USHORT usAntDeviceNumber;
    DOUBLE dRecordInterval;
    DOUBLE dTimeBase;
    DOUBLE dReSyncInterval;
    DSISerialGeneric* pclSerialObject;
    DSIFramerANT* pclMessageObject;
    DSI_THREAD_ID uiDSIThread;
    DSI_CONDITION_VAR condTestDone;
    DSI_MUTEX mutexTestDone;
    time_t previousRxTime;
    UCHAR ucPowerOnlyUpdateEventCount;
    DOUBLE dRxTimeTePs;

    BOOL bDisplay;

    UCHAR aucTransmitBuffer[ANT_STANDARD_DATA_PAYLOAD_SIZE];

    unsigned long ulNewEventTime;
    unsigned short usPreviousEventTime;
};

#endif
