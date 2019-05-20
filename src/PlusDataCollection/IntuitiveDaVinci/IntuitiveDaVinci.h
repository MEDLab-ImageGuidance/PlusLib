/*=Plus=header=begin======================================================
Program: Plus
Copyright (c) Laboratory for Percutaneous Surgery. All rights reserved.
See License.txt for details.
=========================================================Plus=header=end*/

#include "PlusConfigure.h"

#ifndef _INTUITIVE_DAVINCI_H_
#define _INTUITIVE_DAVINCI_H_

#ifdef WIN32
  #define _CRT_SECURE_NO_WARNINGS
  #include <windows.h>
#endif

#include "IntuitiveDaVinciManipulator.h"

// Intuitive includes
#include <isi_api_types.h>
#include <dv_api.h>

#define ISI_FAIL 0x0001 // IntuitiveDaVinci expects this for some reason, not provided by isi_types.h

class IntuitiveDaVinci
{
public:
  // Constructor
  IntuitiveDaVinci();

  // Destructor
  ~IntuitiveDaVinci();

  // Start streaming from the da Vinci
  bool start();

  // Stop streaming from the da Vinci
  void stop();

  // Make a request to connect to the da Vinci
  ISI_STATUS connect();

  // Make a request to disconnect from the da Vinci
  ISI_STATUS disconnect();

  // Print out the 6DOF from the given transform.
  void printTransform(const ISI_TRANSFORM* T);

  // Added. Accessor for connected state.
  bool isConnected();
  bool isStreaming();
  
  IntuitiveDaVinciManipulator* GetPsm1();
  IntuitiveDaVinciManipulator* GetPsm2();
  IntuitiveDaVinciManipulator* GetEcm();

  ISI_TRANSFORM* GetPsm1BaseToWorld();
  ISI_TRANSFORM* GetPsm2BaseToWorld();
  ISI_TRANSFORM* GetEcmBaseToWorld();

  // Update joints
  ISI_STATUS UpdateAllJointValues();
  ISI_STATUS UpdateAllJointValuesSineWave();
  void PrintAllJointValues();
  void PrintAllKinematicsTransforms();

  // Run kinematics
  ISI_STATUS UpdateBaseToWorldTransforms();
  ISI_STATUS UpdateAllKinematicsTransforms();

protected:
  void copyTransform(ISI_TRANSFORM* in, ISI_TRANSFORM* out);

protected:
  ISI_STATUS        mStatus;
  bool              mConnected;
  bool              mStreaming;
  unsigned int      mRateHz;

  ISI_TRANSFORM*    mPsm1BaseToWorld;
  ISI_TRANSFORM*    mPsm2BaseToWorld;
  ISI_TRANSFORM*    mEcmBaseToWorld;

  ISI_TRANSFORM*    mViewToWorld;
  ISI_TRANSFORM*    mPsm1BaseToView;
  ISI_TRANSFORM*    mPsm2BaseToView;

  IntuitiveDaVinciManipulator* mPsm1;
  IntuitiveDaVinciManipulator* mPsm2;
  IntuitiveDaVinciManipulator* mEcm;
};

#endif
