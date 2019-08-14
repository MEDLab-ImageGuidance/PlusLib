/*=Plus=header=begin======================================================
Program: Plus
Copyright (c) Laboratory for Percutaneous Surgery. All rights reserved.
See License.txt for details.
=========================================================Plus=header=end*/

#ifndef _INTUITIVE_DAVINCIXI_H_
#define _INTUITIVE_DAVINCIXI_H_

#include "PlusConfigure.h"
#include "IntuitiveDaVinciManipulatorXi.h"

#include "isi_api_types.h"

class IntuitiveDaVinciXi
{
public:
  /*! Constructor. */
  IntuitiveDaVinciXi();

  /*! Destructor. */
  ~IntuitiveDaVinciXi();

  /*! Start streaming from the da Vinci API system */
  ISI_STATUS Start();
  ISI_STATUS StartDebugSineWaveMode();

  /*! Stop streaming */
  void Stop();

  /*! Make a request to connect to the da Vinci */
  ISI_STATUS Connect();
  ISI_STATUS ConnectDebugSineWaveMode();

  /*! Make a request to disconnect from the da Vinci Xi */
  void Disconnect();

  /*! Accessor for connected state. */
  bool IsConnected() const;

  /*! Accessor for streaming state. */
  bool IsStreaming() const;
  
  /*! Accessor for each of the manipulators. */
  IntuitiveDaVinciManipulatorXi* GetUsm1() const;
  IntuitiveDaVinciManipulatorXi* GetUsm2() const;
	IntuitiveDaVinciManipulatorXi* GetUsm3() const;
	IntuitiveDaVinciManipulatorXi* GetUsm4() const;
  IntuitiveDaVinciManipulatorXi* GetEcm() const;

  /*! Accessor for each of the manipulator base transforms. */
  ISI_TRANSFORM* GetUsm1BaseToWorld() const;
  ISI_TRANSFORM* GetUsm2BaseToWorld() const;
	ISI_TRANSFORM* GetUsm3BaseToWorld() const;
	ISI_TRANSFORM* GetUsm4BaseToWorld() const;
  ISI_TRANSFORM* GetEcmBaseToWorld() const;

  /*! Update joint values using the da Vinci Xi API for all of the manipulators. */
	ISI_STATUS UpdateAllJointValues();

  /*! Update joint values using sine functions for debugging purposes. */
  ISI_STATUS UpdateAllJointValuesSineWave();

  /*! Print all of the joint values for all of the manipulators. */
  void PrintAllJointValues() const;

  /*! Get all of the joint values as a string. */
  std::string GetAllJointValuesAsString() const;

  /*! Print all of the kinematics transforms for all of the manipulators. */
  void PrintAllKinematicsTransforms() const;

  /*! Update all of the base frames of the manipulators using the da Vinci Xi API. */
  void UpdateBaseToWorldTransforms();

  /*! Update every transform for each DH row in the kinematic chain. */
  ISI_STATUS UpdateAllKinematicsTransforms();

  /*! Copy data from one ISI_TRANSFORM to another. */
  static void CopyIsiTransform(ISI_TRANSFORM* srcTransform, ISI_TRANSFORM* destTransform);

protected:
  /*! Variables for storing the state of the da Vinci Xi API. */
  ISI_STATUS mStatus; // An integer, error if != 0
  bool mConnected;
  bool mStreaming;
  unsigned int mRateHz; // Rate of data streaming from the da Vinci Xi system

  /*! The intuitive da Vinci Xi will have four manipulators: four USMs. Any of the manipulators could be used as ECM. */
	IntuitiveDaVinciManipulatorXi* mUsm1;
  IntuitiveDaVinciManipulatorXi* mUsm2;
	IntuitiveDaVinciManipulatorXi* mUsm3;
	IntuitiveDaVinciManipulatorXi* mUsm4;
  IntuitiveDaVinciManipulatorXi* mEcm;

  /*! We also want to track all of the manipulator base frames. */
  ISI_TRANSFORM* mUsm1BaseToWorld;
  ISI_TRANSFORM* mUsm2BaseToWorld;
	ISI_TRANSFORM* mUsm3BaseToWorld;
	ISI_TRANSFORM* mUsm4BaseToWorld;
  ISI_TRANSFORM* mEcmBaseToWorld;

  /*! These are some intermediate variables needed for computation of the base frames poses. */
  ISI_TRANSFORM* mViewToWorld;
  ISI_TRANSFORM* mUsm1BaseToView;
  ISI_TRANSFORM* mUsm2BaseToView;
	ISI_TRANSFORM* mUsm3BaseToView;
	ISI_TRANSFORM* mUsm4BaseToView;

  /*! Python object to run Python scripts. */
  PyObject* pName;
  PyObject* pModule;
  PyObject* pDict;
  PyObject* pClass;
  PyObject* pInstance;
  PyObject* pValue;

};

#endif
