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

#include <Python.h>

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

	/*! Get Python list from Xi API and convert it to an C++ array. */
	ISI_FLOAT* IntuitiveDaVinciXi::GetJointValuesFromPy(IXI_MANIP_INDEX mIxiManipIndex);

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

  /*! Python object to run Python scripts. */
  PyObject* pName;
  PyObject* pModule;
  PyObject* pDict;
  PyObject* pClass;
  PyObject* pInstance;
  PyObject* pValue;

};

#endif
