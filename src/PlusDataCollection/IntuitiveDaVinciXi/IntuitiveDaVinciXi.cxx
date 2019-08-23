/*=Plus=header=begin======================================================
Program: Plus
Copyright (c) Laboratory for Percutaneous Surgery. All rights reserved.
See License.txt for details.
=========================================================Plus=header=end*/

#include <cmath>
#include <time.h>

#include "PlusConfigure.h"

#include "dv_api.h"
#include "dv_api_math.h"

#include "IntuitiveDaVinciXi.h"

//----------------------------------------------------------------------------
IntuitiveDaVinciXi::IntuitiveDaVinciXi()
  : mStatus(ISI_SUCCESS)
  , mConnected(false)
  , mStreaming(false)
  , mRateHz(60)
{
  mUsm1 = new IntuitiveDaVinciManipulatorXi(IXI_USM1);
	mUsm2 = new IntuitiveDaVinciManipulatorXi(IXI_USM2);
	mUsm3 = new IntuitiveDaVinciManipulatorXi(IXI_USM3);
	mUsm4 = new IntuitiveDaVinciManipulatorXi(IXI_USM4);

  pName = new PyObject;
  pModule = new PyObject;
  pClass = new PyObject;
  pDict = new PyObject;
  pInstance = new PyObject;
  pValue = new PyObject;
  
  LOG_DEBUG("Created da Vinci Xi.");
}

//----------------------------------------------------------------------------
IntuitiveDaVinciXi::~IntuitiveDaVinciXi()
{
	delete mUsm1, mUsm2, mUsm3, mUsm4;
	mUsm1 = nullptr; mUsm2 = nullptr; mUsm3 = nullptr; mUsm4 = nullptr; 
		
	// Deleting Python API classes.
	delete pName, pModule, pClass, pDict, pInstance, pValue;
	pName = nullptr; pModule = nullptr; pClass = nullptr; pDict = nullptr;
	pInstance = nullptr; pValue = nullptr;

  this->Stop();
  this->Disconnect();

  LOG_DEBUG("Destroyed da Vinci Xi.");
}

ISI_STATUS IntuitiveDaVinciXi::Connect()
{
	bool pyObjectCheck;

	LOG_DEBUG("Connecting to da Vinci Xi API.");

	if (this->IsConnected())
	{
		LOG_WARNING("Cannot connect to da Vinci Xi API because already connected.");
		return mStatus;
	}

	if (this->IsStreaming())
	{
		LOG_WARNING("Cannot connect to da Vinci Xi API because currently streaming data from it.");
		return mStatus;
	}

	// Start Python interpreter
	Py_Initialize();

	pName = PyUnicode_DecodeFSDefault("DaVinciXiApi");
	pModule = PyImport_Import(pName);

	// get the class 
	pDict = PyModule_GetDict(pModule);
	pClass = PyDict_GetItemString(pDict, "DaVinciXiApi");
	pInstance = PyObject_CallObject(pClass, NULL);

	// get the result value
	pValue = PyObject_CallMethod(pInstance, "connect", NULL, NULL);

	pyObjectCheck = PyObject_IsTrue(pValue);

	if (pyObjectCheck) 
	{
		mConnected = true;
		LOG_INFO("Connected to da Vinci system.");
		mStatus = ISI_SUCCESS;
	}
	else
	{
		LOG_ERROR("Could not connect to da Vinci Xi system.");
		mStatus = ISI_UNKNOWN_ERROR;
	}

	return mStatus;
}

//----------------------------------------------------------------------------
ISI_STATUS IntuitiveDaVinciXi::ConnectDebugSineWaveMode()
{
  LOG_WARNING("Connected to a debug version of the da Vinci Xi API. Broadcasting only default values for da Vinci Xi base frames.");
  mConnected = true;
  return mStatus;
}

ISI_STATUS IntuitiveDaVinciXi::Start()
{
	bool pyObjectCheck;

	LOG_DEBUG("Starting data stream from da Vinci Xi API.");

	if (this->IsStreaming())
	{
		LOG_WARNING("Will not attempt to start streaming because da Vinci Xi API already streaming.");
		return mStatus;
	}

	if (!this->IsConnected())
	{
		LOG_WARNING("Not connected, so cannot start streaming.");
		return mStatus;
	}

	// get the result value
	pValue = PyObject_CallMethod(pInstance, "startStream", NULL, mRateHz);

	pyObjectCheck = PyObject_IsTrue(pValue);

	if (pyObjectCheck)
	{
		mStreaming = true;
		LOG_DEBUG("Data stream started.");
		mStatus = ISI_SUCCESS;
	}
	else
	{
		LOG_ERROR("Could not start da Vinci Xi data stream.");
		mStatus = ISI_UNKNOWN_ERROR;
	}

	return mStatus;
}

//----------------------------------------------------------------------------
ISI_STATUS IntuitiveDaVinciXi::StartDebugSineWaveMode()
{
  LOG_DEBUG("Started sine wave debug data stream.");
  mStreaming = true;
  return mStatus;
}

void IntuitiveDaVinciXi::Stop()
{
	LOG_DEBUG("Stopping data stream from da Vinci Xi API.");

	if (this->IsConnected())
	{
		LOG_WARNING("Cannot stop da Vinci Xi stream until disconnected.");
		return;
	}

	if (!this->IsStreaming())
	{
		LOG_WARNING("Stop called, but da Vinci Xi is already stopped.");
		return;
	}

	PyObject_CallMethod(pInstance, "stopStream", NULL, NULL);

	mStreaming = false;
	LOG_DEBUG("Streaming from the da Vinci Xi API stopped.");
}

void IntuitiveDaVinciXi::Disconnect()
{
	// check if system is connected
	if (!this->IsConnected())
	{
		LOG_WARNING("Disconnect cannot be called because not connected.");
		return;
	}

	if (this->IsStreaming())
	{
		LOG_WARNING("You must stop streaming before attempting to disconnect.");
		return;
	}

	PyObject_CallMethod(pInstance, "disconnect", NULL, NULL);

	Py_Finalize();
	mConnected = false;

	LOG_DEBUG("Disconnected from the da Vinci Xi API.");
}

//----------------------------------------------------------------------------
bool IntuitiveDaVinciXi::IsConnected() const
{
  return mConnected;
}

//----------------------------------------------------------------------------
bool IntuitiveDaVinciXi::IsStreaming() const
{
  return mStreaming;
}

ISI_FLOAT* IntuitiveDaVinciXi::GetJointValuesFromPy(IXI_MANIP_INDEX mIxiManipIndex)
{
	PyObject *pList;
	ISI_FLOAT outputArray[IXI_NUM_USM_JOINTS];
	ISI_FLOAT tmpArray[IXI_NUM_USM_JOINT_VALS];
	

	pList = PyList_New(IXI_NUM_USM_JOINT_VALS);
	pList = PyObject_CallMethod(pInstance, "getUsmJointValues", NULL, mIxiManipIndex);

	if (pList != NULL)
	{
		for (int iii = 0; iii < IXI_NUM_USM_JOINT_VALS; iii++)
		{
			tmpArray[iii] = PyLong_AsLong(PyList_GetItem(pList, iii));
			if (iii == 3) tmpArray[iii] *= 1000.0; // because of prismatic joint to convert mm
		}

		/* Arrange array again because of parellelogram of links 2,3 and 4. See Xi Kinematics */
		outputArray[0] = tmpArray[0];
		outputArray[1] = tmpArray[1];
		outputArray[2] = -(tmpArray[2]);
		outputArray[3] = tmpArray[2];
		outputArray[4] = tmpArray[2];
		outputArray[5] = tmpArray[3];
		outputArray[6] = tmpArray[4];
		outputArray[7] = tmpArray[5];
		outputArray[8] = tmpArray[6];
		outputArray[9] = tmpArray[7];
	}
	else
	{
		LOG_ERROR("Could not update the da Vinci Xi manipulator joint values.");
	}
	
	return outputArray;
}

ISI_STATUS IntuitiveDaVinciXi::UpdateAllJointValues()
{

	/*! Hold the current joint values of the manipulators. */
	ISI_FLOAT* usm1JointValues;
	ISI_FLOAT* usm2JointValues;
	ISI_FLOAT* usm3JointValues;
	ISI_FLOAT* usm4JointValues;

	usm1JointValues = GetJointValuesFromPy(IXI_USM1);

	mStatus = this->mUsm1->UpdateJointValues(usm1JointValues);

	if (mStatus != ISI_SUCCESS)
	{
		LOG_ERROR("Could not update the USM1 joint values with the da Vinci API.");
		return mStatus;
	}

	usm2JointValues = GetJointValuesFromPy(IXI_USM2);

	mStatus = this->mUsm2->UpdateJointValues(usm2JointValues);

	if (mStatus != ISI_SUCCESS)
	{
		LOG_ERROR("Could not update the USM2 joint values with the da Vinci API.");
		return mStatus;
	}

	usm3JointValues = GetJointValuesFromPy(IXI_USM3);

	mStatus = this->mUsm3->UpdateJointValues(usm3JointValues);

	if (mStatus != ISI_SUCCESS)
	{
		LOG_ERROR("Could not update the USM3 joint values with the da Vinci API.");
		return mStatus;
	}

	usm4JointValues = GetJointValuesFromPy(IXI_USM4);

	mStatus = this->mUsm4->UpdateJointValues(usm4JointValues);

	if (mStatus != ISI_SUCCESS)
	{
		LOG_ERROR("Could not update the USM4 joint values with the da Vinci API.");
		return mStatus;
	}

	return mStatus;
}

ISI_STATUS IntuitiveDaVinciXi::UpdateAllJointValuesSineWave()
{
	clock_t ticks = clock();
	float t = ((float)ticks) / ((float)CLOCKS_PER_SEC);

	ISI_FLOAT usm1JointValues[IXI_NUM_USM_JOINTS] = { 0.5*sin(1.0*t), 0.5*sin(1.5*t), -sin(1.7*t), sin(1.7*t), sin(1.7*t), 50.0*sin(2.0*t) + 75.0, sin(0.7*t), sin(0.5*t), sin(0.8*t), sin(0.5*t)};
	ISI_FLOAT usm2JointValues[IXI_NUM_USM_JOINTS] = { 0.5*sin(1.1*t), 0.5*sin(1.4*t), -sin(1.6*t), sin(1.6*t), sin(1.6*t), 50.0*sin(2.1*t) + 75.0, sin(0.6*t), sin(0.9*t), sin(1.8*t), sin(0.5*t)};
	ISI_FLOAT usm3JointValues[IXI_NUM_USM_JOINTS] = { 0.5*sin(0.9*t), 0.5*sin(1.6*t), -sin(1.8*t), sin(1.8*t), sin(1.8*t), 50.0*sin(1.9*t) + 75.0, sin(0.8*t), sin(0.8*t), sin(1.6*t), sin(0.5*t)};
	ISI_FLOAT usm4JointValues[IXI_NUM_USM_JOINTS] = { 0.0, 0.0, 0.5*sin(0.9*t), 0.0, 0.0, 0.0, sin(1.7*t), sin(0.7*t), sin(0.5*t), sin(0.8*t) };

	this->mUsm1->SetJointValues(usm1JointValues);
	this->mUsm2->SetJointValues(usm2JointValues);
	this->mUsm3->SetJointValues(usm3JointValues);
	this->mUsm4->SetJointValues(usm4JointValues);

	return mStatus;
}

void IntuitiveDaVinciXi::PrintAllJointValues() const
{
	std::string tmp0 = this->mUsm1->GetJointValuesAsString();
	LOG_DEBUG("USM1 Joint Values: " << tmp0);
	std::string tmp1 = this->mUsm2->GetJointValuesAsString();
	LOG_DEBUG("USM2 Joint Values: " << tmp1);
	std::string tmp2 = this->mUsm3->GetJointValuesAsString();
	LOG_DEBUG("USM3 Joint Values: " << tmp2);
	std::string tmp3 = this->mUsm4->GetJointValuesAsString();
	LOG_DEBUG("USM4 Joint Values: " << tmp3);
}

std::string IntuitiveDaVinciXi::GetAllJointValuesAsString() const
{
	std::stringstream str;
	str << "USM1: " << this->mUsm1->GetJointValuesAsString() << '\n';
	str << "USM2: " << this->mUsm2->GetJointValuesAsString() << '\n';
	str << "USM3: " << this->mUsm3->GetJointValuesAsString() << '\n';
	str << "USM4: " << this->mUsm4->GetJointValuesAsString() << '\n';

	return str.str();
}

ISI_STATUS IntuitiveDaVinciXi::UpdateAllKinematicsTransforms()
{

	mStatus = this->mUsm1->UpdateLinkTransforms();

	if (mStatus != ISI_SUCCESS)
	{
		LOG_ERROR("Error updating USM1 manipulator transforms.");
		return mStatus;
	}

	mStatus = this->mUsm2->UpdateLinkTransforms();

	if (mStatus != ISI_SUCCESS)
	{
		LOG_ERROR("Error updating USM2 manipulator transforms.");
		return mStatus;
	}

	mStatus = this->mUsm3->UpdateLinkTransforms();

	if (mStatus != ISI_SUCCESS)
	{
		LOG_ERROR("Error updating USM1 manipulator transforms.");
		return mStatus;
	}

	mStatus = this->mUsm4->UpdateLinkTransforms();

	if (mStatus != ISI_SUCCESS)
	{
		LOG_ERROR("Error updating USM1 manipulator transforms.");
		return mStatus;
	}

	return mStatus;
}

void IntuitiveDaVinciXi::PrintAllKinematicsTransforms() const
{
	std::string tmp0 = this->mUsm1->GetTransformsAsString();
	LOG_DEBUG("USM1 Kinematics Transforms: " << tmp0);
	std::string tmp1 = this->mUsm2->GetTransformsAsString();
	LOG_DEBUG("USM2 Kinematics Transforms: " << tmp1);
	std::string tmp2 = this->mUsm3->GetTransformsAsString();
	LOG_DEBUG("USM3 Kinematics Transforms: " << tmp2);
	std::string tmp3 = this->mUsm4->GetTransformsAsString();
	LOG_DEBUG("USM4 Kinematics Transforms: " << tmp3);
}

//----------------------------------------------------------------------------
void IntuitiveDaVinciXi::CopyIsiTransform(ISI_TRANSFORM* in, ISI_TRANSFORM* out)
{
  if (in == NULL || out == NULL) { return; }

  out->pos.x = in->pos.x;
  out->pos.y = in->pos.y;
  out->pos.z = in->pos.z;

  out->rot.row0 = in->rot.row0;
  out->rot.row1 = in->rot.row1;
  out->rot.row2 = in->rot.row2;
}

//----------------------------------------------------------------------------
IntuitiveDaVinciManipulatorXi* IntuitiveDaVinciXi::GetUsm1() const
{
  return this->mUsm1;
}

//----------------------------------------------------------------------------
IntuitiveDaVinciManipulatorXi* IntuitiveDaVinciXi::GetUsm2() const
{
	return this->mUsm2;
}

IntuitiveDaVinciManipulatorXi* IntuitiveDaVinciXi::GetUsm3() const
{
	return this->mUsm3;
}

IntuitiveDaVinciManipulatorXi* IntuitiveDaVinciXi::GetUsm4() const
{
	return this->mUsm4;
}

