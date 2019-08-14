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
	mEcm = new IntuitiveDaVinciManipulatorXi(IXI_ECM);

  mUsm1BaseToWorld = new ISI_TRANSFORM;
  mUsm2BaseToWorld = new ISI_TRANSFORM;
	mUsm3BaseToWorld = new ISI_TRANSFORM;
	mUsm4BaseToWorld = new ISI_TRANSFORM;
  mEcmBaseToWorld = new ISI_TRANSFORM;

  mViewToWorld = new ISI_TRANSFORM;
  mUsm1BaseToView = new ISI_TRANSFORM;
	mUsm2BaseToView = new ISI_TRANSFORM;
	mUsm3BaseToView = new ISI_TRANSFORM;
	mUsm4BaseToView = new ISI_TRANSFORM;

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
	delete mUsm1, mUsm2, mUsm3, mUsm4, mEcm;
	mUsm1 = nullptr; mUsm2 = nullptr; mUsm3 = nullptr; mUsm4 = nullptr; mEcm = nullptr;

	delete mUsm1BaseToWorld, mUsm2BaseToWorld, mUsm3BaseToWorld, mUsm4BaseToWorld, mEcmBaseToWorld;
	mUsm1BaseToWorld = nullptr; mUsm2BaseToWorld = nullptr; mUsm3BaseToWorld = nullptr;
	mUsm4BaseToWorld = nullptr; mEcmBaseToWorld = nullptr;

	delete mViewToWorld, mUsm1BaseToView, mUsm2BaseToView, mUsm3BaseToView, mUsm4BaseToView;
	mViewToWorld = nullptr; mUsm1BaseToView = nullptr; mUsm2BaseToView = nullptr;
	mUsm3BaseToView = nullptr; mUsm4BaseToView = nullptr;
	
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

	if (PyObject_IsTrue(pValue) == true)
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

	if (PyObject_IsTrue(pValue) == true)
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

ISI_STATUS IntuitiveDaVinciXi::UpdateAllJointValues()
{
	mStatus = this->mUsm1->UpdateJointValues();

	if (mStatus != ISI_SUCCESS)
	{
		LOG_ERROR("Could not update the USM1 joint values with the da Vinci API.");
		return mStatus;
	}

	mStatus = this->mUsm2->UpdateJointValues();

	if (mStatus != ISI_SUCCESS)
	{
		LOG_ERROR("Could not update the USM2 joint values with the da Vinci API.");
		return mStatus;
	}

	mStatus = this->mUsm3->UpdateJointValues();

	if (mStatus != ISI_SUCCESS)
	{
		LOG_ERROR("Could not update the USM3 joint values with the da Vinci API.");
		return mStatus;
	}

	mStatus = this->mUsm4->UpdateJointValues();

	if (mStatus != ISI_SUCCESS)
	{
		LOG_ERROR("Could not update the USM4 joint values with the da Vinci API.");
		return mStatus;
	}

	mStatus = this->mEcm->UpdateJointValues();

	if (mStatus != ISI_SUCCESS)
	{
		LOG_ERROR("Could not update the ECM joint values with the da Vinci API.");
		return mStatus;
	}

	return mStatus;
}

ISI_STATUS IntuitiveDaVinciXi::UpdateAllJointValuesSineWave()
{
	clock_t ticks = clock();
	float t = ((float)ticks) / ((float)CLOCKS_PER_SEC);

	ISI_FLOAT usm1JointValues[IXI_NUM_USM_JOINTS] = { 0.5*sin(1.0*t), 0.5*sin(1.5*t), 50.0*sin(2.0*t) + 75.0, sin(1.7*t), sin(0.7*t), sin(0.5*t), sin(0.8*t) };
	ISI_FLOAT usm2JointValues[IXI_NUM_USM_JOINTS] = { 0.5*sin(1.1*t), 0.5*sin(1.4*t), 50.0*sin(2.1*t) + 75.0, sin(1.6*t), sin(0.6*t), sin(0.9*t), sin(1.8*t) };
	ISI_FLOAT usm3JointValues[IXI_NUM_USM_JOINTS] = { 0.5*sin(0.9*t), 0.5*sin(1.6*t), 50.0*sin(1.9*t) + 75.0, sin(1.8*t), sin(0.8*t), sin(0.8*t), sin(1.6*t) };
	ISI_FLOAT usm4JointValues[IXI_NUM_USM_JOINTS] = { 0.5*sin(1.2*t), 0.5*sin(1.7*t), 50.0*sin(2.3*t) + 75.0, sin(1.9*t), sin(0.4*t), sin(0.2*t), sin(1.2*t) };
	ISI_FLOAT ecmJointValues[IXI_NUM_ECM_JOINTS] = { 0.5*sin(0.9*t), 0.5*sin(1.3*t), 50.0*sin(1.3*t) + 75.0, sin(1.1*t) };

	this->mUsm1->SetJointValues(usm1JointValues);
	this->mUsm2->SetJointValues(usm2JointValues);
	this->mUsm3->SetJointValues(usm3JointValues);
	this->mUsm4->SetJointValues(usm4JointValues);
	this->mEcm->SetJointValues(ecmJointValues);

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
	std::string tmp4 = this->mEcm->GetJointValuesAsString();
	LOG_DEBUG("ECM Joint Values: " << tmp4);
}

std::string IntuitiveDaVinciXi::GetAllJointValuesAsString() const
{
	std::stringstream str;
	str << "USM1: " << this->mUsm1->GetJointValuesAsString() << '\n';
	str << "USM2: " << this->mUsm2->GetJointValuesAsString() << '\n';
	str << "USM3: " << this->mUsm3->GetJointValuesAsString() << '\n';
	str << "USM4: " << this->mUsm4->GetJointValuesAsString() << '\n';
	str << "ECM:  " << this->mEcm->GetJointValuesAsString() << '\n';

	return str.str();
}

void IntuitiveDaVinciXi::UpdateBaseToWorldTransforms()
{

}


//----------------------------------------------------------------------------
void IntuitiveDaVinciXi::UpdateBaseToWorldTransforms()
{
  // Update the ECM base frame first
  dv_get_reference_frame(ISI_ECM, ISI_BASE_FRAME, mEcmBaseToWorld);

  mEcmBaseToWorld->pos.x *= 1000.0;
  mEcmBaseToWorld->pos.y *= 1000.0;
  mEcmBaseToWorld->pos.z *= 1000.0;

  // Get endoscope view to base transform
  ISI_TRANSFORM* viewToEcmBase = GetEcm()->GetTransforms() + 6;

  // Multiply to get the endoscope view to world transform
  dv_mult_transforms(mEcmBaseToWorld, viewToEcmBase, mViewToWorld);

  // Get the psm1basetoview transforms
  dv_get_reference_frame(ISI_PSM1, ISI_BASE_FRAME, mPsm1BaseToView);

  mPsm1BaseToView->pos.x *= 1000.0; // Convert from m to mm
  mPsm1BaseToView->pos.y *= 1000.0;
  mPsm1BaseToView->pos.z *= 1000.0;

  dv_get_reference_frame(ISI_PSM2, ISI_BASE_FRAME, mPsm2BaseToView);

  mPsm2BaseToView->pos.x *= 1000.0;
  mPsm2BaseToView->pos.y *= 1000.0;
  mPsm2BaseToView->pos.z *= 1000.0;

  // Compute them relative to world
  dv_mult_transforms(mViewToWorld, mPsm1BaseToView, mPsm1BaseToWorld);
  dv_mult_transforms(mViewToWorld, mPsm2BaseToView, mPsm2BaseToWorld);
}

ISI_STATUS IntuitiveDaVinciXi::UpdateAllKinematicsTransforms()
{
	UpdateBaseToWorldTransforms();

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

	mStatus = this->mEcm->UpdateLinkTransforms();

	if (mStatus != ISI_SUCCESS)
	{
		LOG_ERROR("Error updating ECM manipulator transforms.");
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
	std::string tmp4 = this->mEcm->GetTransformsAsString();
	LOG_DEBUG("ECM Kinematics Transforms: " << tmp4);
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
ISI_TRANSFORM* IntuitiveDaVinciXi::GetUsm1BaseToWorld() const
{
  return mUsm1BaseToWorld;
}

//----------------------------------------------------------------------------
ISI_TRANSFORM* IntuitiveDaVinciXi::GetUsm2BaseToWorld() const
{
	return mUsm2BaseToWorld;
}

ISI_TRANSFORM* IntuitiveDaVinciXi::GetUsm3BaseToWorld() const
{
	return mUsm3BaseToWorld;
}

ISI_TRANSFORM* IntuitiveDaVinciXi::GetUsm4BaseToWorld() const
{
	return mUsm4BaseToWorld;
}

//----------------------------------------------------------------------------
ISI_TRANSFORM* IntuitiveDaVinciXi::GetEcmBaseToWorld() const
{
  return mEcmBaseToWorld;
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

//----------------------------------------------------------------------------
IntuitiveDaVinciManipulatorXi* IntuitiveDaVinciXi::GetEcm() const
{
  return this->mEcm;
}
