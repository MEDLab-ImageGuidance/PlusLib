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
  mPsm1 = new IntuitiveDaVinciManipulatorXi(ISI_PSM1);
  mPsm2 = new IntuitiveDaVinciManipulatorXi(ISI_PSM2);
  mEcm = new IntuitiveDaVinciManipulatorXi(ISI_ECM);

  mPsm1BaseToWorld = new ISI_TRANSFORM;
  mPsm2BaseToWorld = new ISI_TRANSFORM;
  mEcmBaseToWorld = new ISI_TRANSFORM;

  mViewToWorld = new ISI_TRANSFORM;
  mPsm1BaseToView = new ISI_TRANSFORM;
  mPsm2BaseToView = new ISI_TRANSFORM;

  LOG_DEBUG("Created da Vinci Xi.");
}

//----------------------------------------------------------------------------
IntuitiveDaVinciXi::~IntuitiveDaVinciXi()
{
  delete mPsm1, mPsm2, mEcm;
  mPsm1 = nullptr; mPsm2 = nullptr; mEcm = nullptr;

  delete mPsm1BaseToWorld, mPsm2BaseToWorld, mEcmBaseToWorld;
  mPsm1BaseToWorld = nullptr; mPsm2BaseToWorld = nullptr; mEcmBaseToWorld = nullptr;

  delete mViewToWorld, mPsm1BaseToView, mPsm2BaseToView;
  mViewToWorld = nullptr; mPsm1BaseToView = nullptr; mPsm2BaseToView = nullptr;

  this->Stop();
  this->Disconnect();

  LOG_DEBUG("Destroyed da Vinci Xi.");
}

//----------------------------------------------------------------------------
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

  mStatus = dv_connect();

  if (mStatus != ISI_SUCCESS)
  {
    LOG_ERROR("Could not connect to da Vinci Xi system.");
    return mStatus;
  }

  mConnected = true;
  LOG_INFO("Connected to da Vinci system.");
  return mStatus;
}

//----------------------------------------------------------------------------
ISI_STATUS IntuitiveDaVinciXi::ConnectDebugSineWaveMode()
{
  LOG_WARNING("Connected to a debug version of the da Vinci Xi API. Broadcasting only default values for da Vinci Xi base frames.");
  mConnected = true;
  return mStatus;
}

//----------------------------------------------------------------------------
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
  
  mStatus = dv_subscribe_all_stream_fields();

  if (mStatus != ISI_SUCCESS)
  {
    LOG_ERROR("Could not subscribe to da Vinci Xi data stream.");
    return mStatus;
  }

  mStatus = dv_start_stream(mRateHz);

  if (mStatus != ISI_SUCCESS)
  {
    LOG_ERROR("Could not start da Vinci Xi data stream.");
    return mStatus;
  }

  mStreaming = true;
  LOG_DEBUG("Data stream started.");
  return mStatus;
}

//----------------------------------------------------------------------------
ISI_STATUS IntuitiveDaVinciXi::StartDebugSineWaveMode()
{
  LOG_DEBUG("Started sine wave debug data stream.");
  mStreaming = true;
  return mStatus;
}

//----------------------------------------------------------------------------
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

  dv_stop_stream();

  mStreaming = false;
  LOG_DEBUG("Streaming from the da Vinci Xi API stopped.");
}

//----------------------------------------------------------------------------
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

  dv_disconnect();

  mConnected = false;
  LOG_DEBUG("Disconnected from the da Vinci Xi API.")
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

//----------------------------------------------------------------------------
ISI_STATUS IntuitiveDaVinciXi::UpdateAllJointValues()
{
  mStatus = this->mPsm1->UpdateJointValues();

  if (mStatus != ISI_SUCCESS)
  {
    LOG_ERROR("Could not update the PSM1 joint values with the da Vinci Xi API.");
    return mStatus;
  }

  mStatus = this->mPsm2->UpdateJointValues();

  if (mStatus != ISI_SUCCESS)
  {
    LOG_ERROR("Could not update the PSM2 joint values with the da Vinci Xi API.");
    return mStatus;
  }

  mStatus = this->mEcm->UpdateJointValues();

  if (mStatus != ISI_SUCCESS)
  {
    LOG_ERROR("Could not update the ECM joint values with the da Vinci Xi API.");
    return mStatus;
  }

  return mStatus;
}

//----------------------------------------------------------------------------
ISI_STATUS IntuitiveDaVinciXi::UpdateAllJointValuesSineWave()
{
  clock_t ticks = clock();
  float t = ((float)ticks) / ((float)CLOCKS_PER_SEC);

  ISI_FLOAT psm1JointValues[ISI_NUM_PSM_JOINTS] = { 0.5*sin(1.0*t), 0.5*sin(1.5*t), 50.0*sin(2.0*t) + 75.0, sin(1.7*t), sin(0.7*t), sin(0.5*t), sin(0.8*t) };
  ISI_FLOAT psm2JointValues[ISI_NUM_PSM_JOINTS] = { 0.5*sin(1.1*t), 0.5*sin(1.4*t), 50.0*sin(2.1*t) + 75.0, sin(1.6*t), sin(0.6*t), sin(0.9*t), sin(1.8*t) };
  ISI_FLOAT ecmJointValues[ISI_NUM_ECM_JOINTS] = { 0.5*sin(0.9*t), 0.5*sin(1.3*t), 50.0*sin(1.3*t) + 75.0, sin(1.1*t) };

  this->mPsm1->SetJointValues(psm1JointValues);
  this->mPsm2->SetJointValues(psm2JointValues);
  this->mEcm->SetJointValues(ecmJointValues);

  return mStatus;
}

//----------------------------------------------------------------------------
void IntuitiveDaVinciXi::PrintAllJointValues() const
{
  std::string tmp0 = this->mPsm1->GetJointValuesAsString();
  LOG_DEBUG("PSM1 Joint Values: " << tmp0);
  std::string tmp1 = this->mPsm2->GetJointValuesAsString();
  LOG_DEBUG("PSM2 Joint Values: " << tmp1);
  std::string tmp2 = this->mEcm->GetJointValuesAsString();
  LOG_DEBUG("ECM Joint Values: " << tmp2);
}

//----------------------------------------------------------------------------
std::string IntuitiveDaVinciXi::GetAllJointValuesAsString() const
{
  std::stringstream str;
  str << "PSM1: " << this->mPsm1->GetJointValuesAsString() << '\n';
  str << "PSM2: " << this->mPsm2->GetJointValuesAsString() << '\n';
  str << "ECM:  " << this->mEcm->GetJointValuesAsString() << '\n';
  
  return str.str();
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

//----------------------------------------------------------------------------
ISI_STATUS IntuitiveDaVinciXi::UpdateAllKinematicsTransforms()
{
  UpdateBaseToWorldTransforms();

  mStatus = this->mPsm1->UpdateLinkTransforms();

  if (mStatus != ISI_SUCCESS)
  {
    LOG_ERROR("Error updating PSM1 manipulator transforms.");
    return mStatus;
  }

  mStatus = this->mPsm2->UpdateLinkTransforms();

  if (mStatus != ISI_SUCCESS)
  {
    LOG_ERROR("Error updating PSM2 manipulator transforms.");
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

//----------------------------------------------------------------------------
void IntuitiveDaVinciXi::PrintAllKinematicsTransforms() const
{
  std::string tmp0 = this->mPsm1->GetTransformsAsString();
  LOG_DEBUG("PSM1 Kinematics Transforms: " << tmp0);
  std::string tmp1 = this->mPsm2->GetTransformsAsString();
  LOG_DEBUG("PSM2 Kinematics Transforms: " << tmp1);
  std::string tmp2 = this->mEcm->GetTransformsAsString();
  LOG_DEBUG("ECM Kinematics Transforms: " << tmp2);
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
ISI_TRANSFORM* IntuitiveDaVinciXi::GetPsm1BaseToWorld() const
{
  return mPsm1BaseToWorld;
}

//----------------------------------------------------------------------------
ISI_TRANSFORM* IntuitiveDaVinciXi::GetPsm2BaseToWorld() const
{
  return mPsm2BaseToWorld;
}

//----------------------------------------------------------------------------
ISI_TRANSFORM* IntuitiveDaVinciXi::GetEcmBaseToWorld() const
{
  return mEcmBaseToWorld;
}

//----------------------------------------------------------------------------
IntuitiveDaVinciManipulatorXi* IntuitiveDaVinciXi::GetPsm1() const
{
  return this->mPsm1;
}

//----------------------------------------------------------------------------
IntuitiveDaVinciManipulatorXi* IntuitiveDaVinciXi::GetPsm2() const
{
  return this->mPsm2;
}

//----------------------------------------------------------------------------
IntuitiveDaVinciManipulatorXi* IntuitiveDaVinciXi::GetEcm() const
{
  return this->mEcm;
}