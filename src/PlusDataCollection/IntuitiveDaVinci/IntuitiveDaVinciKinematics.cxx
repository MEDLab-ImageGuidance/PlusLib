#include "IntuitiveDaVinciKinematics.h"
#include <vtkSmartPointer.h>
#include "igsioCommon.h"

void CopyDhTable(UsmDhRow* setupDhTableOut, UsmDhRow* setupDhTableIn, int numRows)
{
  for (int iii = 0; iii < numRows; iii++)
    setupDhTableOut[iii] = setupDhTableIn[iii];
}

void UsmDhRow::SetRow(UsmJointType jointType, double a, double alpha, double beta, double d, double theta)
{
  mJointType = jointType; mA = a; mAlpha = alpha; mBeta = beta; mD = d; mTheta = theta;
}

void UsmDhRow::GetTransform(vtkMatrix4x4* transformOut, vtkMatrix4x4* transformBase, double jointValue)
{
  vtkSmartPointer<vtkTransform> tempTransform = vtkSmartPointer<vtkTransform>::New();
  tempTransform->SetMatrix(transformBase);

  double d = mD;
  double theta = mTheta;

  if (mJointType == PRISMATIC)
    d += jointValue;
  else if (mJointType == ROTARY)
    theta += jointValue;
  else if (mJointType != DUMMY)
    LOG_ERROR("Unknown joint type for da Vinci Usm.");

  tempTransform->Identity();
  
  tempTransform->RotateX(mAlpha);
  tempTransform->Translate(mA, 0, 0);
  tempTransform->RotateY(mBeta);
  tempTransform->Translate(0, 0, d);
  tempTransform->RotateZ(theta);

  tempTransform->GetMatrix(transformOut);
}

UsmTransforms::UsmTransforms()
{
  sujToWorld = vtkMatrix4x4::New();
  usmToWorld = vtkMatrix4x4::New();

  for (int iii = 0; iii < NUM_USM_DH_ROWS_SETUP; iii++)
    setupToWorld[iii] = vtkMatrix4x4::New();

  for (int iii = 0; iii < NUM_USM_DH_ROWS_ACTIVE; iii++)
    activeToWorld[iii] = vtkMatrix4x4::New();
}

UsmTransforms::~UsmTransforms()
{
  sujToWorld->Delete();
  usmToWorld->Delete();

  for (int iii = 0; iii < NUM_USM_DH_ROWS_SETUP; iii++)
    setupToWorld[iii]->Delete();

  for (int iii = 0; iii < NUM_USM_DH_ROWS_ACTIVE; iii++)
    activeToWorld[iii]->Delete();
}

void UsmTransforms::Copy(UsmTransforms* transformsOut)
{
  sujToWorld->DeepCopy(transformsOut->sujToWorld);
  usmToWorld->DeepCopy(transformsOut->usmToWorld);

  for (int iii = 0; iii < NUM_USM_DH_ROWS_SETUP; iii++)
    setupToWorld[iii]->DeepCopy(transformsOut->setupToWorld[iii]);

  for (int iii = 0; iii < NUM_USM_DH_ROWS_ACTIVE; iii++)
    activeToWorld[iii]->DeepCopy(transformsOut->activeToWorld[iii]);
}

UsmKinematicModel::UsmKinematicModel()
{
  mSujToWorld = vtkMatrix4x4::New();
  mUsmToSetup = vtkMatrix4x4::New();
}

UsmKinematicModel::~UsmKinematicModel()
{
  mSujToWorld->Delete();
  mUsmToSetup->Delete();
}

void UsmKinematicModel::SetSetupDhTable(UsmDhRow* dhTable)
{
  CopyDhTable(mSetupDhTable, dhTable, NUM_USM_DH_ROWS_SETUP);
}

void UsmKinematicModel::SetActiveDhTable(UsmDhRow* dhTable)
{
  CopyDhTable(mActiveDhTable, dhTable, NUM_USM_DH_ROWS_ACTIVE);
}


void UsmKinematicModel::SetSujToWorldTransform(vtkMatrix4x4* sujToWorldTransform)
{
  mSujToWorld->DeepCopy(sujToWorldTransform);
}

void UsmKinematicModel::SetUsmToSujTransform(vtkMatrix4x4* usmToSujTransform)
{
  mUsmToSetup->DeepCopy(usmToSujTransform);
}

void UsmKinematicModel::SetJointValues(UsmJointValues* jointValues)
{
  mJointValues = *jointValues;
}

void UsmKinematicModel::ComputeKinematics()
{
  // Set setup chain to world transform
  mTransforms.sujToWorld->DeepCopy(mSujToWorld);

  // Set all of the setup joints to world transforms
  int jointIndex = 0; // We will only use a joint value when the joint is not a dummy joint
  for (int iii = 0; iii < NUM_USM_DH_ROWS_SETUP; iii++)
  {
    // If this DH row is not a dummy, use a joint value from mJointValues, but if not, just set the joint value to 0
    double jointValue;
    if (mSetupDhTable[iii].GetJointType() != DUMMY)
    {
      jointValue = mJointValues.setupJointValues[jointIndex];
      jointIndex++; // Increment if we used one
    }
    else
      jointValue = 0.0;

    // We want our setup joints to world transforms to be relative to the world frame, so each transform
    // needs to be computed relative to the previous one.  If its the first one, then the previous is the
    // sujToWorld transform.
    vtkMatrix4x4* transformBase = iii == 0 ? mTransforms.sujToWorld : mTransforms.setupToWorld[iii - 1];

    // Write the next DH transform into the correct place
    mSetupDhTable[iii].GetTransform(mTransforms.setupToWorld[iii], transformBase, jointValue);
  }
  
  // Set active chain to world transform
  vtkMatrix4x4::Multiply4x4(mTransforms.setupToWorld[NUM_USM_DH_ROWS_SETUP - 1], mUsmToSetup, mTransforms.usmToWorld);
  
  // Set all of the active joints to world transforms
  jointIndex = 0;
  for (int iii = 0; iii < NUM_USM_DH_ROWS_ACTIVE; iii++)
  {
    double jointValue;
    if (mActiveDhTable[iii].GetJointType() != DUMMY)
    {
      jointValue = mJointValues.activeJointValues[jointIndex];
      jointIndex++;
    }
    else
      jointValue = 0.0;

    vtkMatrix4x4* transformBase = iii == 0 ? mTransforms.usmToWorld : mTransforms.activeToWorld[iii - 1];
    mActiveDhTable[iii].GetTransform(mTransforms.activeToWorld[iii], transformBase, jointValue);
  }
}

void UsmKinematicModel::GetTransforms(UsmTransforms* transformsOut)
{
  mTransforms.Copy(transformsOut);
}