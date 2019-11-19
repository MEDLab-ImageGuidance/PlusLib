#include "IntuitiveDaVinciKinematics.h"
#include <vtkSmartPointer.h>
#include "igsioCommon.h"

void CopyDhTable(UsmDhRow* setupDhTableOut, UsmDhRow* setupDhTableIn, int numRows)
{
  for (int iii = 0; iii < numRows; iii++)
    setupDhTableOut[iii] = setupDhTableIn[iii];
}

void DhTransform(vtkMatrix4x4* transformOut, const UsmDhRow& dhRow, double jointValue)
{
  vtkSmartPointer<vtkTransform> tempTransform = vtkSmartPointer<vtkTransform>::New();

  double d = dhRow.d;
  double theta = dhRow.theta;

  if (dhRow.jointType == PRISMATIC)
    d += jointValue;
  else if (dhRow.jointType == ROTARY)
    theta += jointValue;
  else if (dhRow.jointType != DUMMY)
    LOG_ERROR("Unknown joint type for da Vinci Usm.");

  tempTransform->Identity();

  transformOut->Identity();

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

UsmKinematicModel::UsmKinematicModel()
{
  mSujToWorld = vtkMatrix4x4::New();
  mUsmToSetup = vtkMatrix4x4::New();

  mTempTransform = vtkTransform::New();
}

UsmKinematicModel::~UsmKinematicModel()
{
  mSujToWorld->Delete();
  mUsmToSetup->Delete();

  mTempTransform->Delete();
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
  mTransforms.sujToWorld->DeepCopy(mSujToWorld);

  DhTransform(mTransforms.setupToWorld[0], mSetupDhTable + 0, mJointValues.setupJointValues[0]);
  
}

void UsmKinematicModel::GetTransforms(UsmTransforms* transformsOut);
void UsmKinematicModel::ComputeKinematicsAndGetTransforms(UsmTransforms* transformsOut, UsmJointValues* jointValues);