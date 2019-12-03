#include "IntuitiveDaVinciKinematics.h"
#include <vtkSmartPointer.h>
#include "igsioCommon.h"
#include <vtkMath.h>

//----------------------------------------------------------------------------
void CopyDhTable(UsmDhRow* setupDhTableOut, UsmDhRow* setupDhTableIn, int numRows)
{
  for (int iii = 0; iii < numRows; iii++)
    setupDhTableOut[iii] = setupDhTableIn[iii];
}

//----------------------------------------------------------------------------
void UsmDhRow::SetRow(UsmJointType jointType, double a, double alpha, double beta, double d, double theta)
{
  mJointType = jointType; mA = a; mAlpha = alpha; mBeta = beta; mD = d; mTheta = theta;
}

//----------------------------------------------------------------------------
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
  
  tempTransform->RotateX(vtkMath::DegreesFromRadians(mAlpha));
  tempTransform->Translate(mA, 0, 0);
  tempTransform->RotateY(vtkMath::DegreesFromRadians(mBeta));
  tempTransform->Translate(0, 0, d);
  tempTransform->RotateZ(vtkMath::DegreesFromRadians(theta));

  tempTransform->GetMatrix(transformOut);
}

//----------------------------------------------------------------------------
UsmTransforms::UsmTransforms()
{
  sujToWorld = vtkMatrix4x4::New();
  usmToWorld = vtkMatrix4x4::New();

  for (int iii = 0; iii < NUM_USM_DH_ROWS_SETUP; iii++)
    setupToWorld[iii] = vtkMatrix4x4::New();

  for (int iii = 0; iii < NUM_USM_DH_ROWS_ACTIVE; iii++)
    activeToWorld[iii] = vtkMatrix4x4::New();

  for (int iii = 0; iii < NUM_USM_DH_ROWS_TOOL; iii++)
    toolToWorld[iii] = vtkMatrix4x4::New();
}

//----------------------------------------------------------------------------
UsmTransforms::~UsmTransforms()
{
  sujToWorld->Delete();
  usmToWorld->Delete();

  for (int iii = 0; iii < NUM_USM_DH_ROWS_SETUP; iii++)
    setupToWorld[iii]->Delete();

  for (int iii = 0; iii < NUM_USM_DH_ROWS_ACTIVE; iii++)
    activeToWorld[iii]->Delete();

  for (int iii = 0; iii < NUM_USM_DH_ROWS_TOOL; iii++)
    toolToWorld[iii]->Delete();
}

//----------------------------------------------------------------------------
void UsmTransforms::Copy(UsmTransforms* transformsOut)
{
  transformsOut->sujToWorld->DeepCopy(sujToWorld);
  transformsOut->usmToWorld->DeepCopy(usmToWorld);

  for (int iii = 0; iii < NUM_USM_DH_ROWS_SETUP; iii++)
    transformsOut->setupToWorld[iii]->DeepCopy(setupToWorld[iii]);

  for (int iii = 0; iii < NUM_USM_DH_ROWS_ACTIVE; iii++)
    transformsOut->activeToWorld[iii]->DeepCopy(activeToWorld[iii]);

  for (int iii = 0; iii < NUM_USM_DH_ROWS_TOOL; iii++)
    transformsOut->toolToWorld[iii]->DeepCopy(toolToWorld[iii]);
}

//----------------------------------------------------------------------------
UsmKinematicModel::UsmKinematicModel()
{
  mSujToWorld = vtkMatrix4x4::New();
  mUsmToSetup = vtkMatrix4x4::New();
}

//----------------------------------------------------------------------------
UsmKinematicModel::~UsmKinematicModel()
{
  mSujToWorld->Delete();
  mUsmToSetup->Delete();
}

//----------------------------------------------------------------------------
void UsmKinematicModel::SetSetupDhTable(UsmDhRow* dhTable)
{
  CopyDhTable(mSetupDhTable, dhTable, NUM_USM_DH_ROWS_SETUP);
}

//----------------------------------------------------------------------------
void UsmKinematicModel::SetActiveDhTable(UsmDhRow* dhTable)
{
  CopyDhTable(mActiveDhTable, dhTable, NUM_USM_DH_ROWS_ACTIVE);
}

//----------------------------------------------------------------------------
void UsmKinematicModel::SetToolDhTable(UsmDhRow* dhTable)
{
  CopyDhTable(mToolDhTable, dhTable, NUM_USM_DH_ROWS_TOOL);
}

//----------------------------------------------------------------------------
void UsmKinematicModel::SetSujToWorldTransform(vtkMatrix4x4* sujToWorldTransform)
{
  mSujToWorld->DeepCopy(sujToWorldTransform);
}

//----------------------------------------------------------------------------
void UsmKinematicModel::SetUsmToSujTransform(vtkMatrix4x4* usmToSujTransform)
{
  mUsmToSetup->DeepCopy(usmToSujTransform);
}

//----------------------------------------------------------------------------
void UsmKinematicModel::SetJointValues(UsmJointValues* jointValues)
{
  mJointValues = *jointValues;

  // We need to convert all of the prismatic joint values from m to mm
  mJointValues.setupJointValues[1] *= 1000.0;
  mJointValues.setupJointValues[2] *= 1000.0;
  mJointValues.activeJointValues[3] *= 1000.0;
}

//----------------------------------------------------------------------------
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
  
  if (jointIndex != DAVINCI_NUM_SETUP_JOINTS)
    LOG_WARNING("Did not use all setup joint values when computing kiematics. Check DH table for joint types.");

  // Set active chain to world transform
  vtkMatrix4x4::Multiply4x4(mTransforms.setupToWorld[NUM_USM_DH_ROWS_SETUP - 1], mUsmToSetup, mTransforms.usmToWorld);
  
  float* q = mJointValues.activeJointValues;
  double activeJointsModified[] = { q[0], q[1], q[2], q[2], q[2], q[3], q[4], q[5], q[6] };
  const int numActiveJointsModified = 9;

  // Set all of the active joints to world transforms
  jointIndex = 0;
  for (int iii = 0; iii < NUM_USM_DH_ROWS_ACTIVE; iii++)
  {
    double jointValue;
    if (mActiveDhTable[iii].GetJointType() != DUMMY)
    {
      jointValue = activeJointsModified[jointIndex];
      jointIndex++;
    }
    else
      jointValue = 0.0;

    vtkMatrix4x4* transformBase = iii == 0 ? mTransforms.usmToWorld : mTransforms.activeToWorld[iii - 1];
    mActiveDhTable[iii].GetTransform(mTransforms.activeToWorld[iii], transformBase, jointValue);
  }

  // Set all of the tool joints to world transforms
  for (int iii = 0; iii < NUM_USM_DH_ROWS_TOOL; iii++)
  {
    double jointValue;
    if (mToolDhTable[iii].GetJointType() != DUMMY)
    {
      jointValue = activeJointsModified[jointIndex];
      jointIndex++;
    }
    else
      jointValue = 0.0;

    vtkMatrix4x4* transformBase = iii == 0 ? mTransforms.activeToWorld[NUM_USM_DH_ROWS_ACTIVE - 1] : mTransforms.toolToWorld[iii - 1];
    mToolDhTable[iii].GetTransform(mTransforms.toolToWorld[iii], transformBase, jointValue);
  }

  if (jointIndex != numActiveJointsModified)
    LOG_WARNING("Did not use all active joint values when computing kiematics. Check DH table for joint types.");

}

//----------------------------------------------------------------------------
void UsmKinematicModel::GetTransforms(UsmTransforms* transformsOut)
{
  mTransforms.Copy(transformsOut);
}