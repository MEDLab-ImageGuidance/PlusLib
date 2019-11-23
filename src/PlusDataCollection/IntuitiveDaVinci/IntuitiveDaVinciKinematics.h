#ifndef INTUITIVE_DA_VINCI_KINEMATICS_H
#define INTUITIVE_DA_VINCI_KINEMATICS_H


#include <vtkMatrix4x4.h>
#include <vtkTransform.h>

#include "DaVinciXiCApi.h"

using namespace DaVinciXi;


const int NUM_USM_DH_ROWS_SETUP = 6;
const int NUM_USM_DH_ROWS_ACTIVE = 9;
const int NUM_USM_DH_ROWS_TOOL = 4;
const int NUM_USM_DH_COLS = 6;

enum UsmJointType
{
  DUMMY,
  ROTARY,
  PRISMATIC,
};

class UsmDhRow
{
public:
  void SetRow(UsmJointType jointType, double a, double alpha, double beta, double d, double theta);
  void GetTransform(vtkMatrix4x4* transformOut, vtkMatrix4x4* transformBase, double jointValue);
  UsmJointType GetJointType() { return mJointType; }

protected:
  UsmJointType mJointType;
  double mA, mAlpha, mBeta, mD, mTheta;
};

class UsmTransforms
{
public:
  UsmTransforms();
  ~UsmTransforms();
  void Copy(UsmTransforms* transformsOut);

public:
  vtkMatrix4x4* sujToWorld;
  vtkMatrix4x4* setupToWorld[NUM_USM_DH_ROWS_SETUP];
  vtkMatrix4x4* usmToWorld;
  vtkMatrix4x4* activeToWorld[NUM_USM_DH_ROWS_ACTIVE];
  vtkMatrix4x4* toolToWorld[NUM_USM_DH_ROWS_TOOL];
};

class UsmKinematicModel
{
public:
  UsmKinematicModel();
  ~UsmKinematicModel();

  // Setting usm model parameters
  void SetSetupDhTable(UsmDhRow* setupDhTable);
  void SetActiveDhTable(UsmDhRow* activeDhTable);
  void SetToolDhTable(UsmDhRow* toolDhTable);

  void SetSujToWorldTransform(vtkMatrix4x4* sujToWorldTransform);
  void SetUsmToSujTransform(vtkMatrix4x4* usmToSujTransform);

  // Computing and getting kinematics
  void SetJointValues(UsmJointValues* jointValues);
  void ComputeKinematics();
  void GetTransforms(UsmTransforms* transformsOut);
  
protected:
  UsmTransforms mTransforms;
  UsmJointValues mJointValues;

  UsmDhRow mSetupDhTable[NUM_USM_DH_ROWS_SETUP];
  UsmDhRow mActiveDhTable[NUM_USM_DH_ROWS_ACTIVE];
  UsmDhRow mToolDhTable[NUM_USM_DH_ROWS_TOOL];

  vtkMatrix4x4* mSujToWorld;
  vtkMatrix4x4* mUsmToSetup;
};

#endif