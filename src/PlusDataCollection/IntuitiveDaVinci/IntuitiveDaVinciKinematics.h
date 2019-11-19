#ifndef INTUITIVE_DA_VINCI_KINEMATICS_H
#define INTUITIVE_DA_VINCI_KINEMATICS_H


#include <vtkMatrix4x4.h>
#include <vtkTransform.h>

#include "DaVinciXiCApi.h"


using namespace DaVinciXi;

const int NUM_USM_DH_ROWS_ACTIVE = 13;
const int NUM_USM_DH_ROWS_SETUP = 6;

enum UsmJointType
{
  DUMMY,
  ROTARY,
  PRISMATIC,
};

struct UsmDhRow
{
  UsmJointType jointType;
  double a, alpha, beta, d, theta;
};

class UsmTransforms
{
public:
  UsmTransforms();
  ~UsmTransforms();

public:
  vtkMatrix4x4* sujToWorld;
  vtkMatrix4x4* setupToWorld[NUM_USM_DH_ROWS_SETUP];
  vtkMatrix4x4* usmToWorld;
  vtkMatrix4x4* activeToWorld[NUM_USM_DH_ROWS_ACTIVE];
};

class UsmKinematicModel
{
public:
  UsmKinematicModel();
  ~UsmKinematicModel();

  // Setting usm model parameters
  void SetSetupDhTable(UsmDhRow* setupDhTable);
  void SetActiveDhTable(UsmDhRow* activeDhTable);
  void SetSujToWorldTransform(vtkMatrix4x4* sujToWorldTransform);
  void SetUsmToSujTransform(vtkMatrix4x4* usmToSujTransform);

  // Computing and getting kinematics
  void SetJointValues(UsmJointValues* jointValues);
  void ComputeKinematics();
  void GetTransforms(UsmTransforms* transformsOut);
  void ComputeKinematicsAndGetTransforms(UsmTransforms* transformsOut, UsmJointValues* jointValues);

protected:
  UsmTransforms mTransforms;
  UsmJointValues mJointValues;

  UsmDhRow mSetupDhTable[NUM_USM_DH_ROWS_SETUP];
  UsmDhRow mActiveDhTable[NUM_USM_DH_ROWS_ACTIVE];

  vtkMatrix4x4* mSujToWorld;
  vtkMatrix4x4* mUsmToSetup;
};

#endif