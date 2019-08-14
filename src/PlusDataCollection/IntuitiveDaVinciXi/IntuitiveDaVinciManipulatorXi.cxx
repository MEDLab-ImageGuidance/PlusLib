/*=Plus=header=begin======================================================
Program: Plus
Copyright (c) Laboratory for Percutaneous Surgery. All rights reserved.
See License.txt for details.
=========================================================Plus=header=end*/
#include <sstream>

#include "PlusConfigure.h"

#include "dv_api.h"
#include "dv_api_math.h"

#include "IntuitiveDaVinciManipulatorXi.h"


IntuitiveDaVinciManipulatorXi::IntuitiveDaVinciManipulatorXi(IXI_MANIP_INDEX manipIndex)
	:mManipIndex(manipIndex)
{
	if (manipIndex == IXI_USM1 || manipIndex == IXI_USM2 || manipIndex == IXI_USM3
		|| manipIndex == IXI_USM2)
	{
		mNumJoints = (int)IXI_NUM_USM_JOINTS;
	}
	else
		mNumJoints = (int)IXI_NUM_ECM_JOINTS;

	mDhTable = new ISI_DH_ROW[7];
	mTransforms = new ISI_TRANSFORM[7];
	mJointValues = new ISI_FLOAT[mNumJoints];

	pName = new PyObject;
	pModule = new PyObject;
	pClass = new PyObject;
	pDict = new PyObject;
	pInstance = new PyObject;
	pValue = new PyObject;

	LOG_DEBUG("Created da Vinci Xi manipulator.");
}

//----------------------------------------------------------------------------
IntuitiveDaVinciManipulatorXi::~IntuitiveDaVinciManipulatorXi()
{
  delete[] mDhTable;
  delete[] mTransforms;
  delete[] mJointValues;

  mDhTable = nullptr; mTransforms = nullptr; mJointValues = nullptr;

	delete pName, pModule, pClass, pDict, pInstance, pValue;
	pName = nullptr; pModule = nullptr; pClass = nullptr;
	pDict = nullptr; pInstance = nullptr; pValue = nullptr;

  LOG_DEBUG("Destroyed da Vinci Xi manipulator.");
}

ISI_STATUS IntuitiveDaVinciManipulatorXi::UpdateJointValues()
{
	PyObject* pList;
	ISI_STATUS status;

	Py_Initialize();

	pName = PyUnicode_DecodeFSDefault("DaVinciXiApi");
	pModule = PyImport_Import(pName);
	pDict = PyModule_GetDict(pModule);
	pClass = PyDict_GetItemString(pDict, "DaVinciXiApi");
	pInstance = PyObject_CallObject(pClass, NULL);

	pList = PyList_New(mNumJoints);
	pList = PyObject_CallMethod(pInstance, "getUsmJointValues", NULL, mManipIndex); 

	if (pList != NULL)
	{
		for (int iii = 0; iii < mNumJoints; iii++)
		{
			mJointValues[iii] = PyLong_AsLong(PyList_GetItem(pList, iii));
			if (iii == 2) mJointValues[iii] *= 1000.0;
		}

		status = ISI_SUCCESS;
	}
	else
	{
		LOG_ERROR("Could not update the da Vinci Xi manipulator joint values.");
		status = ISI_UNKNOWN_ERROR;
	}

	Py_Finalize();

	return status;
}

//----------------------------------------------------------------------------
std::string IntuitiveDaVinciManipulatorXi::GetJointValuesAsString() const
{
  std::stringstream str;
  for (int iii = 0; iii < mNumJoints; iii++)
  {
    str << mJointValues[iii] << "  ";
  }

  return str.str();
}

//----------------------------------------------------------------------------
std::string IntuitiveDaVinciManipulatorXi::GetDhTableAsString() const
{
  std::stringstream str;
  for (int iii = 0; iii < 7; iii++)
  {
    str << mDhTable[iii].type << ' ';
    str << mDhTable[iii].l << ' ';
    str << mDhTable[iii].sina << ' ';
    str << mDhTable[iii].cosa << ' ';
    str << mDhTable[iii].d << ' ';
    str << mDhTable[iii].sinq << ' ';
    str << mDhTable[iii].cosq << ' ';
    str << '\n';
  }

  return str.str();
}

//----------------------------------------------------------------------------
void IntuitiveDaVinciManipulatorXi::SetDhTable(ISI_DH_ROW* srcDhTable)
{
  CopyDhTable(srcDhTable, mDhTable);
}

//----------------------------------------------------------------------------
void IntuitiveDaVinciManipulatorXi::SetJointValues(ISI_FLOAT* jointValues)
{
  for (int iii = 0; iii < mNumJoints; iii++)
  {
    mJointValues[iii] = jointValues[iii];
  }
}

//----------------------------------------------------------------------------
void IntuitiveDaVinciManipulatorXi::CopyDhTable(ISI_DH_ROW* srcDhTable, ISI_DH_ROW* destDhTable)
{
  for (int iii = 0; iii < 7; iii++)
  {
    destDhTable[iii].cosa = srcDhTable[iii].cosa;
    destDhTable[iii].cosq = srcDhTable[iii].cosq;
    destDhTable[iii].d = srcDhTable[iii].d;
    destDhTable[iii].l = srcDhTable[iii].l;
    destDhTable[iii].sina = srcDhTable[iii].sina;
    destDhTable[iii].sinq = srcDhTable[iii].sinq;
    destDhTable[iii].type = srcDhTable[iii].type;
  }
}

//----------------------------------------------------------------------------
ISI_STATUS IntuitiveDaVinciManipulatorXi::UpdateLinkTransforms()
{
  ISI_TRANSFORM base = dv_identity_transform(); // Compute relative to identity

  ISI_STATUS status = ISI_SUCCESS;
  for (int iii = 0; iii < 7; iii++)
  {
    status += dv_dh_forward_kinematics(
      &(base), iii + 1, mDhTable, mJointValues, 
      &(mTransforms[iii]), NULL);
  }

  if (status != ISI_SUCCESS)
  {
    LOG_ERROR("Could not run DH forward kinematics.");
  }

  return status;
}

//----------------------------------------------------------------------------
static std::string GetTransformAsString(const ISI_TRANSFORM& t)
{
  std::stringstream str;
  str << t.rot.row0.x << ' ' << t.rot.row0.y << ' '
    << t.rot.row0.z << ' ' << t.pos.x << '\n';
  str << t.rot.row1.x << ' ' << t.rot.row1.y << ' '
    << t.rot.row1.z << ' ' << t.pos.y << '\n';
  str << t.rot.row2.x << ' ' << t.rot.row2.y << ' '
    << t.rot.row2.z << ' ' << t.pos.z << '\n';
  str << "0 0 0 1";

  return str.str();
}

//----------------------------------------------------------------------------
std::string IntuitiveDaVinciManipulatorXi::GetTransformsAsString() const
{
  std::stringstream str;

  for (int iii = 0; iii < 7; iii++)
  {
    str << "Frame" << iii + 1 << "ToBase\n";
    str << GetTransformAsString(mTransforms[iii])<< '\n';
  }

  return str.str();
}

//----------------------------------------------------------------------------
ISI_TRANSFORM* IntuitiveDaVinciManipulatorXi::GetTransforms() const
{
  return mTransforms;
}

//----------------------------------------------------------------------------
ISI_FLOAT* IntuitiveDaVinciManipulatorXi::GetJointValues() const
{
  return mJointValues;
}