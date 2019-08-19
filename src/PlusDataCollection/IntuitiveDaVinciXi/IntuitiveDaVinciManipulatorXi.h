/*=Plus=header=begin======================================================
Program: Plus
Copyright (c) Laboratory for Percutaneous Surgery. All rights reserved.
See License.txt for details.
=========================================================Plus=header=end*/

#ifndef _INTUITIVE_DAVINCI_MANIPULATORXI_H_
#define _INTUITIVE_DAVINCI_MANIPULATORXI_H_

#include "isi_api_types.h"
#include "ixi_api_types.h"

class IntuitiveDaVinciManipulatorXi
{
public:
  /*! Constructor. */
  IntuitiveDaVinciManipulatorXi(IXI_MANIP_INDEX manipIndex);

  /*! Destructor. */
  ~IntuitiveDaVinciManipulatorXi();

  /*! Get all of the link transforms of the robot. */
  ISI_TRANSFORM* GetTransforms() const;

  /*! Get the array of joint values for this manipulator. Be careful as this may not always have the same length. */
  ISI_FLOAT* GetJointValues() const;

  /*! Get the joint value array as a string for printing. */
  std::string GetJointValuesAsString() const;

  /*! Get the dh table as a string for printing. */
  std::string GetDhTableAsString() const;

  /*! Get the transforms as a string for printing. */
  std::string GetTransformsAsString() const;

  /*! Update all of the manipulator joint values using the da Vinci Xi API. */
	ISI_STATUS UpdateJointValues(ISI_FLOAT* jointValuesPy);

	/*! Update every link transform of the manipulator even if not associated with a model in slicer. Useful for debugging. */
  ISI_STATUS UpdateLinkTransforms();

  /*! Set the robot DH table (used by the xml parser). */
  void SetDhTable(ISI_DH_ROW* srcDhTable);

  /*! Set the joint values (used by the sine wave debug mode). */
  void SetJointValues(ISI_FLOAT* jointValues);

  /*! Copy a DH table (used by SetDhTable for assignment). */
  static inline void CopyDhTable(ISI_DH_ROW* srcDhTable, ISI_DH_ROW* destDhTable);

protected:
  /*! The type of manipulator. Either ISI_ECM, ISI_PSM1, or ISI_PSM2. */
  IXI_MANIP_INDEX mManipIndex; 

  /*! How many joints variables the manipulator has. Either ISI_NUM_ECM_JOINTS or ISI_NUM_PSM_JOINTS. */
  int mNumJoints;

  /*! The table of kinematic parameters. Length of 7. Used to compute robot kinematics. */
  ISI_DH_ROW* mDhTable;

  /*! Holds all of the link transforms for the manipulator. */
  ISI_TRANSFORM* mTransforms; 

  /*! Holds the current joint values of the manipulator. */
  ISI_FLOAT* mJointValues; 

	int numberOfJoints;

};

#endif