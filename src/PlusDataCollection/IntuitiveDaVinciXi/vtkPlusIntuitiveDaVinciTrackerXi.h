/*=Plus=header=begin======================================================
Program: Plus
Copyright (c) Laboratory for Percutaneous Surgery. All rights reserved.
See License.txt for details.
=========================================================Plus=header=end*/

#ifndef __vtkPlusIntuitiveDaVinciTrackerXi_h
#define __vtkPlusIntuitiveDaVinciTrackerXi_h

#include <vtkObjectFactory.h>

#include "vtkPlusDataCollectionExport.h"
#include "vtkPlusDevice.h"

#include "IntuitiveDaVinciXi.h"

/* This class talks with the da Vinci Surgical System via the class IntuitiveDaVinci. */
class vtkPlusDataCollectionExport vtkPlusIntuitiveDaVinciTrackerXi : public vtkPlusDevice
{
public:
  static vtkPlusIntuitiveDaVinciTrackerXi* New();
  vtkTypeMacro(vtkPlusIntuitiveDaVinciTrackerXi, vtkPlusDevice);
  virtual void PrintSelf(ostream& os, vtkIndent indent) VTK_OVERRIDE;

  virtual bool IsTracker() const { return true; }

  /*! Probe to see if the tracking system is present. */
  virtual PlusStatus Probe();

  /*! Read da Vinci configuration and update the tracker settings accordingly */
  virtual PlusStatus ReadConfiguration(vtkXMLDataElement* rootConfigElement);

  /*! Write current da Vinci configuration settings to XML */
  virtual PlusStatus WriteConfiguration(vtkXMLDataElement* rootConfigElement);

  IntuitiveDaVinciXi* GetDaVinci() const;

protected:
  vtkPlusIntuitiveDaVinciTrackerXi();
  ~vtkPlusIntuitiveDaVinciTrackerXi();

  /*! Connect to the da Vinci API*/
  virtual PlusStatus InternalConnect();

  /*! Disconnect from the da Vinci API */
  virtual PlusStatus InternalDisconnect();

  /*!  Start the streaming of kinematics data and/or events. */
  virtual PlusStatus InternalStartRecording();

  /*! Stop the system and bring it back to its initial state. */
  virtual PlusStatus InternalStopRecording();

  /*! Update method for updating joint values, base frames, and kinematics transforms. */
  virtual PlusStatus InternalUpdate();

  vtkSetMacro(DebugSineWaveMode, bool);

protected:
  /*! Pointer to the IntuitiveDaVinci class instance */
  IntuitiveDaVinciXi*   DaVinci;

  /*! Index of the last frame number. This is used for providing a frame number when the tracker doesn't return any transform */
  unsigned long       LastFrameNumber;

  /*! These are some additional flags that we can load from the xml to put the system into different modes. */
  bool DebugSineWaveMode;

private:
  vtkPlusIntuitiveDaVinciTrackerXi(const vtkPlusIntuitiveDaVinciTrackerXi&);
  void operator=(const vtkPlusIntuitiveDaVinciTrackerXi&);

  /*! Convert very explicitly between the two represeations of transforms. */
  static inline void ConvertIsiTransformToVtkMatrix(ISI_TRANSFORM* isiMatrix, vtkMatrix4x4& vtkMatrix);

  /*! From three strings (likely obtained from the xml), set the robot DH tables. */
  PlusStatus SetDhTablesFromStrings(std::string psm1DhTable, std::string psm2DhTable, std::string ecmDhTable);

private:
  /*************** ROBOT JOINT VALUES ***************/

  /*! The 7 joint values of PSM1 stored and broadcasted in the first 7 elements of a matrix. */
  vtkPlusDataSource* psm1Joints;

  /*! The 7 joint values of PSM2 stored and broadcasted in the first 7 elements of a matrix. */
  vtkPlusDataSource* psm2Joints;

  /*! The 7 joint values of ECM stored and broadcasted in the first 4 elements of a matrix. */
  vtkPlusDataSource* ecmJoints;

  /*************** ROBOT BASE TRANSFORMS ***************/

  /*! Transform from PSM1 Base frame to the da Vinci world frame. */
  vtkPlusDataSource* psm1Base;

  /*! Transform from PSM2 Base frame to the da Vinci world frame. */
  vtkPlusDataSource* psm2Base;

  /*! Transform from ECM Base frame to the da Vinci world frame. */
  vtkPlusDataSource* ecmBase;

  /*************** PSM1 LINK TRANSFORMS ***************/

  /*! Transform from Frame1 of PSM1 to PSM1 Base. */
  vtkPlusDataSource* psm1Frame1;

  /*! Transform from Frame2 of PSM1 to PSM1 Base. */
  vtkPlusDataSource* psm1Frame2;

  /*! Transform from Frame3 of PSM1 to PSM1 Base. */
  vtkPlusDataSource* psm1Frame3;

  /*! Transform from Frame4 of PSM1 to PSM1 Base. */
  vtkPlusDataSource* psm1Frame4;

  /*! Transform from Frame5 of PSM1 to PSM1 Base. */
  vtkPlusDataSource* psm1Frame5;

  /*! Transform from Frame6 of PSM1 to PSM1 Base. */
  vtkPlusDataSource* psm1Frame6;

  /*! Transform from Frame7 of PSM1 to PSM1 Base. */
  vtkPlusDataSource* psm1Frame7;

  /*************** PSM2 LINK TRANSFORMS ***************/

  /*! Transform from Frame1 of PSM2 to PSM2 Base. */
  vtkPlusDataSource* psm2Frame1;

  /*! Transform from Frame2 of PSM2 to PSM2 Base. */
  vtkPlusDataSource* psm2Frame2;

  /*! Transform from Frame3 of PSM2 to PSM2 Base. */
  vtkPlusDataSource* psm2Frame3;

  /*! Transform from Frame4 of PSM2 to PSM2 Base. */
  vtkPlusDataSource* psm2Frame4;

  /*! Transform from Frame5 of PSM2 to PSM2 Base. */
  vtkPlusDataSource* psm2Frame5;

  /*! Transform from Frame6 of PSM2 to PSM2 Base. */
  vtkPlusDataSource* psm2Frame6;

  /*! Transform from Frame7 of PSM2 to PSM2 Base. */
  vtkPlusDataSource* psm2Frame7;

  /*************** ECM LINK TRANSFORMS ***************/

  /*! Transform from Frame1 of ECM to ECM Base. */
  vtkPlusDataSource* ecmFrame1;

  /*! Transform from Frame2 of ECM to ECM Base. */
  vtkPlusDataSource* ecmFrame2;

  /*! Transform from Frame3 of ECM to ECM Base. */
  vtkPlusDataSource* ecmFrame3;

  /*! Transform from Frame4 of ECM to ECM Base. */
  vtkPlusDataSource* ecmFrame4;

  /*! Transform from Frame5 of ECM to ECM Base. */
  vtkPlusDataSource* ecmFrame5;

  /*! Transform from Frame6 of ECM to ECM Base. */
  vtkPlusDataSource* ecmFrame6;

  /*! Transform from Frame7 of ECM to ECM Base. */
  vtkPlusDataSource* ecmFrame7;
};

// Macro to publish an isiTransform to a given tool. 
#define PUBLISH_ISI_TRANSFORM(tool, isiTransform) \
  if(tool!=NULL) \
  { \
    ConvertIsiTransformToVtkMatrix(isiTransform, *tmpVtkMatrix); \
    unsigned long frameNumber = tool->GetFrameNumber() + 1; \
    ToolTimeStampedUpdate(tool->GetId(), tmpVtkMatrix, TOOL_OK, frameNumber, toolTimestamp); \
  } \

#endif
