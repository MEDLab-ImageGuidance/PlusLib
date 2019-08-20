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
	PlusStatus vtkPlusIntuitiveDaVinciTrackerXi::SetDhTablesFromStrings(std::string usm1DhTable, std::string usm2DhTable, 
		std::string usm3DhTable, std::string usm4DhTable);

private:
  /*************** ROBOT JOINT VALUES ***************/

  /*! The 7 joint values of USM1 stored and broadcasted in the first 7 elements of a matrix. */
  vtkPlusDataSource* usm1Joints;

  /*! The 7 joint values of USM2 stored and broadcasted in the first 7 elements of a matrix. */
  vtkPlusDataSource* usm2Joints;

	/*! The 7 joint values of USM3 stored and broadcasted in the first 7 elements of a matrix. */
	vtkPlusDataSource* usm3Joints;

	/*! The 7 joint values of USM4 stored and broadcasted in the first 7 elements of a matrix. */
	vtkPlusDataSource* usm4Joints;

  /*************** USM1 LINK TRANSFORMS ***************/

  /*! Transform from Frame1 of USM1 to USM1 Base. */
  vtkPlusDataSource* usm1Frame1;

  /*! Transform from Frame2 of USM1 to USM1 Base. */
  vtkPlusDataSource* usm1Frame2;

  /*! Transform from Frame3 of USM1 to USM1 Base. */
  vtkPlusDataSource* usm1Frame3;

  /*! Transform from Frame4 of USM1 to USM1 Base. */
  vtkPlusDataSource* usm1Frame4;

  /*! Transform from Frame5 of USM1 to USM1 Base. */
  vtkPlusDataSource* usm1Frame5;

  /*! Transform from Frame6 of USM1 to USM1 Base. */
  vtkPlusDataSource* usm1Frame6;

  /*! Transform from Frame7 of USM1 to USM1 Base. */
  vtkPlusDataSource* usm1Frame7;

	/*! Transform from Frame8 of USM1 to USM1 Base. */
	vtkPlusDataSource* usm1Frame8;

	/*! Transform from Frame9 of USM1 to USM1 Base. */
	vtkPlusDataSource* usm1Frame9;


  /*************** USM2 LINK TRANSFORMS ***************/

  /*! Transform from Frame1 of USM2 to USM2 Base. */
  vtkPlusDataSource* usm2Frame1;

  /*! Transform from Frame2 of USM2 to USM2 Base. */
  vtkPlusDataSource* usm2Frame2;

  /*! Transform from Frame3 of USM2 to USM2 Base. */
  vtkPlusDataSource* usm2Frame3;

  /*! Transform from Frame4 of USM2 to USM2 Base. */
  vtkPlusDataSource* usm2Frame4;

  /*! Transform from Frame5 of USM2 to USM2 Base. */
  vtkPlusDataSource* usm2Frame5;

  /*! Transform from Frame6 of USM2 to USM2 Base. */
  vtkPlusDataSource* usm2Frame6;

  /*! Transform from Frame7 of USM2 to USM2 Base. */
  vtkPlusDataSource* usm2Frame7;

	/*! Transform from Frame8 of USM2 to USM2 Base. */
	vtkPlusDataSource* usm2Frame8;

	/*! Transform from Frame9 of USM2 to USM2 Base. */
	vtkPlusDataSource* usm2Frame9;

	/*************** USM3 LINK TRANSFORMS ***************/

	/*! Transform from Frame1 of USM3 to USM3 Base. */
	vtkPlusDataSource* usm3Frame1;

	/*! Transform from Frame2 of USM3 to USM3 Base. */
	vtkPlusDataSource* usm3Frame2;

	/*! Transform from Frame3 of USM3 to USM3 Base. */
	vtkPlusDataSource* usm3Frame3;

	/*! Transform from Frame4 of USM3 to USM3 Base. */
	vtkPlusDataSource* usm3Frame4;

	/*! Transform from Frame5 of USM3 to USM3 Base. */
	vtkPlusDataSource* usm3Frame5;

	/*! Transform from Frame6 of USM3 to USM3 Base. */
	vtkPlusDataSource* usm3Frame6;

	/*! Transform from Frame7 of USM3 to USM3 Base. */
	vtkPlusDataSource* usm3Frame7;

	/*! Transform from Frame8 of USM3 to USM3 Base. */
	vtkPlusDataSource* usm3Frame8;

	/*! Transform from Frame9 of USM3 to USM3 Base. */
	vtkPlusDataSource* usm3Frame9;

	/*************** USM4 LINK TRANSFORMS ***************/

	/*! Transform from Frame1 of USM4 to USM4 Base. */
	vtkPlusDataSource* usm4Frame1;

	/*! Transform from Frame2 of USM4 to USM4 Base. */
	vtkPlusDataSource* usm4Frame2;

	/*! Transform from Frame3 of USM4 to USM4 Base. */
	vtkPlusDataSource* usm4Frame3;

	/*! Transform from Frame4 of USM4 to USM4 Base. */
	vtkPlusDataSource* usm4Frame4;

	/*! Transform from Frame5 of USM4 to USM4 Base. */
	vtkPlusDataSource* usm4Frame5;

	/*! Transform from Frame6 of USM4 to USM4 Base. */
	vtkPlusDataSource* usm4Frame6;

	/*! Transform from Frame7 of USM4 to USM4 Base. */
	vtkPlusDataSource* usm4Frame7;

	/*! Transform from Frame8 of USM4 to USM4 Base. */
	vtkPlusDataSource* usm4Frame8;

	/*! Transform from Frame9 of USM4 to USM4 Base. */
	vtkPlusDataSource* usm4Frame9;

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
