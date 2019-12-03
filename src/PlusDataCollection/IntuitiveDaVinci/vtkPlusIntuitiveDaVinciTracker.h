/*=Plus=header=begin======================================================
Program: Plus
Copyright (c) Laboratory for Percutaneous Surgery. All rights reserved.
See License.txt for details.
=========================================================Plus=header=end*/

#ifndef __vtkPlusIntuitiveDaVinciTracker_h
#define __vtkPlusIntuitiveDaVinciTracker_h

#include <vtkObjectFactory.h>

#include "vtkPlusDataCollectionExport.h"
#include "vtkPlusDevice.h"

#include "DaVinciXiCApi.h"
#include "IntuitiveDaVinciKinematics.h"

using namespace DaVinciXi;

/* This class talks with the da Vinci Surgical System via the class IntuitiveDaVinci. */
class vtkPlusDataCollectionExport vtkPlusIntuitiveDaVinciTracker : public vtkPlusDevice
{
public:
  static vtkPlusIntuitiveDaVinciTracker* New();
  vtkTypeMacro(vtkPlusIntuitiveDaVinciTracker, vtkPlusDevice);
  virtual void PrintSelf(ostream& os, vtkIndent indent) VTK_OVERRIDE;

  virtual bool IsTracker() const { return true; }

  /*! Probe to see if the tracking system is present. */
  virtual PlusStatus Probe();

  /*! Read da Vinci configuration and update the tracker settings accordingly */
  virtual PlusStatus ReadConfiguration(vtkXMLDataElement* rootConfigElement);

  /*! Write current da Vinci configuration settings to XML */
  virtual PlusStatus WriteConfiguration(vtkXMLDataElement* rootConfigElement);

protected:
  vtkPlusIntuitiveDaVinciTracker();
  ~vtkPlusIntuitiveDaVinciTracker();

  /*! Connect to the da Vinci Xi API*/
  virtual PlusStatus InternalConnect();

  /*! Disconnect from the da Vinci Xi API */
  virtual PlusStatus InternalDisconnect();

  /*!  Start the streaming of kinematics data and/or events. */
  virtual PlusStatus InternalStartRecording();

  /*! Stop the system and bring it back to its initial state. */
  virtual PlusStatus InternalStopRecording();

  /*! Update method for updating joint values, base frames, and kinematics transforms. */
  virtual PlusStatus InternalUpdate();

  //vtkSetMacro(DebugSineWaveMode, bool);

protected:
  /*! Pointer to the DaVinciXiCApi class instance for streaming joint values*/
  DaVinciXiCApi* api;

  /*! Pointer to each of the IntuitiveDaVinciKinematics class instances for computing transforms*/
  UsmKinematicModel* usm1;
  UsmKinematicModel* usm2;
  UsmKinematicModel* usm3;
  UsmKinematicModel* usm4;

  /*! Index of the last frame number. This is used for providing a frame number when the tracker doesn't return any transform */
  unsigned long LastFrameNumber;

  /*! These are some additional flags that we can load from the xml to put the system into different modes. */
  //bool DebugSineWaveMode;

private:
  vtkPlusIntuitiveDaVinciTracker(const vtkPlusIntuitiveDaVinciTracker&);
  void operator=(const vtkPlusIntuitiveDaVinciTracker&);

  /*! Convert very explicitly between the two represeations of transforms. */
  //static inline void ConvertIsiTransformToVtkMatrix(ISI_TRANSFORM* isiMatrix, vtkMatrix4x4& vtkMatrix);

  /*! From three strings (likely obtained from the xml), set the robot DH tables. */
  //PlusStatus SetDhTablesFromStrings(std::string psm1DhTable, std::string psm2DhTable, std::string ecmDhTable);

private:
  /*************** ROBOT JOINT VALUES ***************/
  vtkPlusDataSource* usm1Joints;
  vtkPlusDataSource* usm2Joints;
  vtkPlusDataSource* usm3Joints;
  vtkPlusDataSource* usm4Joints;

  /*************** USM1 LINK TRANSFORMS ***************/
  vtkPlusDataSource* usm1Shaft;
  vtkPlusDataSource* usm1Wrist;
  vtkPlusDataSource* usm1EndEffectorJaw;
  vtkPlusDataSource* usm1EndEffectorTip;

  /*************** USM2 LINK TRANSFORMS ***************/
  vtkPlusDataSource* usm2Shaft;
  vtkPlusDataSource* usm2Wrist;
  vtkPlusDataSource* usm2EndEffectorJaw;
  vtkPlusDataSource* usm2EndEffectorTip;

  /*************** USM3 LINK TRANSFORMS ***************/
  vtkPlusDataSource* usm3Shaft;
  vtkPlusDataSource* usm3Wrist;
  vtkPlusDataSource* usm3EndEffectorJaw;
  vtkPlusDataSource* usm3EndEffectorTip;

  /*************** USM4 LINK TRANSFORMS ***************/
  vtkPlusDataSource* usm4Shaft;
  vtkPlusDataSource* usm4Wrist;
  vtkPlusDataSource* usm4EndEffectorJaw;
  vtkPlusDataSource* usm4EndEffectorTip;
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
