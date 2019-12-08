/*=Plus=header=begin======================================================
Program: Plus
Copyright (c) Laboratory for Percutaneous Surgery. All rights reserved.
See License.txt for details.
=========================================================Plus=header=end*/

#include "igsioCommon.h"
#include "PlusConfigure.h"
#include "vtkPlusDataSource.h"
#include "vtkPlusIntuitiveDaVinciTracker.h"

#include <vtkImageData.h>
#include <vtkMath.h>
#include <vtkMatrix4x4.h>
#include <vtkTransform.h>

//----------------------------------------------------------------------------
void UsmJointValuesToVtkMatrix(vtkMatrix4x4* matrixOut, UsmJointValues* jointValuesIn)
{
  matrixOut->Zero();

  matrixOut->SetElement(0, 0, jointValuesIn->setupJointValues[0]);
  matrixOut->SetElement(0, 1, jointValuesIn->setupJointValues[1]);
  matrixOut->SetElement(0, 2, jointValuesIn->setupJointValues[2]);
  matrixOut->SetElement(0, 3, jointValuesIn->setupJointValues[3]);

  matrixOut->SetElement(1, 0, jointValuesIn->activeJointValues[0]);
  matrixOut->SetElement(1, 1, jointValuesIn->activeJointValues[1]);
  matrixOut->SetElement(1, 2, jointValuesIn->activeJointValues[2]);
  matrixOut->SetElement(1, 3, jointValuesIn->activeJointValues[3]);
  matrixOut->SetElement(2, 0, jointValuesIn->activeJointValues[4]);
  matrixOut->SetElement(2, 1, jointValuesIn->activeJointValues[5]);
  matrixOut->SetElement(2, 2, jointValuesIn->activeJointValues[6]);
  matrixOut->SetElement(2, 3, jointValuesIn->activeJointValues[7]);
}

//----------------------------------------------------------------------------
vtkStandardNewMacro(vtkPlusIntuitiveDaVinciTracker);

//----------------------------------------------------------------------------
vtkPlusIntuitiveDaVinciTracker::vtkPlusIntuitiveDaVinciTracker()
  : vtkPlusDevice()
  , api(new DaVinciXiCApi())
  , usm1(new UsmKinematicModel()), usm2(new UsmKinematicModel())
  , usm3(new UsmKinematicModel()), usm4(new UsmKinematicModel())
  , LastFrameNumber(0)
  , usm1Joints(NULL), usm2Joints(NULL), usm3Joints(NULL), usm4Joints(NULL)
  , usm1Shaft(NULL), usm1Wrist(NULL), usm1Jaws(NULL), usm1Tip(NULL)
  , usm2Shaft(NULL), usm2Wrist(NULL), usm2Jaws(NULL), usm2Tip(NULL)
  , usm3Shaft(NULL), usm3Wrist(NULL), usm3Jaws(NULL), usm3Tip(NULL)
  , usm4Shaft(NULL), usm4Wrist(NULL), usm4Jaws(NULL), usm4Tip(NULL)
{
  this->StartThreadForInternalUpdates = true; // Want a dedicated thread
  this->RequirePortNameInDeviceSetConfiguration = true;
  this->AcquisitionRate = 1;

  LOG_DEBUG("vktPlusIntuitiveDaVinciTracker created.");
}

//----------------------------------------------------------------------------
vtkPlusIntuitiveDaVinciTracker::~vtkPlusIntuitiveDaVinciTracker()
{
  this->StopRecording();
  this->Disconnect();

  delete this->api;
  this->api = nullptr;

  delete this->usm1;
  this->usm1 = nullptr;

  delete this->usm2;
  this->usm2 = nullptr;

  delete this->usm3;
  this->usm3 = nullptr;

  delete this->usm4;
  this->usm4 = nullptr;

  LOG_DEBUG("vktPlusIntuitiveDaVinciTracker destroyed.");
}

//----------------------------------------------------------------------------
void vtkPlusIntuitiveDaVinciTracker::PrintSelf(ostream& os, vtkIndent indent)
{

}

//----------------------------------------------------------------------------
PlusStatus vtkPlusIntuitiveDaVinciTracker::Probe()
{
  LOG_DEBUG("Probing vtkPlusIntuitiveDaVinciTracker.");

  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusIntuitiveDaVinciTracker::InternalConnect()
{
  LOG_DEBUG("vtkPlusIntuitiveDaVinciTracker::InternalConnect");

  if (this->Connected)
  {
    LOG_WARNING("Cannot run InternalConnect because already connected.");
    return PLUS_SUCCESS;
  }

  GetToolByPortName("usm1Joints", this->usm1Joints);
  GetToolByPortName("usm2Joints", this->usm2Joints);
  GetToolByPortName("usm3Joints", this->usm3Joints);
  GetToolByPortName("usm4Joints", this->usm4Joints);

  GetToolByPortName("usm1Shaft", this->usm1Shaft);
  GetToolByPortName("usm1Wrist", this->usm1Wrist);
  GetToolByPortName("usm1Jaws", this->usm1Jaws);
  GetToolByPortName("usm1Tip", this->usm1Tip);

  GetToolByPortName("usm2Shaft", this->usm2Shaft);
  GetToolByPortName("usm2Wrist", this->usm2Wrist);
  GetToolByPortName("usm2Jaws", this->usm2Jaws);
  GetToolByPortName("usm2Tip", this->usm2Tip);

  GetToolByPortName("usm3Shaft", this->usm3Shaft);
  GetToolByPortName("usm3Wrist", this->usm3Wrist);
  GetToolByPortName("usm3Jaws", this->usm3Jaws);
  GetToolByPortName("usm3Tip", this->usm3Tip);

  GetToolByPortName("usm4Shaft", this->usm4Shaft);
  GetToolByPortName("usm4Wrist", this->usm4Wrist);
  GetToolByPortName("usm4Jaws", this->usm4Jaws);
  GetToolByPortName("usm4Tip", this->usm4Tip);

  LOG_DEBUG("Connection successful.");

  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusIntuitiveDaVinciTracker::InternalStartRecording()
{
  if (!this->Connected)
  {
    LOG_ERROR("InternalStartRecording failed: vtkDevice is not connected.");
    return PLUS_FAIL;
  }

  if (this->api->getStreamRunning() == DAVINCI_SUCCESS)
  {
    LOG_ERROR("InternalStartRecording failed: da Vinci stream is already running.");
    return PLUS_FAIL;
  }

  this->api->startStream();

  if (this->api->getStreamRunning() != DAVINCI_SUCCESS)
  {
    LOG_ERROR("InternalStartRecording: Unable to start streaming.");
    return PLUS_FAIL;
  }

  LOG_DEBUG("InternalStartRecording started.");
  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusIntuitiveDaVinciTracker::InternalUpdate()
{
  this->LastFrameNumber++;
  const double toolTimestamp = vtkIGSIOAccurateTimer::GetSystemTime(); // unfiltered timestamp

  if (this->api->getStreamRunning() != DAVINCI_SUCCESS)
  {
    LOG_ERROR("InternalUpdate: DaVinciXiCApi stream is not running.");
    return PLUS_FAIL;
  }

  if (this->api->getStreamUpToDate() != DAVINCI_SUCCESS)
  {
    LOG_ERROR("InternalUpdate: DaVinciXiCApi stream is not up to date.");
    return PLUS_FAIL;
  }

  DaVinciXiJointValues jointValues;

  //LOG_DEBUG("InternalUpdate called.");
  this->api->getJointValues(jointValues);

  //LOG_DEBUG(jointValues.AsString());

  this->usm1->SetJointValues(&(jointValues.usm1));
  this->usm2->SetJointValues(&(jointValues.usm2));
  this->usm3->SetJointValues(&(jointValues.usm3));
  this->usm4->SetJointValues(&(jointValues.usm4));

  this->usm1->ComputeKinematics();
  this->usm2->ComputeKinematics();
  this->usm3->ComputeKinematics();
  this->usm4->ComputeKinematics();

  UsmTransforms usm1Transforms;
  this->usm1->GetTransforms(&usm1Transforms);
  UsmTransforms usm2Transforms;
  this->usm2->GetTransforms(&usm2Transforms);
  UsmTransforms usm3Transforms;
  this->usm3->GetTransforms(&usm3Transforms);
  UsmTransforms usm4Transforms;
  this->usm4->GetTransforms(&usm4Transforms);

  // We will need these to copy data values 
  vtkSmartPointer<vtkMatrix4x4> tmpVtkMatrix = vtkSmartPointer<vtkMatrix4x4>::New();

  // Send out all of the joint values
  UsmJointValuesToVtkMatrix(tmpVtkMatrix, &(jointValues.usm1));
  unsigned long frameNumber = usm1Joints->GetFrameNumber() + 1;
  ToolTimeStampedUpdate(usm1Joints->GetId(), tmpVtkMatrix, TOOL_OK, frameNumber, toolTimestamp);

  UsmJointValuesToVtkMatrix(tmpVtkMatrix, &(jointValues.usm2));
  frameNumber = usm2Joints->GetFrameNumber() + 1;
  ToolTimeStampedUpdate(usm2Joints->GetId(), tmpVtkMatrix, TOOL_OK, frameNumber, toolTimestamp);

  UsmJointValuesToVtkMatrix(tmpVtkMatrix, &(jointValues.usm3));
  frameNumber = usm3Joints->GetFrameNumber() + 1;
  ToolTimeStampedUpdate(usm3Joints->GetId(), tmpVtkMatrix, TOOL_OK, frameNumber, toolTimestamp);

  UsmJointValuesToVtkMatrix(tmpVtkMatrix, &(jointValues.usm4));
  frameNumber = usm4Joints->GetFrameNumber() + 1;
  ToolTimeStampedUpdate(usm4Joints->GetId(), tmpVtkMatrix, TOOL_OK, frameNumber, toolTimestamp);

  const int shaftIndex = 0;
  const int wristIndex = 1;
  const int endEffectorJawIndex = 2;
  const int endEffectorTipIndex = 3;

  // Send out link transforms for Usm1
  frameNumber = usm1Shaft->GetFrameNumber() + 1;
  ToolTimeStampedUpdate(usm1Shaft->GetId(), usm1Transforms.toolToWorld[shaftIndex], TOOL_OK, frameNumber, toolTimestamp);

  frameNumber = usm1Wrist->GetFrameNumber() + 1;
  ToolTimeStampedUpdate(usm1Wrist->GetId(), usm1Transforms.toolToWorld[wristIndex], TOOL_OK, frameNumber, toolTimestamp);

  frameNumber = usm1Jaws->GetFrameNumber() + 1;
  ToolTimeStampedUpdate(usm1Jaws->GetId(), usm1Transforms.toolToWorld[endEffectorJawIndex], TOOL_OK, frameNumber, toolTimestamp);

  frameNumber = usm1Tip->GetFrameNumber() + 1;
  ToolTimeStampedUpdate(usm1Tip->GetId(), usm1Transforms.toolToWorld[endEffectorTipIndex], TOOL_OK, frameNumber, toolTimestamp);

  // Send out link transforms for Usm2
  frameNumber = usm2Shaft->GetFrameNumber() + 1;
  ToolTimeStampedUpdate(usm2Shaft->GetId(), usm2Transforms.toolToWorld[shaftIndex], TOOL_OK, frameNumber, toolTimestamp);

  frameNumber = usm2Wrist->GetFrameNumber() + 1;
  ToolTimeStampedUpdate(usm2Wrist->GetId(), usm2Transforms.toolToWorld[wristIndex], TOOL_OK, frameNumber, toolTimestamp);

  frameNumber = usm2Jaws->GetFrameNumber() + 1;
  ToolTimeStampedUpdate(usm2Jaws->GetId(), usm2Transforms.toolToWorld[endEffectorJawIndex], TOOL_OK, frameNumber, toolTimestamp);

  frameNumber = usm2Tip->GetFrameNumber() + 1;
  ToolTimeStampedUpdate(usm2Tip->GetId(), usm2Transforms.toolToWorld[endEffectorTipIndex], TOOL_OK, frameNumber, toolTimestamp);

  // Send out link transforms for Usm3
  frameNumber = usm3Shaft->GetFrameNumber() + 1;
  ToolTimeStampedUpdate(usm3Shaft->GetId(), usm3Transforms.toolToWorld[shaftIndex], TOOL_OK, frameNumber, toolTimestamp);

  frameNumber = usm3Wrist->GetFrameNumber() + 1;
  ToolTimeStampedUpdate(usm3Wrist->GetId(), usm3Transforms.toolToWorld[wristIndex], TOOL_OK, frameNumber, toolTimestamp);

  frameNumber = usm3Jaws->GetFrameNumber() + 1;
  ToolTimeStampedUpdate(usm3Jaws->GetId(), usm3Transforms.toolToWorld[endEffectorJawIndex], TOOL_OK, frameNumber, toolTimestamp);

  frameNumber = usm3Tip->GetFrameNumber() + 1;
  ToolTimeStampedUpdate(usm3Tip->GetId(), usm3Transforms.toolToWorld[endEffectorTipIndex], TOOL_OK, frameNumber, toolTimestamp);

  // Send out link transforms for Usm4
  frameNumber = usm4Shaft->GetFrameNumber() + 1;
  ToolTimeStampedUpdate(usm4Shaft->GetId(), usm4Transforms.toolToWorld[shaftIndex], TOOL_OK, frameNumber, toolTimestamp);

  frameNumber = usm4Wrist->GetFrameNumber() + 1;
  ToolTimeStampedUpdate(usm4Wrist->GetId(), usm4Transforms.toolToWorld[wristIndex], TOOL_OK, frameNumber, toolTimestamp);

  frameNumber = usm4Jaws->GetFrameNumber() + 1;
  ToolTimeStampedUpdate(usm4Jaws->GetId(), usm4Transforms.toolToWorld[endEffectorJawIndex], TOOL_OK, frameNumber, toolTimestamp);

  frameNumber = usm4Tip->GetFrameNumber() + 1;
  ToolTimeStampedUpdate(usm4Tip->GetId(), usm4Transforms.toolToWorld[endEffectorTipIndex], TOOL_OK, frameNumber, toolTimestamp);

  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusIntuitiveDaVinciTracker::InternalStopRecording()
{
  if (this->api->getConnected() != DAVINCI_SUCCESS)
  {
    LOG_WARNING("InternalStopRecording: Cannot stop recording when not connected to device.");
    return PLUS_FAIL;
  }

  if (this->api->getStreamRunning() != DAVINCI_SUCCESS)
  {
    LOG_WARNING("InternalStopRecording: Cannot stop recording when stream is not running.");
    return PLUS_FAIL;
  }

  this->api->stopStream();

  LOG_DEBUG("InternalStartRecording stopped.");
  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusIntuitiveDaVinciTracker::InternalDisconnect()
{
  LOG_DEBUG("Disconnected from device.");
  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
void VectorToDhTable(UsmDhRow* dhTable, double* vector, int numRows)
{
  for (int iii = 0; iii < numRows; iii++)
  {
    double* row = vector + iii*NUM_USM_DH_COLS;
    dhTable[iii].SetRow(static_cast<UsmJointType>(static_cast<int>(row[0])), row[1], row[2], row[3], row[4], row[5]);
  }
}

//----------------------------------------------------------------------------
PlusStatus SetUsmModelParameters(UsmKinematicModel* usm, const std::string& usmNumber, const std::string& toolType, const std::string& calibrationFilename)
{
  LOG_DEBUG("Building parameters for " << usmNumber << " with tool: " << toolType << " and calibration file " << calibrationFilename);

  vtkXMLDataElement* calibrationElement = vtkPlusConfig::GetInstance()->CreateDeviceSetConfigurationFromFile(calibrationFilename);

  if (calibrationElement == nullptr)
  {
    return PLUS_FAIL;
  }

  XML_FIND_NESTED_ELEMENT_REQUIRED(usmElement, calibrationElement, usmNumber.c_str());
  XML_FIND_NESTED_ELEMENT_REQUIRED(instrumentDhTables, calibrationElement, "InstrumentDhTables");

  XML_FIND_NESTED_ELEMENT_REQUIRED(sujToWorldTransformElement, usmElement, "SujToWorldTransform");
  XML_FIND_NESTED_ELEMENT_REQUIRED(usmToSujTransformElement, usmElement, "UsmToSujTransform");
  XML_FIND_NESTED_ELEMENT_REQUIRED(setupDhTableElement, usmElement, "SetupDhTable");
  XML_FIND_NESTED_ELEMENT_REQUIRED(activeDhTableElement, usmElement, "ActiveDhTable");
  XML_FIND_NESTED_ELEMENT_REQUIRED(toolDhTableElement, instrumentDhTables, toolType.c_str());

  vtkSmartPointer<vtkMatrix4x4> matrix = vtkSmartPointer<vtkMatrix4x4>::New();
  double vectorBuffer[128] = { 0 };

  // Set the SujToWorldTransform
  if (sujToWorldTransformElement->GetVectorAttribute("Matrix", 4*4, vectorBuffer))
  {
    matrix->DeepCopy(vectorBuffer);
    usm->SetSujToWorldTransform(matrix);
  }
  else
  {
    LOG_ERROR("Unable to find 'Matrix' attribute of SujToWorldTransform in the calibration file.");
    return PLUS_FAIL;
  }

  // Set the UsmToSujTransform
  if (usmToSujTransformElement->GetVectorAttribute("Matrix", 4*4, vectorBuffer))
  {
    matrix->DeepCopy(vectorBuffer);
    usm->SetUsmToSujTransform(matrix);
  }
  else
  {
    LOG_ERROR("Unable to find 'Matrix' attribute of UsmToSujTransform in the calibration file.");
    return PLUS_FAIL;
  }

  // Set the SetupDhTable
  if (setupDhTableElement->GetVectorAttribute("Matrix", NUM_USM_DH_ROWS_SETUP*NUM_USM_DH_COLS, vectorBuffer))
  {
    UsmDhRow setupDhTable[NUM_USM_DH_ROWS_SETUP];
    VectorToDhTable(setupDhTable, vectorBuffer, NUM_USM_DH_ROWS_SETUP);
    usm->SetSetupDhTable(setupDhTable);
  }
  else
  {
    LOG_ERROR("Unable to find 'Matrix' attribute of SetupDhTable in the calibration file.");
    return PLUS_FAIL;
  }

  // Set the ActiveDhTable (without the tool portion)
  if (activeDhTableElement->GetVectorAttribute("Matrix", NUM_USM_DH_ROWS_ACTIVE*NUM_USM_DH_COLS, vectorBuffer))
  {
    UsmDhRow activeDhTable[NUM_USM_DH_ROWS_ACTIVE];
    VectorToDhTable(activeDhTable, vectorBuffer, NUM_USM_DH_ROWS_ACTIVE);
    usm->SetActiveDhTable(activeDhTable);
  }
  else
  {
    LOG_ERROR("Unable to find 'Matrix' attribute of ActiveDhTable in the calibration file.");
    return PLUS_FAIL;
  }

  // Set the ToolDhTable
  if (toolDhTableElement->GetVectorAttribute("Matrix", NUM_USM_DH_ROWS_TOOL*NUM_USM_DH_COLS, vectorBuffer))
  {
    UsmDhRow toolDhTable[NUM_USM_DH_ROWS_TOOL];
    VectorToDhTable(toolDhTable, vectorBuffer, NUM_USM_DH_ROWS_TOOL);
    usm->SetToolDhTable(toolDhTable);
  }
  else
  {
    LOG_ERROR("Unable to find 'Matrix' attribute InstrumentDhTables for the tool " << toolType <<  " in the calibration file.");
    return PLUS_FAIL;
  }

  calibrationElement->Delete();
  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusIntuitiveDaVinciTracker::ReadConfiguration(vtkXMLDataElement* rootConfigElement)
{
  LOG_DEBUG("vtkPlusIntuitiveDaVinciTracker::ReadConfiguration");

  XML_FIND_DEVICE_ELEMENT_REQUIRED_FOR_READING(deviceConfig, rootConfigElement);

  XML_READ_SCALAR_ATTRIBUTE_WARNING(int, AcquisitionRate, deviceConfig); 

  // Read the filename for the calibration file that contains all of our calibrated Usm parameters
  std::string calibrationFilename;
  XML_READ_STRING_ATTRIBUTE_NONMEMBER_REQUIRED(CalibrationFilename, calibrationFilename, deviceConfig);

  // In order to build DH tables for the USM, we have to know which tool it is holding
  std::string usmToolType;
  
  // Build parameters for Usm1
  XML_READ_STRING_ATTRIBUTE_NONMEMBER_REQUIRED(Usm1ToolType, usmToolType, deviceConfig);

  if (SetUsmModelParameters(this->usm1, "Usm1", usmToolType, calibrationFilename) != PLUS_SUCCESS)
  {
    LOG_ERROR("Could not set model parameters for Usm1.");
    return PLUS_FAIL;
  }

  // Build parameters for Usm2
  XML_READ_STRING_ATTRIBUTE_NONMEMBER_REQUIRED(Usm2ToolType, usmToolType, deviceConfig);

  if (SetUsmModelParameters(this->usm2, "Usm2", usmToolType, calibrationFilename) != PLUS_SUCCESS)
  {
    LOG_ERROR("Could not set model parameters for Usm2.");
    return PLUS_FAIL;
  }

  // Build parameters for Usm3
  XML_READ_STRING_ATTRIBUTE_NONMEMBER_REQUIRED(Usm3ToolType, usmToolType, deviceConfig);

  if (SetUsmModelParameters(this->usm3, "Usm3", usmToolType, calibrationFilename) != PLUS_SUCCESS)
  {
    LOG_ERROR("Could not set model parameters for Usm3.");
    return PLUS_FAIL;
  }

  // Build parameters for Usm4
  XML_READ_STRING_ATTRIBUTE_NONMEMBER_REQUIRED(Usm4ToolType, usmToolType, deviceConfig);

  if (SetUsmModelParameters(this->usm4, "Usm4", usmToolType, calibrationFilename) != PLUS_SUCCESS)
  {
    LOG_ERROR("Could not set model parameters for Usm4.");
    return PLUS_FAIL;
  }

  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusIntuitiveDaVinciTracker::WriteConfiguration(vtkXMLDataElement* rootConfigElement)
{
  LOG_DEBUG("vtkPlusIntuitiveDaVinciTracker::WriteConfiguration");
  XML_FIND_DEVICE_ELEMENT_REQUIRED_FOR_WRITING(trackerConfig, rootConfigElement);

  return PLUS_SUCCESS;
}