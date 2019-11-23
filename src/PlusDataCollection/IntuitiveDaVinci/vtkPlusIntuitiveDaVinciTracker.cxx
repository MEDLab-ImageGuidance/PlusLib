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
  //, DebugSineWaveMode(false)
  //, psm1Base(NULL),   psm2Base(NULL),   ecmBase(NULL)
  //, psm1Frame1(NULL), psm1Frame2(NULL), psm1Frame3(NULL), psm1Frame4(NULL), psm1Frame5(NULL), psm1Frame6(NULL), psm1Frame7(NULL)
  //, psm2Frame1(NULL), psm2Frame2(NULL), psm2Frame3(NULL), psm2Frame4(NULL), psm2Frame5(NULL), psm2Frame6(NULL), psm2Frame7(NULL)
  //, ecmFrame1(NULL),  ecmFrame2(NULL),  ecmFrame3(NULL),  ecmFrame4(NULL),  ecmFrame5(NULL),  ecmFrame6(NULL),  ecmFrame7(NULL)
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

  //GetToolByPortName("psm1Base", this->psm1Base);
  //GetToolByPortName("psm2Base", this->psm2Base);
  //GetToolByPortName("ecmBase", this->ecmBase);

  //GetToolByPortName("psm1Frame1", this->psm1Frame1);
  //GetToolByPortName("psm1Frame2", this->psm1Frame2);
  //GetToolByPortName("psm1Frame3", this->psm1Frame3);
  //GetToolByPortName("psm1Frame4", this->psm1Frame4);
  //GetToolByPortName("psm1Frame5", this->psm1Frame5);
  //GetToolByPortName("psm1Frame6", this->psm1Frame6);
  //GetToolByPortName("psm1Frame7", this->psm1Frame7);

  //GetToolByPortName("psm2Frame1", this->psm2Frame1);
  //GetToolByPortName("psm2Frame2", this->psm2Frame2);
  //GetToolByPortName("psm2Frame3", this->psm2Frame3);
  //GetToolByPortName("psm2Frame4", this->psm2Frame4);
  //GetToolByPortName("psm2Frame5", this->psm2Frame5);
  //GetToolByPortName("psm2Frame6", this->psm2Frame6);
  //GetToolByPortName("psm2Frame7", this->psm2Frame7);

  //GetToolByPortName("ecmFrame1", this->ecmFrame1);
  //GetToolByPortName("ecmFrame2", this->ecmFrame2);
  //GetToolByPortName("ecmFrame3", this->ecmFrame3);
  //GetToolByPortName("ecmFrame4", this->ecmFrame4);
  //GetToolByPortName("ecmFrame5", this->ecmFrame5);
  //GetToolByPortName("ecmFrame6", this->ecmFrame6);
  //GetToolByPortName("ecmFrame7", this->ecmFrame7);

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

  LOG_DEBUG(jointValues.AsString());

  this->usm1->SetJointValues(&(jointValues.usm1));
  this->usm2->SetJointValues(&(jointValues.usm2));
  this->usm3->SetJointValues(&(jointValues.usm3));
  this->usm4->SetJointValues(&(jointValues.usm4));

  //this->usm1->ComputeKinematics();
  //this->usm2->ComputeKinematics();
  //this->usm3->ComputeKinematics();
  //this->usm4->ComputeKinematics();

  //UsmTransforms usm1Transforms;
  //this->usm1->GetTransforms(&usm1Transforms);
  //UsmTransforms usm2Transforms;
  //this->usm1->GetTransforms(&usm2Transforms);
  //UsmTransforms usm3Transforms;
  //this->usm1->GetTransforms(&usm3Transforms);
  //UsmTransforms usm4Transforms;
  //this->usm1->GetTransforms(&usm4Transforms);

  // We will need these to copy data values 
  vtkSmartPointer<vtkMatrix4x4> tmpVtkMatrix = vtkSmartPointer<vtkMatrix4x4>::New();

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

  //// Update all of the manipulator base frames
  //PUBLISH_ISI_TRANSFORM(psm1Base, this->DaVinci->GetPsm1BaseToWorld());
  //PUBLISH_ISI_TRANSFORM(psm2Base, this->DaVinci->GetPsm2BaseToWorld());
  //PUBLISH_ISI_TRANSFORM(ecmBase, this->DaVinci->GetEcmBaseToWorld());

  //// Update all of the psm1Frames
  //ISI_TRANSFORM* psm1Transforms = this->DaVinci->GetPsm1()->GetTransforms();

  //PUBLISH_ISI_TRANSFORM(psm1Frame1, psm1Transforms + 0);
  //PUBLISH_ISI_TRANSFORM(psm1Frame2, psm1Transforms + 1);
  //PUBLISH_ISI_TRANSFORM(psm1Frame3, psm1Transforms + 2);
  //PUBLISH_ISI_TRANSFORM(psm1Frame4, psm1Transforms + 3);
  //PUBLISH_ISI_TRANSFORM(psm1Frame5, psm1Transforms + 4);
  //PUBLISH_ISI_TRANSFORM(psm1Frame6, psm1Transforms + 5);
  //PUBLISH_ISI_TRANSFORM(psm1Frame7, psm1Transforms + 6);

  //// Update all of the psm2Frames
  //ISI_TRANSFORM* psm2Transforms = this->DaVinci->GetPsm2()->GetTransforms();

  //PUBLISH_ISI_TRANSFORM(psm2Frame1, psm2Transforms + 0);
  //PUBLISH_ISI_TRANSFORM(psm2Frame2, psm2Transforms + 1);
  //PUBLISH_ISI_TRANSFORM(psm2Frame3, psm2Transforms + 2);
  //PUBLISH_ISI_TRANSFORM(psm2Frame4, psm2Transforms + 3);
  //PUBLISH_ISI_TRANSFORM(psm2Frame5, psm2Transforms + 4);
  //PUBLISH_ISI_TRANSFORM(psm2Frame6, psm2Transforms + 5);
  //PUBLISH_ISI_TRANSFORM(psm2Frame7, psm2Transforms + 6);

  //// Update all of the ecmFrames
  //ISI_TRANSFORM* ecmTransforms = this->DaVinci->GetEcm()->GetTransforms();

  //PUBLISH_ISI_TRANSFORM(ecmFrame1, ecmTransforms + 0);
  //PUBLISH_ISI_TRANSFORM(ecmFrame2, ecmTransforms + 1);
  //PUBLISH_ISI_TRANSFORM(ecmFrame3, ecmTransforms + 2);
  //PUBLISH_ISI_TRANSFORM(ecmFrame4, ecmTransforms + 3);
  //PUBLISH_ISI_TRANSFORM(ecmFrame5, ecmTransforms + 4);
  //PUBLISH_ISI_TRANSFORM(ecmFrame6, ecmTransforms + 5);
  //PUBLISH_ISI_TRANSFORM(ecmFrame7, ecmTransforms + 6);

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

void VectorToDhTable(UsmDhRow* dhTable, double* vector, int numRows)
{
  for (int iii = 0; iii < numRows; iii++)
  {
    double* row = vector + iii*NUM_USM_DH_COLS;
    dhTable[iii].SetRow(static_cast<UsmJointType>(static_cast<int>(row[0])), row[1], row[2], row[3], row[4], row[5]);
  }
}

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

////----------------------------------------------------------------------------
//static bool BothAreSpaces(char lhs, char rhs)
//{
//	return (lhs == rhs) && (lhs == ' ');
//}
//
////----------------------------------------------------------------------------
//static void ProcessDhString(std::string& str)
//{
//	std::vector<std::string> strTokens;
//
//	// Remove all the new lines from the string
//	str.erase(std::remove(str.begin(), str.end(), '\n'), str.end());
//	// Remove all tabs 
//	str.erase(std::remove(str.begin(), str.end(), '\t'), str.end());
//	// Trim the beginning and end
//	str = igsioCommon::Trim(str);
//
//	// Remove all double/triple spaces
//	std::string::iterator new_end = std::unique(str.begin(), str.end(), BothAreSpaces);
//	str.erase(new_end, str.end());
//}

//static void ConvertTokenVectorToDhTable(std::vector<std::string>& srcTokenVector, ISI_DH_ROW* destIsiDhTable)
//{
//  for (int iii = 0; iii < 7; iii++)
//  {
//    try
//    {
//      destIsiDhTable[iii].type = (ISI_FLOAT)std::stof(srcTokenVector[7 * iii + 0]);
//      destIsiDhTable[iii].l = (ISI_FLOAT)std::stof(srcTokenVector[7 * iii + 1]);
//      destIsiDhTable[iii].sina = (ISI_FLOAT)std::stof(srcTokenVector[7 * iii + 2]);
//      destIsiDhTable[iii].cosa = (ISI_FLOAT)std::stof(srcTokenVector[7 * iii + 3]);
//      destIsiDhTable[iii].d = (ISI_FLOAT)std::stof(srcTokenVector[7 * iii + 4]);
//      destIsiDhTable[iii].sinq = (ISI_FLOAT)std::stof(srcTokenVector[7 * iii + 5]);
//      destIsiDhTable[iii].cosq = (ISI_FLOAT)std::stof(srcTokenVector[7 * iii + 6]);
//    }
//    catch (...)
//    {
//      LOG_ERROR("Check input DH table input in config file.");
//    }
//  }
//}

//----------------------------------------------------------------------------
//PlusStatus vtkPlusIntuitiveDaVinciTracker::SetDhTablesFromStrings(std::string psm1DhTable, std::string psm2DhTable, std::string ecmDhTable)
//{
//  std::vector<std::string> psm1TokenVector, psm2TokenVector, ecmTokenVector;
// 
//  const int numDhRows = 7; const int numDhCols = 7;
//  int numElem = numDhRows*numDhCols;
//
//  ProcessDhString(psm1DhTable);
//  ProcessDhString(psm2DhTable);
//  ProcessDhString(ecmDhTable);
//
//  psm1TokenVector = igsioCommon::SplitStringIntoTokens(psm1DhTable, ' ');
//  psm2TokenVector = igsioCommon::SplitStringIntoTokens(psm2DhTable, ' ');
//  ecmTokenVector = igsioCommon::SplitStringIntoTokens(ecmDhTable, ' ');
//
//  if((psm1TokenVector.size() != numElem) || 
//	   (psm2TokenVector.size() != numElem) ||
//	   (ecmTokenVector.size() != numElem))
//  {
//	  LOG_ERROR("Invalid formatting of DH table string. Must have " << numElem << "elements.");
//	  return PLUS_FAIL;
//  }
//
//  ISI_DH_ROW isiPsm1DhTable[numDhRows];
//  ISI_DH_ROW isiPsm2DhTable[numDhRows];
//  ISI_DH_ROW isiEcmDhTable[numDhRows];
//
//  ConvertTokenVectorToDhTable(psm1TokenVector, isiPsm1DhTable);
//  ConvertTokenVectorToDhTable(psm2TokenVector, isiPsm2DhTable);
//  ConvertTokenVectorToDhTable(ecmTokenVector, isiEcmDhTable);
//
//  this->DaVinci->GetPsm1()->SetDhTable(isiPsm1DhTable);
//  LOG_DEBUG("PSM1 DH Table set to: " << this->DaVinci->GetPsm1()->GetDhTableAsString());
//  this->DaVinci->GetPsm2()->SetDhTable(isiPsm2DhTable);
//  LOG_DEBUG("PSM2 DH Table set to: " << this->DaVinci->GetPsm2()->GetDhTableAsString());
//  this->DaVinci->GetEcm()->SetDhTable(isiEcmDhTable);
//  LOG_DEBUG("ECM DH Table set to: " << this->DaVinci->GetEcm()->GetDhTableAsString());
//
//  return PLUS_SUCCESS;
//}

//----------------------------------------------------------------------------
PlusStatus vtkPlusIntuitiveDaVinciTracker::WriteConfiguration(vtkXMLDataElement* rootConfigElement)
{
  LOG_DEBUG("vtkPlusIntuitiveDaVinciTracker::WriteConfiguration");
  XML_FIND_DEVICE_ELEMENT_REQUIRED_FOR_WRITING(trackerConfig, rootConfigElement);

  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
//IntuitiveDaVinci* vtkPlusIntuitiveDaVinciTracker::GetDaVinci() const
//{
//  return this->DaVinci;
//}

//----------------------------------------------------------------------------
//void vtkPlusIntuitiveDaVinciTracker::ConvertIsiTransformToVtkMatrix(ISI_TRANSFORM* srcIsiMatrix, vtkMatrix4x4& destVtkMatrix)
//{
//  destVtkMatrix.Identity();
//
//  // Let's VERY EXPLCITLY copy over the values.
//  destVtkMatrix.SetElement(0, 0, srcIsiMatrix->rot.row0.x);
//  destVtkMatrix.SetElement(0, 1, srcIsiMatrix->rot.row0.y);
//  destVtkMatrix.SetElement(0, 2, srcIsiMatrix->rot.row0.z);
//  destVtkMatrix.SetElement(0, 3, srcIsiMatrix->pos.x);
//
//  destVtkMatrix.SetElement(1, 0, srcIsiMatrix->rot.row1.x);
//  destVtkMatrix.SetElement(1, 1, srcIsiMatrix->rot.row1.y);
//  destVtkMatrix.SetElement(1, 2, srcIsiMatrix->rot.row1.z);
//  destVtkMatrix.SetElement(1, 3, srcIsiMatrix->pos.y);
//
//  destVtkMatrix.SetElement(2, 0, srcIsiMatrix->rot.row2.x);
//  destVtkMatrix.SetElement(2, 1, srcIsiMatrix->rot.row2.y);
//  destVtkMatrix.SetElement(2, 2, srcIsiMatrix->rot.row2.z);
//  destVtkMatrix.SetElement(2, 3, srcIsiMatrix->pos.z);
//
//  return;
//}