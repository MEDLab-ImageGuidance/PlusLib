/*=Plus=header=begin======================================================
Program: Plus
Copyright (c) Laboratory for Percutaneous Surgery. All rights reserved.
See License.txt for details.
=========================================================Plus=header=end*/

#include "igsioCommon.h"
#include "PlusConfigure.h"
#include "vtkPlusDataSource.h"
#include "vtkPlusIntuitiveDaVinciTrackerXi.h"

#include <vtkImageData.h>
#include <vtkMath.h>
#include <vtkMatrix4x4.h>
#include <vtkTransform.h>

//----------------------------------------------------------------------------

vtkStandardNewMacro(vtkPlusIntuitiveDaVinciTrackerXi);

//----------------------------------------------------------------------------
vtkPlusIntuitiveDaVinciTrackerXi::vtkPlusIntuitiveDaVinciTrackerXi()
  : vtkPlusDevice()
  , DaVinci(new IntuitiveDaVinciXi())
  , LastFrameNumber(0)
  , DebugSineWaveMode(false)
  , psm1Joints(NULL), psm2Joints(NULL), ecmJoints(NULL)
  , psm1Base(NULL),   psm2Base(NULL),   ecmBase(NULL)
  , psm1Frame1(NULL), psm1Frame2(NULL), psm1Frame3(NULL), psm1Frame4(NULL), psm1Frame5(NULL), psm1Frame6(NULL), psm1Frame7(NULL)
  , psm2Frame1(NULL), psm2Frame2(NULL), psm2Frame3(NULL), psm2Frame4(NULL), psm2Frame5(NULL), psm2Frame6(NULL), psm2Frame7(NULL)
  , ecmFrame1(NULL),  ecmFrame2(NULL),  ecmFrame3(NULL),  ecmFrame4(NULL),  ecmFrame5(NULL),  ecmFrame6(NULL),  ecmFrame7(NULL)
{
  this->StartThreadForInternalUpdates = true; // Want a dedicated thread
  this->RequirePortNameInDeviceSetConfiguration = true;
  this->AcquisitionRate = 50;

  LOG_DEBUG("vktPlusIntuitiveDaVinciTrackerXi created.");
}

//----------------------------------------------------------------------------
vtkPlusIntuitiveDaVinciTrackerXi::~vtkPlusIntuitiveDaVinciTrackerXi()
{
  this->StopRecording();
  this->Disconnect();

  delete this->DaVinci;
  this->DaVinci = nullptr;

  LOG_DEBUG("vktPlusIntuitiveDaVinciTrackerXi destroyed.");
}

//----------------------------------------------------------------------------
void vtkPlusIntuitiveDaVinciTrackerXi::PrintSelf(ostream& os, vtkIndent indent)
{

}

//----------------------------------------------------------------------------
PlusStatus vtkPlusIntuitiveDaVinciTrackerXi::Probe()
{
  LOG_DEBUG("Probing vtkPlusIntuitiveDaVinciTrackerXi.");

  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusIntuitiveDaVinciTrackerXi::InternalConnect()
{
  LOG_DEBUG("vtkPlusIntuitiveDaVinciTrackerXi::InternalConnect");

  if (this->Connected)
  {
    LOG_WARNING("Cannot run DaVinci->Connect because already connected to da Vinci Xi.");
    return PLUS_SUCCESS;
  }

  ISI_STATUS status;

  if (this->DebugSineWaveMode)
    status = this->DaVinci->ConnectDebugSineWaveMode();
  else
    status = this->DaVinci->Connect();

  if (status != ISI_SUCCESS)
  {
    LOG_ERROR("Failed to connect to da Vinci Xi device.");
    return PLUS_FAIL;
  }

  GetToolByPortName("psm1Joints", this->psm1Joints);
  GetToolByPortName("psm2Joints", this->psm2Joints);
  GetToolByPortName("ecmJoints", this->ecmJoints);

  GetToolByPortName("psm1Base", this->psm1Base);
  GetToolByPortName("psm2Base", this->psm2Base);
  GetToolByPortName("ecmBase", this->ecmBase);

  GetToolByPortName("psm1Frame1", this->psm1Frame1);
  GetToolByPortName("psm1Frame2", this->psm1Frame2);
  GetToolByPortName("psm1Frame3", this->psm1Frame3);
  GetToolByPortName("psm1Frame4", this->psm1Frame4);
  GetToolByPortName("psm1Frame5", this->psm1Frame5);
  GetToolByPortName("psm1Frame6", this->psm1Frame6);
  GetToolByPortName("psm1Frame7", this->psm1Frame7);

  GetToolByPortName("psm2Frame1", this->psm2Frame1);
  GetToolByPortName("psm2Frame2", this->psm2Frame2);
  GetToolByPortName("psm2Frame3", this->psm2Frame3);
  GetToolByPortName("psm2Frame4", this->psm2Frame4);
  GetToolByPortName("psm2Frame5", this->psm2Frame5);
  GetToolByPortName("psm2Frame6", this->psm2Frame6);
  GetToolByPortName("psm2Frame7", this->psm2Frame7);

  GetToolByPortName("ecmFrame1", this->ecmFrame1);
  GetToolByPortName("ecmFrame2", this->ecmFrame2);
  GetToolByPortName("ecmFrame3", this->ecmFrame3);
  GetToolByPortName("ecmFrame4", this->ecmFrame4);
  GetToolByPortName("ecmFrame5", this->ecmFrame5);
  GetToolByPortName("ecmFrame6", this->ecmFrame6);
  GetToolByPortName("ecmFrame7", this->ecmFrame7);

  LOG_DEBUG("Connection successful.");

  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusIntuitiveDaVinciTrackerXi::InternalStartRecording()
{
  if (!this->Connected)
  {
    LOG_ERROR("InternalStartRecording failed: da Vinci Xi has not been initialized.");
    return PLUS_FAIL;
  }

  if (!this->DaVinci->IsConnected())
  {
    LOG_ERROR("InternalStartRecording failed: da Vinci Xi is not connected.");
    return PLUS_FAIL;
  }
  
  ISI_STATUS status;

  if (this->DebugSineWaveMode)
    status = this->DaVinci->StartDebugSineWaveMode();
  else
    status = this->DaVinci->Start();

  if (status != ISI_SUCCESS)
  {
    LOG_ERROR("InternalStartRecording: Unable to start streaming.");
    return PLUS_FAIL;
  }

  LOG_DEBUG("InternalStartRecording started.");
  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusIntuitiveDaVinciTrackerXi::InternalUpdate()
{
  this->LastFrameNumber++;
  const double toolTimestamp = vtkIGSIOAccurateTimer::GetSystemTime(); // unfiltered timestamp

  ISI_STATUS status;
  // Update the robot joint values
  if (this->DebugSineWaveMode)
    status = this->DaVinci->UpdateAllJointValuesSineWave();
  else
    status = this->DaVinci->UpdateAllJointValues();

  if (status != ISI_SUCCESS)
  {
    LOG_ERROR("Could not update da Vinci Xi joint values.");
    return PLUS_FAIL;
  }

  // Update the kinematics transforms
  status = this->DaVinci->UpdateAllKinematicsTransforms();

  if (status != ISI_SUCCESS)
  {
    LOG_ERROR("Could not update da Vinci Xi kinematics.");
    return PLUS_FAIL;
  }

  // We will need these to copy data values 
  vtkSmartPointer<vtkMatrix4x4> tmpVtkMatrix = vtkSmartPointer<vtkMatrix4x4>::New();

  // Update all of the manipulator joint values
  ISI_FLOAT* jointValues;

  jointValues = this->DaVinci->GetPsm1()->GetJointValues();
  tmpVtkMatrix->Identity();
  tmpVtkMatrix->SetElement(0, 0, jointValues[0]);
  tmpVtkMatrix->SetElement(0, 1, jointValues[1]);
  tmpVtkMatrix->SetElement(0, 2, jointValues[2]);
  tmpVtkMatrix->SetElement(0, 3, jointValues[3]);
  tmpVtkMatrix->SetElement(1, 0, jointValues[4]);
  tmpVtkMatrix->SetElement(1, 1, jointValues[5]);
  tmpVtkMatrix->SetElement(1, 2, jointValues[6]);
  unsigned long frameNumber = psm1Joints->GetFrameNumber() + 1;
  ToolTimeStampedUpdate(psm1Joints->GetId(), tmpVtkMatrix, TOOL_OK, frameNumber, toolTimestamp);

  jointValues = this->DaVinci->GetPsm2()->GetJointValues();
  tmpVtkMatrix->Identity();
  tmpVtkMatrix->SetElement(0, 0, jointValues[0]);
  tmpVtkMatrix->SetElement(0, 1, jointValues[1]);
  tmpVtkMatrix->SetElement(0, 2, jointValues[2]);
  tmpVtkMatrix->SetElement(0, 3, jointValues[3]);
  tmpVtkMatrix->SetElement(1, 0, jointValues[4]);
  tmpVtkMatrix->SetElement(1, 1, jointValues[5]);
  tmpVtkMatrix->SetElement(1, 2, jointValues[6]);
  frameNumber = psm2Joints->GetFrameNumber() + 1;
  ToolTimeStampedUpdate(psm2Joints->GetId(), tmpVtkMatrix, TOOL_OK, frameNumber, toolTimestamp);

  jointValues = this->DaVinci->GetEcm()->GetJointValues();
  tmpVtkMatrix->Identity();
  tmpVtkMatrix->SetElement(0, 0, jointValues[0]);
  tmpVtkMatrix->SetElement(0, 1, jointValues[1]);
  tmpVtkMatrix->SetElement(0, 2, jointValues[2]);
  tmpVtkMatrix->SetElement(0, 3, jointValues[3]);
  frameNumber = ecmJoints->GetFrameNumber() + 1;
  ToolTimeStampedUpdate(ecmJoints->GetId(), tmpVtkMatrix, TOOL_OK, frameNumber, toolTimestamp);

  // Update all of the manipulator base frames
  PUBLISH_ISI_TRANSFORM(psm1Base, this->DaVinci->GetPsm1BaseToWorld());
  PUBLISH_ISI_TRANSFORM(psm2Base, this->DaVinci->GetPsm2BaseToWorld());
  PUBLISH_ISI_TRANSFORM(ecmBase, this->DaVinci->GetEcmBaseToWorld());

  // Update all of the psm1Frames
  ISI_TRANSFORM* psm1Transforms = this->DaVinci->GetPsm1()->GetTransforms();

  PUBLISH_ISI_TRANSFORM(psm1Frame1, psm1Transforms + 0);
  PUBLISH_ISI_TRANSFORM(psm1Frame2, psm1Transforms + 1);
  PUBLISH_ISI_TRANSFORM(psm1Frame3, psm1Transforms + 2);
  PUBLISH_ISI_TRANSFORM(psm1Frame4, psm1Transforms + 3);
  PUBLISH_ISI_TRANSFORM(psm1Frame5, psm1Transforms + 4);
  PUBLISH_ISI_TRANSFORM(psm1Frame6, psm1Transforms + 5);
  PUBLISH_ISI_TRANSFORM(psm1Frame7, psm1Transforms + 6);

  // Update all of the psm2Frames
  ISI_TRANSFORM* psm2Transforms = this->DaVinci->GetPsm2()->GetTransforms();

  PUBLISH_ISI_TRANSFORM(psm2Frame1, psm2Transforms + 0);
  PUBLISH_ISI_TRANSFORM(psm2Frame2, psm2Transforms + 1);
  PUBLISH_ISI_TRANSFORM(psm2Frame3, psm2Transforms + 2);
  PUBLISH_ISI_TRANSFORM(psm2Frame4, psm2Transforms + 3);
  PUBLISH_ISI_TRANSFORM(psm2Frame5, psm2Transforms + 4);
  PUBLISH_ISI_TRANSFORM(psm2Frame6, psm2Transforms + 5);
  PUBLISH_ISI_TRANSFORM(psm2Frame7, psm2Transforms + 6);

  // Update all of the ecmFrames
  ISI_TRANSFORM* ecmTransforms = this->DaVinci->GetEcm()->GetTransforms();

  PUBLISH_ISI_TRANSFORM(ecmFrame1, ecmTransforms + 0);
  PUBLISH_ISI_TRANSFORM(ecmFrame2, ecmTransforms + 1);
  PUBLISH_ISI_TRANSFORM(ecmFrame3, ecmTransforms + 2);
  PUBLISH_ISI_TRANSFORM(ecmFrame4, ecmTransforms + 3);
  PUBLISH_ISI_TRANSFORM(ecmFrame5, ecmTransforms + 4);
  PUBLISH_ISI_TRANSFORM(ecmFrame6, ecmTransforms + 5);
  PUBLISH_ISI_TRANSFORM(ecmFrame7, ecmTransforms + 6);

  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusIntuitiveDaVinciTrackerXi::InternalStopRecording()
{
  // Stop the stream from the da Vinci Xi.
  this->DaVinci->Stop();

  LOG_DEBUG("InternalStartRecording stopped.");
  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusIntuitiveDaVinciTrackerXi::InternalDisconnect()
{
  this->DaVinci->Disconnect();

  LOG_DEBUG("Disconnected from da Vinci Xi device.")
  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusIntuitiveDaVinciTrackerXi::ReadConfiguration(vtkXMLDataElement* rootConfigElement)
{
  LOG_DEBUG("vtkPlusIntuitiveDaVinciTrackerXi::ReadConfiguration");

  XML_FIND_DEVICE_ELEMENT_REQUIRED_FOR_READING(deviceConfig, rootConfigElement);

  XML_READ_SCALAR_ATTRIBUTE_WARNING(int, AcquisitionRate, deviceConfig); 
  XML_READ_BOOL_ATTRIBUTE_OPTIONAL(DebugSineWaveMode, deviceConfig);

  std::string psm1DhTable;
  std::string psm2DhTable;
  std::string ecmDhTable;

  XML_READ_STRING_ATTRIBUTE_NONMEMBER_REQUIRED(Psm1DhTable, psm1DhTable, deviceConfig);
  XML_READ_STRING_ATTRIBUTE_NONMEMBER_REQUIRED(Psm2DhTable, psm2DhTable, deviceConfig);
  XML_READ_STRING_ATTRIBUTE_NONMEMBER_REQUIRED(EcmDhTable, ecmDhTable, deviceConfig);

  PlusStatus status = SetDhTablesFromStrings(psm1DhTable, psm2DhTable, ecmDhTable);

  if (status != PLUS_SUCCESS)
  {
    LOG_ERROR("Check the formatting of the DH tables.");
    return status;
  }
	
  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
static bool BothAreSpaces(char lhs, char rhs)
{
	return (lhs == rhs) && (lhs == ' ');
}

//----------------------------------------------------------------------------
static void ProcessDhString(std::string& str)
{
	std::vector<std::string> strTokens;

	// Remove all the new lines from the string
	str.erase(std::remove(str.begin(), str.end(), '\n'), str.end());
	// Remove all tabs 
	str.erase(std::remove(str.begin(), str.end(), '\t'), str.end());
	// Trim the beginning and end
	str = igsioCommon::Trim(str);

	// Remove all double/triple spaces
	std::string::iterator new_end = std::unique(str.begin(), str.end(), BothAreSpaces);
	str.erase(new_end, str.end());
}

static void ConvertTokenVectorToDhTable(std::vector<std::string>& srcTokenVector, ISI_DH_ROW* destIsiDhTable)
{
  for (int iii = 0; iii < 7; iii++)
  {
    try
    {
      destIsiDhTable[iii].type = (ISI_FLOAT)std::stof(srcTokenVector[7 * iii + 0]);
      destIsiDhTable[iii].l = (ISI_FLOAT)std::stof(srcTokenVector[7 * iii + 1]);
      destIsiDhTable[iii].sina = (ISI_FLOAT)std::stof(srcTokenVector[7 * iii + 2]);
      destIsiDhTable[iii].cosa = (ISI_FLOAT)std::stof(srcTokenVector[7 * iii + 3]);
      destIsiDhTable[iii].d = (ISI_FLOAT)std::stof(srcTokenVector[7 * iii + 4]);
      destIsiDhTable[iii].sinq = (ISI_FLOAT)std::stof(srcTokenVector[7 * iii + 5]);
      destIsiDhTable[iii].cosq = (ISI_FLOAT)std::stof(srcTokenVector[7 * iii + 6]);
    }
    catch (...)
    {
      LOG_ERROR("Check input DH table input in config file.");
    }
  }
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusIntuitiveDaVinciTrackerXi::SetDhTablesFromStrings(std::string psm1DhTable, std::string psm2DhTable, std::string ecmDhTable)
{
  std::vector<std::string> psm1TokenVector, psm2TokenVector, ecmTokenVector;
 
  const int numDhRows = 7; const int numDhCols = 7;
  int numElem = numDhRows*numDhCols;

  ProcessDhString(psm1DhTable);
  ProcessDhString(psm2DhTable);
  ProcessDhString(ecmDhTable);

  psm1TokenVector = igsioCommon::SplitStringIntoTokens(psm1DhTable, ' ');
  psm2TokenVector = igsioCommon::SplitStringIntoTokens(psm2DhTable, ' ');
  ecmTokenVector = igsioCommon::SplitStringIntoTokens(ecmDhTable, ' ');

  if((psm1TokenVector.size() != numElem) || 
	   (psm2TokenVector.size() != numElem) ||
	   (ecmTokenVector.size() != numElem))
  {
	  LOG_ERROR("Invalid formatting of DH table string. Must have " << numElem << "elements.");
	  return PLUS_FAIL;
  }

  ISI_DH_ROW isiPsm1DhTable[numDhRows];
  ISI_DH_ROW isiPsm2DhTable[numDhRows];
  ISI_DH_ROW isiEcmDhTable[numDhRows];

  ConvertTokenVectorToDhTable(psm1TokenVector, isiPsm1DhTable);
  ConvertTokenVectorToDhTable(psm2TokenVector, isiPsm2DhTable);
  ConvertTokenVectorToDhTable(ecmTokenVector, isiEcmDhTable);

  this->DaVinci->GetPsm1()->SetDhTable(isiPsm1DhTable);
  LOG_DEBUG("PSM1 DH Table set to: " << this->DaVinci->GetPsm1()->GetDhTableAsString());
  this->DaVinci->GetPsm2()->SetDhTable(isiPsm2DhTable);
  LOG_DEBUG("PSM2 DH Table set to: " << this->DaVinci->GetPsm2()->GetDhTableAsString());
  this->DaVinci->GetEcm()->SetDhTable(isiEcmDhTable);
  LOG_DEBUG("ECM DH Table set to: " << this->DaVinci->GetEcm()->GetDhTableAsString());

  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusIntuitiveDaVinciTrackerXi::WriteConfiguration(vtkXMLDataElement* rootConfigElement)
{
  LOG_DEBUG("vtkPlusIntuitiveDaVinciTrackerXi::WriteConfiguration");
  XML_FIND_DEVICE_ELEMENT_REQUIRED_FOR_WRITING(trackerConfig, rootConfigElement);

  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
IntuitiveDaVinciXi* vtkPlusIntuitiveDaVinciTrackerXi::GetDaVinci() const
{
  return this->DaVinci;
}

//----------------------------------------------------------------------------
void vtkPlusIntuitiveDaVinciTrackerXi::ConvertIsiTransformToVtkMatrix(ISI_TRANSFORM* srcIsiMatrix, vtkMatrix4x4& destVtkMatrix)
{
  destVtkMatrix.Identity();

  // Let's VERY EXPLCITLY copy over the values.
  destVtkMatrix.SetElement(0, 0, srcIsiMatrix->rot.row0.x);
  destVtkMatrix.SetElement(0, 1, srcIsiMatrix->rot.row0.y);
  destVtkMatrix.SetElement(0, 2, srcIsiMatrix->rot.row0.z);
  destVtkMatrix.SetElement(0, 3, srcIsiMatrix->pos.x);

  destVtkMatrix.SetElement(1, 0, srcIsiMatrix->rot.row1.x);
  destVtkMatrix.SetElement(1, 1, srcIsiMatrix->rot.row1.y);
  destVtkMatrix.SetElement(1, 2, srcIsiMatrix->rot.row1.z);
  destVtkMatrix.SetElement(1, 3, srcIsiMatrix->pos.y);

  destVtkMatrix.SetElement(2, 0, srcIsiMatrix->rot.row2.x);
  destVtkMatrix.SetElement(2, 1, srcIsiMatrix->rot.row2.y);
  destVtkMatrix.SetElement(2, 2, srcIsiMatrix->rot.row2.z);
  destVtkMatrix.SetElement(2, 3, srcIsiMatrix->pos.z);

  return;
}