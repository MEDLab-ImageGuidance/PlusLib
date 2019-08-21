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
  , DebugSineWaveMode(true)
	, usm1Joints(NULL), usm2Joints(NULL), usm3Joints(NULL), usm4Joints(NULL)
	, usm1Frame1(NULL), usm1Frame2(NULL), usm1Frame3(NULL), usm1Frame4(NULL), usm1Frame5(NULL), usm1Frame6(NULL), usm1Frame7(NULL), usm1Frame8(NULL), usm1Frame9(NULL)
	, usm2Frame1(NULL), usm2Frame2(NULL), usm2Frame3(NULL), usm2Frame4(NULL), usm2Frame5(NULL), usm2Frame6(NULL), usm2Frame7(NULL), usm2Frame8(NULL), usm2Frame9(NULL)
	, usm3Frame1(NULL), usm3Frame2(NULL), usm3Frame3(NULL), usm3Frame4(NULL), usm3Frame5(NULL), usm3Frame6(NULL), usm3Frame7(NULL), usm3Frame8(NULL), usm3Frame9(NULL)
	, usm4Frame1(NULL), usm4Frame2(NULL), usm4Frame3(NULL), usm4Frame4(NULL), usm4Frame5(NULL), usm4Frame6(NULL), usm4Frame7(NULL), usm4Frame8(NULL), usm4Frame9(NULL)
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

  GetToolByPortName("usm1Joints", this->usm1Joints);
  GetToolByPortName("usm2Joints", this->usm2Joints);
	GetToolByPortName("usm3Joints", this->usm3Joints);
	GetToolByPortName("usm4Joints", this->usm4Joints);

  GetToolByPortName("usm1Frame1", this->usm1Frame1);
  GetToolByPortName("usm1Frame2", this->usm1Frame2);
  GetToolByPortName("usm1Frame3", this->usm1Frame3);
  GetToolByPortName("usm1Frame4", this->usm1Frame4);
  GetToolByPortName("usm1Frame5", this->usm1Frame5);
  GetToolByPortName("usm1Frame6", this->usm1Frame6);
  GetToolByPortName("usm1Frame7", this->usm1Frame7);
	GetToolByPortName("usm1Frame8", this->usm1Frame8);
	GetToolByPortName("usm1Frame9", this->usm1Frame9);

  GetToolByPortName("usm2Frame1", this->usm2Frame1);
  GetToolByPortName("usm2Frame2", this->usm2Frame2);
  GetToolByPortName("usm2Frame3", this->usm2Frame3);
  GetToolByPortName("usm2Frame4", this->usm2Frame4);
  GetToolByPortName("usm2Frame5", this->usm2Frame5);
  GetToolByPortName("usm2Frame6", this->usm2Frame6);
  GetToolByPortName("usm2Frame7", this->usm2Frame7);
	GetToolByPortName("usm2Frame8", this->usm2Frame8);
	GetToolByPortName("usm2Frame9", this->usm2Frame9);

	GetToolByPortName("usm3Frame1", this->usm3Frame1);
	GetToolByPortName("usm3Frame2", this->usm3Frame2);
	GetToolByPortName("usm3Frame3", this->usm3Frame3);
	GetToolByPortName("usm3Frame4", this->usm3Frame4);
	GetToolByPortName("usm3Frame5", this->usm3Frame5);
	GetToolByPortName("usm3Frame6", this->usm3Frame6);
	GetToolByPortName("usm3Frame7", this->usm3Frame7);
	GetToolByPortName("usm3Frame8", this->usm3Frame8);
	GetToolByPortName("usm3Frame9", this->usm3Frame9);

	GetToolByPortName("usm4Frame1", this->usm4Frame1);
	GetToolByPortName("usm4Frame2", this->usm4Frame2);
	GetToolByPortName("usm4Frame3", this->usm4Frame3);
	GetToolByPortName("usm4Frame4", this->usm4Frame4);
	GetToolByPortName("usm4Frame5", this->usm4Frame5);
	GetToolByPortName("usm4Frame6", this->usm4Frame6);
	GetToolByPortName("usm4Frame7", this->usm4Frame7);
	GetToolByPortName("usm4Frame8", this->usm4Frame8);
	GetToolByPortName("usm4Frame9", this->usm4Frame9);

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

	jointValues = this->DaVinci->GetUsm1()->GetJointValues();
  tmpVtkMatrix->Identity();
  tmpVtkMatrix->SetElement(0, 0, jointValues[0]);
  tmpVtkMatrix->SetElement(0, 1, jointValues[1]);
  tmpVtkMatrix->SetElement(0, 2, jointValues[2]);
  tmpVtkMatrix->SetElement(0, 3, jointValues[3]);
  tmpVtkMatrix->SetElement(1, 0, jointValues[4]);
  tmpVtkMatrix->SetElement(1, 1, jointValues[5]);
  tmpVtkMatrix->SetElement(1, 2, jointValues[6]);
	tmpVtkMatrix->SetElement(1, 3, jointValues[7]);
	tmpVtkMatrix->SetElement(2, 0, jointValues[8]);
  unsigned long frameNumber = usm1Joints->GetFrameNumber() + 1;
  ToolTimeStampedUpdate(usm1Joints->GetId(), tmpVtkMatrix, TOOL_OK, frameNumber, toolTimestamp);

  jointValues = this->DaVinci->GetUsm2()->GetJointValues();
  tmpVtkMatrix->Identity();
  tmpVtkMatrix->SetElement(0, 0, jointValues[0]);
  tmpVtkMatrix->SetElement(0, 1, jointValues[1]);
  tmpVtkMatrix->SetElement(0, 2, jointValues[2]);
  tmpVtkMatrix->SetElement(0, 3, jointValues[3]);
  tmpVtkMatrix->SetElement(1, 0, jointValues[4]);
  tmpVtkMatrix->SetElement(1, 1, jointValues[5]);
  tmpVtkMatrix->SetElement(1, 2, jointValues[6]);
	tmpVtkMatrix->SetElement(1, 3, jointValues[7]);
	tmpVtkMatrix->SetElement(2, 0, jointValues[8]);
  frameNumber = usm2Joints->GetFrameNumber() + 1;
  ToolTimeStampedUpdate(usm2Joints->GetId(), tmpVtkMatrix, TOOL_OK, frameNumber, toolTimestamp);

	jointValues = this->DaVinci->GetUsm3()->GetJointValues();
	tmpVtkMatrix->Identity();
	tmpVtkMatrix->SetElement(0, 0, jointValues[0]);
	tmpVtkMatrix->SetElement(0, 1, jointValues[1]);
	tmpVtkMatrix->SetElement(0, 2, jointValues[2]);
	tmpVtkMatrix->SetElement(0, 3, jointValues[3]);
	tmpVtkMatrix->SetElement(1, 0, jointValues[4]);
	tmpVtkMatrix->SetElement(1, 1, jointValues[5]);
	tmpVtkMatrix->SetElement(1, 2, jointValues[6]);
	tmpVtkMatrix->SetElement(1, 3, jointValues[7]);
	tmpVtkMatrix->SetElement(2, 0, jointValues[8]);
	frameNumber = usm3Joints->GetFrameNumber() + 1;
	ToolTimeStampedUpdate(usm3Joints->GetId(), tmpVtkMatrix, TOOL_OK, frameNumber, toolTimestamp);

	jointValues = this->DaVinci->GetUsm4()->GetJointValues();
	tmpVtkMatrix->Identity();
	tmpVtkMatrix->SetElement(0, 0, jointValues[0]);
	tmpVtkMatrix->SetElement(0, 1, jointValues[1]);
	tmpVtkMatrix->SetElement(0, 2, jointValues[2]);
	tmpVtkMatrix->SetElement(0, 3, jointValues[3]);
	tmpVtkMatrix->SetElement(1, 0, jointValues[4]);
	tmpVtkMatrix->SetElement(1, 1, jointValues[5]);
	tmpVtkMatrix->SetElement(1, 2, jointValues[6]);
	tmpVtkMatrix->SetElement(1, 3, jointValues[7]);
	tmpVtkMatrix->SetElement(2, 0, jointValues[8]);
	frameNumber = usm4Joints->GetFrameNumber() + 1;
	ToolTimeStampedUpdate(usm4Joints->GetId(), tmpVtkMatrix, TOOL_OK, frameNumber, toolTimestamp);

  // Update all of the usm1Frames
  ISI_TRANSFORM* usm1Transforms = this->DaVinci->GetUsm1()->GetTransforms();

	PUBLISH_ISI_TRANSFORM(usm1Frame1, usm1Transforms + 0);
	PUBLISH_ISI_TRANSFORM(usm1Frame2, usm1Transforms + 1);
	PUBLISH_ISI_TRANSFORM(usm1Frame3, usm1Transforms + 2);
	PUBLISH_ISI_TRANSFORM(usm1Frame4, usm1Transforms + 3);
	PUBLISH_ISI_TRANSFORM(usm1Frame5, usm1Transforms + 4);
	PUBLISH_ISI_TRANSFORM(usm1Frame6, usm1Transforms + 5);
	PUBLISH_ISI_TRANSFORM(usm1Frame7, usm1Transforms + 6);
	PUBLISH_ISI_TRANSFORM(usm1Frame8, usm1Transforms + 7);
	PUBLISH_ISI_TRANSFORM(usm1Frame9, usm1Transforms + 8);

  // Update all of the usm2Frames
	ISI_TRANSFORM* usm2Transforms = this->DaVinci->GetUsm2()->GetTransforms();

	PUBLISH_ISI_TRANSFORM(usm2Frame1, usm2Transforms + 0);
	PUBLISH_ISI_TRANSFORM(usm2Frame2, usm2Transforms + 1);
	PUBLISH_ISI_TRANSFORM(usm2Frame3, usm2Transforms + 2);
	PUBLISH_ISI_TRANSFORM(usm2Frame4, usm2Transforms + 3);
	PUBLISH_ISI_TRANSFORM(usm2Frame5, usm2Transforms + 4);
	PUBLISH_ISI_TRANSFORM(usm2Frame6, usm2Transforms + 5);
	PUBLISH_ISI_TRANSFORM(usm2Frame7, usm2Transforms + 6);
	PUBLISH_ISI_TRANSFORM(usm2Frame8, usm2Transforms + 7);
	PUBLISH_ISI_TRANSFORM(usm2Frame9, usm2Transforms + 8);

	// Update all of the usm2Frames
	ISI_TRANSFORM* usm3Transforms = this->DaVinci->GetUsm3()->GetTransforms();

	PUBLISH_ISI_TRANSFORM(usm3Frame1, usm3Transforms + 0);
	PUBLISH_ISI_TRANSFORM(usm3Frame2, usm3Transforms + 1);
	PUBLISH_ISI_TRANSFORM(usm3Frame3, usm3Transforms + 2);
	PUBLISH_ISI_TRANSFORM(usm3Frame4, usm3Transforms + 3);
	PUBLISH_ISI_TRANSFORM(usm3Frame5, usm3Transforms + 4);
	PUBLISH_ISI_TRANSFORM(usm3Frame6, usm3Transforms + 5);
	PUBLISH_ISI_TRANSFORM(usm3Frame7, usm3Transforms + 6);
	PUBLISH_ISI_TRANSFORM(usm3Frame8, usm3Transforms + 7);
	PUBLISH_ISI_TRANSFORM(usm3Frame9, usm3Transforms + 8);

	// Update all of the usm2Frames
	ISI_TRANSFORM* usm4Transforms = this->DaVinci->GetUsm4()->GetTransforms();

	PUBLISH_ISI_TRANSFORM(usm4Frame1, usm4Transforms + 0);
	PUBLISH_ISI_TRANSFORM(usm4Frame2, usm4Transforms + 1);
	PUBLISH_ISI_TRANSFORM(usm4Frame3, usm4Transforms + 2);
	PUBLISH_ISI_TRANSFORM(usm4Frame4, usm4Transforms + 3);
	PUBLISH_ISI_TRANSFORM(usm4Frame5, usm4Transforms + 4);
	PUBLISH_ISI_TRANSFORM(usm4Frame6, usm4Transforms + 5);
	PUBLISH_ISI_TRANSFORM(usm4Frame7, usm4Transforms + 6);
	PUBLISH_ISI_TRANSFORM(usm4Frame8, usm4Transforms + 7);
	PUBLISH_ISI_TRANSFORM(usm4Frame9, usm4Transforms + 8);

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

  LOG_DEBUG("Disconnected from da Vinci Xi device.");
  return PLUS_SUCCESS;
}

//----------------------------------------------------------------------------
PlusStatus vtkPlusIntuitiveDaVinciTrackerXi::ReadConfiguration(vtkXMLDataElement* rootConfigElement)
{
  LOG_DEBUG("vtkPlusIntuitiveDaVinciTrackerXi::ReadConfiguration");

  XML_FIND_DEVICE_ELEMENT_REQUIRED_FOR_READING(deviceConfig, rootConfigElement);

  XML_READ_SCALAR_ATTRIBUTE_WARNING(int, AcquisitionRate, deviceConfig); 
  XML_READ_BOOL_ATTRIBUTE_OPTIONAL(DebugSineWaveMode, deviceConfig);

  std::string usm1DhTable;
  std::string usm2DhTable;
	std::string usm3DhTable;
	std::string usm4DhTable;

  XML_READ_STRING_ATTRIBUTE_NONMEMBER_REQUIRED(Usm1DhTable, usm1DhTable, deviceConfig);
  XML_READ_STRING_ATTRIBUTE_NONMEMBER_REQUIRED(Usm2DhTable, usm2DhTable, deviceConfig);
	XML_READ_STRING_ATTRIBUTE_NONMEMBER_REQUIRED(Usm3DhTable, usm3DhTable, deviceConfig);
	XML_READ_STRING_ATTRIBUTE_NONMEMBER_REQUIRED(Usm4DhTable, usm4DhTable, deviceConfig);

	PlusStatus status = SetDhTablesFromStrings(usm1DhTable, usm2DhTable, usm3DhTable, usm4DhTable);

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
	

	for (int iii = 0; iii < 9; iii++)
  {
    try
    {
			// while reaiding for first joint iii = 0 and will read the first row. And so on...
			destIsiDhTable[iii].type = (ISI_FLOAT)std::stof(srcTokenVector[9 * iii + 0]);
			destIsiDhTable[iii].l = (ISI_FLOAT)std::stof(srcTokenVector[9 * iii + 1]);
			destIsiDhTable[iii].sina = (ISI_FLOAT)std::stof(srcTokenVector[9 * iii + 2]);
			destIsiDhTable[iii].cosa = (ISI_FLOAT)std::stof(srcTokenVector[9 * iii + 3]);
			destIsiDhTable[iii].d = (ISI_FLOAT)std::stof(srcTokenVector[9 * iii + 4]);
			destIsiDhTable[iii].sinq = (ISI_FLOAT)std::stof(srcTokenVector[9 * iii + 5]);
			destIsiDhTable[iii].cosq = (ISI_FLOAT)std::stof(srcTokenVector[9 * iii + 6]);
    }
    catch (...)
    {
      LOG_ERROR("Check input DH table input in config file.");
    }
  }
}

PlusStatus vtkPlusIntuitiveDaVinciTrackerXi::SetDhTablesFromStrings(std::string usm1DhTable, std::string usm2DhTable, 
	std::string usm3DhTable,std::string usm4DhTable)
{
	std::vector<std::string> usm1TokenVector, usm2TokenVector, usm3TokenVector, usm4TokenVector;

	const int numDhRows = 9; const int numDhCols = 7;
	int numElem = numDhRows*numDhCols;

	ProcessDhString(usm1DhTable);
	ProcessDhString(usm2DhTable);
	ProcessDhString(usm3DhTable);
	ProcessDhString(usm4DhTable);

	usm1TokenVector = igsioCommon::SplitStringIntoTokens(usm1DhTable, ' ');
	usm2TokenVector = igsioCommon::SplitStringIntoTokens(usm2DhTable, ' ');
	usm3TokenVector = igsioCommon::SplitStringIntoTokens(usm3DhTable, ' ');
	usm4TokenVector = igsioCommon::SplitStringIntoTokens(usm4DhTable, ' ');
	if ((usm1TokenVector.size() != numElem) || (usm2TokenVector.size() != numElem) ||
		(usm3TokenVector.size() != numElem) || (usm4TokenVector.size() != numElem))
	{
		LOG_ERROR("Invalid formatting of DH table string. Must have " << numElem << "elements.");
		return PLUS_FAIL;
	}

	ISI_DH_ROW isiUsm1DhTable[numDhRows];
	ISI_DH_ROW isiUsm2DhTable[numDhRows];
	ISI_DH_ROW isiUsm3DhTable[numDhRows];
	ISI_DH_ROW isiUsm4DhTable[numDhRows];

	ConvertTokenVectorToDhTable(usm1TokenVector, isiUsm1DhTable);
	ConvertTokenVectorToDhTable(usm2TokenVector, isiUsm2DhTable);
	ConvertTokenVectorToDhTable(usm3TokenVector, isiUsm3DhTable);
	ConvertTokenVectorToDhTable(usm4TokenVector, isiUsm4DhTable);

	this->DaVinci->GetUsm1()->SetDhTable(isiUsm1DhTable);
	LOG_DEBUG("USM1 DH Table set to: " << this->DaVinci->GetUsm1()->GetDhTableAsString());
	this->DaVinci->GetUsm2()->SetDhTable(isiUsm2DhTable);
	LOG_DEBUG("USM2 DH Table set to: " << this->DaVinci->GetUsm2()->GetDhTableAsString());
	this->DaVinci->GetUsm3()->SetDhTable(isiUsm3DhTable);
	LOG_DEBUG("USM3 DH Table set to: " << this->DaVinci->GetUsm3()->GetDhTableAsString());
	this->DaVinci->GetUsm4()->SetDhTable(isiUsm4DhTable);
	LOG_DEBUG("USM4 DH Table set to: " << this->DaVinci->GetUsm4()->GetDhTableAsString());

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