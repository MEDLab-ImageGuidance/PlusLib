
class IntuitiveDaVinciTransforms
{

};

using UsmTransforms = vtkMatrix4x4[number of transforms];

class IntuitiveDaVinciKinematicModel
{
public:
  void SetJointValues(DaVinciXiJointValues* jointValues);
  void ComputeKinematics();
  void GetTransforms(IntuitiveDaVinciTransforms* transformsOut);
  void ComputeKinematicsAndGetTransforms(IntuitiveDaVinciTransforms* transformsOut, DaVinciXiJointValues* jointValues);

protected:
  UsmKinematicModel* mUsm1, mUsm2, mUsm3, mUsm4;
  IntuitiveDaVinciTransforms* mTransforms;
};


class UsmKinematicModel
{
  void SetJointValues(UsmJointValues* jointValues);
};
