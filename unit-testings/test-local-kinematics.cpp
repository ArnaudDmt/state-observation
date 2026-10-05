#include <bitset>
#include <iostream>

#include <state-observation/tools/probability-law-simulation.hpp>
#include <state-observation/tools/rigid-body-kinematics.hpp>

using namespace stateObservation;
using namespace kine;

/// @brief test rotationMatrix2Angle
///
/// @param errorCode
/// @return int

int testPositionByIntegration()
{

  double dt = 0.001;
  int numbIterations = 10000;

  double err_pvwwd = 0;
  double err_pwwd = 0;
  double err_pw = 0;
  double err_pwd = 0;
  double err_pvw = 0;
  double err_pvwd = 0;
  double err_pavw = 0;
  double err_pavwd = 0;
  double err_pawwd = 0;
  double err_paw = 0;
  double err_pav = 0;
  double err_pa = 0;
  double err_pv = 0;

  double err2_pvwwd = 0;
  double err2_pwwd = 0;
  double err2_pw = 0;
  double err2_pwd = 0;
  double err2_pvw = 0;
  double err2_pvwd = 0;
  double err2_pavw = 0;
  double err2_pavwd = 0;
  double err2_pawwd = 0;
  double err2_paw = 0;
  double err2_pav = 0;
  double err2_pa = 0;
  double err2_pv = 0;

  for(int i = 0; i < numbIterations; i++)
  {
    LocalKinematics k;
    Vector3 pos = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>();
    kine::Orientation ori = kine::Orientation::randomRotation();
    Vector3 linvel = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>();
    Vector3 angvel = tools::ProbabilityLawSimulation::getGaussianMatrix<Vector3>();
    Vector3 linacc = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>();
    Vector3 angacc = tools::ProbabilityLawSimulation::getGaussianMatrix<Vector3>();

    k.position = pos;
    k.orientation = ori;
    k.linVel = linvel;
    k.angVel = angvel;
    k.linAcc = linacc;
    k.angAcc = angacc;

    Vector3 p1_pavwwd =
        k.position() - k.angVel().cross(dt * (k.position() + dt * (0.5 * k.angVel().cross(k.position() - k.linVel()))))
        + dt * (k.linVel() + 0.5 * dt * (k.linAcc() - k.angAcc().cross(k.position())));

    Vector3 p1_pvwwd =
        k.position() - k.angVel().cross(dt * (k.position() + dt * (0.5 * k.angVel().cross(k.position() - k.linVel()))))
        + dt * (k.linVel() + 0.5 * dt * (-k.angAcc().cross(k.position())));
    Vector3 p1_pwwd = k.position() - k.angVel().cross(dt * (k.position() + dt * (0.5 * k.angVel().cross(k.position()))))
                      + dt * (0.5 * dt * (-k.angAcc().cross(k.position())));
    Vector3 p1_pw = k.position() - k.angVel().cross(dt * (k.position() + dt * (0.5 * k.angVel().cross(k.position()))));
    Vector3 p1_pwd = k.position() + dt * (0.5 * dt * (-k.angAcc().cross(k.position())));
    Vector3 p1_pvw = k.position()
                     - k.angVel().cross(dt * (k.position() + dt * (0.5 * k.angVel().cross(k.position() - k.linVel()))))
                     + dt * (k.linVel());
    Vector3 p1_pvwd = k.position() + dt * (k.linVel() + 0.5 * dt * (-k.angAcc().cross(k.position())));
    Vector3 p1_pavw = k.position()
                      - k.angVel().cross(dt * (k.position() + dt * (0.5 * k.angVel().cross(k.position() - k.linVel()))))
                      + dt * (k.linVel() + 0.5 * dt * (k.linAcc()));
    Vector3 p1_pavwd = k.position() + dt * (k.linVel() + 0.5 * dt * (k.linAcc() - k.angAcc().cross(k.position())));
    Vector3 p1_pawwd = k.position()
                       - k.angVel().cross(dt * (k.position() + dt * (0.5 * k.angVel().cross(k.position()))))
                       + dt * (0.5 * dt * (k.linAcc() - k.angAcc().cross(k.position())));
    Vector3 p1_paw = k.position() - k.angVel().cross(dt * (k.position() + dt * (0.5 * k.angVel().cross(k.position()))))
                     + dt * (0.5 * dt * (k.linAcc()));
    Vector3 p1_pav = k.position() + dt * (k.linVel() + 0.5 * dt * (k.linAcc()));
    Vector3 p1_pa = k.position() + dt * (0.5 * dt * (k.linAcc()));
    Vector3 p1_pv = k.position() + dt * (k.linVel());

    err_pvwwd += (p1_pavwwd - p1_pvwwd).squaredNorm() / p1_pavwwd.squaredNorm();
    err_pwwd += (p1_pavwwd - p1_pwwd).squaredNorm() / p1_pavwwd.squaredNorm();
    err_pw += (p1_pavwwd - p1_pw).squaredNorm() / p1_pavwwd.squaredNorm();
    err_pwd += (p1_pavwwd - p1_pwd).squaredNorm() / p1_pavwwd.squaredNorm();
    err_pvw += (p1_pavwwd - p1_pvw).squaredNorm() / p1_pavwwd.squaredNorm();
    err_pvwd += (p1_pavwwd - p1_pvwd).squaredNorm() / p1_pavwwd.squaredNorm();
    err_pavw += (p1_pavwwd - p1_pavw).squaredNorm() / p1_pavwwd.squaredNorm();
    err_pavwd += (p1_pavwwd - p1_pavwd).squaredNorm() / p1_pavwwd.squaredNorm();
    err_pawwd += (p1_pavwwd - p1_pawwd).squaredNorm() / p1_pavwwd.squaredNorm();
    err_paw += (p1_pavwwd - p1_paw).squaredNorm() / p1_pavwwd.squaredNorm();
    err_pav += (p1_pavwwd - p1_pav).squaredNorm() / p1_pavwwd.squaredNorm();
    err_pa += (p1_pavwwd - p1_pa).squaredNorm() / p1_pavwwd.squaredNorm();
    err_pv += (p1_pavwwd - p1_pv).squaredNorm() / p1_pavwwd.squaredNorm();

    err2_pvwwd += (p1_pavwwd - p1_pvwwd)(0);
    err2_pwwd += (p1_pavwwd - p1_pwwd)(0);
    err2_pw += (p1_pavwwd - p1_pw)(0);
    err2_pwd += (p1_pavwwd - p1_pwd)(0);
    err2_pvw += (p1_pavwwd - p1_pvw)(0);
    err2_pvwd += (p1_pavwwd - p1_pvwd)(0);
    err2_pavw += (p1_pavwwd - p1_pavw)(0);
    err2_pavwd += (p1_pavwwd - p1_pavwd)(0);
    err2_pawwd += (p1_pavwwd - p1_pawwd)(0);
    err2_paw += (p1_pavwwd - p1_paw)(0);
    err2_pav += (p1_pavwwd - p1_pav)(0);
    err2_pa += (p1_pavwwd - p1_pa)(0);
    err2_pv += (p1_pavwwd - p1_pv)(0);
  }

  std::cout << std::endl
            << "Average relative squared difference with the parameters "
            << "pvwwd"
            << " with the full integration expression: " << err_pvwwd / numbIterations << std::endl;
  std::cout << std::endl
            << "Average relative squared difference with the parameters "
            << "pwwd"
            << " with the full integration expression: " << err_pwwd / numbIterations << std::endl;
  std::cout << std::endl
            << "Average relative squared difference with the parameters "
            << "pw"
            << " with the full integration expression: " << err_pw / numbIterations << std::endl;
  std::cout << std::endl
            << "Average relative squared difference with the parameters "
            << "pwd"
            << " with the full integration expression: " << err_pwd / numbIterations << std::endl;
  std::cout << std::endl
            << "Average relative squared difference with the parameters "
            << "pvw"
            << " with the full integration expression: " << err_pvw / numbIterations << std::endl;
  std::cout << std::endl
            << "Average relative squared difference with the parameters "
            << "pvwd"
            << " with the full integration expression: " << err_pvwd / numbIterations << std::endl;
  std::cout << std::endl
            << "Average relative squared difference with the parameters "
            << "pavw"
            << " with the full integration expression: " << err_pavw / numbIterations << std::endl;
  std::cout << std::endl
            << "Average relative squared difference with the parameters "
            << "pavwd"
            << " with the full integration expression: " << err_pavwd / numbIterations << std::endl;
  std::cout << std::endl
            << "Average relative squared difference with the parameters "
            << "pawwd"
            << " with the full integration expression: " << err_pawwd / numbIterations << std::endl;
  std::cout << std::endl
            << "Average relative squared difference with the parameters "
            << "paw"
            << " with the full integration expression: " << err_paw / numbIterations << std::endl;
  std::cout << std::endl
            << "Average relative squared difference with the parameters "
            << "pav"
            << " with the full integration expression: " << err_pav / numbIterations << std::endl;
  std::cout << std::endl
            << "Average relative squared difference with the parameters "
            << "pa"
            << " with the full integration expression: " << err_pa / numbIterations << std::endl;
  std::cout << std::endl
            << "Average relative squared difference with the parameters "
            << "pv"
            << " with the full integration expression: " << err_pv / numbIterations << std::endl;

  std::cout << std::endl
            << "Average difference on the x direction with the parameters "
            << "pvwwd"
            << " with the full integration expression: " << err2_pvwwd / numbIterations << std::endl;
  std::cout << std::endl
            << "Average difference on the x direction with the parameters "
            << "pwwd"
            << " with the full integration expression: " << err2_pwwd / numbIterations << std::endl;
  std::cout << std::endl
            << "Average difference on the x direction with the parameters "
            << "pw"
            << " with the full integration expression: " << err2_pw / numbIterations << std::endl;
  std::cout << std::endl
            << "Average difference on the x direction with the parameters "
            << "pwd"
            << " with the full integration expression: " << err2_pwd / numbIterations << std::endl;
  std::cout << std::endl
            << "Average difference on the x direction with the parameters "
            << "pvw"
            << " with the full integration expression: " << err2_pvw / numbIterations << std::endl;
  std::cout << std::endl
            << "Average difference on the x direction with the parameters "
            << "pvwd"
            << " with the full integration expression: " << err2_pvwd / numbIterations << std::endl;
  std::cout << std::endl
            << "Average difference on the x direction with the parameters "
            << "pavw"
            << " with the full integration expression: " << err2_pavw / numbIterations << std::endl;
  std::cout << std::endl
            << "Average difference on the x direction with the parameters "
            << "pavwd"
            << " with the full integration expression: " << err2_pavwd / numbIterations << std::endl;
  std::cout << std::endl
            << "Average difference on the x direction with the parameters "
            << "pawwd"
            << " with the full integration expression: " << err2_pawwd / numbIterations << std::endl;
  std::cout << std::endl
            << "Average difference on the x direction with the parameters "
            << "paw"
            << " with the full integration expression: " << err2_paw / numbIterations << std::endl;
  std::cout << std::endl
            << "Average difference on the x direction with the parameters "
            << "pav"
            << " with the full integration expression: " << err2_pav / numbIterations << std::endl;
  std::cout << std::endl
            << "Average difference on the x direction with the parameters "
            << "pa"
            << " with the full integration expression: " << err2_pa / numbIterations << std::endl;
  std::cout << std::endl
            << "Average difference on the x direction with the parameters "
            << "pv"
            << " with the full integration expression: " << err2_pv / numbIterations << std::endl;

  return 0;
}

int testSetToDiffNoAliasLocalKinematics(int errcode)
{

  std::cout << "testSetToDiffNoAliasLocalKinematics test started" << std::endl;
  typedef kine::LocalKinematics::Flags Flags;

  kine::LocalKinematics k0;
  kine::LocalKinematics k1;

  Flags::Byte flag0 = BOOST_BINARY(000000);
  Flags::Byte flag1 = BOOST_BINARY(000000);

  kine::LocalKinematics k, l, k2;

  int count = int(pow(2, 6) * pow(2, 6));
  double err = 0;
  double threshold = 1e-29 * count;

  for(int i = 0; i < count; i++)
  {
    /*
    std::cout << std::endl << "-----------------------------------------" << std::endl
                          << "New iteration"
              << std::endl << "-----------------------------------------" << std::endl;
    std::cout << "err: " << err << std::endl;
    */

    Vector3 pos0 = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>();
    kine::Orientation ori0 = kine::Orientation::randomRotation();
    Vector3 linvel0 = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>();
    Vector3 angvel0 = tools::ProbabilityLawSimulation::getGaussianMatrix<Vector3>();
    Vector3 linacc0 = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>();
    Vector3 angacc0 = tools::ProbabilityLawSimulation::getGaussianMatrix<Vector3>();

    Vector3 pos1 = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>();
    kine::Orientation ori1 = kine::Orientation::randomRotation();
    Vector3 linvel1 = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>();
    Vector3 angvel1 = tools::ProbabilityLawSimulation::getGaussianMatrix<Vector3>();
    Vector3 linacc1 = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>();
    Vector3 angacc1 = tools::ProbabilityLawSimulation::getGaussianMatrix<Vector3>();

    k0.reset();
    k1.reset();

    if(true) /// the position has to be set
    {
      k0.position = pos0;
    }
    if(true) /// the orientation has to be set
    {
      k0.orientation = ori0;
    }
    if(flag0 & Flags::linVel)
    {
      k0.linVel = linvel0;
    }
    if(flag0 & Flags::angVel)
    {
      k0.angVel = angvel0;
    }
    if(flag0 & Flags::linAcc)
    {
      k0.linAcc = linacc0;
    }
    if(flag0 & Flags::angAcc)
    {
      k0.angAcc = angacc0;
    }

    if(true) /// the position has to be set
    {
      k1.position = pos1;
    }
    if(true) /// the orientation has to be set
    {
      k1.orientation = ori1;
    }
    if(flag1 & Flags::linVel)
    {
      k1.linVel = linvel1;
    }
    if(flag1 & Flags::angVel)
    {
      k1.angVel = angvel1;
    }
    if(flag1 & Flags::linAcc)
    {
      k1.linAcc = linacc1;
    }
    if(flag1 & Flags::angAcc)
    {
      k1.angAcc = angacc1;
    }

    if((flag0 = (flag0 + 1) & Flags::all) == 0) /// update the flags to span all the possibilties
      flag1 = (flag1 + 1) & Flags::all;

    k2 = k1;
    if(!k1.orientation.isSet())
    {
      k2.orientation.setZeroRotation();
    }
    LocalKinematics k3;
    LocalKinematics k4;

    k3.setToDiffNoAlias(k2, k0);

    k4.setToProductNoAlias(k2, k0.getInverse());

    // std::cout << std::endl << "k3: " << std::endl << k3 << std::endl;
    // std::cout << std::endl << "k4: " << std::endl << k4 << std::endl;

    k = k4 * k3.getInverse();

    if(k.position.isSet())
    {
      err += k.position().squaredNorm();
    }
    if(k.orientation.isSet())
    {
      err += k.orientation.toRotationVector().squaredNorm();
    }
    if(k.linVel.isSet())
    {
      err += k.linVel().squaredNorm();
    }
    if(k.angVel.isSet())
    {
      err += k.angVel().squaredNorm();
    }
    if(k.linAcc.isSet())
    {
      err += k.linAcc().squaredNorm();
    }
    if(k.angAcc.isSet())
    {
      err += k.angAcc().squaredNorm();
    }
  }

  std::cout << "Error 1 : " << err << std::endl;

  if(err > threshold)
  {
    std::cout << "Error too large : " << err << ". Threshold: " << threshold << std::endl;
    return errcode;
  }
  return 0;
}

int testLocalVsGlobalIntegrates(int errcode)
{
  double threshold = 1e-4;
  double err = 0;

  double dt = 0.001;

  Kinematics k;
  Vector3 pos = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>();
  kine::Orientation ori = kine::Orientation::randomRotation();
  Vector3 linvel = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>();
  Vector3 angvel = tools::ProbabilityLawSimulation::getGaussianMatrix<Vector3>();
  Vector3 linacc = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>();
  Vector3 angacc = tools::ProbabilityLawSimulation::getGaussianMatrix<Vector3>();

  k.position = pos;
  k.orientation = ori;
  k.linVel = linvel;
  k.angVel = angvel;
  k.linAcc = linacc;
  k.angAcc = angacc;

  LocalKinematics lk1(k);
  k.integrate(dt);
  lk1.integrate(dt);
  Kinematics k1 = Kinematics(lk1);

  std::cout << std::endl << "k1 : " << std::endl << k1 << std::endl;
  std::cout << std::endl << "k : " << std::endl << k << std::endl;

  Kinematics diff = k1 * k.getInverse();

  std::cout << std::endl << "diff : " << std::endl << diff << std::endl;

  std::cout << std::endl << "diff : " << std::endl << diff << std::endl;

  if(diff.position.isSet())
  {
    err += diff.position().squaredNorm();
  }
  if(diff.orientation.isSet())
  {
    err += diff.orientation.toRotationVector().squaredNorm();
  }
  if(diff.linVel.isSet())
  {
    err += diff.linVel().squaredNorm();
  }
  if(diff.angVel.isSet())
  {
    err += diff.angVel().squaredNorm();
  }

  std::cout << "Error between integrations with local and global frames: " << err << std::endl;

  if(err > threshold)
  {
    return errcode;
  }
  return 0;
}

int testRotationOperations(int errorCode)
{
  unsigned numberOfTests = 1000;
  for(unsigned currentTest = 0; currentTest < numberOfTests; ++currentTest)
  {
    { /// test isRotation

      Matrix3 R = tools::ProbabilityLawSimulation::getUniformMatrix<Matrix3>();
      double testPecision = cst::epsilon1 * 20;
      if(isRotationMatrix(R, testPecision))
      {
        std::cout << "Test number " << currentTest << "isRotationTest failed: false positive" << std::endl;
        return errorCode;
      }
      R = randomRotationQuaternion().toRotationMatrix();
      if(!isRotationMatrix(R, testPecision))
      {
        std::cout << R.isUnitary() << " " << R.topLeftCorner<2, 2>().determinant() << " " << R(2, 2) << " "
                  << R.bottomLeftCorner<2, 2>().determinant() << " " << R(0, 2) << " "
                  << R.topLeftCorner<2, 2>().determinant() - R(2, 2) << " "
                  << R.bottomLeftCorner<2, 2>().determinant() - R(0, 2) << " " << testPecision << std::endl;
        std::cout << "Test number " << currentTest << "isRotationTest failed: false negative" << std::endl;
        return errorCode;
      }
      Matrix3 R2;
      R2 << R.col(1), R.col(0), R.col(2); /// Non right handed orthogonal matrix
      if(isRotationMatrix(R2, testPecision))
      {
        std::cout << "isRotationTest failed: false positive (right-handedness)" << std::endl;
        return errorCode;
      }
    }
    {
      /// test pure yaw
      if(!isPureYaw(AngleAxis(randomAngle(), Vector3::UnitZ()).matrix())
         || isPureYaw(randomRotationQuaternion().toRotationMatrix()))
      {
        std::cout << "Test number " << currentTest << "Pure yaw detection failed " << std::endl;
        return errorCode;
      }
    }
    {
      /// Test the angle made by one vector by a rotation
      Vector3 axis = tools::ProbabilityLawSimulation::getGaussianMatrix<Vector3>().normalized();
      Vector3 v = tools::ProbabilityLawSimulation::getGaussianMatrix<Vector3>().cross(axis).normalized();
      double angle = randomAngle();
      Matrix3 m = (AngleAxis(angle, axis)).matrix() * AngleAxis(randomAngle(), v).matrix();

      double error = fabs(angle - kine::rotationMatrixToAngle(m, axis, v));

      if(error > cst::epsilon1 * 2 * M_PI)
      {
        std::cout << "Test number " << currentTest << "Test vector angle failed. Angle error " << error << std::endl;
        return errorCode;
      }
    }
    {
      /// Test yaw extraction with custom vector
      double angle = randomAngle();

      Vector2 v = tools::ProbabilityLawSimulation::getGaussianMatrix<Vector2>().normalized();
      Vector3 v3;
      v3 << v, 0;
      Matrix3 m = AngleAxis(angle, Vector3::UnitZ()).matrix() * AngleAxis(randomAngle(), v3).matrix();

      double error = fabs(angle - kine::rotationMatrixToYaw(m, v));

      if(error > cst::epsilon1 * 2 * M_PI)
      {
        std::cout << "Test number " << currentTest << "Test Yaw extraction with custom vector failed. Angle error "
                  << error << std::endl;
        return errorCode;
      }
    }
    {
      /// Test yaw extraction from roll pitch yaw using the x axis alignment
      double rollangle = randomAngle();
      double pitchangle = randomAngle() / 2; /// constrain the pitch to be smaller than pi/2
      double yawangle = randomAngle();

      Matrix3 m = rollPitchYawToRotationMatrix(rollangle, pitchangle, yawangle);

      double error = fabs(yawangle - kine::rotationMatrixToYaw(m));

      if(error > cst::epsilon1 * 1e5 * M_PI) /// this function is really not precise
      {
        std::cout << "Test number " << currentTest << " axis-based failed. Angle" << yawangle << " Angle error "
                  << error << std::endl;
        return errorCode;
      }

      /// Test yaw extraction from roll pitch yaw using the traditional conversion
      Vector3 rpy = kine::rotationMatrixToRollPitchYaw(m);
      error = fabs(yawangle - rpy(2));

      if(error > cst::epsilon1 * 1e5 * M_PI) /// this function is really not precise
      {
        std::cout << "Test number " << currentTest << "eigen-based failed. Angle" << yawangle << " Angle error "
                  << error << std::endl;
        return errorCode;
      }
    }

    {
      /// Test the automatic detection of the vector to use to extract yaw from a rotation matrix
      double angle = randomAngle(); /// random value
      Vector2 v = tools::ProbabilityLawSimulation::getGaussianMatrix<Vector2>().normalized();
      Vector3 v3;
      v3 << v, 0;

      Matrix3 m = AngleAxis(angle, Vector3::UnitZ()).matrix() * AngleAxis(randomAngle(), v3).matrix();

      double error = fabs(angle - kine::rotationMatrixToYawAxisAgnostic(m));

      if(error > cst::epsilon1 * 2 * M_PI)
      {
        std::cout << " Test rotationMatrixToYawAxisAgnostic failed. Angle error " << error << std::endl;

        return errorCode;
      }
    }
    {
      Vector3 horizAxis1;
      horizAxis1(2) = 0;
      horizAxis1.head<2>() = tools::ProbabilityLawSimulation::getGaussianMatrix<Vector2>().normalized();
      double tiltAngle1 = randomAngle();
      double yawAngle = randomAngle();

      Matrix3 yaw = AngleAxis(yawAngle, Vector3::UnitZ()).matrix();

      /// we should get back this matrix
      Matrix3 initialMatrix = yaw * AngleAxis(tiltAngle1, horizAxis1).matrix();

      Vector3 tilt = initialMatrix.transpose() * Vector3::UnitZ();

      /// m sis a random horizontal vector
      Vector3 m;
      m(2) = 0;
      m.head<2>() = tools::ProbabilityLawSimulation::getGaussianMatrix<Vector2>().normalized();

      Vector3 ml = initialMatrix.transpose() * m;

      Vector3 newTilt = tools::ProbabilityLawSimulation::getGaussianMatrix<Vector3>();
      Matrix3 mTemp1, mTemp2;

      mTemp1 << m, m.cross(Vector3::UnitZ().cross(m)).normalized(), m.cross(Vector3::UnitZ()).normalized();

      mTemp2 << ml, ml.cross(newTilt.cross(ml)).normalized(), ml.cross(newTilt).normalized();

      /// This is a matrix such that M2.transpose()*m is orthogonal to tilt
      Matrix3 M2 = mTemp1 * mTemp2.transpose();

      Matrix3 estimatedMatrix = kine::mergeTiltWithYawAxisAgnostic(tilt, M2);

      if(!isRotationMatrix(estimatedMatrix, cst::epsilon1 * 10))
      {
        std::cout << "Test mergeTiltWithYawAxisAgnostic failed. Reconstructed matrix is not a Rotation Matrix"
                  << std::endl;
        return errorCode;
      }

      double error = AngleAxis(initialMatrix * estimatedMatrix.transpose()).angle();

      if(error > cst::epsilon1 * 1e5 * M_PI) /// this function is really not precise
      {
        std::cout << "Test mergeTiltWithYawAxisAgnostic failed. Reconstructed matrix is wrong" << std::endl;
        return errorCode;
      }
    }
  } /// end of for
  return 0;
}

int testOrientation(int errcode)
{

  Vector4 q_i;

  typedef tools::ProbabilityLawSimulation ran;

  /// random orientation
  q_i = ran::getGaussianMatrix<Vector4>();
  q_i.normalize();

  /// several representations of the orientation
  Quaternion q(q_i);
  AngleAxis aa(q);
  Matrix3 M(q.toRotationMatrix());

  double err = 0.;

  std::cout << "Orientation test started " << err << std::endl;

  {
    kine::Orientation ori1;
    ori1 = q;
    Matrix3 M1 = ori1;

    err +=
        (Quaternion(ori1.toVector4()).toRotationMatrix() - Quaternion(q_i).toRotationMatrix()).norm() + (M1 - M).norm();
    std::cout << "Assignment operaton test 1 (Quaternion) done. Current error " << err << std::endl;
  }

  {
    kine::Orientation ori2;
    ori2 = M;

    err += (Quaternion(ori2.toVector4()).toRotationMatrix() - Quaternion(q_i).toRotationMatrix()).norm()
           + (ori2.toMatrix3() - M).norm();
    std::cout << "Assignment operaton test 2 (Matrix3) done. Current error " << err << std::endl;
  }

  {
    kine::Orientation ori3;
    ori3 = aa;

    err += (Quaternion(ori3.toVector4()).toRotationMatrix() - Quaternion(q_i).toRotationMatrix()).norm()
           + (ori3.toMatrix3() - M).norm();
    std::cout << "Assignment operaton test 3 (AngleAxis) done. Current error " << err << std::endl;
  }

  {
    kine::Orientation ori4;
    ori4 = Vector3(aa.angle() * aa.axis());

    err += (Quaternion(ori4.toVector4()).toRotationMatrix() - Quaternion(q_i).toRotationMatrix()).norm()
           + (ori4.toMatrix3() - M).norm();
    std::cout << "Assignment operaton test 3 (Vector3) done. Current error " << err << std::endl;
  }

  {
    kine::Orientation ori1(q);
    Matrix3 M1 = ori1;

    err +=
        (Quaternion(ori1.toVector4()).toRotationMatrix() - Quaternion(q_i).toRotationMatrix()).norm() + (M1 - M).norm();
    std::cout << "Cast operaton test 1 (Matrix3) done. Current error " << err << std::endl;
  }

  {
    kine::Orientation ori2(M);

    err += (Quaternion(ori2.toVector4()).toRotationMatrix() - Quaternion(q_i).toRotationMatrix()).norm()
           + (ori2.toMatrix3() - M).norm();
    std::cout << "copy constructor test 1 (Matrix3) done. Current error " << err << std::endl;
  }

  {
    kine::Orientation ori3(aa);

    err += (Quaternion(ori3.toVector4()).toRotationMatrix() - Quaternion(q_i).toRotationMatrix()).norm()
           + (ori3.toMatrix3() - M).norm();
    std::cout << "copy constructor test 2 (AngleAxis) done. Current error " << err << std::endl;
  }

  {
    kine::Orientation ori4(Vector3(aa.angle() * aa.axis()));

    err += (Quaternion(ori4.toVector4()).toRotationMatrix() - M).norm() + (ori4.toMatrix3() - M).norm();
    std::cout << "copy constructor test 3 (AngleAxis) done. Current error " << err << std::endl;
  }

  {
    kine::Orientation ori4(q, M);

    err += (Quaternion(ori4.toVector4()).toRotationMatrix() - M).norm() + (ori4.toMatrix3() - M).norm();
    std::cout << "Constructor test 1 (Quaternion, Matrix3) done. Current error " << err << std::endl;
  }

  {
    kine::Orientation ori4;
    ori4.setRandom();
    ori4 = M;

    err += (Quaternion(ori4.toVector4()).toRotationMatrix() - M).norm() + (ori4.toMatrix3() - M).norm();
    std::cout << "Update Assignement operator test 1 (Matrix3) done. Current error " << err << std::endl;
  }

  {
    kine::Orientation ori4;
    ori4.setRandom();
    ori4 = q;

    err += (Quaternion(ori4.toVector4()).toRotationMatrix() - M).norm() + (ori4.toMatrix3() - M).norm();
    std::cout << "Update Assignement operator test 2 (Quaternion) done. Current error " << err << std::endl;
  }

  {
    kine::Orientation ori1(q);

    kine::Orientation ori2 = ori1.inverse();
    kine::Orientation ori3 = ori2.inverse();

    Matrix3 M3 = ori1;

    err += (Quaternion(ori3.toVector4()).toRotationMatrix() - M).norm() + (M3 - M).norm();
    std::cout << "Inverse operator test 1 done. Current error " << err << std::endl;
  }

  {

    kine::Orientation ori0(q);
    kine::Orientation ori1(M);
    ori1 = ori1.inverse();
    const kine::Orientation & ori2 = ori0;
    const kine::Orientation & ori3 = ori1;

    kine::Orientation ori00 = ori0 * ori1;
    ori0 = q;
    ori1 = M;
    ori1 = ori1.inverse();
    kine::Orientation ori01 = ori1 * ori0;
    ori0 = q;
    ori1 = M;
    ori1 = ori1.inverse();
    kine::Orientation ori02 = ori0 * ori3;
    ori0 = q;
    ori1 = M;
    ori1 = ori1.inverse();
    kine::Orientation ori03 = ori3 * ori0;
    ori0 = q;
    ori1 = M;
    ori1 = ori1.inverse();
    kine::Orientation ori04 = ori2 * ori1;
    ori0 = q;
    ori1 = M;
    ori1 = ori1.inverse();
    kine::Orientation ori05 = ori1 * ori2;
    ori0 = q;
    ori1 = M;
    ori1 = ori1.inverse();
    kine::Orientation ori06 = ori2 * ori3;
    kine::Orientation ori07 = ori3 * ori2;

    kine::Orientation ori10(ori0, ori1);
    ori0 = q;
    ori1 = M;
    ori1 = ori1.inverse();
    kine::Orientation ori11(ori1, ori0);
    ori0 = q;
    ori1 = M;
    ori1 = ori1.inverse();
    kine::Orientation ori12(ori0, ori3);
    ori0 = q;
    ori1 = M;
    ori1 = ori1.inverse();
    kine::Orientation ori13(ori3, ori0);
    ori0 = q;
    ori1 = M;
    ori1 = ori1.inverse();
    kine::Orientation ori14(ori2, ori1);
    ori0 = q;
    ori1 = M;
    ori1 = ori1.inverse();
    kine::Orientation ori15(ori1, ori2);
    ori0 = q;
    ori1 = M;
    ori1 = ori1.inverse();
    kine::Orientation ori16(ori2, ori3);
    kine::Orientation ori17(ori3, ori2);

    err += (ori00.toMatrix3() - Matrix3::Identity()).norm();
    // std::cout << "Multiplication test  1 done. Current error " << err << std::endl;
    err += (ori01.toMatrix3() - Matrix3::Identity()).norm();
    // std::cout << "Multiplication test  2 done. Current error " << err << std::endl;
    err += (ori02.toMatrix3() - Matrix3::Identity()).norm();
    // std::cout << "Multiplication test  3 done. Current error " << err << std::endl;
    err += (ori03.toMatrix3() - Matrix3::Identity()).norm();
    // std::cout << "Multiplication test  4 done. Current error " << err << std::endl;
    err += (ori04.toMatrix3() - Matrix3::Identity()).norm();
    // std::cout << "Multiplication test  5 done. Current error " << err << std::endl;
    err += (ori05.toMatrix3() - Matrix3::Identity()).norm();
    // std::cout << "Multiplication test  6 done. Current error " << err << std::endl;
    err += (ori06.toMatrix3() - Matrix3::Identity()).norm();
    // std::cout << "Multiplication test  7 done. Current error " << err << std::endl;
    err += (ori07.toMatrix3() - Matrix3::Identity()).norm();
    // std::cout << "Multiplication test  8 done. Current error " << err << std::endl;
    err += (ori10.toMatrix3() - Matrix3::Identity()).norm();
    // std::cout << "Multiplication test  9 done. Current error " << err << std::endl;
    err += (ori11.toMatrix3() - Matrix3::Identity()).norm();
    // std::cout << "Multiplication test  10 done. Current error " << err << std::endl;
    err += (ori12.toMatrix3() - Matrix3::Identity()).norm();
    // std::cout << "Multiplication test  11 done. Current error " << err << std::endl;
    err += (ori13.toMatrix3() - Matrix3::Identity()).norm();
    // std::cout << "Multiplication test  12 done. Current error " << err << std::endl;
    err += (ori14.toMatrix3() - Matrix3::Identity()).norm();
    // std::cout << "Multiplication test  13 done. Current error " << err << std::endl;
    err += (ori15.toMatrix3() - Matrix3::Identity()).norm();
    // std::cout << "Multiplication test  14 done. Current error " << err << std::endl;
    err += (ori16.toMatrix3() - Matrix3::Identity()).norm();
    // std::cout << "Multiplication test  15 done. Current error " << err << std::endl;
    err += (ori17.toMatrix3() - Matrix3::Identity()).norm();
    // std::cout << "Multiplication test  16 done. Current error " << err << std::endl;
  }

  {

    kine::Orientation ori0(q);
    kine::Orientation ori1(q);
    ori1 = ori1.inverse();
    const kine::Orientation & ori2 = ori0;
    const kine::Orientation & ori3 = ori1;

    kine::Orientation ori00 = ori0 * ori1;
    ori0 = q;
    ori1 = q;
    ori1 = ori1.inverse();
    kine::Orientation ori01 = ori1 * ori0;
    ori0 = q;
    ori1 = q;
    ori1 = ori1.inverse();
    kine::Orientation ori02 = ori0 * ori3;
    ori0 = q;
    ori1 = q;
    ori1 = ori1.inverse();
    kine::Orientation ori03 = ori3 * ori0;
    ori0 = q;
    ori1 = q;
    ori1 = ori1.inverse();
    kine::Orientation ori04 = ori2 * ori1;
    ori0 = q;
    ori1 = q;
    ori1 = ori1.inverse();
    kine::Orientation ori05 = ori1 * ori2;
    ori0 = q;
    ori1 = q;
    ori1 = ori1.inverse();
    kine::Orientation ori06 = ori2 * ori3;
    kine::Orientation ori07 = ori3 * ori2;

    kine::Orientation ori10(ori0, ori1);
    ori0 = q;
    ori1 = q;
    ori1 = ori1.inverse();
    kine::Orientation ori11(ori1, ori0);
    ori0 = q;
    ori1 = q;
    ori1 = ori1.inverse();
    kine::Orientation ori12(ori0, ori3);
    ori0 = q;
    ori1 = q;
    ori1 = ori1.inverse();
    kine::Orientation ori13(ori3, ori0);
    ori0 = q;
    ori1 = q;
    ori1 = ori1.inverse();
    kine::Orientation ori14(ori2, ori1);
    ori0 = q;
    ori1 = q;
    ori1 = ori1.inverse();
    kine::Orientation ori15(ori1, ori2);
    ori0 = q;
    ori1 = q;
    ori1 = ori1.inverse();
    kine::Orientation ori16(ori2, ori3);
    kine::Orientation ori17(ori3, ori2);

    err += (ori00.toMatrix3() - Matrix3::Identity()).norm();
    // std::cout << "Multiplication test  17 done. Current error " << err << std::endl;
    err += (ori01.toMatrix3() - Matrix3::Identity()).norm();
    // std::cout << "Multiplication test  18 done. Current error " << err << std::endl;
    err += (ori02.toMatrix3() - Matrix3::Identity()).norm();
    // std::cout << "Multiplication test  19 done. Current error " << err << std::endl;
    err += (ori03.toMatrix3() - Matrix3::Identity()).norm();
    // std::cout << "Multiplication test  20 done. Current error " << err << std::endl;
    err += (ori04.toMatrix3() - Matrix3::Identity()).norm();
    // std::cout << "Multiplication test  21 done. Current error " << err << std::endl;
    err += (ori05.toMatrix3() - Matrix3::Identity()).norm();
    // std::cout << "Multiplication test  22 done. Current error " << err << std::endl;
    err += (ori06.toMatrix3() - Matrix3::Identity()).norm();
    // std::cout << "Multiplication test  23 done. Current error " << err << std::endl;
    err += (ori07.toMatrix3() - Matrix3::Identity()).norm();
    // std::cout << "Multiplication test  24 done. Current error " << err << std::endl;
    err += (ori10.toMatrix3() - Matrix3::Identity()).norm();
    // std::cout << "Multiplication test  25 done. Current error " << err << std::endl;
    err += (ori11.toMatrix3() - Matrix3::Identity()).norm();
    // std::cout << "Multiplication test  26 done. Current error " << err << std::endl;
    err += (ori12.toMatrix3() - Matrix3::Identity()).norm();
    // std::cout << "Multiplication test  27 done. Current error " << err << std::endl;
    err += (ori13.toMatrix3() - Matrix3::Identity()).norm();
    // std::cout << "Multiplication test  28 done. Current error " << err << std::endl;
    err += (ori14.toMatrix3() - Matrix3::Identity()).norm();
    // std::cout << "Multiplication test  29 done. Current error " << err << std::endl;
    err += (ori15.toMatrix3() - Matrix3::Identity()).norm();
    // std::cout << "Multiplication test  30 done. Current error " << err << std::endl;
    err += (ori16.toMatrix3() - Matrix3::Identity()).norm();
    // std::cout << "Multiplication test  31 done. Current error " << err << std::endl;
    err += (ori17.toMatrix3() - Matrix3::Identity()).norm();
    // std::cout << "Multiplication test  32 done. Current error " << err << std::endl;
  }

  {

    kine::Orientation ori0(M);
    kine::Orientation ori1(M);
    ori1 = ori1.inverse();
    const kine::Orientation & ori2 = ori0;
    const kine::Orientation & ori3 = ori1;

    ori0 = M;
    ori1 = M;
    ori1 = ori1.inverse();
    kine::Orientation ori00 = ori0 * ori1;
    ori0 = M;
    ori1 = M;
    ori1 = ori1.inverse();
    kine::Orientation ori01 = ori1 * ori0;
    ori0 = M;
    ori1 = M;
    ori1 = ori1.inverse();
    kine::Orientation ori02 = ori0 * ori3;
    ori0 = M;
    ori1 = M;
    ori1 = ori1.inverse();
    kine::Orientation ori03 = ori3 * ori0;
    ori0 = M;
    ori1 = M;
    ori1 = ori1.inverse();
    kine::Orientation ori04 = ori2 * ori1;
    ori0 = M;
    ori1 = M;
    ori1 = ori1.inverse();
    kine::Orientation ori05 = ori1 * ori2;
    ori0 = M;
    ori1 = M;
    ori1 = ori1.inverse();
    kine::Orientation ori06 = ori2 * ori3;
    kine::Orientation ori07 = ori3 * ori2;

    kine::Orientation ori10(ori0, ori1);
    ori0 = M;
    ori1 = M;
    ori1 = ori1.inverse();
    kine::Orientation ori11(ori1, ori0);
    ori0 = M;
    ori1 = M;
    ori1 = ori1.inverse();
    kine::Orientation ori12(ori0, ori3);
    ori0 = M;
    ori1 = M;
    ori1 = ori1.inverse();
    kine::Orientation ori13(ori3, ori0);
    ori0 = M;
    ori1 = M;
    ori1 = ori1.inverse();
    kine::Orientation ori14(ori2, ori1);
    ori0 = M;
    ori1 = M;
    ori1 = ori1.inverse();
    kine::Orientation ori15(ori1, ori2);
    ori0 = M;
    ori1 = M;
    ori1 = ori1.inverse();
    kine::Orientation ori16(ori2, ori3);
    kine::Orientation ori17(ori3, ori2);

    err += (ori00.toMatrix3() - Matrix3::Identity()).norm();
    // std::cout << "Multiplication test  33 done. Current error " << err << std::endl;
    err += (ori01.toMatrix3() - Matrix3::Identity()).norm();
    // std::cout << "Multiplication test  34 done. Current error " << err << std::endl;
    err += (ori02.toMatrix3() - Matrix3::Identity()).norm();
    // std::cout << "Multiplication test  35 done. Current error " << err << std::endl;
    err += (ori03.toMatrix3() - Matrix3::Identity()).norm();
    // std::cout << "Multiplication test  36 done. Current error " << err << std::endl;
    err += (ori04.toMatrix3() - Matrix3::Identity()).norm();
    // std::cout << "Multiplication test  37 done. Current error " << err << std::endl;
    err += (ori05.toMatrix3() - Matrix3::Identity()).norm();
    // std::cout << "Multiplication test  38 done. Current error " << err << std::endl;
    err += (ori06.toMatrix3() - Matrix3::Identity()).norm();
    // std::cout << "Multiplication test  39 done. Current error " << err << std::endl;
    err += (ori07.toMatrix3() - Matrix3::Identity()).norm();
    // std::cout << "Multiplication test  40 done. Current error " << err << std::endl;
    err += (ori10.toMatrix3() - Matrix3::Identity()).norm();
    // std::cout << "Multiplication test  41 done. Current error " << err << std::endl;
    err += (ori11.toMatrix3() - Matrix3::Identity()).norm();
    // std::cout << "Multiplication test  42 done. Current error " << err << std::endl;
    err += (ori12.toMatrix3() - Matrix3::Identity()).norm();
    // std::cout << "Multiplication test  43 done. Current error " << err << std::endl;
    err += (ori13.toMatrix3() - Matrix3::Identity()).norm();
    // std::cout << "Multiplication test  44 done. Current error " << err << std::endl;
    err += (ori14.toMatrix3() - Matrix3::Identity()).norm();
    // std::cout << "Multiplication test  45 done. Current error " << err << std::endl;
    err += (ori15.toMatrix3() - Matrix3::Identity()).norm();
    // std::cout << "Multiplication test  46 done. Current error " << err << std::endl;
    err += (ori16.toMatrix3() - Matrix3::Identity()).norm();
    // std::cout << "Multiplication test  47 done. Current error " << err << std::endl;
    err += (ori17.toMatrix3() - Matrix3::Identity()).norm();
    // std::cout << "Multiplication test  48 done. Current error " << err << std::endl;
  }

  {
    kine::Orientation ori1(q);
    kine::Orientation ori2(M);

    kine::Orientation ori3 = ori2.inverse() * ori1;

    err += (Quaternion(ori3.toVector4()).toRotationMatrix() - Matrix3::Identity()).norm();
    // std::cout << "Multiplication test  49 done. Current error " << err << std::endl;
  }

  {
    Vector3 v1 = tools::ProbabilityLawSimulation::getGaussianMatrix<Vector3>() * 0.1;
    kine::Orientation ori1(M);
    kine::Orientation ori2(ori1);
    ori2.integrateRightSide(v1);

    Vector3 v2 = ori1.differentiateRightSide(ori2);

    err += (v1 - v2).norm();
    std::cout << "Integration/differentiation test 1 done. Current error " << err << std::endl;
  }

  {
    Vector3 v1;
    kine::Orientation ori1;
    kine::Orientation ori2;

    ori1.setRandom();
    ori2.setRandom();

    v1 = ori2.differentiateRightSide(ori1);
    kine::Orientation oris = ori2.integrateRightSide(v1);

    err += oris.differentiateRightSide(ori1).norm();
    std::cout << "Integration/differentiation test 2 done. Current error " << err << std::endl;
  }

  if(err > 1e-13)
  {
    return errcode;
  }
  return 0;
}

int testLocalKinematics(int errcode)
{

  std::cout << "LocalKinematics test started" << std::endl;
  typedef kine::LocalKinematics::Flags Flags;

  kine::LocalKinematics k0;
  kine::LocalKinematics k1;

  Flags::Byte flag0 = BOOST_BINARY(000000);
  Flags::Byte flag1 = BOOST_BINARY(000000);

  kine::LocalKinematics k, l, k2;

  int count = int(pow(2, 6) * pow(2, 6));
  double err = 0;
  double threshold = 1e-30 * count;

  for(int i = 0; i < count; i++)
  {
    Vector3 pos0 = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>();
    kine::Orientation ori0 = kine::Orientation::randomRotation();
    Vector3 linvel0 = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>();
    Vector3 angvel0 = tools::ProbabilityLawSimulation::getGaussianMatrix<Vector3>();
    Vector3 linacc0 = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>();
    Vector3 angacc0 = tools::ProbabilityLawSimulation::getGaussianMatrix<Vector3>();

    Vector3 pos1 = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>();
    kine::Orientation ori1 = kine::Orientation::randomRotation();
    Vector3 linvel1 = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>();
    Vector3 angvel1 = tools::ProbabilityLawSimulation::getGaussianMatrix<Vector3>();
    Vector3 linacc1 = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>();
    Vector3 angacc1 = tools::ProbabilityLawSimulation::getGaussianMatrix<Vector3>();

    k0.reset();
    k1.reset();

    if(flag0 & Flags::position)
    {
      k0.position = pos0;
    }
    if(true) /// the orientation has to be set
    {
      k0.orientation = ori0;
    }
    if(flag0 & Flags::linVel)
    {
      k0.linVel = linvel0;
    }
    if(flag0 & Flags::angVel)
    {
      k0.angVel = angvel0;
    }
    if(flag0 & Flags::linAcc)
    {
      k0.linAcc = linacc0;
    }
    if(flag0 & Flags::angAcc)
    {
      k0.angAcc = angacc0;
    }

    if(flag1 & Flags::position)
    {
      k1.position = pos1;
    }
    if(!k1.position.isSet() || flag1 & Flags::orientation) /// the orientation has to be set
    {
      k1.orientation = ori1;
    }
    if(flag1 & Flags::linVel)
    {
      k1.linVel = linvel1;
    }
    if(flag1 & Flags::angVel)
    {
      k1.angVel = angvel1;
    }
    if(flag1 & Flags::linAcc)
    {
      k1.linAcc = linacc1;
    }
    if(flag1 & Flags::angAcc)
    {
      k1.angAcc = angacc1;
    }

    if((flag0 = (flag0 + 1) & Flags::all) == 0) /// update the flags to span all the possibilties
      flag1 = (flag1 + 1) & Flags::all;

    k2 = k1;
    if(!k1.orientation.isSet())
    {
      k2.orientation.setZeroRotation();
    }

    k = ((k0 * k2) * k2.getInverse()) * k0.getInverse();

    if(k.position.isSet())
    {
      err += k.position().squaredNorm();
    }
    if(k.orientation.isSet())
    {
      err += k.orientation.toRotationVector().squaredNorm();
    }
    if(k.linVel.isSet())
    {
      err += k.linVel().squaredNorm();
    }
    if(k.angVel.isSet())
    {
      err += k.angVel().squaredNorm();
    }
    if(k.linAcc.isSet())
    {
      err += k.linAcc().squaredNorm();
    }
    if(k.angAcc.isSet())
    {
      err += k.angAcc().squaredNorm();
    }
  }

  std::cout << "Error 1 : " << err << std::endl;

  if(err > threshold)
  {
    std::cout << "Error too large : " << err << std::endl;
    return errcode;
  }

  err = 0;

  threshold = 1e-19 * count;
  // threshold = 1e-9;

  flag0 = BOOST_BINARY(000000);
  flag1 = BOOST_BINARY(000000);

  for(int i = 0; i < count; i++)
  {
    /*
    std::cout << std::endl << "-----------------------------------------" << std::endl
                          << "New iteration"
              << std::endl << "-----------------------------------------" << std::endl;
    std::cout << "err: " << err << std::endl;
    */

    Vector3 pos0 = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>();
    kine::Orientation ori0 = kine::Orientation::randomRotation();
    Vector3 linvel0 = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>();
    Vector3 angvel0 = tools::ProbabilityLawSimulation::getGaussianMatrix<Vector3>();
    Vector3 linacc0 = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>();
    Vector3 angacc0 = tools::ProbabilityLawSimulation::getGaussianMatrix<Vector3>();

    k.reset();
    k.position = pos0;
    k.orientation = ori0;
    k.linVel = linvel0;
    k.angVel = angvel0;
    k.linAcc = linacc0;
    k.angAcc = angacc0;

    l = k;

    double dt = 0.001;
    // std::cout << std::endl << "k before integrate: " << std::endl << k << std::endl;
    k.integrate(dt);
    // std::cout << std::endl << "k after integrate: " << std::endl << k << std::endl;

    k0.reset();
    k1.reset();

    if(flag0 & Flags::position)
    {
      k0.position = l.position;
    }
    if(true)
    {
      k0.orientation = l.orientation;
    }
    if(flag0 & Flags::linVel)
    {
      k0.linVel = l.linVel;
    }
    if(flag0 & Flags::angVel)
    {
      k0.angVel = l.angVel;
    }
    if(flag0 & Flags::linAcc)
    {
      k0.linAcc = l.linAcc;
    }
    if(flag0 & Flags::angAcc)
    {
      k0.angAcc = l.angAcc;
    }

    if(flag1 & Flags::position)
    {
      k1.position = k.position;
    }
    if(flag1 & Flags::orientation)
    {
      k1.orientation = k.orientation;
    }
    if(flag1 & Flags::linVel)
    {
      k1.linVel = k.linVel;
    }
    if(flag1 & Flags::angVel)
    {
      k1.angVel = k.angVel;
    }
    if(flag1 & Flags::linAcc)
    {
      k1.linAcc = k.linAcc;
    }
    if(flag1 & Flags::angAcc)
    {
      k1.angAcc = k.angAcc;
    }

    if((flag0 = (flag0 + 1) & Flags::all) == 0) /// update the flags to span all the possibilties
      flag1 = (flag1 + 1) & Flags::all;

    kine::LocalKinematics::Flags::Byte flag = BOOST_BINARY(000000);

    if(k1.position.isSet()
       || (k0.position.isSet() && k0.linVel.isSet() && k0.linAcc.isSet() && k0.angVel.isSet() && k0.angAcc.isSet()))
    {
      flag = flag | kine::Kinematics::Flags::position;
    }
    if(true) // the orientation has to be computed
    {
      if(!k1.orientation.isSet() || !(k0.orientation.isSet() && k0.angVel.isSet() && k0.angAcc.isSet()))
      {
        continue; // if the the orientation can't be computed, the update function has to prompt an error, so we musn't
                  // test this combination
      }
      flag = flag | kine::LocalKinematics::Flags::orientation;
    }
    if(k1.linVel.isSet() || (k0.position.isSet() && k1.position.isSet() && k1.angVel.isSet())
       || (k0.linAcc.isSet() && k0.linVel.isSet() && k0.angVel.isSet()))
    {
      flag = flag | kine::Kinematics::Flags::linVel;
    }
    if(k1.angVel.isSet() || (k0.orientation.isSet() && k1.orientation.isSet())
       || (k0.angVel.isSet() && k0.angAcc.isSet()))
    {
      flag = flag | kine::Kinematics::Flags::angVel;
    }
    if(k1.linAcc.isSet() || (k0.linVel.isSet() && k1.linVel.isSet() && k1.angVel.isSet())
       || (k0.linVel.isSet() && k0.position.isSet() && k1.position.isSet() && k1.angAcc.isSet() && k1.angVel.isSet()))
    {
      flag = flag | kine::Kinematics::Flags::linAcc;
    }
    if(k1.angAcc.isSet() || (k0.angVel.isSet() && k1.angVel.isSet())
       || (k0.angVel.isSet() && k0.orientation.isSet() && k1.orientation.isSet()))
    {
      flag = flag | kine::Kinematics::Flags::angAcc;
    }

    // std::cout << std::endl << "k0 before update: " << std::endl << k0 << std::endl;
    // std::cout << std::endl << "k1 before update: " << std::endl << k1 << std::endl;
    k0.update(k1, dt, flag);
    // std::cout << std::endl << "k0 after update: " << std::endl << k0 << std::endl;

    // std::cout << "Error before all : " << err << std::endl;
    if(k0.position.isSet())
    {
      if((k.position() - k0.position()).squaredNorm() < 1e-13)
      {
        err += (k.position() - k0.position()).squaredNorm();
      }
      else
      {
        err += (l.position()
                - l.angVel().cross(dt * (l.position() + dt * (l.linVel() - 0.5 * l.angVel().cross(l.position()))))
                + dt * (l.linVel() + 0.5 * dt * (l.linAcc() - l.angAcc().cross(l.position()))) - k0.position())
                   .squaredNorm();
      }
      // std::cout << "Error after pos : " << err << std::endl;
    }
    if(k0.orientation.isSet())
    {
      err += (k0.orientation.differentiateRightSide(k.orientation)).squaredNorm();
      // std::cout << "Error after ori : " << err << std::endl;
    }
    if(k0.linVel.isSet())
    {

      if((k.linVel() - k0.linVel()).squaredNorm() < 1e-10)
      {
        err += (k.linVel() - k0.linVel()).squaredNorm();
      }
      else if(((k.position() - l.position()) / dt + k.angVel().cross(k.position()) - k0.linVel()).squaredNorm() < 1e-10)
      {
        err += ((k.position() - l.position()) / dt + k.angVel().cross(k.position()) - k0.linVel()).squaredNorm();
      }
      else
      {
        Vector3 tempAngVel = (l.orientation.differentiateRightSide(k.orientation) / dt);
        err += ((k.position() - l.position()) / dt + tempAngVel.cross(k.position()) - k0.linVel()).squaredNorm();
      }

      // std::cout << "Error after linVel : " << err << std::endl;
    }
    if(k0.angVel.isSet())
    {
      if((k.angVel() - k0.angVel()).squaredNorm() < 1e-10)
      {
        err += (k.angVel() - k0.angVel()).squaredNorm();
      }
      else
      {
        err += (l.orientation.differentiateRightSide(k.orientation) / dt - k0.angVel()).squaredNorm();
      }
      // std::cout << "Error after angVel : " << err << std::endl;
    }
    if(k0.linAcc.isSet())
    {

      if((k.linAcc() - k0.linAcc()).squaredNorm() < 1e-10)
      {
        err += (k.linAcc() - k0.linAcc()).squaredNorm();
      }
      else if((k.linAcc() - 2 * k0.linAcc()).squaredNorm() < 1e-10)
      {
        err += (k.linAcc() - 2 * k0.linAcc()).squaredNorm();
      }
      else if((k.angVel().cross(k.linVel()) + (k.linVel() - l.linVel()) / dt - k0.linAcc()).squaredNorm() < 1e-10)
      {
        err += (k.angVel().cross(k.linVel()) + (k.linVel() - l.linVel()) / dt - k0.linAcc()).squaredNorm();
      }
      else
      {
        Vector3 tempLinVel = (k.position() - l.position()) / dt + k.angVel().cross(k.position());
        err += (k.angVel().cross(tempLinVel) + (tempLinVel - l.linVel()) / dt - k0.linAcc()).squaredNorm();
      }

      // std::cout << "Error after linAcc : " << err << std::endl;
    }
    if(k0.angAcc.isSet())
    {
      if((k.angAcc() - k0.angAcc()).squaredNorm() < 1e-10)
      {
        err += (k.angAcc() - k0.angAcc()).squaredNorm();
      }
      else
      {
        err += (k.angAcc() - 2 * k0.angAcc()).squaredNorm();
      }

      // std::cout << "Error after angAcc : " << err << std::endl;
    }

    //        std::cout<< i<<" "<<err << std::endl;
    if(err > threshold)
    {
      break;
    }
  }

  std::cout << "Error 2 : " << err << std::endl;

  if(err > threshold)
  {
    std::cout << "Error too large !" << std::endl;
    return errcode;
  }

  return 0;
}

int testSE3_Integration_vs_euler(int errcode)
{
  double threshold = 1e-4;
  double err = 0;

  double dt = 0.001;
  int steps = 100;

  for(int i = 0; i < 10; i++)
  { // two different variables to also test when vl and omega_l are not zero at the same time.
    for(int j = 0; j < 10; j++)
    {
      LocalKinematics k;
      Vector3 pos = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>();
      kine::Orientation ori = kine::Orientation::randomRotation();
      Vector3 linvel = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>();
      Vector3 angvel = tools::ProbabilityLawSimulation::getGaussianMatrix<Vector3>();
      Vector3 linacc = tools::ProbabilityLawSimulation::getUniformMatrix<Vector3>();
      Vector3 angacc = tools::ProbabilityLawSimulation::getGaussianMatrix<Vector3>();

      pos *= i;
      linvel *= i;
      angvel *= j;
      linacc *= i;
      angacc *= j;

      k.position = pos;
      k.orientation = ori;
      k.linVel = linvel;
      k.angVel = angvel;
      k.linAcc = linacc;
      k.angAcc = angacc;

      LocalKinematics se3_IntegrationResult(k);
      LocalKinematics euler_IntegrationResult(k);

      Kinematics testGlobKine(se3_IntegrationResult);

      se3_IntegrationResult.SE3_integration(dt);
      testGlobKine.SE3_integration(linvel * dt, angvel * dt);
      LocalKinematics test2(testGlobKine);

      for(int l = 0; l < steps; l++)
      {
        euler_IntegrationResult.integrate(dt / steps);
      }
      LocalKinematics diff_se3_euler = se3_IntegrationResult * euler_IntegrationResult.getInverse();
      if(diff_se3_euler.position.isSet())
      {
        err += diff_se3_euler.position().squaredNorm();
      }
      if(diff_se3_euler.orientation.isSet())
      {
        err += diff_se3_euler.orientation.toRotationVector().squaredNorm();
      }
    }
  }

  std::cout << "Error between Euler integration and SE3 integration: " << err << std::endl;

  if(err > threshold)
  {
    return errcode;
  }
  return 0;
}

int main()
{
  int returnVal;
  int errorcode = 0;

  if((returnVal = testPositionByIntegration()))
  {
    std::cout << "testPositionByIntegration Failed, error code: " << returnVal << std::endl;
    return returnVal;
  }
  else
  {
    std::cout << "testPositionByIntegration succeeded" << std::endl;
  }

  if((returnVal = testSetToDiffNoAliasLocalKinematics(++errorcode)))
  {
    std::cout << "testSetToDiffNoAliasLocalKinematics Failed, error code: " << returnVal << std::endl;
    return returnVal;
  }
  else
  {
    std::cout << "testSetToDiffNoAliasLocalKinematics succeeded" << std::endl;
  }

  if((returnVal = testLocalVsGlobalIntegrates(++errorcode)))
  {
    std::cout << "testLocalVsGlobalIntegrates Failed, error code: " << returnVal << std::endl;
    return returnVal;
  }
  else
  {
    std::cout << "testLocalVsGlobalIntegrates succeeded" << std::endl;
  }

  if((returnVal = testRotationOperations(++errorcode)))
  {
    std::cout << "testRotationOperations Failed, error code: " << returnVal << std::endl;
    return returnVal;
  }
  else
  {
    std::cout << "testRotationOperations succeeded" << std::endl;
  }

  if((returnVal = testOrientation(++errorcode))) /// it is not an equality check
  {
    std::cout << "Orientation test failed, code : 1" << std::endl;
    return returnVal;
  }
  else
  {
    std::cout << "Orientation test succeeded" << std::endl;
  }

  if((returnVal = testLocalKinematics(++errorcode))) /// it is not an equality check
  {
    std::cout << "LocalKinematics test failed, code : 2" << std::endl;
    return returnVal;
  }
  else
  {
    std::cout << "LocalKinematics test succeeded" << std::endl;
  }

  if((returnVal = testSE3_Integration_vs_euler(++errorcode))) /// it is not an equality check
  {
    std::cout << "SE3 integration test failed, code :" << std::endl;
    return returnVal;
  }
  else
  {
    std::cout << "SE3 integration test succeeded" << std::endl;
  }

  std::cout << "test succeeded" << std::endl;
  return 0;
}
