/**
 * \file      rigid-body-kinematics.hpp
 * \author    Mehdi Benallegue
 * \date       2013
 * \brief      Implements integrators for the kinematics, in terms or rotations
 *             and translations.
 *
 * \details
 *
 *
 */

#ifndef StATEOBSERVATIONRIGIDBODYKINEMATICS_H
#define StATEOBSERVATIONRIGIDBODYKINEMATICS_H

#include <Eigen/SVD>

#include <state-observation/api.h>
#include <state-observation/tools/definitions.hpp>
#include <state-observation/tools/miscellaneous-algorithms.hpp>
#include <state-observation/tools/probability-law-simulation.hpp>

namespace stateObservation
{
namespace kine
{
inline void integrateKinematics(Vector3 & position, const Vector3 & velocity, double dt);

inline void integrateKinematics(Vector3 & position, Vector3 & velocity, const Vector3 & acceleration, double dt);

inline void integrateKinematics(Matrix3 & orientation, const Vector3 & rotationVelocity, double dt);

inline void integrateKinematics(Matrix3 & orientation,
                                Vector3 & rotationVelocity,
                                const Vector3 & rotationAcceleration,
                                double dt);

inline void integrateKinematics(Quaternion & orientation, const Vector3 & rotationVelocity, double dt);

inline void integrateKinematics(Quaternion & orientation,
                                Vector3 & rotationVelocity,
                                const Vector3 & rotationAcceleration,
                                double dt);

/// integrates the position/orientation and their time derivatives, given the
/// accelerations, and initial velocities and positions. The rotations are
/// expressed by rotation matrix
inline void integrateKinematics(Vector3 & position,
                                Vector3 & velocity,
                                const Vector3 & acceleration,
                                Matrix3 & orientation,
                                Vector3 & rotationVelocity,
                                const Vector3 & rotationAcceleration,
                                double dt);

/// integrates the position/orientation and their time derivatives, given the
/// accelerations, and initial velocities and positions. The orientations are
/// expressed by quaternions
inline void integrateKinematics(Vector3 & position,
                                Vector3 & velocity,
                                const Vector3 & acceleration,
                                Quaternion & orientation,
                                Vector3 & rotationVelocity,
                                const Vector3 & rotationAcceleration,
                                double dt);

/// integrates the postition/orientation given the velocities
inline void integrateKinematics(Vector3 & position,
                                const Vector3 & velocity,
                                Matrix3 & orientation,
                                const Vector3 & rotationVelocity,
                                double dt);

/// integrates the postition/orientation given the velocities
inline void integrateKinematics(Vector3 & position,
                                const Vector3 & velocity,
                                Quaternion & orientation,
                                const Vector3 & rotationVelocity,
                                double dt);

/// Puts the orientation vector norm between 0 and Pi if it
/// gets close to 2pi
inline Vector regulateRotationVector(const Vector3 & v);

/// Transforms the rotation vector into angle axis
inline AngleAxis rotationVectorToAngleAxis(const Vector3 & v);

/// Transforms the rotation vector into rotation matrix
inline Matrix3 rotationVectorToRotationMatrix(const Vector3 & v);

/// Transforms the rotation vector into quaternion
inline Quaternion rotationVectorToQuaternion(const Vector3 & v);

/// Transforms the rotation matrix into rotation vector
inline Vector3 rotationMatrixToRotationVector(const Matrix3 & R);

/// Tranbsform a quaternion into rotation vector
inline Vector3 quaternionToRotationVector(const Quaternion & q);

/// Transforms a quaternion into rotation vector
inline Vector3 quaternionToRotationVector(const Vector4 & v);

/// scalar component of a quaternion
inline double scalarComponent(const Quaternion & q);

/// vector part of the quaternion
inline Vector3 vectorComponent(const Quaternion & q);

/// Transforms the rotation matrix into roll pitch yaw
///(decompose R into Ry*Rp*Rr)
inline Vector3 rotationMatrixToRollPitchYaw(const Matrix3 & R, Vector3 & v);

inline Vector3 rotationMatrixToRollPitchYaw(const Matrix3 & R);

/// Transform the roll pitch yaw into rotation matrix
///( R = Ry*Rp*Rr)
inline Matrix3 rollPitchYawToRotationMatrix(double roll, double pitch, double yaw);

inline Matrix3 rollPitchYawToRotationMatrix(const Vector3 & rpy);

/// Transform the roll pitch yaw into rotation matrix
///( R = Ry*Rp*Rr)
inline Quaternion rollPitchYawToQuaternion(double roll, double pitch, double yaw);

inline Quaternion rollPitchYawToQuaternion(const Vector3 & rpy);

/// Projects the Matrix to so(3)
inline Matrix3 orthogonalizeRotationMatrix(const Matrix3 & M);

/// transform a 3d vector into a skew symmetric 3x3 matrix
inline Matrix3 skewSymmetric(const Vector3 & v, Matrix3 & R);

/// transform a 3d vector into a skew symmetric 3x3 matrix
inline Matrix3 skewSymmetric(const Vector3 & v);

/// transform a 3d vector into a squared skew symmetric 3x3 matrix
inline Matrix3 skewSymmetric2(const Vector3 & v, Matrix3 & R);

/// transform a 3d vector into a squared skew symmetric 3x3 matrix
inline Matrix3 skewSymmetric2(const Vector3 & v);

/// computes the V matrix used for the integration of the local linear velocity into the local position using SE(3)
/// formalism. Allows to take the evolution of the orientation over dt into account.
inline Matrix3 v_matrix(const Vector3 & rotVec);

/// transforms a homogeneous matrix into 6d vector (position theta mu)
inline Vector6 homogeneousMatrixToVector6(const Matrix4 & M);

/// transforms a 6d vector (position theta mu) into a homogeneous matrix
inline Matrix4 vector6ToHomogeneousMatrix(const Vector6 & v);

/// @brief Builds the smallest angle matrix allowing to get from a NORMALIZED vector v1 to its imahe Rv1
/// This is based on Rodrigues formula
///
/// @param v1 the NORMALIZED vector
/// @param Rv1 the NORMALIZED image of this vector by the rotation matrix R
/// @return Matrix3 the rotation matrix R
inline Matrix3 twoVectorsToRotationMatrix(const Vector3 & v1, const Vector3 Rv1);

/// @brief checks if this matrix is a pure yaw matrix or not
///
/// @param R the rotation matrix
/// @return true is pure yaw
/// @return false is not pure yaw
inline bool isPureYaw(const Matrix3 & R);

/// @brief Gets a vector that remains horizontal with this rotation. This vector is NOT normalized
/// @details There is a general version in getInvariantOrthogonalVector(). This can be used to extract yaw angle from
/// a rotation matrix without needing to specify an order in the tils (e.g. roll then pich).
///
/// @param R the input rotation
/// @return Vector3 the output horizontal vector
inline Vector3 getInvariantHorizontalVector(const Matrix3 & R);

/// @brief Gets a vector \f$v\f$ that is orthogonal to \f$e_z\f$ and such that \f$\hat{R}^T e_z\f$ is orthogonal to
/// the tilt \f$R^T e_z\f$. This vector is NOT normalized.
/// @details This is a generalization of getInvariantHorizontalVector() which corresponds to no tilt
/// \f$\hat{R}^T e_z=e_z\f$. This function is useful to merge the yaw from the rotation matrix with the tilt.
///
/// @param Rhat the input rotation matrix \f$\hat{R}^T\f$
/// @param Rtez the input tilt \f$\hat{R}^T e_z\f$
/// @return Vector3 the output horizontal vector
inline Vector3 getInvariantOrthogonalVector(const Matrix3 & Rhat, const Vector3 & Rtez);

/// @brief Merge the roll and pitch from the tilt (R^T e_z) with the yaw from a rotation matrix (minimizes the
/// deviation of the v vector)
/// @details throws exception when the orientation is singlular (likely gimbal lock)
/// to avoid these issues, we recommend to use mergeTiltWithYawAxisAgnostic()
/// @param Rtez the tilt \f$R_1^T e_z\f$ (the local image of \f$e_z\f$ unit vector)
/// @param R2 is the second rotation matrix from which the "yaw" needs to be extracted
/// @param v is the vector to use as reference it must be horizontal and normalized (for a traditional yaw v is by
/// deftault \f$e_x\f$)
/// @return Matrix3 the merged rotation matrix
inline Matrix3 mergeTiltWithYaw(const Vector3 & Rtez,
                                const Matrix3 & R2,
                                const Vector3 & v = Vector3::UnitX()) noexcept(false);

/// @brief Merge the roll and pitch with the yaw from a rotation matrix (minimizes the deviation of the v vector)
///
/// @param R1 is the first rotation to get the roll and pitch
/// @param R2 is the second rotation matrix from which the "yaw" needs to be extracted
/// @param v is the vector to use as reference (for a traditional yaw v is initialized to \f$e_x\f$)
/// @return Matrix3 the merged rotation matrix
inline Matrix3 mergeRoll1Pitch1WithYaw2(const Matrix3 & R1, const Matrix3 & R2, const Vector3 & v = Vector3::UnitX());

/// @brief Merge the roll and pitch from the tilt (R^T e_z) with the yaw from a rotation matrix (minimizes the deviation
/// of the v vector)
/// @param Rtez the tilt \f$R_1^T e_z\f$ (the local image of \f$e_z\f$ unit vector)
/// @param R2 is the second rotation matrix from which the "yaw" needs to be extracted
/// @param v is the vector to use as reference (for a traditional yaw v is initialized to \f$e_x\f$)
/// @return Matrix3 the merged rotation matrix
inline Matrix3 mergeTiltWithYawAxisAgnostic(const Vector3 & Rtez, const Matrix3 & R2);

/// @brief Merge the roll and pitch with the yaw from a rotation matrix with optimal reference vector
///
/// @param R1 is the first rotation to get the roll and pitch
/// @param R2 is the second rotation matrix from which the "yaw" needs to be extracted
/// @param v is the vector to use as reference (for a traditional yaw v is initialized to \f$e_x\f$)
/// @return Matrix3 the merged rotation matrix
inline Matrix3 mergeRoll1Pitch1WithYaw2AxisAgnostic(const Matrix3 & R1, const Matrix3 & R2);

/// @brief take 3x3 matrix represeting a rotation and gives the angle that vector v turns around the axis with this
/// rotation
/// @param rotation The 3x3 rotation matrix
/// @param axis the axis of rotation (must be normalized)
/// @param v the vector that is rotated with the rotation (must be orthogonal to axis and normalized)
/// @return double the angle
inline double rotationMatrixToAngle(const Matrix3 & rotation, const Vector3 & axis, const Vector3 & v);

/// @brief take 3x3 matrix represeting a rotation and gives the angle that vector v turns around the upward vertical
/// axis with this rotation
/// @details this is a generalization of yaw extraction (yaw is equivalent to v = Matrix3::UnitX(), but it is more
/// efficiant to calll the dedicated  rotationMatrixToYaw() without vector parameter).
/// @param rotation The 3x3 rotation matrix
/// @param v the rotated vector (expressed in the horizontal plane, must be normalized)
/// @return double the angle
inline double rotationMatrixToYaw(const Matrix3 & rotation, const Vector2 & v);

/// @brief take 3x3 matrix represeting a rotation and gives the yaw angle from roll pitch yaw representation
/// @param rotation The 3x3 rotation matrix
/// @return double the angle
inline double rotationMatrixToYaw(const Matrix3 & rotation);

/// @brief take 3x3 matrix represeting a rotation and gives a corresponding angle around upward vertical axis
/// @details This is similar to yaw angle but here we identify a horizontal vector that stays horizontal after rotation.
/// this can be called axis agnostic yaw extraction.
/// and get the angle between them
/// @param rotation The 3x3 rotation matrix
/// @return double the angle
inline double rotationMatrixToYawAxisAgnostic(const Matrix3 & rotation);

/// @brief Get the Identity Quaternion
///
/// @return Quaternion
inline Quaternion zeroRotationQuaternion();

/// @brief Get a uniformly random Quaternion
///
/// @return Quaternion
inline Quaternion randomRotationQuaternion();

/// @brief get a randomAngle between -pi and pu
///
/// @return double the random angle
inline double randomAngle();

/// @brief Checks if it is a rotation matrix (right-hand orthonormal) or not
/// @param precision the absolute precision of the test
/// @return true when it is a rotation matrix
/// @return false when not
inline bool isRotationMatrix(const Matrix3 &, double precision = 2 * cst::epsilon1);

/// transforms a rotation into translation given a constraint of a fixed point
inline void fixedPointRotationToTranslation(const Matrix3 & R,
                                            const Vector3 & rotationVelocity,
                                            const Vector3 & rotationAcceleration,
                                            const Vector3 & fixedPoint,
                                            Vector3 & outputTranslation,
                                            Vector3 & outputLinearVelocity,
                                            Vector3 & outputLinearAcceleration);

/// derivates a quaternion using finite difference to get a angular velocity vector
inline Vector3 derivateRotationFD(const Quaternion & q1, const Quaternion & q2, double dt);

/// derivates a rotation vector using finite difference to get a angular velocity vector
inline Vector3 derivateRotationFD(const Vector3 & o1, const Vector3 & o2, double dt);

inline Vector6 derivateHomogeneousMatrixFD(const Matrix4 & m1, const Matrix4 & m2, double dt);

inline Vector6 derivatePoseThetaUFD(const Vector6 & v1, const Vector6 & v2, double dt);

/// Computes the "multiplicative Jacobian" for
/// Kalman filtering for example
/// orientation is the current orientation
/// dR is the rotation delta between the current orientation and the orientation
/// at the next step.
/// dRdR is the "multiplicative" Jacobian with regard to variations of orientation
/// dRddeltaR is the "multiplicative" Jacobian with regard to variations of deltaR
inline void derivateRotationMultiplicative(const Vector3 & deltaR, Matrix3 & dRdR, Matrix3 & dRddeltaR);

/// Computes the "multiplicative Jacobian" for
/// a function R^T.v giving a vector v expressed in a local frame
/// with regard to Rotations of this local frame
inline Matrix3 derivateRtvMultiplicative(const Matrix3 & R, const Vector3 & v);

/// uses the derivation to reconstruct the velocities and accelerations given
/// trajectories in positions and orientations only
inline IndexedVectorArray reconstructStateTrajectory(const IndexedVectorArray & positionOrientation, double dt);

inline Vector invertState(const Vector & state);

inline Matrix4 invertHomoMatrix(const Matrix4 & m);

enum rotationType
{
  matrix = 0,
  rotationVector = 1,
  quaternion = 2,
  angleaxis = 3
};

template<rotationType = rotationVector>
struct indexes
{
};

template<>
struct indexes<rotationVector>
{
  /// indexes of the different components of a vector of the kinematic state
  /// when the orientation is represented using a 3D rotation vector
  static const Index pos = 0;
  static const Index ori = 3;
  static const Index linVel = 6;
  static const Index angVel = 9;
  static const Index linAcc = 12;
  static const Index angAcc = 15;
  static const Index size = 18;
};

template<>
struct indexes<quaternion>
{
  /// indexes of the different components of a vector of the kinematic state
  /// when the orientation is represented using a quaternion
  static const Index pos = 0;
  static const Index ori = 3;
  static const Index linVel = 7;
  static const Index angVel = 10;
  static const Index linAcc = 13;
  static const Index angAcc = 16;
  static const Index size = 19;
};

/// relative tolereance to the square of quaternion norm.
constexpr double quatNormTol = 1e-6;

class Orientation
{
public:
  /// The parameter initialize should be set to true except when it is
  /// certain that the initial value will not be used
  /// And that the first operation would be to set its value
  explicit Orientation(bool initialize = true);

  /// this is the rotation vector and NOT Euler angles
  explicit Orientation(const Vector3 & v);

  explicit Orientation(const Quaternion & q);

  explicit Orientation(const Matrix3 & m);

  explicit Orientation(const AngleAxis & aa);

  Orientation(const Quaternion & q, const Matrix3 & m);

  Orientation(const double & roll, const double & pitch, const double & yaw);

  Orientation(const Orientation & multiplier1, const Orientation & multiplier2);

  inline Orientation & operator=(const Vector3 & v);

  inline Orientation & operator=(const Quaternion & q);

  inline Orientation & operator=(const Matrix3 & m);

  inline Orientation & operator=(const AngleAxis & aa);

  inline Orientation & setValue(const Quaternion & q, const Matrix3 & m);

  inline Orientation & fromVector4(const Vector4 & v);

  inline Orientation & setRandom();

  template<typename t = Quaternion>
  inline Orientation & setZeroRotation();

  /// get a const reference on the matrix or the quaternion
  inline const Matrix3 & toMatrix3() const;
  inline const Quaternion & toQuaternion() const;

  inline operator const Matrix3 &() const;
  inline operator const Quaternion &() const;

  inline Vector4 toVector4() const;

  inline Vector3 toRotationVector() const;
  inline Vector3 toRollPitchYaw() const;
  inline AngleAxis toAngleAxis() const;

  /// Multiply the rotation (orientation) by another rotation R2
  /// the non const versions allow to use more optimized methods

  inline Orientation operator*(const Orientation & R2) const;

  /// Noalias versions of the operator*
  inline const Orientation & setToProductNoAlias(const Orientation & R1, const Orientation & R2);

  inline Orientation inverse() const;

  /// uses the vector dt_x_omega as the increment of rotation expressed in the
  /// world frame. Which gives R_{k+1}=\exp(S(dtxomega))R_k.
  /// This function is also used to sum two Orientations expressed in the same frame at the same time k, even for the
  /// LocalKinematics (the integration of the orientation is different but not the sum)
  inline const Orientation & integrate(Vector3 dt_x_omega);

  /// use the vector dt_x_omega as the increment of rotation expressed in the
  /// local frame. Which gives R_{k+1}=R_k*exp(S(dtxomega))
  inline const Orientation & integrateRightSide(Vector3 dt_x_omega);

  /// @brief gives the log (rotation vector) of the "left-side" difference of orientation: log of
  /// R_k1*(*this).inverse().
  /// @details This function is also used to differentiate two Orientations expressed in the same frame at the same time
  /// k, even for the LocalKinematics (the integration of the orientation is different and therefore the associated
  /// differentiation also is, but the difference remains the same)
  /// @param R_k1 the other orientation with which we compute the difference.
  /// @return Vector3
  inline Vector3 differentiate(Orientation R_k1) const;

  /// @brief gives the log (rotation vector) of the "right-side" difference of orientation: log of (*this).inverse() *
  /// R_k1.
  /// @details This function is also used to differentiate two Orientations expressed in the same frame at the same time
  /// k, even for the LocalKinematics (the integration of the orientation is different and therefore the associated
  /// differentiation also is, but the difference remains the same)
  /// @param R_k1 the other orientation with which we compute the difference.
  /// @return Vector3
  inline Vector3 differentiateRightSide(Orientation R_k1) const;

  /// Rotate a vector
  inline Vector3 operator*(const Vector3 & v) const;

  /// @brief checks that the orientation has been assigned a value.
  /// @return bool
  inline bool isSet() const;

  /// @brief resets the Orientation object.
  /// @details The orientation is then considered as outdated and cannot be used until it is assigned a value.
  inline void reset();

  /// @brief checks that the matrix representation of the orientation has been assigned a value.
  /// @return bool
  inline bool isMatrixSet() const;
  /// @brief checks that the quaternion representation of the orientation has been assigned a value.
  /// @return bool
  inline bool isQuaternionSet() const;

  /// switch the state of the Matrix or quaternion to set or not
  /// this can be used for forward initialization
  inline void setMatrix(bool b = true);
  inline void setQuaternion(bool b = true);

  /// no checks are performed for these functions, use with caution

  /// @brief get a reference to the matrix representation of the orientation without calling the check functions.
  /// @details Has to be used with care as this function doesn't check that the matrix has been assigned values.
  /// @return CheckedMatrix3
  inline CheckedMatrix3 & getMatrixRefUnsafe();
  /// @brief get a reference to the quaternion representation of the orientation without calling the check functions.
  /// @details Has to be used with care as this function doesn't check that the quaternion has been assigned values.
  /// @return CheckedQuaternion
  inline CheckedQuaternion & getQuaternionRefUnsafe();

  /// synchronizes the representations (quaternion and rotation matrix)
  inline void synchronize();

  /// retruns a zero rotation
  static inline Orientation zeroRotation();

  /// Returns a uniformly distributed random rotation
  static inline Orientation randomRotation();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
  void check_() const;

  inline const Matrix3 & quaternionToMatrix_() const;
  inline const Quaternion & matrixToQuaternion_() const;

  mutable CheckedQuaternion q_;
  mutable CheckedMatrix3 m_;
};

namespace internal
{

template<class T>
class KinematicsInternal
{
public:
  KinematicsInternal() {}

  /// @brief constructor of a Kinematics object given each variable independently.
  /// @param position the position composing the kinematics
  /// @param linVel the linear velocity composing the kinematics
  /// @param linAcc the linear acceleration composing the kinematics
  /// @param orientation the orientation composing the kinematics
  /// @param angVel the angular velocity composing the kinematics
  /// @param angAcc the angular acceleration composing the kinematics
  KinematicsInternal(const CheckedVector3 & position,
                     const CheckedVector3 & linVel,
                     const CheckedVector3 & linAcc,
                     const Orientation & orientation,
                     const CheckedVector3 & angVel,
                     const CheckedVector3 & angAcc);
  struct Flags
  {
    typedef unsigned char Byte;

    static const Byte position = BOOST_BINARY(000001);
    static const Byte orientation = BOOST_BINARY(000010);
    static const Byte linVel = BOOST_BINARY(000100);
    static const Byte angVel = BOOST_BINARY(001000);
    static const Byte linAcc = BOOST_BINARY(010000);
    static const Byte angAcc = BOOST_BINARY(100000);

    static const Byte pose = position | orientation;
    static const Byte vel = linVel | angVel;
    static const Byte acc = linAcc | angAcc;
    static const Byte all = pose | vel | acc;
  };

  CheckedVector3 position;
  Orientation orientation;

  CheckedVector3 linVel;
  CheckedVector3 angVel;

  CheckedVector3 linAcc;
  CheckedVector3 angAcc;

  inline void reset();

  /// Fills from vector
  /// the flags show which parts of the kinematics to be loaded from the vector
  /// the order of the vector is
  /// position orientation (quaternion) linevel angvel linAcc angAcc
  /// use the flags to define the structure of the vector
  T & fromVector(const Vector & v, typename Flags::Byte = Flags::all);

  /// initializes at zero all the flagged fields
  /// the typename allows to set if the prefered type for rotation
  /// is a Matrix3 or a Quaternion (Quaternion by default)
  template<typename t = Quaternion>
  T & setZero(typename Flags::Byte = Flags::all);

  /// @brief returns an object corresponding to zero kinematics on the desired variables.
  /// @param Flags defines of which variables the new object must be filled.
  /// @return Kinematics
  static inline T zeroKinematics(typename Flags::Byte = Flags::all);

  /// converts the object to a vector
  /// the order of the vector is
  /// position orientation (quaternion) linevel angvel linAcc angAcc
  /// use the flags to define the structure of the vector
  inline Vector toVector(typename Flags::Byte) const;
  inline Vector toVector() const;
};
} // namespace internal

struct LocalKinematics;

/// @brief Class facilitating the manipulation of the kinematics of a frame within another and the associated
/// operations.
/// @details  The Kinematics object contains the position, the orientation, the velocities and the accelerations of a
/// frame 1 within another frame 2. The object Kinematics is the expression of these kinematics in the global frame 2,
/// while the LocalKinematics object is their expression in the local frame 1.
struct Kinematics : public internal::KinematicsInternal<Kinematics>
{

  Kinematics() {}

  /// Constructor from a vector
  /// the flags show which parts of the kinematics to be loaded from the vector
  /// the order of the vector is
  /// position orientation (quaternion) linevel angvel linAcc angAcc
  /// use the flags to define the structure of the vector
  Kinematics(const Vector & v, Flags::Byte = Flags::all);

  /// @brief constructor of a Kinematics object resulting from the composition of two others.
  /// @param multiplier1 the first Kinematics object used for the composition
  /// @param multiplier2 the second Kinematics object used for the composition
  Kinematics(const Kinematics & multiplier1, const Kinematics & multiplier2);

  /// @brief constructor of a Kinematics object given each variable independently.
  /// @param position the position composing the kinematics
  /// @param linVel the linear velocity composing the kinematics
  /// @param linAcc the linear acceleration composing the kinematics
  /// @param orientation the orientation composing the kinematics
  /// @param angVel the angular velocity composing the kinematics
  /// @param angAcc the angular acceleration composing the kinematics
  Kinematics(const CheckedVector3 & position,
             const CheckedVector3 & linVel,
             const CheckedVector3 & linAcc,
             const Orientation & orientation,
             const CheckedVector3 & angVel,
             const CheckedVector3 & angAcc);

  /// @brief constructor of a Kinematics object given its equivalent in the local frame.
  /// @details performs the conversion from the local to the global expression of the kinematics.
  /// @param locK the local kinematics to convert
  explicit inline Kinematics(const LocalKinematics & locK);

  /// @brief fills the Kinematics object given its equivalent in the local frame.
  /// @details performs the conversion from the local to the global expression of the kinematics.
  /// @param locK the local kinematics to convert
  inline Kinematics & operator=(const LocalKinematics & locK);

  /// @brief integrates the current kinematics over the timestep dt.
  /// @details can be used to predict the future kinematics from the current ones.
  /// @param dt the timestep used for the integration
  /// @return const Kinematics &
  inline const Kinematics & integrate(double dt);

  /// @brief integrates the current kinematics over the timestep dt using the SE3 integration.
  /// @details can be used to predict the future kinematics from the current ones.
  /// @param dt the timestep used for the integration
  /// @param vl_dt local linear velocity times dt
  /// @param omega_l_dt local angular velocity times dt
  /// @return const Kinematics &
  inline const Kinematics & SE3_integration(const Vector3 & vl_dt, const Vector3 & omega_l_dt);

  /// @brief updates the current kinematics (k) with the new ones (k+1).
  /// @details flags allow to chose what variables must be contained in the new kinematics. If a variable is not given
  /// in the updated Kinematics object, it computed using either integration or finite differences depending on the
  /// available information.
  /// @param newValue the new values of the kinematics
  /// @param dt the time ellapsed between the current kinematics and the new ones.
  /// @param Flags the flags indicating which variables are desired in the updated kinematics.
  /// @return const Kinematics &
  inline const Kinematics & update(const Kinematics & newValue, double dt, Flags::Byte = Flags::all);

  /// @brief returns the inverse of the current kinematics.
  /// @details if the current object represents the kinematics of a frame 1 in a frame 2, will return the kinematics of
  /// the frame 2 in 1.
  /// @return Kinematics
  inline Kinematics getInverse() const;

  /// composition of transformation
  inline Kinematics operator*(const Kinematics &) const;

  /// @brief computes the composition of two Kinematics object.
  /// @param operand1 the first Kinematics object used in the composition
  /// @param operand2 the second Kinematics object used in the composition
  /// @return Kinematics
  inline Kinematics & setToProductNoAlias(const Kinematics & operand1, const Kinematics & operand2);

  /// Allows to compute the difference between two Kinematics objects. Has the same effect than calling
  /// setToProductNoAlias(operand1, operand2.getInverse()) but is computationally faster
  inline Kinematics & setToDiffNoAlias(const Kinematics & multiplier1, const Kinematics & multiplier2);

  /// Linear part of the setToDiffNoAlias(const Kinematics &, const Kinematics &) function.
  inline Kinematics & setToDiffNoAliasLinPart(const Kinematics & multiplier1, const Kinematics & multiplier2);

  /// Angular part of the setToDiffNoAlias(const Kinematics &, const Kinematics &) function.
  inline Kinematics & setToDiffNoAliasAngPart(const Kinematics & multiplier1, const Kinematics & multiplier2);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
  Vector3 tempVec_;
};

/// @brief Class facilitating the manipulation of the local kinematics of a frame within another and the associated
/// operations.
/// @details  The Kinematics object contains the position, the orientation, the velocities and the accelerations of a
/// frame 1 within another frame 2. The object Kinematics is the expression of these kinematics in the global frame 2,
/// while the LocalKinematics object is their expression in the local frame 1.
struct LocalKinematics : public internal::KinematicsInternal<LocalKinematics>
{
  LocalKinematics() {}

  /// Constructor from a vector
  /// the flags show which parts of the kinematics to be loaded from the vector
  /// the order of the vector is
  /// position orientation (quaternion) linevel angvel linAcc angAcc
  /// use the flags to define the structure of the vector
  inline LocalKinematics(const Vector & v, Flags::Byte flags);

  /// @brief constructor of a LocalKinematics object resulting from the composition of two others.
  /// @param multiplier1 the first LocalKinematics object used for the composition
  /// @param multiplier2 the second LocalKinematics object used for the composition
  inline LocalKinematics(const LocalKinematics & multiplier1, const LocalKinematics & multiplier2);

  /// @brief constructor of a Kinematics object given each variable independently.
  /// @param position the position composing the kinematics
  /// @param linVel the linear velocity composing the kinematics
  /// @param linAcc the linear acceleration composing the kinematics
  /// @param orientation the orientation composing the kinematics
  /// @param angVel the angular velocity composing the kinematics
  /// @param angAcc the angular acceleration composing the kinematics
  LocalKinematics(const CheckedVector3 & position,
                  const CheckedVector3 & linVel,
                  const CheckedVector3 & linAcc,
                  const Orientation & orientation,
                  const CheckedVector3 & angVel,
                  const CheckedVector3 & angAcc);

  /// @brief constructor of a LocalKinematics object given its equivalent in the global frame.
  /// @details performs the conversion from the global to the local expression of the kinematics.
  /// @param locK the global kinematics to convert
  explicit inline LocalKinematics(const Kinematics & kin);

  /// @brief fills the LocalKinematics object given its equivalent in the global frame.
  /// @details performs the conversion from the global to the local expression of the kinematics.
  /// @param locK the global kinematics to convert
  inline LocalKinematics & operator=(const Kinematics & kine);

  /// initializes at zero all the flagged fields
  /// the typename allows to set if the prefered type for rotation
  /// is a Matrix3 or a Quaternion (Quaternion by default)
  template<typename t = Quaternion>
  LocalKinematics & setZero(Flags::Byte = Flags::all);

  /// @brief integrates the current local kinematics over the timestep dt.
  /// @details can be used to predict the future local kinematics from the current ones.
  /// @param dt the timestep used for the integration
  /// @return const LocalKinematics &
  inline const LocalKinematics & integrate(double dt);

  /// @brief integrates the current kinematics over the timestep dt using the SE3 integration.
  /// @details can be used to predict the future kinematics from the current ones.
  /// @param dt the timestep used for the integration
  /// @return const Kinematics &
  inline const LocalKinematics & SE3_integration(double dt);

  /// @brief updates the current local kinematics (k) with the new ones (k+1).
  /// @details flags allow to chose what variables must be contained in the new local kinematics. If a variable is not
  /// given in the updated LocalKinematics object, it computed using either integration or finite differences depending
  /// on the available information.
  /// @param newValue the new values of the local kinematics
  /// @param dt the time ellapsed between the current kinematics and the new ones.
  /// @param Flags the flags indicating which variables are desired in the updated local kinematics.
  /// @return const LocalKinematics &
  inline const LocalKinematics & update(const LocalKinematics & newValue, double dt, Flags::Byte = Flags::all);

  /// @brief returns the inverse of the current local kinematics.
  /// @details if the current object represents the local kinematics of a frame 1 in a frame 2, will return the local
  /// kinematics of the frame 2 in 1.
  /// @return LocalKinematics
  inline LocalKinematics getInverse() const;

  /// composition of transformation
  inline LocalKinematics operator*(const LocalKinematics &) const;

  /// @brief computes the composition of two LocalKinematics object.
  /// @param operand1 the first LocalKinematics object used in the composition
  /// @param operand2 the second LocalKinematics object used in the composition
  /// @return LocalKinematics
  inline LocalKinematics & setToProductNoAlias(const LocalKinematics & operand1, const LocalKinematics & operand2);

  /// Allows to compute the difference between two LocalKinematics objects. Has the same effect that calling
  /// setToProductNoAlias(operand1, operand2.getInverse()) but is computationally faster
  inline LocalKinematics & setToDiffNoAlias(const LocalKinematics & multiplier1, const LocalKinematics & multiplier2);

  /// Linear part of the setToDiffNoAlias(const LocalKinematics &, const LocalKinematics &) function.
  inline LocalKinematics & setToDiffNoAliasLinPart(const LocalKinematics & multiplier1,
                                                   const LocalKinematics & multiplier2);

  /// Angular part of the setToDiffNoAlias(const LocalKinematics &, const LocalKinematics &) function.
  inline LocalKinematics & setToDiffNoAliasAngPart(const LocalKinematics & multiplier1,
                                                   const LocalKinematics & multiplier2);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
  Vector3 tempVec_;
  Vector3 tempVec_2;
  Vector3 tempVec_3;
  Vector3 tempVec_4;
  Vector3 tempVec_5;
};

} // namespace kine
} // namespace stateObservation

inline std::ostream & operator<<(std::ostream & os, const stateObservation::kine::Kinematics & k);

inline std::ostream & operator<<(std::ostream & os, const stateObservation::kine::LocalKinematics & k);

#include <state-observation/tools/rigid-body-kinematics.hxx>

#endif // StATEOBSERVATIONRIGIDBODYKINEMATICS_H
