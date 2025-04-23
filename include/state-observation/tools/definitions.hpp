/**
 * \file     definitions.hpp
 * \author   Mehdi Benallegue
 * \date     2013
 * \brief    Definitions of types and some structures.
 *
 * \details
 *
 *
 */

#ifndef STATEOBSERVATIONDEFINITIONSHPP
#define STATEOBSERVATIONDEFINITIONSHPP

// #define STATEOBSERVATION_VERBOUS_CONSTRUCTORS

#include <chrono>
#include <deque>
#include <memory>
#include <stdexcept>
#include <vector>

#ifdef STATEOBSERVATION_VERBOUS_CONSTRUCTORS
#  include <iostream>
#endif

#include <boost/assert.hpp>
#include <boost/timer/timer.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#ifndef M_PI
#  include <boost/math/constants/constants.hpp>
#  define M_PI boost::math::constants::pi<double>()
#endif

#include <state-observation/api.h>

// basic file operations
#include <fstream>

namespace stateObservation
{
/// @brief Checks if it is derived from EigenBase (the base class of all dense functions)
///
/// @tparam T The class that you want to check
template<typename T>
struct isEigen
{
  static constexpr bool value = std::is_base_of<Eigen::EigenBase<T>, T>::value;
};

/// @brief Checks if a class is a specialization of Eigen::Matrix
/// @tparam T The class that you want to check
template<typename T>
struct isMatrix : std::false_type
{
};

template<typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
struct isMatrix<Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>> : std::true_type
{
};

template<typename T>
struct EigenType : std::enable_if<isEigen<T>::value, T>
{
};

template<typename T>
struct MatrixType : std::enable_if<isMatrix<T>::value, T>
{
};

/// Dynamic sized scalar vector
typedef Eigen::VectorXd Vector;

/// 1D Vector
typedef Eigen::Matrix<double, 1, 1> Vector1;

/// 2d Vector
typedef Eigen::Matrix<double, 2, 1> Vector2;

/// 3D vector
typedef Eigen::Vector3d Vector3;

/// 3D vector unaligned
typedef Eigen::Matrix<double, 3, 1, Eigen::DontAlign> Vector3Unaligned;

/// 4D vector
typedef Eigen::Vector4d Vector4;

/// 5D vector
typedef Eigen::Matrix<double, 5, 1> Vector5;

/// 6D vector
typedef Eigen::Matrix<double, 6, 1> Vector6;

/// Dynamic sized Matrix
typedef Eigen::MatrixXd Matrix;

/// 1D scalar Matrix
typedef Eigen::Matrix<double, 1, 1> Matrix1;

/// 2D scalar Matrix
typedef Eigen::Matrix2d Matrix2;

/// 3x3 Scalar Matrix
typedef Eigen::Matrix3d Matrix3;

/// 3x3 Scalar Matrix Unaligned
typedef Eigen::Matrix<double, 3, 3, Eigen::DontAlign> Matrix3Unaligned;

/// 4x4 Scalar Matrix
typedef Eigen::Matrix4d Matrix4;

/// 5x5 Scalar Matrix
typedef Eigen::Matrix<double, 5, 5> Matrix5;

/// 6x6 Scalar Matrix
typedef Eigen::Matrix<double, 6, 6> Matrix6;

/// 12x12 scalar Matrix
typedef Eigen::Matrix<double, 12, 12> Matrix12;

/// Quaternion
typedef Eigen::Quaterniond Quaternion;

/// Quaternion Unaligned
typedef Eigen::Quaternion<double, Eigen::DontAlign> QuaternionUnaligned;

/// Euler Axis/Angle representation of orientation
typedef Eigen::AngleAxis<double> AngleAxis;

/// 2D rotations
typedef Eigen::Rotation2D<double> Rotation2D;

typedef Eigen::Index Index;
typedef long int TimeIndex;
typedef Index TimeSize;

#ifndef NDEBUG
static const bool isDebug = true;
#else
static const bool isDebug = false;
#endif // NDEBUG

template<int compileTimeRows = -1, int compileTimeCols = -1>
class FixOrDynMatrixToolsBySize
{
public:
  template<typename CustomNullaryOp>
  static const Eigen::CwiseNullaryOp<CustomNullaryOp, typename Eigen::Matrix<double, compileTimeRows, compileTimeCols>>
      nullaryExp(const CustomNullaryOp & func, Index rows = compileTimeRows, Index cols = compileTimeCols)
  {
    /// Prevent warnings
    (void)rows;
    (void)cols;

    return Eigen::Matrix<double, compileTimeRows, compileTimeCols>::NullaryExpr(func);
  }
};

template<int compileTimeCols>
class FixOrDynMatrixToolsBySize<-1, compileTimeCols>
{
public:
  template<typename CustomNullaryOp>
  static const Eigen::CwiseNullaryOp<CustomNullaryOp, typename Eigen::Matrix<double, -1, compileTimeCols>> nullaryExp(
      const CustomNullaryOp & func,
      Index rows = -1,
      Index cols = compileTimeCols)
  {
    return Eigen::Matrix<double, -1, compileTimeCols>::NullaryExpr(rows, cols, func);
  }
};

template<int compileTimeRows>
class FixOrDynMatrixToolsBySize<compileTimeRows, -1>
{
public:
  template<typename CustomNullaryOp>
  static const Eigen::CwiseNullaryOp<CustomNullaryOp, typename Eigen::Matrix<double, compileTimeRows, -1>> nullaryExp(
      const CustomNullaryOp & func,
      Index rows = compileTimeRows,
      Index cols = -1)
  {
    return Eigen::Matrix<double, compileTimeRows, -1>::NullaryExpr(rows, cols, func);
  }
};

template<>
class FixOrDynMatrixToolsBySize<-1, -1>
{
public:
  template<typename CustomNullaryOp>
  static const Eigen::CwiseNullaryOp<CustomNullaryOp, typename Eigen::Matrix<double, -1, -1>> nullaryExp(
      const CustomNullaryOp & func,
      Index rows = -1,
      Index cols = -1)
  {
    return Eigen::Matrix<double, -1, -1>::NullaryExpr(rows, cols, func);
  }
};

template<typename MatrixT>
class FixOrDynMatrixTools : public FixOrDynMatrixToolsBySize<MatrixType<MatrixT>::type::RowsAtCompileTime,
                                                             MatrixType<MatrixT>::type::ColsAtCompileTime>
{
};

/// Debug item default value is just a way to give a default value to debug item
/// this was required by a compilation issue on Visual Studio
template<typename T, const T defaultValue = T()>
class DebugItemDefaultValue
{
public:
  static const T v;
};

template<typename T, const T defaultValue>
const T DebugItemDefaultValue<T, defaultValue>::v = defaultValue;

namespace detail
{
typedef DebugItemDefaultValue<bool, true> defaultTrue;

enum errorType
{
  message,
  exception,
  exceptionAddr
};

template<errorType i = message, int dummy = 0>
class DebugItemDefaultError
{
};

template<int dummy>
class DebugItemDefaultError<message, dummy>
{
public:
  static const char * v;
};

template<int dummy>
class DebugItemDefaultError<exception, dummy>
{
public:
  static const std::runtime_error v;
};

template<int dummy>
class DebugItemDefaultError<exceptionAddr, dummy>
{
public:
  static const std::runtime_error * v;
};

template<int dummy>
const char * DebugItemDefaultError<message, dummy>::v = "The Object is not initialized. \
       If this happened during initialization then run command chckitm_set() \
       to switch it to set. And if the initialization is incomplete, run \
       chckitm_reset() afterwards.";

template<int dummy>
const std::runtime_error DebugItemDefaultError<exception, dummy>::v =
    std::runtime_error(DebugItemDefaultError<message>::v);

template<int dummy>
const std::runtime_error * DebugItemDefaultError<exceptionAddr, dummy>::v = &DebugItemDefaultError<exception>::v;

typedef DebugItemDefaultError<message> defaultErrorMSG;
typedef DebugItemDefaultError<exception> defaultException;
typedef DebugItemDefaultError<exceptionAddr> defaultExceptionAddr;

void STATE_OBSERVATION_DLLAPI defaultSum(const Vector & stateVector, const Vector & tangentVector, Vector & sum);
void STATE_OBSERVATION_DLLAPI defaultDifference(const Vector & stateVector1,
                                                const Vector & stateVector2,
                                                Vector & difference);

} // namespace detail

/// Debug item is an item that exists when the debug variable is true,
/// otherwise it is empty and returns only the default value
template<typename T, typename defaultValue = DebugItemDefaultValue<T>, bool debug = true>
class DebugItem
{
public:
  DebugItem() : b_(defaultValue::v) {}
  explicit DebugItem(const T & v) : b_(v) {}
  inline T & operator=(T v)
  {
    return b_ = v;
  }
  inline operator T() const
  {
    return b_;
  }
  inline T set(const T & v)
  {
    return b_ = v;
  }
  T get() const
  {
    return b_;
  }

private:
  T b_;
};

/// this specialization contains no object
template<typename T, typename defaultValue>
class DebugItem<T, defaultValue, false>
{
public:
  DebugItem() {}
  explicit DebugItem(T) {}
  inline T & operator=(T v)
  {
    /// I am not sure what is the best behaviour, is it to return v ot defaultValue::v?
    return v;
  }
  inline operator T() const
  {
    return defaultValue::v;
  }
  inline T set(const T &)
  {
    return defaultValue::v;
  }
  T get() const
  {
    return defaultValue::v;
  }

private:
  /// This object is not used but it is still declared to keep the same signature
  T b_;
};

/// @brief This structure is used as an additionalChecker for a CheckedItem that doesn't require additional tests.
/// @details This structure's check operations are always true.
struct EmptyChecker
{
  template<typename T>
  static bool check(const T &)
  {
    return true;
  }
  template<typename T>
  static bool checkAssert(const T &)
  {
    return true;
  }
  inline static constexpr char errorMessage[] = "";
  using ExceptionT = std::runtime_error;
};

/// @brief this is a structure allowing for automatically verifying that the item has been initialized or not. The
/// chckitm_reset() function allows to set it back to "not initialized" state.
/// @tparam T is the contained type
/// @tparam lazy means that the "set" value is true all the time if NDEBUG is defined
/// @tparam alwaysCheck means that the check is always performed and throws exception if it fails. Otherwise, the check
/// is performed only for debug warning, this has no effect if lazy is set to true
/// @tparam assertion means that an assertion will be introduced for the check.
/// @tparam eigenAlignedNew should be set to true if any alignment is required for the new operator (see eigen
/// documentation)
/// @tparam additionalChecker defines a check function that is called in addition to the initialization check
template<typename T,
         bool lazy = false,
         bool alwaysCheck = false,
         bool assertion = true,
         bool eigenAlignedNew = false,
         typename additionalChecker = EmptyChecker>
class CheckedItem
{
public:
  /// The parameter initialize sets whether the isSet() parameter is initialized to false
  CheckedItem(bool initialize = true);
  explicit CheckedItem(const T &);
  CheckedItem(const CheckedItem &);
  virtual ~CheckedItem() {}

  inline CheckedItem & operator=(const CheckedItem &);
  inline T & operator=(const T &);
  inline operator T() const;
  inline operator const T &() const;

  inline const T & chckitm_getValue() const;

  inline T & operator()();
  inline const T & operator()() const;

  inline T & getRefUnchecked();
  inline const T & getRefUnchecked() const;

  inline bool isSet() const;
  inline void reset();

  /// set the value of the initialization check boolean
  inline void set(bool value);

  void setAssertMessage(std::string s);
  void setExceptionPtr(std::exception * e);

  /// allows to set the initialization boolean to true and give a reference
  /// to the object with the same instruction
  /// should be used to initialize the object without using the
  /// assignment operator
  inline T & set();

protected:
  static const bool do_check_ = !lazy || isDebug;
  static const bool do_assert_ = do_check_ && assertion;
  static const bool do_exception_ = do_check_ && !assertion;

  typedef DebugItem<bool, detail::defaultTrue, do_check_> IsSet;
  typedef DebugItem<const char *, detail::defaultErrorMSG, do_assert_> AssertMsg;
  typedef DebugItem<const std::exception *, detail::defaultExceptionAddr, do_exception_> ExceptionPtr;

  IsSet isSet_;
  AssertMsg assertMsg_;
  ExceptionPtr exceptionPtr_;

  bool chckitm_check_() const; /// this can throw(std::exception)
  T v_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(eigenAlignedNew)
};

/// @brief Additional checker that allows to check for the presence of NaN values in the item.
struct CheckNaN
{
  template<typename T>
  static bool check(const T & m)
  {
    return !m.hasNaN();
  }
  template<typename T>
  static bool checkAssert(const T & m)
  {
    BOOST_ASSERT(check(m) && errorMessage);
    return check(m);
  }
  inline static constexpr char errorMessage[] = "Matrix contains a NaN.";
  using ExceptionT = std::runtime_error;
};

typedef CheckedItem<Matrix3, false, false, true, true, CheckNaN> CheckedMatrix3;
typedef CheckedItem<Matrix6, false, false, true, true, CheckNaN> CheckedMatrix6;
typedef CheckedItem<Matrix12, false, false, true, true, CheckNaN> CheckedMatrix12;
typedef CheckedItem<Vector3, false, false, true, true, CheckNaN> CheckedVector3;
typedef CheckedItem<Vector6, false, false, true, true, CheckNaN> CheckedVector6;
typedef CheckedItem<Quaternion, false, false, true, true> CheckedQuaternion;

/**
 * \class    IndexedMatrixT
 * \brief    This class describes a structure composed by a matrix
 *           of a given size and a time-index parameter. It can tell also if
 *           it is initialized or not.
 *
 *
 */
template<typename MatrixType = Matrix, bool lazy = false>
class IndexedMatrixT : protected DebugItem<bool, detail::defaultTrue, !lazy || isDebug>
{
  typedef DebugItem<bool, detail::defaultTrue, !lazy || isDebug> IsSet;

public:
  /// Default constructor
  IndexedMatrixT();

  /// A constructor with a given matrix value and a time index
  IndexedMatrixT(const MatrixType & v, TimeIndex k);

  /// Set the value of the matrix and the time sample
  inline IndexedMatrixT & set(const MatrixType & v, TimeIndex k);

  /// Switch the vector to "initialized" state
  inline void set(bool value = true);

  /// set the index of the matrix
  inline void setIndex(TimeIndex index);

  /// Get the matrix value (const version)
  inline const MatrixType & operator()() const;

  /// Get the matrix value
  inline MatrixType & operator()();

  /// Get the time index
  inline TimeIndex getTime() const;

  /// Says whether the matrix is initialized or not
  inline bool isSet() const;

  /// Switch off the initalization flag, the value is no longer accessible
  inline void reset();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:
  /// Checks whether the matrix is set or not (assert)
  /// does nothing in release mode
  inline bool check_() const;
  TimeIndex k_;
  MatrixType v_;
};

typedef IndexedMatrixT<Matrix> IndexedMatrix;
typedef IndexedMatrixT<Vector> IndexedVector;
typedef IndexedMatrixT<Vector3> IndexedVector3;
typedef IndexedMatrixT<Matrix3> IndexedMatrix3;

/**
 * \class    IndexedObjectArrayT
 * \brief    This class describes a structure that enables to store array of objects
 *           with time indexation.
 *
 */

class InputBase
{
protected:
  InputBase(){};

public:
  virtual ~InputBase() = default;
};

template<typename InputType = Vector>
class InputT : public InputBase
{
public:
  ~InputT() {}
  InputType v;
};

class VectorInput : public Vector, public InputBase
{
  using Vector::Vector;
};

class IndexedObjectArrayBase
{
protected:
  /// Default constructor
  IndexedObjectArrayBase() : k_(0) {}

  /// @brief Construct a new IndexedObjectArrayT with a predifined size
  ///
  /// @param size is the size of the array
  /// @param initTime is the index of the initial time. It is zero by default
  IndexedObjectArrayBase(TimeIndex initTime) : k_(initTime) {}

  virtual ~IndexedObjectArrayBase() {}

  /// removes all the elements with larger indexes than timeIndex
  virtual void truncateAfter(TimeIndex timeIndex) = 0;

  /// removes all the elements with smaller indexes than timeIndex
  virtual void truncateBefore(TimeIndex timeIndex) = 0;

  /// Get the time index
  virtual TimeIndex getLastIndex() const = 0;

  /// Get the time index of the next value that will be pushed back
  /// Can be used in for loops
  virtual TimeIndex getNextIndex() const = 0;

  /// Set the time index of the last element
  virtual TimeIndex setLastIndex(int index) = 0;

  /// Get the time index
  virtual TimeIndex getFirstIndex() const = 0;

  /// set the time index of the first element
  virtual TimeIndex setFirstIndex(int index) = 0;

  virtual TimeSize size() const = 0;

  /// Resets the array to initial state
  /// the value is no longer accessible
  virtual void reset() = 0;

  /// Clears the vector but keeps the last index
  virtual void clear() = 0;

  /// checks whether the index is present in the array
  virtual bool checkIndex(TimeIndex k) const = 0;

  /// Asserts that the index is present in the array
  /// does nothing in release mode
  virtual void check_(TimeIndex time) const = 0;

  /// Asserts that the array is not empty
  /// does nothing in release mode
  virtual void check_() const = 0;

  /// Asserts that the given time can be pushed at the back of the vector
  /// does nothing in release mode
  virtual void checkNext_(TimeIndex time) const = 0;

  TimeIndex k_;
};

template<typename ObjectType, typename Allocator>
class IndexedObjectArrayT : public IndexedObjectArrayBase
{
public:
  /// Default constructor
  IndexedObjectArrayT();

  /// @brief Construct a new IndexedObjectArrayT with a predifined size
  ///
  /// @param size is the size of the array
  /// @param initTime is the index of the initial time. It is zero by default
  IndexedObjectArrayT(TimeSize size, TimeIndex initTime = 0);

  void truncateAfter(TimeIndex timeIndex) override;

  /// removes all the elements with smaller indexes than timeIndex
  void truncateBefore(TimeIndex timeIndex) override;

  /// Get the time index
  inline TimeIndex getLastIndex() const override;

  /// Get the time index of the next value that will be pushed back
  /// Can be used in for loops
  inline TimeIndex getNextIndex() const override;

  /// Set the time index of the last element
  inline TimeIndex setLastIndex(int index) override;

  /// Get the time index
  inline TimeIndex getFirstIndex() const override;

  /// set the time index of the first element
  inline TimeIndex setFirstIndex(int index) override;

  inline TimeSize size() const override;

  /// Resets the array to initial state
  /// the value is no longer accessible
  inline void reset() override;

  /// Clears the vector but keeps the last index
  inline void clear() override;

  /// checks whether the index is present in the array
  inline bool checkIndex(TimeIndex k) const override;

  /// Sets the value v at the time index k
  /// It can be used to push a value into the back of the array
  inline void setValue(const ObjectType & value, TimeIndex k);

  /// Pushes back the object to the array, the new value will take the next time
  inline void pushBack(const ObjectType & v);

  /// removes the first (oldest) element of the array
  inline void popFront();

  /// gets a constant reference to the given time index
  inline const ObjectType & operator[](TimeIndex time) const;

  /// gets the value with the given time index, non const version
  inline ObjectType & operator[](TimeIndex index);

  /// gets the first value
  inline const ObjectType & front() const;

  /// gets the first value
  inline ObjectType & front();

  /// gets the last value
  inline const ObjectType & back() const;

  /// gets the last value
  inline ObjectType & back();

protected:
  /// Asserts that the index is present in the array
  /// does nothing in release mode
  inline void check_(TimeIndex time) const override;

  /// Asserts that the array is not empty
  /// does nothing in release mode
  inline void check_() const override;

  /// Asserts that the given time can be pushed at the back of the vector
  /// does nothing in release mode
  inline void checkNext_(TimeIndex time) const override;

  typedef std::deque<ObjectType> Deque;
  Deque v_;
};

/// @brief Class that DECLARES all the neccessary functions for the handling of an array of index inputs.
/// @details Functions are not defined here since they will be in \ref IndexInputArrayT, which inherits from this class.
/// Allows to call functions on the array for any type of input, while avoiding to template all the code. Here, only
/// \ref IndexInputArrayT is templated within estimators, and not the estimators themselves.
class IndexedInputArrayInterface : public IndexedObjectArrayBase
{
public:
  /// Default constructor
  IndexedInputArrayInterface() : IndexedObjectArrayBase() {}

  /// @brief Construct a new IndexedInputArrayInterface with a predifined size
  ///
  /// @param size is the size of the array
  /// @param initTime is the index of the initial time. It is zero by default
  IndexedInputArrayInterface(TimeIndex initTime) : IndexedObjectArrayBase(initTime) {}

  virtual void truncateAfter(TimeIndex timeIndex) override = 0;

  /// removes all the elements with smaller indexes than timeIndex
  virtual void truncateBefore(TimeIndex timeIndex) override = 0;

  /// Get the time index
  virtual TimeIndex getLastIndex() const override = 0;

  /// Get the time index of the next value that will be pushed back
  /// Can be used in for loops
  virtual TimeIndex getNextIndex() const override = 0;

  /// Set the time index of the last element
  virtual TimeIndex setLastIndex(int index) override = 0;

  /// Get the time index
  virtual TimeIndex getFirstIndex() const override = 0;

  /// set the time index of the first element
  virtual TimeIndex setFirstIndex(int index) override = 0;

  virtual TimeSize size() const override = 0;

  /// Resets the array to initial state
  /// the value is no longer accessible
  virtual void reset() override = 0;

  /// Clears the vector but keeps the last index
  virtual void clear() override = 0;

  /// checks whether the index is present in the array
  virtual bool checkIndex(TimeIndex k) const override = 0;

  /// Sets the value v at the time index k
  /// It can be used to push a value into the back of the array
  virtual void setValue(const InputBase & value, TimeIndex k) = 0;

  /// Pushes back the object to the array, the new value will take the next time
  virtual void pushBack(const InputBase & v) = 0;

  /// removes the first (oldest) element of the array
  virtual void popFront() = 0;

  /// gets a constant reference to the given time index
  virtual const InputBase & operator[](TimeIndex time) const = 0;

  /// gets the value with the given time index, non const version
  virtual InputBase & operator[](TimeIndex index) = 0;

  /// gets a pointer to the input at time index. Unsafe since this is not a shared pointer, use with care.
  // virtual const InputBase * getInputPtrUnsafe(TimeIndex index) const = 0;

  /// gets the first value
  virtual const InputBase & front() const = 0;

  /// gets the first value
  virtual InputBase & front() = 0;

  /// gets the last value
  virtual const InputBase & back() const = 0;

  /// gets the last value
  virtual InputBase & back() = 0;

protected:
  /// Asserts that the index is present in the array
  /// does nothing in release mode
  void check_(TimeIndex time) const override = 0;

  /// Asserts that the array is not empty
  /// does nothing in release mode
  void check_() const override = 0;

  /// Asserts that the given time can be pushed at the back of the vector
  /// does nothing in release mode
  void checkNext_(TimeIndex time) const override = 0;
};

template<typename InputType = InputT<>, typename Allocator = std::allocator<InputT<>>>
class IndexedInputArrayT : public IndexedInputArrayInterface
{
public:
  /// Default constructor
  IndexedInputArrayT();

  /// @brief Construct a new IndexedInputArrayT with a predifined size
  ///
  /// @param size is the size of the array
  /// @param initTime is the index of the initial time. It is zero by default
  IndexedInputArrayT(TimeIndex initTime);

  void truncateAfter(TimeIndex timeIndex) override;

  /// removes all the elements with smaller indexes than timeIndex
  void truncateBefore(TimeIndex timeIndex) override;

  /// Get the time index
  inline TimeIndex getLastIndex() const override;

  /// Get the time index of the next value that will be pushed back
  /// Can be used in for loops
  inline TimeIndex getNextIndex() const override;

  /// Set the time index of the last element
  inline TimeIndex setLastIndex(int index) override;

  /// Get the time index
  inline TimeIndex getFirstIndex() const override;

  /// set the time index of the first element
  inline TimeIndex setFirstIndex(int index) override;

  inline TimeSize size() const override;

  /// Resets the array to initial state
  /// the value is no longer accessible
  inline void reset() override;

  /// Clears the vector but keeps the last index
  inline void clear() override;

  /// checks whether the index is present in the array
  inline bool checkIndex(TimeIndex k) const override;

  /// Sets the value v at the time index k
  /// It can be used to push a value into the back of the array
  inline void setValue(const InputBase & value, TimeIndex k) override;

  /// Pushes back the object to the array, the new value will take the next time
  inline void pushBack(const InputBase & v) override;

  /// removes the first (oldest) element of the array
  inline void popFront() override;

  /// gets a constant reference to the given time index
  inline const InputType & operator[](TimeIndex time) const override;

  /// gets the value with the given time index, non const version
  inline InputType & operator[](TimeIndex index) override;

  // virtual const InputBase * getInputPtrUnsafe(TimeIndex index) const override;

  /// gets the first value
  inline const InputType & front() const override;

  /// gets the first value
  inline InputType & front() override;

  /// gets the last value
  inline const InputType & back() const override;

  /// gets the last value
  inline InputType & back() override;

protected:
  /// Asserts that the index is present in the array
  /// does nothing in release mode
  inline void check_(TimeIndex time) const override;

  /// Asserts that the array is not empty
  /// does nothing in release mode
  inline void check_() const override;

  /// Asserts that the given time can be pushed at the back of the vector
  /// does nothing in release mode
  inline void checkNext_(TimeIndex time) const override;

protected:
  typedef std::deque<InputType> Deque;
  Deque v_;
};

typedef IndexedInputArrayT<VectorInput> IndexedInputVectorArray;

template<typename MatrixType = Matrix, typename Allocator = std::allocator<MatrixType>>
class IndexedMatrixArrayT : public IndexedObjectArrayT<MatrixType, Allocator>
{
public:
  /// Default constructor
  IndexedMatrixArrayT();

  /// @brief Construct a new Indexed Vector Array T with a predifined size
  ///
  /// @param size is the size of the array
  /// @param initTime is the index of the initial time. It is zero by default
  IndexedMatrixArrayT(TimeSize size, TimeIndex initTime = 0);

  typedef std::vector<MatrixType, Allocator> Array;

  /// resizes the array
  inline void resize(TimeSize i, const MatrixType & m = MatrixType());

  /// converts the array into a standard vector
  Array getArray() const;

  /// gets the array from a file
  /// the line starts with the time index and then the matrix is read
  /// row by row
  /// WARNING: this resets the array
  void readFromFile(const char * filename, Index rows, Index cols = 1, bool withTimeStamp = true);
  void readFromFile(const std::string & filename, Index rows, Index cols = 1, bool withTimeStamp = true);

  /// gets the array from a file
  /// the line starts with the time index and then every line of the file
  /// is converted into a vector
  /// WARNING: this resets the array
  void readVectorsFromFile(const std::string & filename, bool withTimeStamp = true);
  void readVectorsFromFile(const char * filename, bool withTimeStamp = true);

  /// write the array in a a file
  /// the line starts with the time index and then the matrix is described
  /// row by row
  /// When clear is set, the array is cleared but the time index is conserved
  /// When append is set to true, the output is appended to file
  void writeInFile(const char * filename, bool clear = false, bool append = false);

  /// write the array in a a file
  /// the line starts with the time index and then the matrix is described
  /// row by row
  /// When clear is set, the array is cleared but the time index is conserved
  /// When append is set to true, the output is appended to file
  void writeInFile(const std::string & filename, bool clear = false, bool append = false);
};

typedef IndexedMatrixArrayT<Matrix> IndexedMatrixArray;
typedef IndexedMatrixArrayT<Vector> IndexedVectorArray;

namespace cst
{
constexpr double gravityConstant = 9.80665;

/// Gravity Vector along Z
const Vector gravity = gravityConstant * Vector3::UnitZ();

/// angles considered Zero
constexpr double epsilonAngle = 1e-16;

/// number considered zero when compared to 1
constexpr double epsilon1 = std::numeric_limits<double>::epsilon();

} // namespace cst

/// @brief checks if two scalars have approximately the same value up to a given relative precision
///
/// @param a the first scalar
/// @param b the second scalar
/// @param relativePrecision the relative precision (no need to multiply by the scales of a and b)
/// @return true they are equal
/// @return false they are not
inline bool isApprox(double a, double b, double relativePrecision = cst::epsilon1);

/// @brief checks if two scalars have approximately the same value up to a given absolute precision
///
/// @param a the first scalar
/// @param b the second scalar
/// @param absolutePrecision the absoilute precision
/// @return true they are equal
/// @return false they are not
inline bool isApproxAbs(double a, double b, double absolutePrecision = cst::epsilon1);

template<typename InputType>
inline InputType & convert_input(InputBase & u);

template<typename InputType>
inline const InputType & convert_input(const InputBase & u);

typedef boost::timer::auto_cpu_timer auto_cpu_timer;
typedef boost::timer::cpu_timer cpu_timer;
typedef boost::timer::cpu_timer cpu_times;

namespace tools
{
struct STATE_OBSERVATION_DLLAPI SimplestStopwatch
{
  /** Always pick a steady clock */
  using clock = typename std::conditional<std::chrono::high_resolution_clock::is_steady,
                                          std::chrono::high_resolution_clock,
                                          std::chrono::steady_clock>::type;
  using time_ns = clock::time_point;
  time_ns startTime;

  inline void start()
  {
    startTime = clock::now();
  }

  /// provides the time since the start
  /// the value is in nanoseconds
  inline double stop()
  {
    auto elapsed = clock::now() - startTime;
    return static_cast<double>(elapsed.count());
  }
};

std::string STATE_OBSERVATION_DLLAPI matrixToString(const Matrix & mat);

std::string STATE_OBSERVATION_DLLAPI vectorToString(const Vector & v);

Matrix STATE_OBSERVATION_DLLAPI stringToMatrix(const std::string & str, Index rows, Index cols);

Vector STATE_OBSERVATION_DLLAPI stringToVector(const std::string & str, Index length);

Vector STATE_OBSERVATION_DLLAPI stringToVector(const std::string & str);
} // namespace tools

} // namespace stateObservation
#include <state-observation/tools/definitions.hxx>

#endif // STATEOBSERVATIONDEFINITIONSHPP
