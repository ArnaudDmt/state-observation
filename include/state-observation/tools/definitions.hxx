namespace stateObservation
{

template<typename T, bool lazy, bool alwaysCheck, bool assertion, bool eigenAlignedNew, typename additionalChecker>
inline CheckedItem<T, lazy, alwaysCheck, assertion, eigenAlignedNew, additionalChecker>::CheckedItem(bool initialize)
{
  if(initialize) isSet_ = false;
}

template<typename T, bool lazy, bool alwaysCheck, bool assertion, bool eigenAlignedNew, typename additionalChecker>
inline CheckedItem<T, lazy, alwaysCheck, assertion, eigenAlignedNew, additionalChecker>::CheckedItem(
    const CheckedItem & c)
: v_(c.v_)
{
  if(do_check_)
  {
    isSet_ = c.isSet_;
  }

  if(do_assert_)
  {
    assertMsg_ = c.assertMsg_;
  }

  if(do_exception_)
  {
    exceptionPtr_ = c.exceptionPtr_;
  }
}

template<typename T, bool lazy, bool alwaysCheck, bool assertion, bool eigenAlignedNew, typename additionalChecker>
inline CheckedItem<T, lazy, alwaysCheck, assertion, eigenAlignedNew, additionalChecker>::CheckedItem(const T & v)
: isSet_(true), v_(v)
{
}

template<typename T, bool lazy, bool alwaysCheck, bool assertion, bool eigenAlignedNew, typename additionalChecker>
inline T & CheckedItem<T, lazy, alwaysCheck, assertion, eigenAlignedNew, additionalChecker>::operator=(const T & v)
{
  isSet_.set(true);
  return v_ = v;
}

template<typename T, bool lazy, bool alwaysCheck, bool assertion, bool eigenAlignedNew, typename additionalChecker>
inline CheckedItem<T, lazy, alwaysCheck, assertion, eigenAlignedNew, additionalChecker> &
    CheckedItem<T, lazy, alwaysCheck, assertion, eigenAlignedNew, additionalChecker>::operator=(const CheckedItem & c)
{
  v_ = c.v_;

  if(do_check_)
  {
    isSet_ = c.isSet_;
  }

  if(do_assert_)
  {
    assertMsg_ = c.assertMsg_;
  }

  if(do_exception_)
  {
    exceptionPtr_ = c.exceptionPtr_;
  }

  return *this;
}

template<typename T, bool lazy, bool alwaysCheck, bool assertion, bool eigenAlignedNew, typename additionalChecker>
inline CheckedItem<T, lazy, alwaysCheck, assertion, eigenAlignedNew, additionalChecker>::operator T() const
{
  return (*this)();
}

template<typename T, bool lazy, bool alwaysCheck, bool assertion, bool eigenAlignedNew, typename additionalChecker>
inline CheckedItem<T, lazy, alwaysCheck, assertion, eigenAlignedNew, additionalChecker>::operator const T &() const
{
  return (*this)();
}

template<typename T, bool lazy, bool alwaysCheck, bool assertion, bool eigenAlignedNew, typename additionalChecker>
inline const T & CheckedItem<T, lazy, alwaysCheck, assertion, eigenAlignedNew, additionalChecker>::chckitm_getValue()
    const
{
  return (*this)();
}

template<typename T, bool lazy, bool alwaysCheck, bool assertion, bool eigenAlignedNew, typename additionalChecker>
inline const T & CheckedItem<T, lazy, alwaysCheck, assertion, eigenAlignedNew, additionalChecker>::operator()() const
{
  chckitm_check_();
  return v_;
}

template<typename T, bool lazy, bool alwaysCheck, bool assertion, bool eigenAlignedNew, typename additionalChecker>
inline T & CheckedItem<T, lazy, alwaysCheck, assertion, eigenAlignedNew, additionalChecker>::operator()()
{
  chckitm_check_();
  return v_;
}

template<typename T, bool lazy, bool alwaysCheck, bool assertion, bool eigenAlignedNew, typename additionalChecker>
inline const T & CheckedItem<T, lazy, alwaysCheck, assertion, eigenAlignedNew, additionalChecker>::getRefUnchecked()
    const
{
  return v_;
}

template<typename T, bool lazy, bool alwaysCheck, bool assertion, bool eigenAlignedNew, typename additionalChecker>
inline T & CheckedItem<T, lazy, alwaysCheck, assertion, eigenAlignedNew, additionalChecker>::getRefUnchecked()
{
  return v_;
}

template<typename T, bool lazy, bool alwaysCheck, bool assertion, bool eigenAlignedNew, typename additionalChecker>
inline bool CheckedItem<T, lazy, alwaysCheck, assertion, eigenAlignedNew, additionalChecker>::chckitm_check_() const
{
  if(assertion)
  {
    BOOST_ASSERT_MSG(isSet(), assertMsg_.get());
    additionalChecker::checkAssert(v_);
  }

  if(alwaysCheck || isDebug)
  {
    if(!isSet())
    {
      throw(*(exceptionPtr_.get()));
    }
    if(!additionalChecker::check(v_))
    {
      using ExceptionT = typename additionalChecker::ExceptionT;
      throw ExceptionT(additionalChecker::errorMessage);
    }
  }
  return (isSet());
}

template<typename T, bool lazy, bool alwaysCheck, bool assertion, bool eigenAlignedNew, typename additionalChecker>
inline bool CheckedItem<T, lazy, alwaysCheck, assertion, eigenAlignedNew, additionalChecker>::isSet() const
{
  return isSet_.get();
}

template<typename T, bool lazy, bool alwaysCheck, bool assertion, bool eigenAlignedNew, typename additionalChecker>
inline void CheckedItem<T, lazy, alwaysCheck, assertion, eigenAlignedNew, additionalChecker>::reset()
{
  isSet_.set(false);
}

template<typename T, bool lazy, bool alwaysCheck, bool assertion, bool eigenAlignedNew, typename additionalChecker>
inline T & CheckedItem<T, lazy, alwaysCheck, assertion, eigenAlignedNew, additionalChecker>::set()
{
  isSet_.set(true);
  return v_;
}

template<typename T, bool lazy, bool alwaysCheck, bool assertion, bool eigenAlignedNew, typename additionalChecker>
inline void CheckedItem<T, lazy, alwaysCheck, assertion, eigenAlignedNew, additionalChecker>::set(bool value)
{
  isSet_.set(value);
}

template<typename T, bool lazy, bool alwaysCheck, bool assertion, bool eigenAlignedNew, typename additionalChecker>
inline void CheckedItem<T, lazy, alwaysCheck, assertion, eigenAlignedNew, additionalChecker>::setAssertMessage(
    std::string s)
{
  assertMsg_.set(s);
}

template<typename T, bool lazy, bool alwaysCheck, bool assertion, bool eigenAlignedNew, typename additionalChecker>
inline void CheckedItem<T, lazy, alwaysCheck, assertion, eigenAlignedNew, additionalChecker>::setExceptionPtr(
    std::exception * e)
{
  exceptionPtr_.set(e);
}

template<typename MatrixType, bool lazy>
inline IndexedMatrixT<MatrixType, lazy>::IndexedMatrixT(const MatrixType & v, TimeIndex k) : IsSet(true), k_(k), v_(v)
{
}

template<typename MatrixType, bool lazy>
inline IndexedMatrixT<MatrixType, lazy>::IndexedMatrixT() : IsSet(false), k_(0)
{
}

/// Says whether the matrix is initialized or not
template<typename MatrixType, bool lazy>
inline bool IndexedMatrixT<MatrixType, lazy>::isSet() const
{
  return (IsSet::get());
}

/// Set the value of the matrix and the time sample
template<typename MatrixType, bool lazy>
inline IndexedMatrixT<MatrixType, lazy> & IndexedMatrixT<MatrixType, lazy>::set(const MatrixType & v, TimeIndex k)
{
  IsSet::set(true);
  k_ = k;
  v_ = v;
  return *this;
}

/// Switch the matrix to set
template<typename MatrixType, bool lazy>
inline void IndexedMatrixT<MatrixType, lazy>::set(bool b)
{
  IsSet::set(b);
}

/// Set the value of the matrix and the time sample
template<typename MatrixType, bool lazy>
inline void IndexedMatrixT<MatrixType, lazy>::reset()
{
  IsSet::set(false);
}

/// Checks whether the matrix is set or not (assert)
/// does nothing in release mode
template<typename MatrixType, bool lazy>
inline bool IndexedMatrixT<MatrixType, lazy>::check_() const
{
  BOOST_ASSERT(isSet() && "Error: Matrix not initialized, if you are initializing it, \
                            use set() function.");
  if(isSet())
  {
    BOOST_ASSERT(!v_.hasNaN() && "Error: Matrix has NaN.");
    return !v_.hasNaN(); // returns false if v has a NaN : test failed
  }
  else
  {
    return isSet();
  }
}

template<typename MatrixType, bool lazy>
inline void IndexedMatrixT<MatrixType, lazy>::setIndex(TimeIndex k)
{
  check_();
  k_ = k;
}

/// Get the matrix value
template<typename MatrixType, bool lazy>
inline const MatrixType & IndexedMatrixT<MatrixType, lazy>::operator()() const
{
  check_();
  return v_;
}

/// Get the matrix value
template<typename MatrixType, bool lazy>
inline MatrixType & IndexedMatrixT<MatrixType, lazy>::operator()()
{
  check_();
  return v_;
}

/// Get the time index
template<typename MatrixType, bool lazy>
TimeIndex IndexedMatrixT<MatrixType, lazy>::getTime() const
{
  check_();
  return k_;
}

/// resizes the array
template<typename MatrixType, typename Allocator>
inline void IndexedMatrixArrayT<MatrixType, Allocator>::resize(TimeSize i, const MatrixType & m)
{
  this->v_.resize(size_t(i), m);
}

/// Default constructor
template<typename MatrixType, typename Allocator>
IndexedMatrixArrayT<MatrixType, Allocator>::IndexedMatrixArrayT() : IndexedObjectArrayT<MatrixType, Allocator>()
{
}

/// size based constructor
template<typename MatrixType, typename Allocator>
IndexedMatrixArrayT<MatrixType, Allocator>::IndexedMatrixArrayT(TimeSize size, TimeIndex initTime)
: IndexedObjectArrayT<MatrixType, Allocator>(size, initTime)
{
}

template<typename MatrixType, typename Allocator>
typename IndexedMatrixArrayT<MatrixType, Allocator>::Array IndexedMatrixArrayT<MatrixType, Allocator>::getArray() const
{
  Array v;

  for(TimeSize i = 0; i < this->v_.size(); ++i)
  {
    this->v.push_back(this->v_[i]);
  }

  return v;
}

template<typename MatrixType, typename Allocator>
void IndexedMatrixArrayT<MatrixType, Allocator>::readFromFile(const std::string & filename,
                                                              Index rows,
                                                              Index cols,
                                                              bool withTimeStamp)
{
  readFromFile(filename.c_str(), rows, cols, withTimeStamp);
}

template<typename MatrixType, typename Allocator>
void IndexedMatrixArrayT<MatrixType, Allocator>::readFromFile(const char * filename,
                                                              Index rows,
                                                              Index cols,
                                                              bool withTimeStamp)
{
  this->reset();

  std::ifstream f;

  f.open(filename);

  if(f.is_open())
  {

    Matrix m(Matrix::Zero(rows, cols));

    bool continuation = true;
    int k = 0;

    while(continuation)
    {

      if(withTimeStamp)
      {
        f >> k;
      }

      if(f.fail())
        continuation = false;
      else
      {
        for(Index i = 0; i < rows; ++i)
        {
          for(Index j = 0; j < cols; ++j)
          {
            f >> m(i, j);
          }
        }

        this->setValue(m, k);
        ++k;
      }
    }
  }
}

template<typename MatrixType, typename Allocator>
void IndexedMatrixArrayT<MatrixType, Allocator>::readVectorsFromFile(const std::string & filename, bool withTimeStamp)
{
  readVectorsFromFile(filename.c_str(), withTimeStamp);
}

template<typename MatrixType, typename Allocator>
void IndexedMatrixArrayT<MatrixType, Allocator>::readVectorsFromFile(const char * filename, bool withTimeStamp)
{
  this->reset();

  std::ifstream f;

  f.open(filename);

  if(f.is_open())
  {
    std::string s;
    Vector v;
    int k = 0;

    bool continuation = true;

    while(continuation)
    {
      std::getline(f, s);

      std::stringstream ss(s);

      if(withTimeStamp)
      {
        ss >> k;
      }

      if(f.fail())
        continuation = false;
      else
      {
        std::vector<double> doublecontainer;
        double component;
        bool readingVector = true;
        while(readingVector)
        {
          ss >> component;

          if(ss.fail())
          {
            readingVector = false;
          }
          else
          {
            doublecontainer.push_back(component);
          }
        }
        v.resize(Index(doublecontainer.size()));
        for(Index i = 0; i < Index(doublecontainer.size()); ++i)
        {
          v(i) = doublecontainer[size_t(i)];
        }
        this->setValue(v, k);
        ++k;
      }
    }
  }
}

template<typename MatrixType, typename Allocator>
void IndexedMatrixArrayT<MatrixType, Allocator>::writeInFile(const std::string & filename, bool clearLog, bool append)
{
  writeInFile(filename.c_str(), clearLog, append);
}

template<typename MatrixType, typename Allocator>
void IndexedMatrixArrayT<MatrixType, Allocator>::writeInFile(const char * filename, bool clearLog, bool append)
{
  std::ofstream f;
  if(!append)
  {
    f.open(filename);
  }
  else
  {
    f.open(filename, std::ofstream::app);
  }

  if(f.is_open())
  {
    if(this->size() > 0)
    {

      for(TimeIndex k = this->getFirstIndex(); k < this->getNextIndex(); ++k)
      {

        f << k;

        MatrixType & m = this->operator[](k);

        for(int i = 0; i < m.rows(); ++i)
        {
          for(int j = 0; j < m.cols(); ++j)
          {
            f << " " << m(i, j);
          }
        }
        f << std::endl;
      }
    }

    if(clearLog)
    {
      this->clear();
    }
  }
  else
  {
    std::stringstream ss;
    ss << "Logger: File " << filename << " could not be created/opened.";
    std::runtime_error e(ss.str().c_str());
    throw e;
  }
}

template<typename ObjectType, typename Allocator>
inline TimeIndex IndexedObjectArrayT<ObjectType, Allocator>::getLastIndex() const
{
  return k_ + TimeIndex(v_.size()) - 1;
}

template<typename ObjectType, typename Allocator>
inline TimeIndex IndexedObjectArrayT<ObjectType, Allocator>::getNextIndex() const
{
  return k_ + TimeIndex(v_.size());
}

template<typename ObjectType, typename Allocator>
inline TimeIndex IndexedObjectArrayT<ObjectType, Allocator>::getFirstIndex() const
{
  return k_;
}

template<typename ObjectType, typename Allocator>
inline TimeIndex IndexedObjectArrayT<ObjectType, Allocator>::setLastIndex(int index)
{
  return k_ = index - (v_.size() + 1);
}

template<typename ObjectType, typename Allocator>
inline TimeIndex IndexedObjectArrayT<ObjectType, Allocator>::setFirstIndex(int index)
{
  return k_ = index;
}

template<typename ObjectType, typename Allocator>
inline TimeSize IndexedObjectArrayT<ObjectType, Allocator>::size() const
{
  return TimeSize(v_.size());
}

/// Switch off the initialization flag, the value is no longer accessible
template<typename ObjectType, typename Allocator>
inline void IndexedObjectArrayT<ObjectType, Allocator>::reset()
{
  k_ = 0;
  v_.clear();
}

template<typename ObjectType, typename Allocator>
inline void IndexedObjectArrayT<ObjectType, Allocator>::clear()
{
  k_ = k_ + TimeIndex(v_.size());
  v_.clear();
}

template<typename ObjectType, typename Allocator>
inline bool IndexedObjectArrayT<ObjectType, Allocator>::checkIndex(TimeIndex time) const
{
  return (v_.size() > 0 && k_ <= time && k_ + TimeIndex(v_.size()) > time);
}

template<typename ObjectType, typename Allocator>
inline void IndexedObjectArrayT<ObjectType, Allocator>::check_(TimeIndex time) const
{
  (void)time; // avoid warning in release mode
  BOOST_ASSERT(checkIndex(time) && "Error: Time out of range");
}

template<typename ObjectType, typename Allocator>
inline void IndexedObjectArrayT<ObjectType, Allocator>::check_() const
{
  BOOST_ASSERT(v_.size() && "Error: Matrix array is empty");
}

template<typename ObjectType, typename Allocator>
inline void IndexedObjectArrayT<ObjectType, Allocator>::checkNext_(TimeIndex time) const
{
  (void)time; // avoid warning
  BOOST_ASSERT((v_.size() == 0 || k_ + TimeIndex(v_.size()) == time)
               && "Error: New time instants must be consecutive to existing ones");
}

template<typename ObjectType, typename Allocator>
inline void IndexedObjectArrayT<ObjectType, Allocator>::truncateAfter(TimeIndex time)
{
  if(v_.size() > 0)
  {
    if(time >= getFirstIndex())
    {
      for(TimeIndex i = getLastIndex(); i > time; --i)
      {
        v_.pop_back();
      }
    }
    else
    {
      v_.clear();
    }
  }
}

template<typename ObjectType, typename Allocator>
inline void IndexedObjectArrayT<ObjectType, Allocator>::truncateBefore(TimeIndex time)
{
  if(v_.size() > 0)
  {
    if(time < getLastIndex())
    {
      for(TimeIndex i = getFirstIndex(); i < time; ++i)
      {
        v_.pop_front();
      }

      setFirstIndex(time);
    }
    else
    {
      v_.clear();
    }
  }
}

template<typename ObjectType, typename Allocator>
IndexedObjectArrayT<ObjectType, Allocator>::IndexedObjectArrayT() : IndexedObjectArrayBase(), v_()
{
}

template<typename ObjectType, typename Allocator>
IndexedObjectArrayT<ObjectType, Allocator>::IndexedObjectArrayT(TimeSize size, TimeIndex initial)
: v_(size), IndexedObjectArrayBase(initial)
{
}

template<typename ObjectType, typename Allocator>
inline void IndexedObjectArrayT<ObjectType, Allocator>::setValue(const ObjectType & v, TimeIndex k)
{
  if(checkIndex(k))
  {
    (*this)[k] = v;
  }
  else
  {
    checkNext_(k);
    if(v_.size() == 0) k_ = k;

    v_.emplace_back(v);
  }
}

template<typename ObjectType, typename Allocator>
inline void IndexedObjectArrayT<ObjectType, Allocator>::pushBack(const ObjectType & v)
{
  v_.push_back(v);
}

template<typename ObjectType, typename Allocator>
inline void IndexedObjectArrayT<ObjectType, Allocator>::popFront()
{
  check_();
  v_.pop_front();
  ++k_;
}

/// Get the matrix value
template<typename ObjectType, typename Allocator>
inline ObjectType & IndexedObjectArrayT<ObjectType, Allocator>::operator[](TimeIndex time)
{
  check_(time);
  return v_[size_t(time - k_)];
}

template<typename ObjectType, typename Allocator>
inline const ObjectType & IndexedObjectArrayT<ObjectType, Allocator>::operator[](TimeIndex time) const
{
  check_(time);
  return v_[size_t(time - k_)];
}

template<typename ObjectType, typename Allocator>
inline const ObjectType & IndexedObjectArrayT<ObjectType, Allocator>::front() const
{
  return v_.front();
}

template<typename ObjectType, typename Allocator>
inline ObjectType & IndexedObjectArrayT<ObjectType, Allocator>::front()
{
  return v_.front();
}

template<typename ObjectType, typename Allocator>
inline const ObjectType & IndexedObjectArrayT<ObjectType, Allocator>::back() const
{
  return v_.back();
}

template<typename ObjectType, typename Allocator>
inline ObjectType & IndexedObjectArrayT<ObjectType, Allocator>::back()
{
  return v_.back();
}

template<typename InputType, typename Allocator>
IndexedInputArrayT<InputType, Allocator>::IndexedInputArrayT() : IndexedInputArrayInterface()
{
}

template<typename InputType, typename Allocator>
IndexedInputArrayT<InputType, Allocator>::IndexedInputArrayT(TimeIndex initial) : IndexedInputArrayInterface(initial)
{
}

template<typename InputType, typename Allocator>
inline TimeIndex IndexedInputArrayT<InputType, Allocator>::getLastIndex() const
{
  return k_ + TimeIndex(v_.size()) - 1;
}

template<typename InputType, typename Allocator>
inline TimeIndex IndexedInputArrayT<InputType, Allocator>::getNextIndex() const
{
  return k_ + TimeIndex(v_.size());
}

template<typename InputType, typename Allocator>
inline TimeIndex IndexedInputArrayT<InputType, Allocator>::getFirstIndex() const
{
  return k_;
}

template<typename InputType, typename Allocator>
inline TimeIndex IndexedInputArrayT<InputType, Allocator>::setLastIndex(int index)
{
  return k_ = index - (v_.size() + 1);
}

template<typename InputType, typename Allocator>
inline TimeIndex IndexedInputArrayT<InputType, Allocator>::setFirstIndex(int index)
{
  return k_ = index;
}

template<typename InputType, typename Allocator>
inline TimeSize IndexedInputArrayT<InputType, Allocator>::size() const
{
  return TimeSize(v_.size());
}

/// Switch off the initialization flag, the value is no longer accessible
template<typename InputType, typename Allocator>
inline void IndexedInputArrayT<InputType, Allocator>::reset()
{
  k_ = 0;
  v_.clear();
}

template<typename InputType, typename Allocator>
inline void IndexedInputArrayT<InputType, Allocator>::clear()
{
  k_ = k_ + TimeIndex(v_.size());
  v_.clear();
}

template<typename InputType, typename Allocator>
inline bool IndexedInputArrayT<InputType, Allocator>::checkIndex(TimeIndex time) const
{
  return (v_.size() > 0 && k_ <= time && k_ + TimeIndex(v_.size()) > time);
}

template<typename InputType, typename Allocator>
inline void IndexedInputArrayT<InputType, Allocator>::check_(TimeIndex time) const
{
  (void)time; // avoid warning in release mode
  BOOST_ASSERT(checkIndex(time) && "Error: Time out of range");
}

template<typename InputType, typename Allocator>
inline void IndexedInputArrayT<InputType, Allocator>::check_() const
{
  BOOST_ASSERT(v_.size() && "Error: Input array is empty");
}

template<typename InputType, typename Allocator>
inline void IndexedInputArrayT<InputType, Allocator>::checkNext_(TimeIndex time) const
{
  (void)time; // avoid warning
  BOOST_ASSERT((v_.size() == 0 || k_ + TimeIndex(v_.size()) == time)
               && "Error: New time instants must be consecutive to existing ones");
}

template<typename InputType, typename Allocator>
inline void IndexedInputArrayT<InputType, Allocator>::truncateAfter(TimeIndex time)
{
  if(v_.size() > 0)
  {
    if(time >= getFirstIndex())
    {
      for(TimeIndex i = getLastIndex(); i > time; --i)
      {
        v_.pop_back();
      }
    }
    else
    {
      v_.clear();
    }
  }
}

template<typename InputType, typename Allocator>
inline void IndexedInputArrayT<InputType, Allocator>::truncateBefore(TimeIndex time)
{
  if(v_.size() > 0)
  {
    if(time < getLastIndex())
    {
      for(TimeIndex i = getFirstIndex(); i < time; ++i)
      {
        v_.pop_front();
      }

      setFirstIndex(time);
    }
    else
    {
      v_.clear();
    }
  }
}

template<typename InputType, typename Allocator>
inline void IndexedInputArrayT<InputType, Allocator>::setValue(const InputBase & v, TimeIndex k)
{
  if(checkIndex(k))
  {
    (*this)[k] = static_cast<const InputType &>(v);
  }
  else
  {
    checkNext_(k);
    if(v_.size() == 0) k_ = k;

    v_.emplace_back(static_cast<const InputType &>(v));
  }
}

template<typename InputType, typename Allocator>
inline void IndexedInputArrayT<InputType, Allocator>::pushBack(const InputBase & v)
{
  v_.push_back(static_cast<const InputType &>(v));
}

template<typename InputType, typename Allocator>
inline void IndexedInputArrayT<InputType, Allocator>::popFront()
{
  check_();
  v_.pop_front();
  ++k_;
}

template<typename InputType, typename Allocator>
inline const InputType & IndexedInputArrayT<InputType, Allocator>::operator[](TimeIndex time) const
{
  check_(time);
  return static_cast<const InputType &>(v_[size_t(time - k_)]);
}

template<typename InputType, typename Allocator>
inline InputType & IndexedInputArrayT<InputType, Allocator>::operator[](TimeIndex time)
{
  check_(time);
  return static_cast<InputType &>(v_[size_t(time - k_)]);
}

// template<typename InputType, typename Allocator>
// inline const InputBase * IndexedInputArrayT<InputType, Allocator>::getInputPtrUnsafe(TimeIndex time) const
// {
//   check_(time);
//   return &(v_[size_t(time - k_)]);
// }

template<typename InputType, typename Allocator>
inline const InputType & IndexedInputArrayT<InputType, Allocator>::front() const
{
  return v_.front();
}

template<typename InputType, typename Allocator>
inline InputType & IndexedInputArrayT<InputType, Allocator>::front()
{
  return v_.front();
}

template<typename InputType, typename Allocator>
inline const InputType & IndexedInputArrayT<InputType, Allocator>::back() const
{
  return v_.back();
}

template<typename InputType, typename Allocator>
inline InputType & IndexedInputArrayT<InputType, Allocator>::back()
{
  return v_.back();
}

template<typename InputType>
inline InputType & convert_input(InputBase & u)
{
  return static_cast<InputType &>(u);
}

template<typename InputType>
inline const InputType & convert_input(const InputBase & u)
{
  return static_cast<const InputType &>(u);
}

inline bool isApprox(double a, double b, double relativePrecision)
{
  return fabs(a - b) < fabs(a + b) * relativePrecision;
}

inline bool isApproxAbs(double a, double b, double absolutePrecision)
{
  return fabs(a - b) < absolutePrecision;
}

} // namespace stateObservation