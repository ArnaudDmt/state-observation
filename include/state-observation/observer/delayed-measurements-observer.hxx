namespace stateObservation
{
template<typename DataType>
inline bool AsynchronousDataMapT<DataType>::checkIndex(TimeIndex k)
{
  return v_.count(k) != 0;
}

template<typename DataType>
inline void AsynchronousDataMapT<DataType>::pushValue(const AsynchronousDataBase & v, TimeIndex k)
{
  if(checkIndex(k))
  {
    v_.at(k).merge(static_cast<const DataType &>(v));
  }
  else
  {
    v_.insert({k, static_cast<const DataType &>(v)});
  }
}

template<typename DataType>
inline void AsynchronousDataMapT<DataType>::clear()
{
  v_.clear();
}

template<typename DataType>
inline void AsynchronousDataMapT<DataType>::erase(TimeIndex k)
{
  BOOST_ASSERT(!v_.empty() && "The asynchronous data vector is empty.");
  BOOST_ASSERT(checkIndex(k) && "The element to erase does not exist.");
  v_.erase(k);
}

template<typename DataType>
inline bool AsynchronousDataMapT<DataType>::empty() const
{
  return v_.empty();
}

template<typename DataType>
inline AsynchronousDataBase & AsynchronousDataMapT<DataType>::getElement(TimeIndex k)
{
  BOOST_ASSERT(!v_.empty() && "The asynchronous data vector is empty.");
  BOOST_ASSERT(checkIndex(k) && "The element does not exist in the asynchronous data map.");
  return v_.at(k);
}

template<typename DataType>
inline TimeIndex AsynchronousDataMapT<DataType>::getFirstIndex()
{
  BOOST_ASSERT(!v_.empty() && "The asynchronous data vector is empty.");
  return v_.begin()->first;
}

template<typename DataType>
inline DataType & convert_async_data(AsynchronousDataBase & u)
{
  return static_cast<DataType &>(u);
}

template<typename DataType>
inline const DataType & convert_async_data(const AsynchronousDataBase & u)
{
  return static_cast<const DataType &>(u);
}

} // namespace stateObservation