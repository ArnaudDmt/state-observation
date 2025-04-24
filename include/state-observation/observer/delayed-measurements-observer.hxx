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
  v_.erase(k);
}

template<typename DataType>
inline bool AsynchronousDataMapT<DataType>::empty() const
{
  v_.empty();
}

template<typename DataType>
inline AsynchronousDataBase & AsynchronousDataMapT<DataType>::getElement(TimeIndex k)
{
  return v_.at(k);
}

template<typename DataType>
inline TimeIndex AsynchronousDataMapT<DataType>::getFirstIndex()
{
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