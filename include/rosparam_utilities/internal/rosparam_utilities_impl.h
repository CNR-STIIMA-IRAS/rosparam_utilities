#pragma once  // workaround qtcreator clang-tidy

#ifndef ROSPARAM_UTILITIES__ROSPARAM_UTILITIES_IMPL_H
#define ROSPARAM_UTILITIES__ROSPARAM_UTILITIES_IMPL_H

#include <ros/ros.h>
#include <XmlRpc.h>
#include <boost/array.hpp>
#include <bitset>
#include <typeinfo>
#include <typeindex>

#include <rosparam_utilities/rosparam_utilities.h>

namespace utils
{

  //Using const & and const_cast to modify value ... weird syntax of eigen
  template<typename Derived, typename OtherDerived,
           std::enable_if_t< (Eigen::MatrixBase<Derived>::RowsAtCompileTime == Eigen::Dynamic)
                          || (Eigen::MatrixBase<Derived>::ColsAtCompileTime == Eigen::Dynamic)
                          , int> = 0>
  inline bool resize(Eigen::MatrixBase<Derived> const & m1, const Eigen::MatrixBase<Derived>& m2)
  {
    Eigen::MatrixBase<Derived>& mat = const_cast< Eigen::MatrixBase<Derived>& >(m1);
    if((Eigen::MatrixBase<Derived>::RowsAtCompileTime == Eigen::Dynamic)
    && (Eigen::MatrixBase<Derived>::ColsAtCompileTime == Eigen::Dynamic))
    {
      mat.derived().resize(m2.rows(),m2.cols());
    }
    else if(Eigen::MatrixBase<Derived>::RowsAtCompileTime == Eigen::Dynamic)
    {
      mat.derived().resize(m2.rows(),Eigen::NoChange);
    }
    else if(Eigen::MatrixBase<Derived>::ColsAtCompileTime ==Eigen::Dynamic)
    {
      mat.derived().resize(Eigen::NoChange, m2.cols());
    }
    return (mat.derived().rows() == m2.rows()) && (mat.derived().cols() == m2.cols());
  }

  template<typename Derived, typename OtherDerived,
           std::enable_if_t< (Eigen::MatrixBase<Derived>::RowsAtCompileTime != Eigen::Dynamic)
                          && (Eigen::MatrixBase<Derived>::ColsAtCompileTime != Eigen::Dynamic)
                          , int> = 0>
  inline bool resize(Eigen::MatrixBase<Derived> const & m1, const Eigen::MatrixBase<Derived>& m2)
  {
    return (m1.rows() == m2.rows()) && (m1.cols() == m2.cols());
  }

  template<typename T>
  inline bool resize(std::vector<T>& m1, const std::vector<T>& m2)
  {
    m1.resize(m2.size());
    return true;
  }

  template<typename T>
  inline bool resize(T& /*m1*/, const T& /*m2*/)
  {
    return true;
  }
}

namespace rosparam_utilities
{

template<typename T>
inline std::string to_string(const std::vector<T>& vv)
{
  std::string ret="[ ";
  for(auto const & v : vv) ret += std::to_string(v) +" ";
  ret +="]";
  return ret;
}

template<>
inline std::string to_string(const std::vector<std::string>& vv)
{
  std::string ret="[ ";
  for(auto const & v : vv) ret += v +" ";
  ret +="]";
  return ret;
}


template<typename T>
inline std::string to_string(const std::vector<std::vector<T>>& vv)
{
  std::string ret="[\n";
  for(auto const & v : vv) ret += to_string(v) +"\n";
  ret +="]";
  return ret;
}


//=============================================================================================
template <typename T>
inline T fromXmlRpcValue(const XmlRpc::XmlRpcValue& node,
                         const std::string& key,
                         const std::string& log)
{
  T ret;
  bool ok = true;
  std::string error = "Error. "
  + std::string(key != "" ? "Key: '"  + key + "'." : "")
  + std::string(log != "" ? "Log: '"  + log + "'." : "");

  try
  {
    XmlRpc::XmlRpcValue config(node);
    if(key != "")
    {
      if(!config.hasMember(key.c_str()))
      {
        throw std::runtime_error(("The param '" + key + "' is not in the rosparam server. ").c_str());
      }
      fromXmlRpcValue(config[key.c_str()], ret);
    }
    else
    {
      fromXmlRpcValue(config, ret);
    }
  }
  catch(XmlRpc::XmlRpcException& e)
  {
    error += e.getMessage();
    ok = false;
  }
  catch(std::exception& e)
  {
    error += e.what();
    ok = false;
  }

  if(!ok)
  {
    throw std::runtime_error(error.c_str());
  }

  return ret;
}
//=============================================================================================

inline void fromXmlRpcValue(const XmlRpc::XmlRpcValue& node, unsigned int& val)
{
  XmlRpc::XmlRpcValue config(node);

  switch(config.getType())
  {
    case XmlRpc::XmlRpcValue::TypeBoolean:
    {
      bool _val = static_cast<bool>(config);
      val = _val;
    }
    break;

    case XmlRpc::XmlRpcValue::TypeInt:
    {
      int    _val = static_cast<int>(config);
      val = _val;
    }
    break;

    case XmlRpc::XmlRpcValue::TypeDouble:
    {
      double _val = static_cast<double>(config);
      val = _val;
    }
    break;

    case XmlRpc::XmlRpcValue::TypeString:
    {
      unsigned int x;
      std::stringstream ss;
      ss <<std::hex << static_cast<std::string>(config);
      ss>> x;
      // output it as a signed type
      std::cout <<static_cast<int>(x) <<std::endl;
      val = x;
    }
    break;

    case XmlRpc::XmlRpcValue::TypeBase64:
    case XmlRpc::XmlRpcValue::TypeInvalid:
    case XmlRpc::XmlRpcValue::TypeDateTime:
    case XmlRpc::XmlRpcValue::TypeArray:
    case XmlRpc::XmlRpcValue::TypeStruct:
      throw std::runtime_error("Type inconsistency");
      break;
  }
}


inline void fromXmlRpcValue(const XmlRpc::XmlRpcValue& node, bool& val)
{
  XmlRpc::XmlRpcValue config(node);

  switch(config.getType())
  {
    case XmlRpc::XmlRpcValue::TypeBoolean:
    {
      bool   _val = static_cast<bool>(config);
      val = _val;
    }
    break;

    case XmlRpc::XmlRpcValue::TypeInt:
    {
      int    _val = static_cast<int>(config);
      val = _val> 0;
    }
    break;

    case XmlRpc::XmlRpcValue::TypeDouble:
    {
      double _val = static_cast<double>(config);
      val = _val> 0;
    }
    break;

    case XmlRpc::XmlRpcValue::TypeBase64:
    case XmlRpc::XmlRpcValue::TypeInvalid:
    case XmlRpc::XmlRpcValue::TypeString:
    case XmlRpc::XmlRpcValue::TypeDateTime:
    case XmlRpc::XmlRpcValue::TypeArray:
    case XmlRpc::XmlRpcValue::TypeStruct:
      throw std::runtime_error("Type inconsistency");
      break;
  }
}


inline void fromXmlRpcValue(const XmlRpc::XmlRpcValue& node, double& val)
{
  XmlRpc::XmlRpcValue config(node);

  switch(config.getType())
  {
    case XmlRpc::XmlRpcValue::TypeBoolean:
    {
      bool   _val = static_cast<bool>(config);
      val = _val;
    }
    break;

    case XmlRpc::XmlRpcValue::TypeInt:
    {
      int    _val = static_cast<int>(config);
      val = _val;
    }
    break;

    case XmlRpc::XmlRpcValue::TypeDouble:
    {
      double _val = static_cast<double>(config);
      val = _val;
    }
    break;

    case XmlRpc::XmlRpcValue::TypeBase64:
    case XmlRpc::XmlRpcValue::TypeInvalid:
    case XmlRpc::XmlRpcValue::TypeString:
    case XmlRpc::XmlRpcValue::TypeDateTime:
    case XmlRpc::XmlRpcValue::TypeArray:
    case XmlRpc::XmlRpcValue::TypeStruct:
      throw std::runtime_error("Type inconsistency");
      break;
  }
}


inline void fromXmlRpcValue(const XmlRpc::XmlRpcValue& node, int& val)
{
  XmlRpc::XmlRpcValue config(node);

  switch(config.getType())
  {
    case XmlRpc::XmlRpcValue::TypeBoolean:
    {
      bool   _val = static_cast<bool>(config);
      val = _val;
    }
    break;

    case XmlRpc::XmlRpcValue::TypeInt:
    {
      int    _val = static_cast<int>(config);
      val = _val;
    }
    break;

    case XmlRpc::XmlRpcValue::TypeDouble:
    {
      double _val = static_cast<double>(config);
      val = _val;
    }
    break;

    case XmlRpc::XmlRpcValue::TypeString:
    {
      std::string s = config;
      unsigned int x;
      std::stringstream ss;
      ss <<std::hex << s;
      ss>> x;
      val = x;
    }
    break;

    case XmlRpc::XmlRpcValue::TypeBase64:
    case XmlRpc::XmlRpcValue::TypeInvalid:
    case XmlRpc::XmlRpcValue::TypeDateTime:
    case XmlRpc::XmlRpcValue::TypeArray:
    case XmlRpc::XmlRpcValue::TypeStruct:
      throw std::runtime_error("Type inconsistency");
      break;
  }
}


inline void fromXmlRpcValue(const XmlRpc::XmlRpcValue& node, long unsigned int& val)
{
  XmlRpc::XmlRpcValue config(node);

  switch(config.getType())
  {
    case XmlRpc::XmlRpcValue::TypeBoolean:
    {
      bool   _val = static_cast<bool>(config);
      val = _val;
    }
    break;

    case XmlRpc::XmlRpcValue::TypeInt:
    {
      int    _val = static_cast<int>(config);
      val = _val;
    }
    break;

    case XmlRpc::XmlRpcValue::TypeDouble:
    {
      double _val = static_cast<double>(config);
      val = _val;
    }
    break;

    case XmlRpc::XmlRpcValue::TypeString:
    {
      unsigned int x;
      std::stringstream ss;
      ss <<std::hex << static_cast<std::string>(config);
      ss>> x;
      // output it as a signed type
      std::cout <<static_cast<int>(x) <<std::endl;
      val = x;
    }
    break;

    case XmlRpc::XmlRpcValue::TypeBase64:
    case XmlRpc::XmlRpcValue::TypeInvalid:
    case XmlRpc::XmlRpcValue::TypeDateTime:
    case XmlRpc::XmlRpcValue::TypeArray:
    case XmlRpc::XmlRpcValue::TypeStruct:
      throw std::runtime_error("Type inconsistency");
      break;
  }
}

inline void fromXmlRpcValue(const XmlRpc::XmlRpcValue& node, std::string& val)
{
  XmlRpc::XmlRpcValue config(node);

  switch(config.getType())
  {
    case XmlRpc::XmlRpcValue::TypeBoolean:
    {
      bool   _val = static_cast<bool>(config);
      val = std::to_string(_val);
    }
    break;

    case XmlRpc::XmlRpcValue::TypeInt:
    {
      int    _val = static_cast<int>(config);
      val = std::to_string(_val);
    }
    break;

    case XmlRpc::XmlRpcValue::TypeDouble:
    {
      double _val = static_cast<double>(config);
      val = std::to_string(_val);
    }
    break;

    case XmlRpc::XmlRpcValue::TypeString:
    {
      std::string _val = static_cast<std::string>(config);
      val = _val ;
      val.erase(std::remove_if(val.begin(), val.end(), isspace), val.end());
    }
    break;

    case XmlRpc::XmlRpcValue::TypeBase64:
    case XmlRpc::XmlRpcValue::TypeInvalid:
    case XmlRpc::XmlRpcValue::TypeDateTime:
    case XmlRpc::XmlRpcValue::TypeArray:
    case XmlRpc::XmlRpcValue::TypeStruct:
      throw std::runtime_error("Type inconsistency");
      break;
  }
}

//!
template<typename T>
inline void fromXmlRpcValue(const XmlRpc::XmlRpcValue& node, std::vector<T>& val)
{
  XmlRpc::XmlRpcValue config(node);
  if(config.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    throw std::runtime_error("Type inconsistency (expected type a vector, param not a vector)");
  }

  val.clear();
  for(int j=0; j<config.size(); ++j)
  {
    try
    {
      T element;
      fromXmlRpcValue(config[j], element);
      val.push_back(element);
    }
    catch (std::exception& e)
    {
      throw std::runtime_error(e.what());
    }
    catch (...)
    {
      throw std::runtime_error("Wrong Format.");
    }
  }
}

template<typename T,  size_t n>
inline void fromXmlRpcValue(const XmlRpc::XmlRpcValue& node, boost::array<T,n>& val)
{
  try
  {
    std::vector<T> v;
    fromXmlRpcValue(node, v);
    if(v.size() != n)
    {
      throw std::runtime_error("Dimension Error.");
    }

    for(size_t i=0; i<n; i++)
    {
      val[i] = v[i];
    }
  }
  catch (std::exception& e)
  {
    throw std::runtime_error(e.what());
  }
  catch (...)
  {
    throw std::runtime_error("Wrong Format.");
  }
}

template<typename T,  size_t n>
inline void fromXmlRpcValue(const XmlRpc::XmlRpcValue& node, std::array<T,n>& val)
{
  try
  {
    std::vector<T> v;
    fromXmlRpcValue(node, v);
    if(v.size() != n)
    {
      throw std::runtime_error("Dimension Error.");
    }

    for(size_t i=0; i<n; i++)
    {
      val[i] = v[i];
    }
  }
  catch (std::exception& e)
  {
    throw std::runtime_error(e.what());
  }
  catch (...)
  {
    throw std::runtime_error("Wrong Format.");
  }
}

//!
template<typename T>
inline void fromXmlRpcValue(const XmlRpc::XmlRpcValue& node, std::vector<std::vector<T>>& vv)
{
  XmlRpc::XmlRpcValue config(node);
  if(config.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    throw std::runtime_error("Type inconsistency (expected type a vector, param not a vector)");
  }

  vv.clear();
  for(int i=0; i<config.size(); ++i)
  {
    try
    {
      std::vector<T> v;
      fromXmlRpcValue(config[i], v);
      vv.push_back(v);
    }
    catch (std::exception& e)
    {
      throw std::runtime_error(e.what());
    }
    catch (...)
    {
      throw std::runtime_error("Wrong Format.");
    }
  }
}


template<typename Derived>
inline void fromXmlRpcValue(const XmlRpc::XmlRpcValue& node, Eigen::MatrixBase<Derived> const & val)
{
  XmlRpc::XmlRpcValue config(node);
  if(config.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    throw std::runtime_error("Type inconsistency (expected type a vector, while the param is not a vector)");
  }

  try
  {
    Eigen::MatrixBase<Derived>& _val = const_cast< Eigen::MatrixBase<Derived>& >(val);
    if(Eigen::MatrixBase<Derived>::ColsAtCompileTime == 1)
    {
      std::vector<double> v;
      fromXmlRpcValue(config, v);
      std::cout << rosparam_utilities::to_string(v) << std::endl;
      if(Eigen::MatrixBase<Derived>::RowsAtCompileTime == -1)
      {
        _val.derived().resize(v.size(),1);
      }
      else
      {
        if(Eigen::MatrixBase<Derived>::RowsAtCompileTime != v.size())
        {
          throw std::runtime_error("Vector in param has a size differen from the expected fixed-size vector.");
        }
      }
      for(size_t i=0;i<v.size();i++)
      {
        _val(i) = v.at(i);
      }
    }
    else
    {
      std::vector< std::vector<double> > v;
      fromXmlRpcValue(config, v);
      if((Eigen::MatrixBase<Derived>::RowsAtCompileTime == -1)
      && (Eigen::MatrixBase<Derived>::ColsAtCompileTime == -1) )
      {
        _val.derived().resize(v.size(), v.front().size());
      }
      else if(Eigen::MatrixBase<Derived>::RowsAtCompileTime == -1)
      {
        if(Eigen::MatrixBase<Derived>::ColsAtCompileTime != v.front().size())
        {
          throw std::runtime_error("Matrix in param has a colum-size different from the expected fixed-colums-size matrix.");
        }
      }
      else if(Eigen::MatrixBase<Derived>::ColsAtCompileTime == -1)
      {
        if(Eigen::MatrixBase<Derived>::RowsAtCompileTime != v.size())
        {
          throw std::runtime_error("Matrix in param has a row-size different from the expected fixed-rows-size matrix.");
        }
      }
      else
      {
        if((Eigen::MatrixBase<Derived>::RowsAtCompileTime != v.size())
        || (Eigen::MatrixBase<Derived>::ColsAtCompileTime != v.front().size()))
        {
          throw std::runtime_error("Matrix in param has a different from the expected fixed-size matrix.");
        }
      }

      for(size_t i=0;i<v.size();i++)
      {
        if(v.at(i).size() != v.front().size())
        {
          throw std::runtime_error("Not all the matrix-rows have the same size");
        }
        for(size_t j=0;j<v.at(i).size();j++)
        {
          _val(i,j) = v.at(i).at(j);
        }
      }
    }
  }
  catch (std::exception& e)
  {
    throw std::runtime_error(e.what());
  }
  catch (...)
  {
    throw std::runtime_error("Wrong Format.");
  }


}



















template<class T>
inline bool getParam(const XmlRpc::XmlRpcValue& node, const std::string& key,  T& ret, const std::string& log_key)
{
  XmlRpc::XmlRpcValue config(node);
  try
  {
    ret = fromXmlRpcValue<T>(config, key, log_key);
  }
  catch(std::exception& e)
  {
    ROS_ERROR("Error %s", e.what());
    return false;
  }
  return true;
}

template<class T>
inline bool getParam(const XmlRpc::XmlRpcValue& node,  T& ret, const std::string& log_key)
{
  return getParam(node, "", ret, log_key) ;
}


//template<class T>
//inline void extractParam(const XmlRpc::XmlRpcValue& node, const std::string& key,  T& ret)
//{
//  XmlRpc::XmlRpcValue config(node);
//  try
//  {
//    ret = fromXmlRpcValue<T>(config, key);
//  }
//  catch(std::exception& e)
//  {
//    ROS_ERROR("The node '%s' is corrupted. %s", key.c_str(), e.what());
//    throw std::runtime_error("The node '" + key + "' is corrupted. "+ std::string(e.what()));
//  }
//}


template<class T>
inline bool getParamVector(const XmlRpc::XmlRpcValue& node,  std::vector<T>& ret, const std::string& log_key)
{
  XmlRpc::XmlRpcValue config(node);
  if(config.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("The node '%s' is not of type array. %d/%d", log_key.c_str(), int(config.getType()), int(XmlRpc::XmlRpcValue::TypeArray));
    return false;
  }

  ret.clear();
  for(int j=0; j<config.size(); ++j)
  {
    try
    {
      T val;
      if(typeid(T) == typeid(double))
      {
        val = toDouble(config[j]);
      }
      else if(typeid(T) == typeid(std::string))
      {
        if(config[j].getType() == XmlRpc::XmlRpcValue::TypeString)
          fromXmlRpcValue(config[j], val);
        else
          throw std::runtime_error(("Type Error: the node '" + log_key + "' is not a 'XmlRpc::XmlRpcValue::TypeString' but it is a '%d'!" + std::to_string( int(config[j].getType()))) .c_str());

      }
      else
        fromXmlRpcValue(config[j], val);

      ret.push_back(val);
    }
    catch (std::exception& e)
    {
      ROS_ERROR_STREAM("Error: " <<e.what());
      return false;
    }
    catch (...)
    {
      ROS_ERROR_STREAM("Wrong Format.");
      return false;
    }
  }
  return true;
}

template<class T>
inline bool getParamVector(const XmlRpc::XmlRpcValue& node,  const std::string& key, std::vector<T>& ret)
{
  try
  {
    if(!node.hasMember(key))
    {
      ROS_ERROR("The node '%s' is corrupted or type is not '%s'", key.c_str(), typeid(T).name());
      return false;
    }

    XmlRpc::XmlRpcValue config(node);
    return getParamVector(config[key],  ret, key);
  }
  catch(std::exception& e)
  {
    ROS_ERROR("The node '%s' is corrupted or type is not '%s'", key.c_str(), typeid(T).name());
    return false;
  }
  return true;
}

template<class T, size_t n>
inline bool getParamArray(const XmlRpc::XmlRpcValue& node,  boost::array<T, n>& ret, const std::string& log_key)
{
  std::vector<T> v;
  if(!getParamVector(node, v,  log_key))
    return false;
  if(v.size() != n)
    return false;

  for(size_t i=0; i<n; i++)
    ret[i] = v[i];
  return true;
}

template<class T, size_t n>
inline bool getParamArray(const XmlRpc::XmlRpcValue& node,  const std::string& key, boost::array<T, n>& ret)
{
  std::vector<T> v;
  if(!getParamVector(node, key, v))
    return false;
  if(v.size() != n)
    return false;

  for(size_t i=0; i<n; i++)
    ret[i] = v[i];
  return true;
}

template<class T>
inline bool getParamMatrix(const XmlRpc::XmlRpcValue& node, std::vector<std::vector<T>>& ret, const std::string& log_key)
{
  XmlRpc::XmlRpcValue config(node);
  if(config.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("The node '%s' is not of type array. %d/%d", log_key.c_str(), int(config.getType()), int(XmlRpc::XmlRpcValue::TypeArray));
    return false;
  }

  ret.clear();
  for(auto i=0; i<config.size(); ++i)
  {

    XmlRpc::XmlRpcValue row = config[i];
    if(row.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("The row[%d] is not an array.", i);
      return false;
    }
    std::vector<T> vct;
    for(int j=0; j<row.size(); ++j)
    {
      T value;
      try
      {
        XmlRpc::XmlRpcValue rpcval = row[j];
        if(typeid(T) == typeid(double))
        {
          value = toDouble(row[j]);
        }
        else if(typeid(T) == typeid(std::string))
        {
          if(row[j].getType() == XmlRpc::XmlRpcValue::TypeString)
            fromXmlRpcValue(row[j], value);
          else
            throw std::runtime_error(("Type Error: the node '" + log_key + "' is not a 'XmlRpc::XmlRpcValue::TypeString' but it is a '%d'!" + std::to_string( int(row[j].getType()))).c_str());

        }
        else
        {
          fromXmlRpcValue(row[j], value);
        }
      }
      catch (std::exception& e)
      {
        ROS_ERROR_STREAM("Error: " <<e.what());
        if (typeid(T) == typeid(double))
          ROS_ERROR_STREAM("you have to specify the fractional part even if it is zero");
        return false;
      }
      catch (...)
      {
        if (typeid(T) == typeid(double))
          ROS_ERROR_STREAM("you have to specify the fractional part even if it is zero");
        return false;
      }
      vct.push_back(value);
    }
    ret.push_back(vct);
  }
  return true;
}

template<class T>
inline bool getParamMatrix(const XmlRpc::XmlRpcValue& node,  const std::string& key, std::vector<T>& ret)
{
  try
  {
    if(!node.hasMember(key))
    {
      ROS_ERROR("The node '%s' is corrupted or type is not '%s'", key.c_str(), typeid(T).name());
      return false;
    }

    XmlRpc::XmlRpcValue config(node);
    return getParamMatrix(config[key],  ret, key);
  }
  catch(std::exception& e)
  {
    ROS_ERROR("The node '%s' is corrupted or type is not '%s'", key.c_str(), typeid(T).name());
    return false;
  }
  return true;
}



template<typename T>
inline
bool getParam(const ros::NodeHandle& nh, const std::string& key, T& ret, std::string& what, const T* default_val)
{
  if(!nh.hasParam(key))
  {
    what = "The param '"
         + ( (key.rfind("/") == 0) ? key : nh.getNamespace() + "/" + key)
         + "' is not in ros param server.";

    if(default_val)
    {
      if(!utils::resize(ret, *default_val))
      {
        what += " Mismatch between the default value dimension and the value dimension.";
        return false;
      }
      what += " Default value superimposed";
      ret = *default_val;
      return true;
    }
    else
    {
      return false;
    }
  }

  XmlRpc::XmlRpcValue config;
  if(!nh.getParam(key,config))
  {
    what = "Failed in getting thw XmlRpc struct from parameter '" + nh.getNamespace() +"/" + key + "', weird ..";
    return false;
  }
  try
  {
    fromXmlRpcValue(config,ret);
  }
  catch(std::exception& e)
  {
    what = "Failed in getting thw XmlRpc struct from parameter '" + nh.getNamespace() +"/" + key + "':\n";
    what += e.what();
    return false;
  }

  return true;
}

template<typename T>
inline void extractParam(const ros::NodeHandle& nh, const std::string& key, T& ret)
{
XmlRpc::XmlRpcValue config;
  if(!nh.getParam(key,config))
  {
    ROS_WARN_STREAM("Parameter '" <<nh.getNamespace() <<"/" <<key <<"' not found!");
    throw std::runtime_error("The parameter '" + key + "' has been not found");
  }
  return extractParam(config, "", ret);
}

template<>
inline void extractParam<XmlRpc::XmlRpcValue>(const ros::NodeHandle& nh, const std::string& key, XmlRpc::XmlRpcValue& ret)
{
  if(!nh.getParam(key,ret))
  {
    ROS_WARN_STREAM("Parameter '" <<nh.getNamespace() <<"/" <<key <<"' not found!");
    throw std::runtime_error("The parameter '" + key + "' has been not found");
  }
}

template<class T>
inline bool getParamVector(const ros::NodeHandle& nh, const std::string& key,  std::vector<T>& ret)
{
  XmlRpc::XmlRpcValue config;
  if(!nh.getParam(key,config))
  {
    ROS_WARN_STREAM("Parameter '" <<nh.getNamespace() <<"/" <<key <<"' not found!");
    return false;
  }
  return getParamVector<T>(config,ret, key);
}

template<class T, size_t n>
inline bool getParamArray(ros::NodeHandle& nh, const std::string& key,  boost::array<T, n>& ret)
{
  std::vector<T> v;
  if(!getParamVector(nh, key,  v))
    return false;
  if(v.size() != n)
    return false;

  for(size_t i=0; i<n; i++)
    ret[i] = v[i];
  return true;
}


template<class T>
inline bool getParamMatrix(const ros::NodeHandle& nh,  const std::string& key, std::vector<std::vector<T>>& ret)
{
  XmlRpc::XmlRpcValue config;
  nh.getParam(key,config);
  return getParamMatrix(config,ret, key);
  }

template<class T>
inline bool setParam(ros::NodeHandle& nh, const std::string& key, const std::vector<std::vector<T>>& mtx)
{
  XmlRpc::XmlRpcValue data_;
  int iRow = 0;
  for(auto itRow=mtx.begin(); itRow!=mtx.end(); itRow++)
  {
    int iEl = 0;
    XmlRpc::XmlRpcValue row_;
    for(auto itCol=(*itRow).begin(); itCol!=(*itRow).end(); itCol++)
      row_[iEl++] = *itCol;

    data_[iRow++] = (XmlRpc::XmlRpcValue)row_;
  }

  nh.setParam(key, data_);
  return true;
}

template<class T>
inline bool setParamNum (ros::NodeHandle& nh,  const std::string& key, const std::vector<std::vector<T>>& mtx,
                          unsigned int precision)
{
  const std::vector<std::type_index> allowed_type = { std::type_index(typeid(double)),
                                                        std::type_index(typeid(long double)),
                                                        std::type_index(typeid(float)) };
  bool ok = false;

  for(auto typ : allowed_type)
    if(std::type_index(typeid(T)) == typ)
      ok = true;

  if(ok)
  {
    XmlRpc::XmlRpcValue data_;
    int iRow = 0;
    for(auto itRow=mtx.begin(); itRow!=mtx.end(); itRow++)
    {
      int iEl = 0;
      XmlRpc::XmlRpcValue row_;

      for(auto itCol=(*itRow).begin(); itCol!=(*itRow).end(); itCol++)
      {
        if(precision != 0)
          row_[iEl++] =(lround((long double)(*itCol) * pow(10, precision))) / pow(10, precision);
        else
          row_[iEl++] = *itCol;
      }
      data_[iRow++] = row_;
    }

    nh.setParam(key, data_);
    return true;
  }
  else
  {
    ROS_ERROR("Unable to detect type id in setParamNum");
    return false;
  }
}

}

#endif  // ROSPARAM_UTILITIES__ROSPARAM_UTILITIES_IMPL_H
