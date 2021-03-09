/*
 *  Software License Agreement (New BSD License)
 *
 *  Copyright 2020 National Council of Research of Italy (CNR)
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#pragma once  // workaround qtcreator clang-tidy

#ifndef ROSPARAM_UTILITIES_INTERNAL_ROSPARAM_UTILITIES_IMPL_H
#define ROSPARAM_UTILITIES_INTERNAL_ROSPARAM_UTILITIES_IMPL_H

#include <string>
#include <vector>
#include <type_traits>
#include <ros/param.h>
#include <rosparam_utilities/rosparam_utilities.h>

namespace utils
{

/**
 * RESIZE - SAFE FUNCTION CALLED ONLY IF THE MATRIX IS DYNAMICALLY CREATED AT RUNTIME
 */
template<typename Derived,
         	typename std::enable_if<
                            (Eigen::MatrixBase<Derived>::RowsAtCompileTime == Eigen::Dynamic)
                             || (Eigen::MatrixBase<Derived>::ColsAtCompileTime == Eigen::Dynamic)
                        , int>::type = 0 >
inline bool resize(Eigen::MatrixBase<Derived> const & m, int rows, int cols)
{
  Eigen::MatrixBase<Derived>& mat = const_cast< Eigen::MatrixBase<Derived>& >(m);
  if((Eigen::MatrixBase<Derived>::RowsAtCompileTime ==Eigen::Dynamic)
  && (Eigen::MatrixBase<Derived>::ColsAtCompileTime ==Eigen::Dynamic))
  {
    mat.derived().resize(rows,cols);
  }
  else if(Eigen::MatrixBase<Derived>::RowsAtCompileTime ==Eigen::Dynamic)
  {
    mat.derived().resize(rows,Eigen::NoChange);
  }
  else if(Eigen::MatrixBase<Derived>::ColsAtCompileTime ==Eigen::Dynamic)
  {
    mat.derived().resize(Eigen::NoChange, cols);
  }
  return (mat.derived().rows() == rows) && (mat.derived().cols() == cols);
}


template<typename Derived,
           typename std::enable_if< (Eigen::MatrixBase<Derived>::RowsAtCompileTime != Eigen::Dynamic)
                        && (Eigen::MatrixBase<Derived>::ColsAtCompileTime != Eigen::Dynamic)
                        , int>::type = 0 >
inline bool resize(Eigen::MatrixBase<Derived> const & /*m*/, int rows, int cols)
{
  return Eigen::MatrixBase<Derived>::RowsAtCompileTime == rows
      && Eigen::MatrixBase<Derived>::ColsAtCompileTime == cols;
}

inline bool resize(const double& /*m*/, int rows, int cols)
{
  return rows==1 && cols ==1;
}

// Using const & and const_cast to modify value ... weird syntax of eigen
template < typename Derived, typename OtherDerived,
           typename std::enable_if < (Eigen::MatrixBase<Derived>::RowsAtCompileTime == Eigen::Dynamic)
                                     || (Eigen::MatrixBase<Derived>::ColsAtCompileTime == Eigen::Dynamic)
                                     , int >::type = 0 >
inline bool resize(Eigen::MatrixBase<Derived> const & m1, const Eigen::MatrixBase<Derived>& m2)
{
  Eigen::MatrixBase<Derived>& mat = const_cast< Eigen::MatrixBase<Derived>& >(m1);
  if ((Eigen::MatrixBase<Derived>::RowsAtCompileTime == Eigen::Dynamic)
      && (Eigen::MatrixBase<Derived>::ColsAtCompileTime == Eigen::Dynamic))
  {
    mat.derived().resize(m2.rows(), m2.cols());
  }
  else if (Eigen::MatrixBase<Derived>::RowsAtCompileTime == Eigen::Dynamic)
  {
    mat.derived().resize(m2.rows(), Eigen::NoChange);
  }
  else if (Eigen::MatrixBase<Derived>::ColsAtCompileTime == Eigen::Dynamic)
  {
    mat.derived().resize(Eigen::NoChange, m2.cols());
  }
  return (mat.derived().rows() == m2.rows()) && (mat.derived().cols() == m2.cols());
}

template < typename Derived, typename OtherDerived,
           typename std::enable_if < (Eigen::MatrixBase<Derived>::RowsAtCompileTime != Eigen::Dynamic)
                                     && (Eigen::MatrixBase<Derived>::ColsAtCompileTime != Eigen::Dynamic)
                                     , int >::type = 0 >
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

}  // namespace utils

namespace rosparam_utilities
{

template<typename T>
inline std::string to_string(const std::vector<T>& vv)
{
  std::string ret = "[ ";
  for (auto const & v : vv) ret += std::to_string(v) + " ";
  ret += "]";
  return ret;
}

template<>
inline std::string to_string(const std::vector<std::string>& vv)
{
  std::string ret = "[ ";
  for (auto const & v : vv) ret += v + " ";
  ret += "]";
  return ret;
}

template<typename T>
inline std::string to_string(const std::vector<std::vector<T>>& vv)
{
  std::string ret = "[\n";
  for (auto const & v : vv) ret += to_string(v) + "\n";
  ret += "]";
  return ret;
}

// =============================================================================================
template<typename T>
inline bool get(const std::string& key, T& ret, std::string& what, const T* default_val)
{
  what = "";
  if (!ros::param::has(key))
  {
    what = "The param '" + key + "' is not in ros param server.";

    if (default_val)
    {
      if (!utils::resize(ret, *default_val))
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
  if (!ros::param::get(key, config))
  {
    what = "Failed in getting thw XmlRpc struct from parameter '" + key + "', weird ..";
    return false;
  }
  try
  {
    fromXmlRpcValue(config, ret);
  }
  catch (std::exception& e)
  {
    what = "Failed in getting thw XmlRpc struct from parameter '" + key + "':\n";
    what += e.what();
    return false;
  }
  return true;
}

template<typename T>
inline bool set(const std::string& key, const T& val, std::string& what)
{
  XmlRpc::XmlRpcValue config;
  if (ros::param::has(key))
  {
    if (!ros::param::get(key, config))
    {
      what = "Weird error in getting the data from XmlRpc before override the content";
      return false;
    }
  }

  try
  {
    toXmlRpcValue(val, config);
    ros::param::set(key,config);
  }
  catch (std::exception& e)
  {
    what = "Failed in getting thw XmlRpc struct from parameter '" + key + "':\n";
    what += e.what();
    return false;
  }
  return true;
}
// =============================================================================================

template<typename T>
inline bool getParam(const ros::NodeHandle& nh, const std::string& key, T& ret, std::string& what, const T* default_val)
{
  std::string key_ = (key.find("/") == 0) ? key : nh.getNamespace() + "/" + key;
  return get(key_, ret, what, default_val);
}

template<typename T>
bool setParam(const ros::NodeHandle& nh,
                const std::string& key,
                  const T& ret,
                    std::string& what)
{
  std::string key_ = (key.find("/") == 0) ? key : nh.getNamespace() + "/" + key;
  return set(key_, ret, what);
}
// =============================================================================================

template<class T>
inline bool getParam(const XmlRpc::XmlRpcValue& node, const std::string& key,  T& ret, const std::string& log_key)
{
  XmlRpc::XmlRpcValue config(node);
  try
  {
    ret = fromXmlRpcValue<T>(config, key, log_key);
  }
  catch (std::exception& e)
  {
    ROS_ERROR("Error %s", e.what());
    return false;
  }
  return true;
}

template<class T>
inline bool getParam(const XmlRpc::XmlRpcValue& node,  T& ret, const std::string& log_key)
{
  return getParam(node, "", ret, log_key);
}

template<class T>
inline bool setParam(ros::NodeHandle& nh, const std::string& key, const std::vector<std::vector<T>>& mtx)
{
  XmlRpc::XmlRpcValue data_;
  int iRow = 0;
  for (auto itRow = mtx.begin(); itRow != mtx.end(); itRow++)
  {
    int iEl = 0;
    XmlRpc::XmlRpcValue row_;
    for (auto itCol = (*itRow).begin(); itCol != (*itRow).end(); itCol++)
      row_[iEl++] = *itCol;

    data_[iRow++] = (XmlRpc::XmlRpcValue)row_;
  }

  nh.setParam(key, data_);
  return true;
}

template<class T>
inline bool setParamNum(ros::NodeHandle& nh,  const std::string& key, const std::vector<std::vector<T>>& mtx,
                        unsigned int precision)
{
  const std::vector<std::type_index> allowed_type =
    {
      std::type_index(typeid(double)),
      std::type_index(typeid(long double)),
      std::type_index(typeid(float))
    };
  bool ok = false;

  for (auto typ : allowed_type)
    if (std::type_index(typeid(T)) == typ)
      ok = true;

  if (ok)
  {
    XmlRpc::XmlRpcValue data_;
    int iRow = 0;
    for (auto itRow = mtx.begin(); itRow != mtx.end(); itRow++)
    {
      int iEl = 0;
      XmlRpc::XmlRpcValue row_;

      for (auto itCol = (*itRow).begin(); itCol != (*itRow).end(); itCol++)
      {
        if (precision != 0)
          row_[iEl++] = (lround((long double)(*itCol) * pow(10, precision))) / pow(10, precision);
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




// =============================================================================================
template <typename T>
inline T fromXmlRpcValue(const XmlRpc::XmlRpcValue& node, const std::string& key, const std::string& log)
{
  T ret;
  bool ok = true;
  std::string error = "Error. "
                      + std::string(key != "" ? "Key: '"  + key + "'." : "")
                      + std::string(log != "" ? "Log: '"  + log + "'." : "");

  try
  {
    XmlRpc::XmlRpcValue config(node);
    if (key != "")
    {
      if (!config.hasMember(key.c_str()))
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
  catch (XmlRpc::XmlRpcException& e)
  {
    error += e.getMessage();
    ok = false;
  }
  catch (std::exception& e)
  {
    error += e.what();
    ok = false;
  }

  if (!ok)
  {
    throw std::runtime_error(error.c_str());
  }

  return ret;
}
// =============================================================================================

template<typename T>
inline void fromXmlRpcValue(const XmlRpc::XmlRpcValue& node, std::vector<T>& val)
{
  val.clear();
  try
  {
    XmlRpc::XmlRpcValue config(node);
    if (config.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      T element;
      fromXmlRpcValue(node,element);
      val.push_back(element);
    }
    else
    {
      for (int j=0; j<config.size(); ++j)
      {
        T element;
        fromXmlRpcValue(config[j], element);
        val.push_back(element);
      }
    }
  }
  catch(std::exception& e)
  {
    throw std::runtime_error(("Type inconsistency (expected a vector<>): "
                                + std::string(e.what())).c_str());
  }
}

template<typename T>
void toXmlRpcValue(const std::vector<T>& val, XmlRpc::XmlRpcValue& node)
{
  for (auto i=0; i<val.size();++i)
  {
    try
    {
      XmlRpc::XmlRpcValue leaf;
      toXmlRpcValue(val.at(i), leaf);
      node[i]=leaf;
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
inline void fromXmlRpcValue(const XmlRpc::XmlRpcValue& node, boost::array<T, n>& val)
{
  try
  {
    std::vector<T> v;
    fromXmlRpcValue(node, v);
    if (v.size() != n)
    {
      throw std::runtime_error("Dimension Error.");
    }

    for (size_t i = 0; i < n; i++)
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

//! Overload to specialize on vector (SET)
template<typename T, size_t n>
void toXmlRpcValue(const boost::array<T, n>& val, XmlRpc::XmlRpcValue& node)
{
  std::vector<T> v(val.size());
  for (size_t i = 0; i < val.size(); i++)
  {
    v[i] = val[i];
  }
  toXmlRpcValue(v,node);
}

template<typename T,  size_t n>
inline void fromXmlRpcValue(const XmlRpc::XmlRpcValue& node, std::array<T, n>& val)
{
  try
  {
    std::vector<T> v;
    fromXmlRpcValue(node, v);
    if (v.size() != n)
    {
      throw std::runtime_error("Dimension Error.");
    }

    for (size_t i = 0; i < n; i++)
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

//! Overload to specialize on vector (SET)
template<typename T, size_t n>
void toXmlRpcValue(const std::array<T, n>& val, XmlRpc::XmlRpcValue& node)
{
  std::vector<T> v(val.size());
  for (size_t i = 0; i < val.size(); i++)
  {
    v[i] = val[i];
  }
  toXmlRpcValue(v,node);
}

template<typename T>
inline void fromXmlRpcValue(const XmlRpc::XmlRpcValue& node, std::vector<std::vector<T>>& vv)
{
  try
  {
    vv.clear();
    XmlRpc::XmlRpcValue config(node);
    if (config.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      std::vector<T> element;
      fromXmlRpcValue(node,element);
      vv.push_back(element);
    }
    else
    {
      for (int i = 0; i < config.size(); ++i)
      {
        std::vector<T> v;
        fromXmlRpcValue(config[i], v);
        vv.push_back(v);
      }
    }
  }
  catch(std::exception& e)
  {
    throw std::runtime_error(("Type inconsistency (expected a vector<vector<>>): "
                                + std::string(e.what())).c_str());
  }
}

template<typename T>
void toXmlRpcValue(const std::vector<std::vector<T>>& val, XmlRpc::XmlRpcValue& node)
{
  for (auto i=0; i<val.size();++i)
  {
    try
    {
      XmlRpc::XmlRpcValue leaf;
      toXmlRpcValue(val.at(i), leaf);
      node[i]=leaf;
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

  int expected_rows = Eigen::MatrixBase<Derived>::RowsAtCompileTime;
  int expected_cols = Eigen::MatrixBase<Derived>::ColsAtCompileTime;
  bool should_be_a_vector = (expected_rows==1 || expected_cols==1);

  try
  {
    Eigen::MatrixBase<Derived>& _val = const_cast< Eigen::MatrixBase<Derived>& >(val);

    if(should_be_a_vector)
    {
      std::vector<double> vv;
      fromXmlRpcValue(node,vv);
      int dim = static_cast<int>(vv.size());
      if(!utils::resize(_val, (expected_rows==1 ? 1 : dim), (expected_rows==1 ? dim : 1)))
      {
          throw std::runtime_error("It was expected a vector (" +
                                     std::to_string(expected_rows) + "x" + std::to_string(expected_rows) +
                                      ") while the param store a " + std::to_string(dim) +"-vector");
      }
      for(int i=0;i<dim;i++)
          _val(i) = vv.at(static_cast<size_t>(i));
    }
    else // matrix expected
    {
      std::vector<std::vector<double>> vv;
      fromXmlRpcValue(node,vv);
      int rows = vv.size();
      int cols = vv.front().size();
      if(!utils::resize(_val,rows,cols))
      {
          throw std::runtime_error("It was expected a vector (" +
                                     std::to_string(expected_rows) + "x" + std::to_string(expected_rows) +
                                      ") while the param store a (" + std::to_string(rows)+"x"+ std::to_string(cols)+ +") matrix");
      }
      for(int i=0;i<rows;i++)
        for(int j=0;j<cols;j++)
          _val(i,j) = vv.at(static_cast<int>(i)).at(static_cast<int>(j));
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

template<typename Derived>
void toXmlRpcValue(const Eigen::MatrixBase<Derived>& val, XmlRpc::XmlRpcValue& node)
{
  if(val.cols()==1)
  {
    std::vector<double> v(val.rows());
    for(size_t i=0;i<v.size();++i)
    {
      v.at(i) = val(i);
    }
    toXmlRpcValue(v,node);
  }
  else
  {
    std::vector<std::vector<double>> mtx(val.rows(),std::vector<double>(val.cols()));
    for(int i=0;i<val.rows();++i)
    {
      for(int j=0;j<val.cols();++j)
      {
        mtx.at(i).at(j)=val(i,j);
      }
    }
    toXmlRpcValue(mtx,node);
  }
}

template<class T>
inline bool getParamVector(const XmlRpc::XmlRpcValue& node,  std::vector<T>& ret, const std::string& log_key)
{
  XmlRpc::XmlRpcValue config(node);
  if (config.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("The node '%s' is not of type array. %d/%d", log_key.c_str(), static_cast<int>(config.getType()),
                static_cast<int>(XmlRpc::XmlRpcValue::TypeArray));
    return false;
  }

  ret.clear();
  for (int j = 0; j < config.size(); ++j)
  {
    try
    {
      T val;
      if (typeid(T) == typeid(double))
      {
        val = toDouble(config[j]);
      }
      else if (typeid(T) == typeid(std::string))
      {
        if (config[j].getType() == XmlRpc::XmlRpcValue::TypeString)
          fromXmlRpcValue(config[j], val);
        else
          throw std::runtime_error(("Type Error: the node '" + log_key +
                                    "' is not a 'XmlRpc::XmlRpcValue::TypeString' but it is a '%d'!" +
                                    std::to_string(static_cast<int>(config[j].getType()))) .c_str());
      }
      else
        fromXmlRpcValue(config[j], val);

      ret.push_back(val);
    }
    catch (std::exception& e)
    {
      ROS_ERROR_STREAM("Error: " << e.what());
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
    if (!node.hasMember(key))
    {
      ROS_ERROR("The node '%s' is corrupted or type is not '%s'", key.c_str(), typeid(T).name());
      return false;
    }

    XmlRpc::XmlRpcValue config(node);
    return getParamVector(config[key],  ret, key);
  }
  catch (std::exception& e)
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
  if (!getParamVector(node, v,  log_key))
    return false;
  if (v.size() != n)
    return false;

  for (size_t i = 0; i < n; i++)
    ret[i] = v[i];
  return true;
}

template<class T, size_t n>
inline bool getParamArray(const XmlRpc::XmlRpcValue& node,  const std::string& key, boost::array<T, n>& ret)
{
  std::vector<T> v;
  if (!getParamVector(node, key, v))
    return false;
  if (v.size() != n)
    return false;

  for (size_t i = 0; i < n; i++)
    ret[i] = v[i];
  return true;
}

template<class T>
inline bool getParamMatrix(const XmlRpc::XmlRpcValue& node,
                            std::vector<std::vector<T>>& ret,
                              const std::string& log_key)
{
  XmlRpc::XmlRpcValue config(node);
  if (config.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("The node '%s' is not of type array. %d/%d",
                log_key.c_str(), static_cast<int>(config.getType()),  static_cast<int>(XmlRpc::XmlRpcValue::TypeArray));
    return false;
  }

  ret.clear();
  for (auto i = 0; i < config.size(); ++i)
  {
    XmlRpc::XmlRpcValue row = config[i];
    if (row.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("The row[%d] is not an array.", i);
      return false;
    }
    std::vector<T> vct;
    for (int j = 0; j < row.size(); ++j)
    {
      T value;
      try
      {
        XmlRpc::XmlRpcValue rpcval = row[j];
        if (typeid(T) == typeid(double))
        {
          value = toDouble(row[j]);
        }
        else if (typeid(T) == typeid(std::string))
        {
          if (row[j].getType() == XmlRpc::XmlRpcValue::TypeString)
            fromXmlRpcValue(row[j], value);
          else
            throw std::runtime_error(("Type Error: the node '" + log_key +
                                      "' is not a 'XmlRpc::XmlRpcValue::TypeString' but it is a '%d'!" +
                                      std::to_string(static_cast<int>(row[j].getType()))).c_str());
        }
        else
        {
          fromXmlRpcValue(row[j], value);
        }
      }
      catch (std::exception& e)
      {
        ROS_ERROR_STREAM("Error: " << e.what());
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
    if (!node.hasMember(key))
    {
      ROS_ERROR("The node '%s' is corrupted or type is not '%s'", key.c_str(), typeid(T).name());
      return false;
    }

    XmlRpc::XmlRpcValue config(node);
    return getParamMatrix(config[key],  ret, key);
  }
  catch (std::exception& e)
  {
    ROS_ERROR("The node '%s' is corrupted or type is not '%s'", key.c_str(), typeid(T).name());
    return false;
  }
  return true;
}

template<typename T>
inline void extractParam(const ros::NodeHandle& nh, const std::string& key, T& ret)
{
  XmlRpc::XmlRpcValue config;
  if (!nh.getParam(key, config))
  {
    ROS_WARN_STREAM("Parameter '" << nh.getNamespace() << "/" << key << "' not found!");
    throw std::runtime_error("The parameter '" + key + "' has been not found");
  }
  return fromXmlRpcValue(config, ret);
}

template<>
inline void extractParam<XmlRpc::XmlRpcValue>(const ros::NodeHandle& nh,
                                                const std::string& key,
                                                  XmlRpc::XmlRpcValue& ret)
{
  if (!nh.getParam(key, ret))
  {
    ROS_WARN_STREAM("Parameter '" << nh.getNamespace() << "/" << key << "' not found!");
    throw std::runtime_error("The parameter '" + key + "' has been not found");
  }
}

template<class T>
inline bool getParamVector(const ros::NodeHandle& nh, const std::string& key,  std::vector<T>& ret)
{
  XmlRpc::XmlRpcValue config;
  if (!nh.getParam(key, config))
  {
    ROS_WARN_STREAM("Parameter '" << nh.getNamespace() << "/" << key << "' not found!");
    return false;
  }
  return getParamVector<T>(config, ret, key);
}

template<class T, size_t n>
inline bool getParamArray(ros::NodeHandle& nh, const std::string& key,  boost::array<T, n>& ret)
{
  std::vector<T> v;
  if (!getParamVector(nh, key,  v))
    return false;
  if (v.size() != n)
    return false;

  for (size_t i = 0; i < n; i++)
    ret[i] = v[i];
  return true;
}

template<class T>
inline bool getParamMatrix(const ros::NodeHandle& nh,  const std::string& key, std::vector<std::vector<T>>& ret)
{
  XmlRpc::XmlRpcValue config;
  nh.getParam(key, config);
  return getParamMatrix(config, ret, key);
}
}  // namespace rosparam_utilities

#endif  // ROSPARAM_UTILITIES_INTERNAL_ROSPARAM_UTILITIES_IMPL_H
