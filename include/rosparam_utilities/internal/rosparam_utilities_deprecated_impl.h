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

#ifndef ROSPARAM_UTILITIES_INTERNAL_ROSPARAM_UTILITIES_DEPRECATED_IMPL_H
#define ROSPARAM_UTILITIES_INTERNAL_ROSPARAM_UTILITIES_DEPRECATED_IMPL_H

#include <typeinfo>       // operator typeid
#include <typeindex>
#include <string>
#include <vector>
#include <type_traits>
#include <ros/param.h>
#include <rosparam_utilities/rosparam_utilities.h>

namespace rosparam_utilities
{

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
        fromXmlRpcValue(config[j],val);
        //val = toDouble(config[j]);
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
inline bool getParamVector(const XmlRpc::XmlRpcValue& node, const std::string& key, std::vector<T>& ret)
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
          fromXmlRpcValue(row[j],value);
          //value = toDouble(row[j]);
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
  return getParamVector(config, ret, key);
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

#endif  //  ROSPARAM_UTILITIES_INTERNAL_ROSPARAM_UTILITIES_DEPRECATED_IMPL_H
