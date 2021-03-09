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
#ifndef ROSPARAM_UTILITIES_ROSPARAM_UTILITIES_H
#define ROSPARAM_UTILITIES_ROSPARAM_UTILITIES_H

#include <vector>
#include <string>
#include <boost/array.hpp>
#include <bitset>
#include <typeinfo>
#include <typeindex>
#include <Eigen/Core>

#include <ros/ros.h>
#include <XmlRpc.h>

namespace rosparam_utilities
{

//! utils
template<typename T>
std::string to_string(const std::vector<T>& vv);

//! The function simply check if all the needed keys are in the rosparam server
bool check(const XmlRpc::XmlRpcValue& node, const std::vector<std::string>& required_keys);

XmlRpc::XmlRpcValue get(const ros::NodeHandle& nh);
XmlRpc::XmlRpcValue get(const std::string& key);

//======================================================================================================================
//=== MAIN FUNCTIONS ===================================================================================================
/**
 * @brief get the param, and return true if found and ok. Store the error(s) in 'what'. If a default value is present,
 * it superimposes the defaul values, and it return true, but it stores a warning in 'what'. Typical error is the
 * mismatch of types between the actual parameter and the required type
 *
 * @param[in] key to find (full path)
 * @param[out] ret the value of the element
 * @param[out] what: a message with the error (return false) or a warning (return true)
 * @param[in] default_val: it is superimposed if the value is not in rosparam value. A warning is stored in 'what'
 * @return true if ok, or if default value has been superimposed, false otherwise.
 */
template<typename T>
bool get(const std::string& key, T& ret, std::string& what, const T* default_val = nullptr);

/**
 * @brief set the param, and return true if found and ok. Store the error(s) in 'what'. Typical error is the
 * mismatch of types between the actual parameter and the required type
 *
 * @param[in] key: full path
 * @param[in] val: element to be stored
 * @param[out] what: a message with the error (return false) or a warning (return true)
 * @return true if ok, or if default value has been superimposed, false otherwise.
 */
template<typename T>
bool set(const std::string& key, T& ret, std::string& what);
//======================================================================================================================


//======================================================================================================================
//== OLD FUNCTIONS BUT STILL ALIVE ======
//======================================================================================================================
/**
 * @brief get the param, and return true if found and ok. Store the error(s) in 'what'. If a default value is present,
 * it superimposes the defaul values, and it return true, but it stores a warning in 'what'. Typical error is the
 * mismatch of types between the actual parameter and the required type
 *
 * @param[in] nh node handle
 * @param[in] key to find
 * @param[out] ret the value of the element
 * @param[out] what: a message with the error (return false) or a warning (return true)
 * @param[in] default_val: it is superimposed if the value is not in rosparam value. A warning is stored in what
 * @return true if ok, or if default value has been superimposed, false otherwise.
 */
template<typename T>
bool getParam(const ros::NodeHandle& nh,
                const std::string& key,
                  T& ret,
                    std::string& what,
                      const T* default_val = nullptr);

/**
 * @brief set the param, and return true if found and ok. Store the error(s) in 'what'. Typical error is the
 * mismatch of types between the actual parameter and the required type
 *
 * @param[in] key: full path
 * @param[in] val: element to be stored
 * @param[out] what: a message with the error (return false) or a warning (return true)
 * @return true if ok, or if default value has been superimposed, false otherwise.
 */
template<typename T>
bool setParam(const ros::NodeHandle& nh,
                const std::string& key,
                  const T& ret,
                    std::string& what);

//==
/**
 * @brief get the param, and return true if found and ok. If error, it prints the error using ROS_ERROR/ROS_WARN.
 * Typical error is the mismatch of types between the actual parameter and the required type
 *
 * @param[in] node
 * @param[in] key
 * @param[out] ret the value of the element
 * @param[log_key] default_val: it is superimposed if the value is not in rosparam value. A warning is stored in what
 * @return true if ok, or if default value has been superimposed, false otherwise.
 */
template<class T>
bool getParam(const XmlRpc::XmlRpcValue& node, const std::string& key, T& ret, const std::string& log_key = "");

template<class T>
bool getParam(const XmlRpc::XmlRpcValue& node, T& ret, const std::string& log_key = "");


template<class T>
bool setParam(ros::NodeHandle& nh, const std::string& key, const std::vector< std::vector<T> >& mtx);

bool setParam(ros::NodeHandle& nh,  const std::string& key, const std::vector<Eigen::VectorXd>& vector);

template<class T>
inline bool setParamNum(ros::NodeHandle& nh,
                          const std::string& key,
                            const std::vector< std::vector<T> >& mtx,
                              unsigned int precision = 0);
//======================================================================================================================



//======================================================================================================================
//== EXCEPTION IF FAIL ===== OUTPUT PASSED AS ARGUMENT WITH REFERENCE
//======================================================================================================================
//! Overload DOUBLE (GET)
void fromXmlRpcValue(const XmlRpc::XmlRpcValue& node, double& val);

//! Overload DOUBLE (SET)
void toXmlRpcValue(const double& t, XmlRpc::XmlRpcValue& xml_value);

//! Overload INT (GET)
void fromXmlRpcValue(const XmlRpc::XmlRpcValue& node, int& val);

//! Overload INT (SET)
void toXmlRpcValue(const int& t, XmlRpc::XmlRpcValue& xml_value, const std::string& format = "dec");

//! Overload UNSIGNED INT (GET)
void fromXmlRpcValue(const XmlRpc::XmlRpcValue& node, uint32_t& val);

//! Overload UNSIGNED INT (SET)
void toXmlRpcValue(uint32_t& val, const XmlRpc::XmlRpcValue& node);

//! Overload LONG UNSIGNED INT (GET)
void fromXmlRpcValue(const XmlRpc::XmlRpcValue& node, uint64_t& val);

//! Overload UNSIGNED INT (SET)
void toXmlRpcValue(uint64_t& val, const XmlRpc::XmlRpcValue& node);

//! Overload BOOL (GET)
void fromXmlRpcValue(const XmlRpc::XmlRpcValue& node, bool& val);

//! Overload BOOL (SET)
void toXmlRpcValue(const bool& t, XmlRpc::XmlRpcValue& xml_value);

//! Overload STRING (GET)
void fromXmlRpcValue(const XmlRpc::XmlRpcValue& node, std::string& val);

//! Overload STRING (SET)
void toXmlRpcValue(const std::string& t, XmlRpc::XmlRpcValue& xml_value);

//! Overload to specialize on vector (GET)
template<typename T>
void fromXmlRpcValue(const XmlRpc::XmlRpcValue& node, std::vector<T>& val);

//! Overload to specialize on vector (SET)
template<typename T>
void toXmlRpcValue(const std::vector<T>& val, XmlRpc::XmlRpcValue& node);

//! Overload to specialize on matrixes (GET)
template<typename T>
void fromXmlRpcValue(const XmlRpc::XmlRpcValue& node, std::vector<std::vector<T>>& val);

//! Overload to specialize on matrixes (SET)
template<typename T>
void toXmlRpcValue(const std::vector<std::vector<T>>& val, XmlRpc::XmlRpcValue& node);

//! Overload to specialize on eigen vector/matrices (GET)
template<typename Derived>
void fromXmlRpcValue(const XmlRpc::XmlRpcValue& node, Eigen::MatrixBase<Derived> const & val);

//! Overload to specialize on eigen vector/matrices (SET)
template<typename Derived>
void toXmlRpcValue(const Eigen::MatrixBase<Derived>& val, XmlRpc::XmlRpcValue& node);

//! Overload to specialize on std array (GET)
template<typename T, size_t n>
void fromXmlRpcValue(const XmlRpc::XmlRpcValue& node, std::array<T, n>& val);

//! Overload to specialize on vector (SET)
template<typename T, size_t n>
void toXmlRpcValue(const std::array<T, n>& val, XmlRpc::XmlRpcValue& node);

//! Overload to specialize on boost array (GET)
template<typename T, size_t n>
void fromXmlRpcValue(const XmlRpc::XmlRpcValue& node, boost::array<T, n>& val);

//! Overload to specialize on vector (SET)
template<typename T, size_t n>
void toXmlRpcValue(const boost::array<T, n>& val, XmlRpc::XmlRpcValue& node);
//======================================================================================================================


//======================================================================================================================
//=== RETURN TYPED VALUE EXCEPTION IF FAIL ===
//======================================================================================================================
//! Adds string for error tracking
template<typename T>
T fromXmlRpcValue(const XmlRpc::XmlRpcValue& node, const std::string& key = "", const std::string& log = "");

//! "Specialization" of template<typename T> T fromXmlRpcValue(...) for double
double toDouble(const XmlRpc::XmlRpcValue& node, const std::string& key = "", const std::string& log = "");

//! "Specialization" of template<typename T> T fromXmlRpcValue(...) for Int
int toInt(const XmlRpc::XmlRpcValue& node, const std::string& key = "", const std::string& log = "");

//! "Specialization" of template<typename T> T fromXmlRpcValue(...) for bool
bool toBool(const XmlRpc::XmlRpcValue& node, const std::string& key = "", const std::string& log = "");

//! "Specialization" of template<typename T> T fromXmlRpcValue(...) for string
std::string toString(const XmlRpc::XmlRpcValue& node, const std::string& key = "", const std::string& log = "");
//======================================================================================================================




//======================================================================================================================
//== DEPRECATED ========================================================================================================
//======================================================================================================================
template<class T>
[[deprecated("Use the getParam")]]
bool getParamVector(const XmlRpc::XmlRpcValue& node, std::vector<T>& ret, const std::string& log_key = "");

template<class T>
[[deprecated("Use the getParam")]]
bool getParamVector(const XmlRpc::XmlRpcValue& node, const std::string& key, std::vector<T>& ret);

template<class T, size_t n>
[[deprecated("Use the getParam")]]
bool getParamArray(const XmlRpc::XmlRpcValue& node, boost::array<T, n>& ret, const std::string& log_key = "");

template<class T, size_t n>
[[deprecated("Use the getParam")]]
bool getParamArray(const XmlRpc::XmlRpcValue& node, const std::string& key, boost::array<T, n>& ret);

template<class T>
[[deprecated("Use the getParam")]]
bool getParamMatrix(const XmlRpc::XmlRpcValue& node,
                      std::vector< std::vector<T> >& ret,
                        const std::string& log_key = "");

template<class T>
[[deprecated("Use the getParam")]]
bool getParamMatrix(const XmlRpc::XmlRpcValue& node, const std::string& key, std::vector<T>& ret);

template<typename T>
[[deprecated("Use the getParam(const ros::NodeHandle&, const std::string&, T&, std::string&, const T*)")]]
void extractParam(const ros::NodeHandle& nh, const std::string& key, T& ret);

template<class T>
[[deprecated("Use the getParam(const ros::NodeHandle&, const std::string&, T&, std::string&, const T*)")]]
bool getParamVector(const ros::NodeHandle& nh, const std::string& key, std::vector<T>& ret);

template<class T, size_t n>
[[deprecated("Use the getParam(const ros::NodeHandle&, const std::string&, T&, std::string&, const T*)")]]
bool getParamArray(ros::NodeHandle& nh, const std::string& key, boost::array<T, n>& ret);

template<class T>
[[deprecated("Use the getParam(const ros::NodeHandle&, const std::string&, T&, std::string&, const T*)")]]
bool getParamMatrix(const ros::NodeHandle& nh, const std::string& key, std::vector< std::vector<T> >& ret);

}  // namespace rosparam_utilities

#include <rosparam_utilities/internal/rosparam_utilities_impl.h>

#endif  // ROSPARAM_UTILITIES_ROSPARAM_UTILITIES_H
