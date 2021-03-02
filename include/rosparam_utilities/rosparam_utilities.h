#ifndef ROSPARAM_UTILITIES__ROSPARAM_UTILITIES_H
#define ROSPARAM_UTILITIES__ROSPARAM_UTILITIES_H

#include <ros/ros.h>
#include <XmlRpc.h>
#include <boost/array.hpp>
#include <bitset>
#include <typeinfo>
#include <typeindex>
#include <Eigen/Core>

namespace rosparam_utilities
{

//! utils
template<typename T>
std::string to_string(const std::vector<T>& vv);

//=== NODEHANDLE =======================================================================================================
template<typename T>
bool getParam(const ros::NodeHandle& nh, const std::string& key, T& ret, std::string& what, const T* default_val=nullptr);

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

template<class T>
bool setParam(ros::NodeHandle& nh, const std::string& key, const std::vector< std::vector<T> >& mtx);

bool setParam( ros::NodeHandle& nh,  const std::string& key, const std::vector<Eigen::VectorXd>& vector );

template<class T>
inline bool setParamNum(ros::NodeHandle& nh, const std::string& key, const std::vector< std::vector<T> >& mtx, unsigned int precision = 0);
//======================================================================================================================


//! The function simply check if all the needed keys are in the rosparam server
bool check(const XmlRpc::XmlRpcValue& node, const std::vector<std::string>& required_keys);

//======================================================================================================================
//== EXCEPTION IF FAIL =================================================================================================
//======================================================================================================================
//! Overload DOUBLE
void fromXmlRpcValue(const XmlRpc::XmlRpcValue& node, double& val);

//! Overload INT
void fromXmlRpcValue(const XmlRpc::XmlRpcValue& node, int& val);

//! Overload UNSIGNED INT
void fromXmlRpcValue(const XmlRpc::XmlRpcValue& node, unsigned int& val);

//! Overload LONG UNSIGNED INT
void fromXmlRpcValue(const XmlRpc::XmlRpcValue& node, long unsigned int& val);

//! Overload BOOL
void fromXmlRpcValue(const XmlRpc::XmlRpcValue& node, bool& val);

//! Overload STRING
void fromXmlRpcValue(const XmlRpc::XmlRpcValue& node, std::string& val);

//! Overload to specialize on vector
template<typename T>
void fromXmlRpcValue(const XmlRpc::XmlRpcValue& node, std::vector<T>& val);

//! Overload to specialize on matrixes
template<typename T>
void fromXmlRpcValue(const XmlRpc::XmlRpcValue& node, std::vector<std::vector<T>>& val);

//! Overload to specialize on eigen vector/matrices
template<typename Derived>
void fromXmlRpcValue(const XmlRpc::XmlRpcValue& node, Eigen::MatrixBase<Derived> const & val);

//! Overload to specialize on vector
template<typename T, size_t n>
void fromXmlRpcValue(const XmlRpc::XmlRpcValue& node, std::array<T, n>& val);

template<typename T, size_t n>
void fromXmlRpcValue(const XmlRpc::XmlRpcValue& node, boost::array<T, n>& val);

//! Adds string for error tracking
template<typename T>
T fromXmlRpcValue(const XmlRpc::XmlRpcValue& node, const std::string& key="", const std::string& log="");

//! "Specialization" of template<typename T> T fromXmlRpcValue(...) for double
double toDouble(const XmlRpc::XmlRpcValue& node, const std::string& key="", const std::string& log="");

//! "Specialization" of template<typename T> T fromXmlRpcValue(...) for Int
int toInt(const XmlRpc::XmlRpcValue& node, const std::string& key="", const std::string& log="");

//! "Specialization" of template<typename T> T fromXmlRpcValue(...) for bool
bool toBool(const XmlRpc::XmlRpcValue& node, const std::string& key="", const std::string& log="");

//! "Specialization" of template<typename T> T fromXmlRpcValue(...) for string
std::string toString(const XmlRpc::XmlRpcValue& node, const std::string& key="", const std::string& log="");


//template<class T>
//void extractParam(const XmlRpc::XmlRpcValue& node, const std::string& key, T& ret);

//======================================================================================================================



//======================================================================================================================
//== TRUE/FALSE RETURN =================================================================================================
//======================================================================================================================
template<class T>
bool getParam(const XmlRpc::XmlRpcValue& node, const std::string& key, T& ret, const std::string& log_key="");

template<class T>
bool getParam(const XmlRpc::XmlRpcValue& node, T& ret, const std::string& log_key="");


//======================================================================================================================
//== DEPRECATED ========================================================================================================
//======================================================================================================================
template<class T>
[[deprecated("Use the getParam")]]
bool getParamVector(const XmlRpc::XmlRpcValue& node, std::vector<T>& ret, const std::string& log_key="");

template<class T>
[[deprecated("Use the getParam")]]
bool getParamVector(const XmlRpc::XmlRpcValue& node, const std::string& key, std::vector<T>& ret );

template<class T, size_t n>
[[deprecated("Use the getParam")]]
bool getParamArray(const XmlRpc::XmlRpcValue& node, boost::array<T, n>& ret, const std::string& log_key="");

template<class T, size_t n>
[[deprecated("Use the getParam")]]
bool getParamArray(const XmlRpc::XmlRpcValue& node, const std::string& key, boost::array<T, n>& ret );

template<class T>
[[deprecated("Use the getParam")]]
bool getParamMatrix(const XmlRpc::XmlRpcValue& node, std::vector< std::vector<T> >& ret, const std::string& log_key="");

template<class T>
[[deprecated("Use the getParam")]]
bool getParamMatrix(const XmlRpc::XmlRpcValue& node, const std::string& key, std::vector<T>& ret );


void toXmlRpcValue (const double& t, XmlRpc::XmlRpcValue& xml_value);
void toXmlRpcValue (const int& t, XmlRpc::XmlRpcValue& xml_value, const std::string& format ="dec" );
void toXmlRpcValue (const std::string& t, XmlRpc::XmlRpcValue& xml_value);
void toXmlRpcValue (const bool& t, XmlRpc::XmlRpcValue& xml_value);


}

#include <rosparam_utilities/internal/rosparam_utilities_impl.h>

#endif
