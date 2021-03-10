# rosparam_utilities #

[![Build Status][t]][1]
[![codecov][c]][2] 
[![Codacy Badge][y]][3]
[![FOSSA Status][f]][4]

## Aim ##

The package has been designed to speedup the get/set of data from/to the rosparam server.

## Usage ##

### Dependencies and Building ###

The package is based on [eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page).

### Class initialization and usage ###

The main functions are template that supports `double`, `std::string`, `int`, `int32`, `int64`, `uint32`, and 
`std::vector<typename T>`, `std::vector<std::vector<typename T>>`, `Eigen::Matrix<,,>` etc.

```cpp
// 'what' stores warning(s) and error(s). The return is 'true' unless errors are raised. If default_val is different 
// from nullptr, the default value is superimposed in case the param is not found (a warning is stored, and the 
// true is returned)
template<typename T>
bool get(const std::string& key, T& ret, std::string& what, const T* default_val = nullptr);

template<typename T>
bool set(const std::string& key, const T& ret, std::string& what);
```

and, alternatively, the functions are provided with `NodeHandle` and `XmlRpc` 

```cpp
//======================================================================================================================
//== OLD FUNCTIONS BUT STILL ALIVE ======
//======================================================================================================================
template<typename T>
bool getParam(const ros::NodeHandle& nh,
                const std::string& key,
                  T& ret,
                    std::string& what,
                      const T* default_val = nullptr);

template<typename T>
bool setParam(const ros::NodeHandle& nh,
                const std::string& key,
                  const T& ret,
                    std::string& what);

template<class T>
bool getParam(const XmlRpc::XmlRpcValue& node, const std::string& key, T& ret, const std::string& log_key = "");

template<class T>
bool getParam(const XmlRpc::XmlRpcValue& node, T& ret, const std::string& log_key = "");
//======================================================================================================================
```

There are also a bunch of raw methods, that check the type and solve possibile problms when needed

```cpp
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
```

### Contact ###

<mailto:<mailto:nicola.pedrocchi@stiima.cnr.it>>

## License ##
[![FOSSA Status][o]][5]

[t]:https://travis-ci.org/CNR-STIIMA-IRAS/rosparam_utilities.svg?branch=master
[1]:https://travis-ci.org/CNR-STIIMA-IRAS/rosparam_utilities

[c]:https://codecov.io/gh/CNR-STIIMA-IRAS/rosparam_utilities/branch/master/graph/badge.svg?token=VELYXJ2FUJ
[2]:https://codecov.io/gh/CNR-STIIMA-IRAS/rosparam_utilities

[y]:https://api.codacy.com/project/badge/Grade/7f1834c02aa84b959ee9b7529deb48d6
[3]:https://app.codacy.com/gh/CNR-STIIMA-IRAS/rosparam_utilities?utm_source=github.com&utm_medium=referral&utm_content=CNR-STIIMA-IRAS/rosparam_utilities&utm_campaign=Badge_Grade_Dashboard

[f]:https://app.fossa.com/api/projects/git%2Bgithub.com%2FCNR-STIIMA-IRAS%2Frosparam_utilities.svg?type=shield
[4]:https://app.fossa.com/projects/git%2Bgithub.com%2FCNR-STIIMA-IRAS%2Frosparam_utilities?ref=badge_shield

[o]:https://app.fossa.com/api/projects/git%2Bgithub.com%2FCNR-STIIMA-IRAS%2Frosparam_utilities.svg?type=large
[5]:https://app.fossa.com/projects/git%2Bgithub.com%2FCNR-STIIMA-IRAS%2Frosparam_utilities?ref=badge_large
