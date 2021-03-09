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

#include <cstdlib>
#include <ctime>
#include <iostream>
#include <vector>
#include <string>

#include <gtest/gtest.h>
#include <gtest/gtest-death-test.h>

#include <Eigen/Dense>
#include <ros/ros.h>
#include <rosparam_utilities/rosparam_utilities.h>

namespace ru = rosparam_utilities;

#define COUT std::cout << __LINE__

int int_i = 0;
int int_i2 = 0;
double double_d = 0.0;
double double_d2 = 0.0;
std::string string_s;
std::vector<std::string> vector_string;
std::vector<int> vector_int;
std::vector<double> vector_double;
std::vector<std::vector<int>> matrix_int;
std::vector<std::vector<double>> matrix_double;
std::vector<std::vector<std::string>> matrix_string;

Eigen::VectorXd vectorxd;
Eigen::Matrix<double, 10, 1> vector10;
Eigen::Matrix<double, 4, 1> vector4;

Eigen::MatrixXd matrixxd;
Eigen::Matrix<double, 10, 10> matrix1010;
Eigen::Matrix<double, 2, 2>   matrix22;

// same numbers/data that should be in the yaml/launch
const int d_int_i= 1;
const int d_int_i2= 1.0;
const double d_double_d= 2.0;
const double d_double_d2= 2;
const std::string d_string_s = "ciao";
const std::vector<std::string> d_vector_string{"ciao", "ciao", "ciao"};
const std::vector<int> d_vector_int{1, 2, 3, 4};
const std::vector<double> d_vector_double{1.01, 2.002, 3.0003, 4.00004};
const std::vector<std::vector<int>> d_matrix_int{{11, 12}, {21, 22}};
const std::vector<std::vector<double>> d_matrix_double{{11.0, 12.0},{21.0, 22}};
const std::vector<std::vector<std::string>> d_matrix_string{{"ciao", "bau"},{"aaaa", "1234"}};

Eigen::VectorXd d_vectorxd = Eigen::Map<const Eigen::VectorXd>(&d_vector_double[0],4);
Eigen::Matrix<double, 4, 1> d_vector4 = Eigen::Map<const Eigen::Matrix<double, 4, 1>>(&d_vector_double[0]);

Eigen::MatrixXd d_matrixxd(2,2);
Eigen::Matrix<double, 2, 2> d_matrix22;

template<typename T>
bool equal(const std::vector<T>& lhs, const std::vector<T>& rhs)
{
  if(lhs.size()!=rhs.size())
    return false;
  for(size_t i=0;i<lhs.size();i++)
  {
    if(lhs.at(i)!=rhs.at(i))
      return false;
  }
  return true;
}

template<typename Mat>
bool equal(const Mat& lhs, const Mat& rhs)
{
  return (lhs-rhs).cwiseAbs().maxCoeff()<1e-6;
}

// Declare a test
TEST(TestSuite, rawMethods)
{
  ros::NodeHandle nh("/ns_exist");

  std::vector<std::string> required_fields =
  {
    "int_i",
    "int_i2",
    "double_d",
    "double_d2",
    "string_s",
    "vector_string",
    "vector_int",
    "vector_double",
    "matrix_int",
    "matrix_double",
    "matrix_string"
  };

  d_matrixxd << 11.0, 12.0, 21.0, 22;
  d_matrix22 << 11.0, 12.0, 21.0, 22;

  vector10.setZero();
  vector4.setZero();
  matrix1010.setZero();
  matrix22.setZero();

  XmlRpc::XmlRpcValue node;
  EXPECT_TRUE(nh.getParam("struct", node));

  EXPECT_TRUE(ru::check(node, required_fields));

  EXPECT_NO_FATAL_FAILURE(ru::fromXmlRpcValue(node["int_i"         ], int_i));
  EXPECT_NO_FATAL_FAILURE(ru::fromXmlRpcValue(node["int_i2"        ], int_i2));
  EXPECT_NO_FATAL_FAILURE(ru::fromXmlRpcValue(node["double_d"      ], double_d));
  EXPECT_NO_FATAL_FAILURE(ru::fromXmlRpcValue(node["double_d2"     ], double_d2));
  EXPECT_NO_FATAL_FAILURE(ru::fromXmlRpcValue(node["string_s"      ], string_s));
  EXPECT_NO_FATAL_FAILURE(ru::fromXmlRpcValue(node["vector_string" ], vector_string));
  EXPECT_NO_FATAL_FAILURE(ru::fromXmlRpcValue(node["vector_int"    ], vector_int));
  EXPECT_NO_FATAL_FAILURE(ru::fromXmlRpcValue(node["vector_double" ], vector_double));

  EXPECT_NO_FATAL_FAILURE(ru::fromXmlRpcValue(node["matrix_int"    ], matrix_int));
  EXPECT_NO_FATAL_FAILURE(ru::fromXmlRpcValue(node["matrix_double" ], matrix_double));
  EXPECT_NO_FATAL_FAILURE(ru::fromXmlRpcValue(node["matrix_string" ], matrix_string));

  EXPECT_NO_FATAL_FAILURE(ru::fromXmlRpcValue(node["vector_double" ], vectorxd));
  EXPECT_NO_FATAL_FAILURE(ru::fromXmlRpcValue(node["vector_double" ], vector4));
  EXPECT_ANY_THROW(ru::fromXmlRpcValue(node["vector_double" ], vector10));

  EXPECT_NO_FATAL_FAILURE(ru::fromXmlRpcValue(node["matrix_double" ], matrixxd));
  EXPECT_NO_FATAL_FAILURE(ru::fromXmlRpcValue(node["matrix_double" ], matrix22));
  EXPECT_ANY_THROW(ru::fromXmlRpcValue(node["matrix_double" ], matrix1010));
  EXPECT_ANY_THROW(ru::fromXmlRpcValue(node["matrix_double" ], vectorxd));

  EXPECT_TRUE(d_int_i           == int_i     );
  EXPECT_TRUE(d_int_i2          == int_i2    );
  EXPECT_TRUE(d_double_d        == double_d  );
  EXPECT_TRUE(d_double_d2       == double_d2 );
  EXPECT_TRUE(d_string_s        == string_s  );
  EXPECT_TRUE(equal(d_vector_string   , vector_string   ));
  EXPECT_TRUE(equal(d_vector_int      , vector_int      ));
  EXPECT_TRUE(equal(d_vector_double   , vector_double   ));
  EXPECT_TRUE(equal(d_matrix_int      , matrix_int      ));
  EXPECT_TRUE(equal(d_matrix_double   , matrix_double   ));
  EXPECT_TRUE(equal(d_matrix_string   , matrix_string   ));

  EXPECT_TRUE(equal(d_vectorxd   , vectorxd   ));
  EXPECT_TRUE(equal(d_vector4    , vector4    ));

  EXPECT_TRUE(equal(d_matrixxd   , matrixxd   ));
  EXPECT_TRUE(equal(d_matrix22   , matrix22   ));
}

// Declare a test
TEST(TestSuite, rawSetMethods)
{
  ros::NodeHandle nh("/ns_exist");

  std::vector<std::string> required_fields =
  {
    "int_i",
    "int_i2",
    "double_d",
    "double_d2",
    "string_s",
    "vector_string",
    "vector_int",
    "vector_double",
    "matrix_int",
    "matrix_double",
    "matrix_string"
  };

  XmlRpc::XmlRpcValue node;

  int int_i = 0;
  int int_i2 = 0;
  double double_d = 0.0;
  double double_d2 = 0.0;
  std::string string_s;
  std::vector<std::string> vector_string;
  std::vector<int> vector_int;
  std::vector<double> vector_double;
  std::vector<std::vector<int>> matrix_int;
  std::vector<std::vector<double>> matrix_double;
  std::vector<std::vector<std::string>> matrix_string;

  Eigen::VectorXd vectorxd;
  Eigen::Matrix<double, 10, 1> vector10;
  vector10.setZero();
  Eigen::Matrix<double, 4, 1> vector4;
  vector4.setZero();

  Eigen::MatrixXd matrixxd;
  Eigen::Matrix<double, 10, 10> matrix1010;
  matrix1010.setZero();
  Eigen::Matrix<double, 2, 2>   matrix22;
  matrix22.setZero();

  EXPECT_NO_FATAL_FAILURE(ru::toXmlRpcValue(int_i, node["int_i"]));
  EXPECT_NO_FATAL_FAILURE(ru::toXmlRpcValue(int_i2, node["int_i2"]));
  EXPECT_NO_FATAL_FAILURE(ru::toXmlRpcValue(double_d, node["double_d"]));
  EXPECT_NO_FATAL_FAILURE(ru::toXmlRpcValue(double_d2, node["double_d2"]));
  EXPECT_NO_FATAL_FAILURE(ru::toXmlRpcValue(string_s, node["string_s"]));
  EXPECT_NO_FATAL_FAILURE(ru::toXmlRpcValue(vector_string, node["vector_string" ]));
  EXPECT_NO_FATAL_FAILURE(ru::toXmlRpcValue(vector_int, node["vector_int"]));
  EXPECT_NO_FATAL_FAILURE(ru::toXmlRpcValue(vector_double, node["vector_double" ]));

  EXPECT_NO_FATAL_FAILURE(ru::toXmlRpcValue( matrix_int, node["matrix_int" ]));
  EXPECT_NO_FATAL_FAILURE(ru::toXmlRpcValue(matrix_double, node["matrix_double"]));
  EXPECT_NO_FATAL_FAILURE(ru::toXmlRpcValue(matrix_string, node["matrix_string"]));

  EXPECT_NO_FATAL_FAILURE(ru::toXmlRpcValue(vectorxd, node["vector_double"]));
  EXPECT_NO_FATAL_FAILURE(ru::toXmlRpcValue(vector4,  node["vector_double"]));
  EXPECT_NO_FATAL_FAILURE(ru::toXmlRpcValue(vector10, node["vector_double" ]));

  EXPECT_NO_FATAL_FAILURE(ru::toXmlRpcValue(matrixxd, node["matrix_double" ]));
  EXPECT_NO_FATAL_FAILURE(ru::toXmlRpcValue(matrix22, node["matrix_double" ]));
  EXPECT_NO_FATAL_FAILURE(ru::toXmlRpcValue(matrix1010, node["matrix_double" ]));

  EXPECT_NO_FATAL_FAILURE(ru::toXmlRpcValue(vectorxd, node["matrix_double" ]));

  EXPECT_NO_FATAL_FAILURE(nh.setParam("struct_new", node));
}


// Declare a test
TEST(TestSuite, getParamMethods)
{
  ros::NodeHandle nh("/ns_exist/struct");

  int int_i = 0;
  int int_i2 = 0;
  double double_d = 0.0;
  double double_d2 = 0.0;
  std::string string_s;
  std::vector<std::string> vector_string;
  std::vector<int> vector_int;
  std::vector<double> vector_double;
  std::vector<std::vector<int>> matrix_int;
  std::vector<std::vector<double>> matrix_double;
  std::vector<std::vector<std::string>> matrix_string;

  Eigen::VectorXd vectorxd;
  Eigen::Matrix<double, 10, 1> vector10;
  vector10.setZero();
  Eigen::Matrix<double, 4, 1> vector4;
  vector4.setZero();

  Eigen::MatrixXd matrixxd;
  Eigen::Matrix<double, 10, 10> matrix1010;
  matrix1010.setZero();
  Eigen::Matrix<double, 2, 2>   matrix22;
  matrix22.setZero();

  std::string what;
  EXPECT_TRUE(ru::getParam(nh, "int_i", int_i, what));
  EXPECT_TRUE(ru::getParam(nh, "int_i2", int_i2, what));
  EXPECT_TRUE(ru::getParam(nh, "double_d", double_d, what));
  EXPECT_TRUE(ru::getParam(nh, "double_d2", double_d2, what));
  EXPECT_TRUE(ru::getParam(nh, "string_s", string_s, what));
  EXPECT_TRUE(ru::getParam(nh, "vector_string", vector_string, what));
  EXPECT_TRUE(ru::getParam(nh, "vector_int", vector_int, what));
  EXPECT_TRUE(ru::getParam(nh, "vector_double", vector_double, what));
  EXPECT_TRUE(ru::getParam(nh, "matrix_int", matrix_int, what));
  EXPECT_TRUE(ru::getParam(nh, "matrix_double", matrix_double, what));
  EXPECT_TRUE(ru::getParam(nh, "matrix_string", matrix_string, what));
  EXPECT_TRUE(ru::getParam(nh, "vector_double", vectorxd, what));
  EXPECT_TRUE(ru::getParam(nh, "vector_double", vector4, what));

  EXPECT_FALSE(ru::getParam(nh, "vector_double", vector10, what));
  COUT << ":vector100:" << vector10.transpose() << " what:" << what  << std::endl;

  Eigen::Matrix<double, 4, 1> vector4def;
  vector4def.setConstant(2.0);
  EXPECT_TRUE(ru::getParam(nh, "vector_double__", vector4, what, &vector4def));

  Eigen::Matrix < double, -1, 1 > vectorxdef;
  vectorxdef.resize(7);
  vectorxdef.setConstant(3.0);
  EXPECT_TRUE(ru::getParam(nh, "vector_double__", vectorxd, what, &vectorxdef));
  EXPECT_TRUE(ru::getParam(nh, "matrix_double", matrixxd, what));
  EXPECT_TRUE(ru::getParam(nh, "matrix_double", matrix22, what));
  EXPECT_FALSE(ru::getParam(nh, "matrix_double", matrix1010, what));
  EXPECT_FALSE(ru::getParam(nh, "matrix_double", vectorxd, what));
}

// Declare a test
TEST(TestSuite, paramVectors1dVector)
{
  ros::NodeHandle nh("/test_vectors");

  std::vector<double> vector_double;

  std::string what;
  EXPECT_TRUE(ru::getParam(nh, "/test_vectors_1d/A", vector_double, what));
  EXPECT_TRUE(ru::getParam(nh, "/test_vectors_1d/B", vector_double, what));
  EXPECT_FALSE(ru::getParam(nh, "/test_vectors_1d/C", vector_double, what));
  COUT << ":vectors:" << " what:" << what << std::endl;
}


TEST(TestSuite, paramVectors1dMatrix)
{
  ros::NodeHandle nh("/test_vectors");

  std::vector<std::vector<double>> matrix_double;

  std::string what;

//=====================
  EXPECT_TRUE(ru::getParam(nh, "/test_vectors_1d/A", matrix_double, what));
  EXPECT_TRUE(ru::getParam(nh, "/test_vectors_1d/B", matrix_double, what));
  EXPECT_TRUE(ru::getParam(nh, "/test_vectors_1d/C", matrix_double, what));
}


// Declare a test
TEST(TestSuite, paramEigenVectors1dVectorXd)
{
  ros::NodeHandle nh("/test_vectors");

  Eigen::VectorXd vector_double;

  std::string what;
  EXPECT_TRUE(ru::getParam(nh, "/test_vectors_1d/A", vector_double, what));
  EXPECT_TRUE(ru::getParam(nh, "/test_vectors_1d/B", vector_double, what));
  EXPECT_FALSE(ru::getParam(nh, "/test_vectors_1d/C", vector_double, what));
  COUT << ":vectors:" << (vector_double) << ( what.size()>0? " what: " + what : "")  << std::endl;
}

// Declare a test
TEST(TestSuite, paramEigenVectors1dMatrixXd)
{
  ros::NodeHandle nh("/test_vectors");

  Eigen::MatrixXd matrix_double;

  std::string what;
  EXPECT_TRUE(ru::getParam(nh, "/test_vectors_1d/A", matrix_double, what));
  EXPECT_TRUE(ru::getParam(nh, "/test_vectors_1d/B", matrix_double, what));
  EXPECT_TRUE(ru::getParam(nh, "/test_vectors_1d/C", matrix_double, what));
  COUT << ":vectors:" << (matrix_double) << ( what.size()>0? " what: " + what : "")  << std::endl;
}
// Declare a test
TEST(TestSuite, paramVectors2dVector)
{
  ros::NodeHandle nh("/test_vectors");

  std::vector<double> vector_double;

  std::string what;
  EXPECT_TRUE(ru::getParam(nh, "/test_vectors_2d/A", vector_double, what));
  EXPECT_FALSE(ru::getParam(nh, "/test_vectors_2d/B", vector_double, what));
  COUT << ":vectors:" << ( what.size()>0? " what: " + what : "")  << std::endl;
}


TEST(TestSuite, paramVectors2dMatrix)
{
  ros::NodeHandle nh("/test_vectors");

  std::vector<std::vector<double>> matrix_double;

  std::string what;

//=====================
  EXPECT_TRUE(ru::getParam(nh, "/test_vectors_2d/A", matrix_double, what));
  EXPECT_TRUE(ru::getParam(nh, "/test_vectors_2d/B", matrix_double, what));
}


// Declare a test
TEST(TestSuite, paramEigenVectors2dVectorXd)
{
  ros::NodeHandle nh("/test_vectors_2d");

  Eigen::VectorXd vector_double;

  std::string what;
  EXPECT_TRUE(ru::getParam(nh, "A", vector_double, what));
  EXPECT_FALSE(ru::getParam(nh, "B", vector_double, what));
  COUT << ":vectors:" << ( what.size()>0? " what: " + what : "")  << std::endl;
}

// Declare a test
TEST(TestSuite, paramEigenVectors2dMatrixXd)
{
  ros::NodeHandle nh("/test_vectors");

  Eigen::MatrixXd matrix_double;

  std::string what;
  EXPECT_TRUE(ru::getParam(nh, "/test_vectors_2d/A", matrix_double, what));
  EXPECT_TRUE(ru::getParam(nh, "/test_vectors_2d/B", matrix_double, what));
}



// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "test");

  return RUN_ALL_TESTS();
}

#undef COUT
