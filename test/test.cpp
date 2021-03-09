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

  XmlRpc::XmlRpcValue node;
  EXPECT_TRUE(nh.getParam("struct", node));

  EXPECT_TRUE(ru::check(node, required_fields));

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


  EXPECT_NO_FATAL_FAILURE(ru::fromXmlRpcValue(node["int_i"         ], int_i));
  COUT << ":int_i:" << int_i << std::endl;
  EXPECT_NO_FATAL_FAILURE(ru::fromXmlRpcValue(node["int_i2"        ], int_i2));
  COUT << ":int_i2:" << int_i2 << std::endl;
  EXPECT_NO_FATAL_FAILURE(ru::fromXmlRpcValue(node["double_d"      ], double_d));
  COUT << ":int_d:" << double_d << std::endl;
  EXPECT_NO_FATAL_FAILURE(ru::fromXmlRpcValue(node["double_d2"     ], double_d2));
  COUT << ":int_d2:" << double_d2 << std::endl;
  EXPECT_NO_FATAL_FAILURE(ru::fromXmlRpcValue(node["string_s"      ], string_s));
  COUT << ":string_s:" << string_s << std::endl;
  EXPECT_NO_FATAL_FAILURE(ru::fromXmlRpcValue(node["vector_string" ], vector_string));
  COUT << ":vector_string:" << ru::to_string(vector_string) << std::endl;
  EXPECT_NO_FATAL_FAILURE(ru::fromXmlRpcValue(node["vector_int"    ], vector_int));
  COUT << ":vector_int:" << ru::to_string(vector_int) << std::endl;
  EXPECT_NO_FATAL_FAILURE(ru::fromXmlRpcValue(node["vector_double" ], vector_double));
  COUT << ":vector_double:" << ru::to_string(vector_double) << std::endl;

  EXPECT_NO_FATAL_FAILURE(ru::fromXmlRpcValue(node["matrix_int"    ], matrix_int));
  COUT << ":matrix_int:" << ru::to_string(matrix_int) << std::endl;
  EXPECT_NO_FATAL_FAILURE(ru::fromXmlRpcValue(node["matrix_double" ], matrix_double));
  COUT << ":matrix_double:" << ru::to_string(matrix_double) << std::endl;
  EXPECT_NO_FATAL_FAILURE(ru::fromXmlRpcValue(node["matrix_string" ], matrix_string));
  COUT << ":matrix_string:" << ru::to_string(matrix_string) << std::endl;

  EXPECT_NO_FATAL_FAILURE(ru::fromXmlRpcValue(node["vector_double" ], vectorxd));
  COUT << ":vectorxd:" << vectorxd.transpose() << std::endl;
  EXPECT_NO_FATAL_FAILURE(ru::fromXmlRpcValue(node["vector_double" ], vector4));
  COUT << ":vector4:" << vector4.transpose() << std::endl;
  EXPECT_ANY_THROW(ru::fromXmlRpcValue(node["vector_double" ], vector10));
  COUT << ":vector100:" << vector10.transpose() << std::endl;

  EXPECT_NO_FATAL_FAILURE(ru::fromXmlRpcValue(node["matrix_double" ], matrixxd));
  COUT << ":matrixxd:" << matrixxd << std::endl;
  EXPECT_NO_FATAL_FAILURE(ru::fromXmlRpcValue(node["matrix_double" ], matrix22));
  COUT << ":matrix22:" << matrix22 << std::endl;
  EXPECT_ANY_THROW(ru::fromXmlRpcValue(node["matrix_double" ], matrix1010));
  COUT << ":matrix1010:" << matrix1010 << std::endl;

  EXPECT_ANY_THROW(ru::fromXmlRpcValue(node["matrix_double" ], vectorxd));
  COUT << ":vectorxd:" << vectorxd.transpose() << std::endl;
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
  COUT << ":int_i:" << int_i << " what:" << what << std::endl;

  EXPECT_TRUE(ru::getParam(nh, "int_i2", int_i2, what));
  COUT << ":int_i2:" << int_i2 << " what:" << what << std::endl;

  EXPECT_TRUE(ru::getParam(nh, "double_d", double_d, what));
  COUT << ":int_d:" << double_d << " what:" << what << std::endl;

  EXPECT_TRUE(ru::getParam(nh, "double_d2", double_d2, what));
  COUT << ":int_d2:" << double_d2 << " what:" << what << std::endl;

  EXPECT_TRUE(ru::getParam(nh, "string_s", string_s, what));
  COUT << ":string_s:" << string_s << " what:" << what << std::endl;

  EXPECT_TRUE(ru::getParam(nh, "vector_string", vector_string, what));
  COUT << ":vector_string:" << ru::to_string(vector_string) << " what:" << what << std::endl;

  EXPECT_TRUE(ru::getParam(nh, "vector_int", vector_int, what));
  COUT << ":vector_int:" << ru::to_string(vector_int) << " what:" << what << std::endl;

  EXPECT_TRUE(ru::getParam(nh, "vector_double", vector_double, what));
  COUT << ":vector_double:" << ru::to_string(vector_double) << " what:" << what << std::endl;

  EXPECT_TRUE(ru::getParam(nh, "matrix_int", matrix_int, what));
  COUT << ":matrix_int:" << ru::to_string(matrix_int) << " what:" << what << std::endl;

  EXPECT_TRUE(ru::getParam(nh, "matrix_double", matrix_double, what));
  COUT << ":matrix_double:" << ru::to_string(matrix_double) << " what:" << what << std::endl;

  EXPECT_TRUE(ru::getParam(nh, "matrix_string", matrix_string, what));
  COUT << ":matrix_string:" << ru::to_string(matrix_string) << " what:" << what << std::endl;

  EXPECT_TRUE(ru::getParam(nh, "vector_double", vectorxd, what));
  COUT << ":vectorxd:" << vectorxd.transpose() << " what:" << what << std::endl;

  EXPECT_TRUE(ru::getParam(nh, "vector_double", vector4, what));
  COUT << ":vector4:" << vector4.transpose() << " what:" << what << std::endl;

  EXPECT_FALSE(ru::getParam(nh, "vector_double", vector10, what));
  COUT << ":vector100:" << vector10.transpose() << " what:" << what  << std::endl;

  Eigen::Matrix<double, 4, 1> vector4def;
  vector4def.setConstant(2.0);
  EXPECT_TRUE(ru::getParam(nh, "vector_double__", vector4, what, &vector4def));
  COUT << ":vector4:" << vector4def.transpose() << "/" << vector4.transpose() << " what:" << what  << std::endl;

  Eigen::Matrix < double, -1, 1 > vectorxdef;
  vectorxdef.resize(7);
  vectorxdef.setConstant(3.0);
  EXPECT_TRUE(ru::getParam(nh, "vector_double__", vectorxd, what, &vectorxdef));
  COUT << ":vectorxd:" << vectorxd.transpose() << "/" << vectorxdef.transpose() << " what:" << what  << std::endl;

  EXPECT_TRUE(ru::getParam(nh, "matrix_double", matrixxd, what));
  COUT << ":matrixxd:" << matrixxd << " what:" << what << std::endl;

  EXPECT_TRUE(ru::getParam(nh, "matrix_double", matrix22, what));
  COUT << ":matrix22:" << matrix22 << " what:" << what << std::endl;

  EXPECT_FALSE(ru::getParam(nh, "matrix_double", matrix1010, what));
  COUT << ":matrix1010:" << matrix1010 << " what:" << what  << std::endl;

  EXPECT_FALSE(ru::getParam(nh, "matrix_double", vectorxd, what));
  COUT << ":vectorxd:" << vectorxd.transpose() << " what:" << what  << std::endl;

  // To be added test for ru::setParam
}

// Declare a test
TEST(TestSuite, paramVectors)
{
  ros::NodeHandle nh("/test_vectors");

  std::vector<double> vector_double;

  std::string what;
  EXPECT_TRUE(ru::getParam(nh, "/test_vectors/vectors/A", vector_double, what));
  COUT << ":vectors:" << " what:" << what << ", " << ru::to_string(vector_double) << std::endl;
  EXPECT_TRUE(ru::getParam(nh, "/test_vectors/vectors/B", vector_double, what));
  COUT << ":vectors:" << " what:" << what << ", " << ru::to_string(vector_double) << std::endl;
  EXPECT_FALSE(ru::getParam(nh, "/test_vectors/vectors/C", vector_double, what));
  COUT << ":vectors:" << " what:" << what << ", " << ru::to_string(vector_double) << std::endl;

}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "test");

  return RUN_ALL_TESTS();
}

#undef COUT
