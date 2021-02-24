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

//template<typename T>
//std::string to_string(const std::vector<T>& vv)
//{
//  std::string ret="[ ";
//  for(auto const & v : vv) ret += std::to_string(v) +" ";
//  ret +="]";
//  return ret;
//}

//template<>
//std::string to_string(const std::vector<std::string>& vv)
//{
//  std::string ret="[ ";
//  for(auto const & v : vv) ret += v +" ";
//  ret +="]";
//  return ret;
//}


//template<typename T>
//std::string to_string(const std::vector<std::vector<T>>& vv)
//{
//  std::string ret="[ ";
//  for(auto const & v : vv) ret += to_string(v) +"\n";
//  ret +="]";
//  return ret;
//}



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
  EXPECT_TRUE(nh.getParam("struct", node) );

  EXPECT_TRUE(rosparam_utilities::check(node, required_fields));

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
  Eigen::Matrix<double, 10, 1> vector10;vector10.setZero();
  Eigen::Matrix<double, 4, 1> vector4;vector4.setZero();

  Eigen::MatrixXd matrixxd;
  Eigen::Matrix<double, 10, 10> matrix1010;matrix1010.setZero();
  Eigen::Matrix<double, 2, 2>   matrix22;matrix22.setZero();


  EXPECT_NO_FATAL_FAILURE( rosparam_utilities::fromXmlRpcValue(node["int_i"         ], int_i         ));
  std::cout << __LINE__ <<":int_i:" << int_i << std::endl;
  EXPECT_NO_FATAL_FAILURE( rosparam_utilities::fromXmlRpcValue(node["int_i2"        ], int_i2        ));
  std::cout << __LINE__ <<":int_i2:" << int_i2 << std::endl;
  EXPECT_NO_FATAL_FAILURE( rosparam_utilities::fromXmlRpcValue(node["double_d"      ], double_d      ));
  std::cout << __LINE__ <<":int_d:" << double_d << std::endl;
  EXPECT_NO_FATAL_FAILURE( rosparam_utilities::fromXmlRpcValue(node["double_d2"     ], double_d2     ));
  std::cout << __LINE__ <<":int_d2:" << double_d2 << std::endl;
  EXPECT_NO_FATAL_FAILURE( rosparam_utilities::fromXmlRpcValue(node["string_s"      ], string_s      ));
  std::cout << __LINE__ <<":string_s:" << string_s << std::endl;
  EXPECT_NO_FATAL_FAILURE( rosparam_utilities::fromXmlRpcValue(node["vector_string" ], vector_string ));
  std::cout << __LINE__ <<":vector_string:" << rosparam_utilities::to_string(vector_string) << std::endl;
  EXPECT_NO_FATAL_FAILURE( rosparam_utilities::fromXmlRpcValue(node["vector_int"    ], vector_int    ));
  std::cout << __LINE__ <<":vector_int:" << rosparam_utilities::to_string(vector_int) << std::endl;
  EXPECT_NO_FATAL_FAILURE( rosparam_utilities::fromXmlRpcValue(node["vector_double" ], vector_double ));
  std::cout << __LINE__ <<":vector_double:" << rosparam_utilities::to_string(vector_double) << std::endl;

  EXPECT_NO_FATAL_FAILURE( rosparam_utilities::fromXmlRpcValue(node["matrix_int"    ], matrix_int    ));
  std::cout << __LINE__ <<":matrix_int:" << rosparam_utilities::to_string(matrix_int) << std::endl;
  EXPECT_NO_FATAL_FAILURE( rosparam_utilities::fromXmlRpcValue(node["matrix_double" ], matrix_double ));
  std::cout << __LINE__ <<":matrix_double:" << rosparam_utilities::to_string(matrix_double) << std::endl;
  EXPECT_NO_FATAL_FAILURE( rosparam_utilities::fromXmlRpcValue(node["matrix_string" ], matrix_string ));
  std::cout << __LINE__ <<":matrix_string:" << rosparam_utilities::to_string(matrix_string) << std::endl;

  EXPECT_NO_FATAL_FAILURE( rosparam_utilities::fromXmlRpcValue(node["vector_double" ], vectorxd ));
  std::cout << __LINE__ <<":vectorxd:" << vectorxd.transpose() << std::endl;
  EXPECT_NO_FATAL_FAILURE( rosparam_utilities::fromXmlRpcValue(node["vector_double" ], vector4 ));
  std::cout << __LINE__ <<":vector4:" << vector4.transpose() << std::endl;
  EXPECT_ANY_THROW(        rosparam_utilities::fromXmlRpcValue(node["vector_double" ], vector10 ));
  std::cout << __LINE__ <<":vector100:" << vector10.transpose() << std::endl;

  EXPECT_NO_FATAL_FAILURE( rosparam_utilities::fromXmlRpcValue(node["matrix_double" ], matrixxd ));
  std::cout << __LINE__ <<":matrixxd:" << matrixxd << std::endl;
  EXPECT_NO_FATAL_FAILURE( rosparam_utilities::fromXmlRpcValue(node["matrix_double" ], matrix22 ));
  std::cout << __LINE__ <<":matrix22:" << matrix22 << std::endl;
  EXPECT_ANY_THROW(        rosparam_utilities::fromXmlRpcValue(node["matrix_double" ], matrix1010 ));
  std::cout << __LINE__ <<":matrix1010:" << matrix1010 << std::endl;

  EXPECT_ANY_THROW(        rosparam_utilities::fromXmlRpcValue(node["matrix_double" ], vectorxd ));
  std::cout << __LINE__ <<":vectorxd:" << vectorxd.transpose() << std::endl;

}


// Declare a test
TEST(TestSuite, getParamMethods)
{
  ros::NodeHandle nh("/ns_exist/struct");

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
  Eigen::Matrix<double, 10, 1> vector10;vector10.setZero();
  Eigen::Matrix<double, 4, 1> vector4;vector4.setZero();

  Eigen::MatrixXd matrixxd;
  Eigen::Matrix<double, 10, 10> matrix1010;matrix1010.setZero();
  Eigen::Matrix<double, 2, 2>   matrix22;matrix22.setZero();

  std::string what;
  EXPECT_TRUE( rosparam_utilities::getParam(nh, "int_i", int_i, what));
  std::cout << __LINE__ <<":int_i:" << int_i <<" what:" << what << std::endl;

  EXPECT_TRUE( rosparam_utilities::getParam(nh, "int_i2", int_i2, what));
  std::cout << __LINE__ <<":int_i2:" << int_i2 <<" what:" << what << std::endl;

  EXPECT_TRUE( rosparam_utilities::getParam(nh, "double_d", double_d, what));
  std::cout << __LINE__ <<":int_d:" << double_d <<" what:" << what << std::endl;

  EXPECT_TRUE( rosparam_utilities::getParam(nh, "double_d2", double_d2, what));
  std::cout << __LINE__ <<":int_d2:" << double_d2 <<" what:" << what << std::endl;

  EXPECT_TRUE( rosparam_utilities::getParam(nh, "string_s", string_s, what));
  std::cout << __LINE__ <<":string_s:" << string_s <<" what:" << what << std::endl;

  EXPECT_TRUE( rosparam_utilities::getParam(nh, "vector_string", vector_string, what));
  std::cout << __LINE__ <<":vector_string:" << rosparam_utilities::to_string(vector_string) <<" what:" << what << std::endl;

  EXPECT_TRUE( rosparam_utilities::getParam(nh, "vector_int", vector_int, what));
  std::cout << __LINE__ <<":vector_int:" << rosparam_utilities::to_string(vector_int) <<" what:" << what << std::endl;

  EXPECT_TRUE( rosparam_utilities::getParam(nh, "vector_double", vector_double, what));
  std::cout << __LINE__ <<":vector_double:" << rosparam_utilities::to_string(vector_double) <<" what:" << what << std::endl;

  EXPECT_TRUE( rosparam_utilities::getParam(nh, "matrix_int", matrix_int, what));
  std::cout << __LINE__ <<":matrix_int:" << rosparam_utilities::to_string(matrix_int) <<" what:" << what << std::endl;

  EXPECT_TRUE( rosparam_utilities::getParam(nh, "matrix_double", matrix_double, what));
  std::cout << __LINE__ <<":matrix_double:" << rosparam_utilities::to_string(matrix_double) <<" what:" << what << std::endl;

  EXPECT_TRUE( rosparam_utilities::getParam(nh, "matrix_string", matrix_string, what));
  std::cout << __LINE__ <<":matrix_string:" << rosparam_utilities::to_string(matrix_string) <<" what:" << what << std::endl;

  EXPECT_TRUE( rosparam_utilities::getParam(nh, "vector_double", vectorxd, what));
  std::cout << __LINE__ <<":vectorxd:" << vectorxd.transpose() <<" what:" << what << std::endl;

  EXPECT_TRUE( rosparam_utilities::getParam(nh, "vector_double", vector4, what));
  std::cout << __LINE__ <<":vector4:" << vector4.transpose() <<" what:" << what << std::endl;

  EXPECT_FALSE( rosparam_utilities::getParam(nh, "vector_double", vector10, what));
  std::cout << __LINE__ <<":vector100:" << vector10.transpose() <<" what:" << what  << std::endl;

  Eigen::Matrix<double, 4, 1> vector4def;vector4def.setConstant(2.0);
  EXPECT_TRUE( rosparam_utilities::getParam(nh, "vector_double__", vector4, what, &vector4def));
  std::cout << __LINE__ <<":vector4:" << vector4def.transpose() <<"/" <<vector4.transpose() <<" what:" << what  << std::endl;

  Eigen::Matrix<double, -1, 1> vectorxdef;vectorxdef.resize(7); vectorxdef.setConstant(3.0);
  EXPECT_TRUE( rosparam_utilities::getParam(nh, "vector_double__", vectorxd, what, &vectorxdef));
  std::cout << __LINE__ <<":vectorxd:" << vectorxd.transpose() <<"/" <<vectorxdef.transpose() <<" what:" << what  << std::endl;

  EXPECT_TRUE( rosparam_utilities::getParam(nh, "matrix_double", matrixxd, what));
  std::cout << __LINE__ <<":matrixxd:" << matrixxd <<" what:" << what << std::endl;

  EXPECT_TRUE( rosparam_utilities::getParam(nh, "matrix_double", matrix22, what));
  std::cout << __LINE__ <<":matrix22:" << matrix22 <<" what:" << what << std::endl;

  EXPECT_FALSE(rosparam_utilities::getParam(nh, "matrix_double", matrix1010, what));
  std::cout << __LINE__ <<":matrix1010:" << matrix1010<<" what:" << what  << std::endl;

  EXPECT_FALSE(rosparam_utilities::getParam(nh, "matrix_double", vectorxd, what));
  std::cout << __LINE__ <<":vectorxd:" << vectorxd.transpose() <<" what:" << what  << std::endl;

  // To be added test for rosparam_utilities::setParam

}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc,argv,"test");

  return RUN_ALL_TESTS();
}
