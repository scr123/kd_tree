/**
 * MIT License
 *
 * Copyright (c) 2019 Sean Crutchlow
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file node_test.cpp
 * @brief Unit tests using Google Test framework to validate class
 *  implementation.
 * @author Sean Crutchlow <sean.GH1231@gmail.com>
 * @version 1.0
 */

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <sstream>

#include "kd_tree/node.h"

namespace kd_tree {

class NodeTest : public ::testing::Test {
 protected:
    std::shared_ptr<Node<double>> node_a_ptr;
    std::shared_ptr<Node<double>> node_b_ptr;
    std::shared_ptr<Node<double>> node_c_ptr;
    std::shared_ptr<Node<double>> node_d_ptr;

  virtual void SetUp() {
    node_a_ptr = std::make_shared<Node<double>>(2.2);
    node_b_ptr = std::make_shared<Node<double>>(4.2);
    node_c_ptr = std::make_shared<Node<double>>(6.4);
    node_d_ptr = std::make_shared<Node<double>>(8.6);
  }

  virtual void TearDown() {}
};

TEST_F(NodeTest, Copy_Constructor) {
  /// Set children of node
  node_a_ptr->SetLeftChild(node_b_ptr);
  node_a_ptr->SetRightChild(node_c_ptr);

  /// Construct node using copy constructor
  Node<double> node(*node_a_ptr);

  /// Check that node data and children are the same
  bool same_contents = (node == *node_a_ptr
    && *node.GetLeftChild() == *node_a_ptr->GetLeftChild()
    && *node.GetRightChild() == *node_a_ptr->GetRightChild());

  EXPECT_TRUE(same_contents);
}

TEST_F(NodeTest, Assignment_Operator) {
  /// Set children of node
  node_a_ptr->SetLeftChild(node_b_ptr);
  node_a_ptr->SetRightChild(node_c_ptr);

  /// Copy node from right-hand side using assignment operator
  Node<double> node = *node_a_ptr;

  /// Check that node data and children are the same
  bool same_contents = (node == *node_a_ptr
    && *node.GetLeftChild() == *node_a_ptr->GetLeftChild()
    && *node.GetRightChild() == *node_a_ptr->GetRightChild());

  EXPECT_TRUE(same_contents);
}

TEST_F(NodeTest, Equal_Nodes) {
  EXPECT_TRUE(*node_a_ptr == *node_a_ptr);
}

TEST_F(NodeTest, Equal_Nodes_False) {
  EXPECT_FALSE(*node_a_ptr == *node_b_ptr);
}

TEST_F(NodeTest, Not_Equal_Nodes) {
  EXPECT_TRUE(*node_a_ptr != *node_b_ptr);
}

TEST_F(NodeTest, Not_Equal_Nodes_False) {
  EXPECT_FALSE(*node_a_ptr == *node_b_ptr);
}

TEST_F(NodeTest, Equal_Children) {
  node_a_ptr->SetLeftChild(node_c_ptr);
  node_a_ptr->SetRightChild(node_d_ptr);

  node_b_ptr->SetRightChild(node_c_ptr);
  node_b_ptr->SetLeftChild(node_d_ptr);

  EXPECT_TRUE(*node_a_ptr->GetLeftChild() == *node_b_ptr->GetRightChild());
}

TEST_F(NodeTest, Output_Node) {
  node_a_ptr->SetLeftChild(node_c_ptr);
  node_a_ptr->SetRightChild(node_d_ptr);

  std::string node_str = "Node = [Data: (2.2)   Left Child: (6.4)"
    "   Right Child: (8.6)]";

  std::stringstream output;
  output << *node_a_ptr;

  EXPECT_EQ(node_str, output.str());
}
}   //  namespace kd_tree

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
