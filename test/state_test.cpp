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
 * @file state_test.cpp
 * @brief Unit tests using Google Test framework to validate class
 *  implementation.
 * @author Sean Crutchlow <sean.GH1231@gmail.com>
 * @version 1.0
 */

#include <gtest/gtest.h>

#include <limits>
#include <string>
#include <sstream>

#include "kd_tree/state.h"

namespace kd_tree {

class StateTest : public ::testing::Test {
 protected:
    State<double> *state_a_ptr;
    State<double> *state_b_ptr;

    State<double> *state_type_max_ptr;
    State<double> *state_type_min_ptr;

    State<double> *sum_ptr;
    State<double> *difference_ptr;

  virtual void SetUp() {
    state_a_ptr = new State<double>({1.1, 2.2, 3.3});
    state_b_ptr = new State<double>({2.2, 3.3, 4.4});

    sum_ptr = new State<double>({3.3, 5.5, 7.7});
    difference_ptr = new State<double>({-1.1, -1.1, -1.1});

    state_type_max_ptr = new State<double>(3,
      std::numeric_limits<double>::max());
    state_type_min_ptr = new State<double>(3,
      std::numeric_limits<double>::min());
  }

  virtual void TearDown() {
    delete state_a_ptr;
    delete state_b_ptr;

    delete sum_ptr;
    delete difference_ptr;

    delete state_type_max_ptr;
    delete state_type_min_ptr;
  }
};

TEST_F(StateTest, Size_State_3d) {
  EXPECT_EQ(state_a_ptr->size(), 3);
}

TEST_F(StateTest, At_State_3d) {
  EXPECT_EQ(state_a_ptr->operator[](1), 2.2);
}

TEST_F(StateTest, Equal_States_3d_Copy_Constructor) {
  State<double> state(*state_a_ptr);

  EXPECT_TRUE(state == *state_a_ptr);
}

TEST_F(StateTest, Equal_States_3d_Assignment_Operator) {
  State<double> state = *state_a_ptr;

  EXPECT_TRUE(state == *state_a_ptr);
}

TEST_F(StateTest, Equal_States_3d_False) {
  State<double> state(3, std::numeric_limits<float>::max());

  EXPECT_FALSE(state == *state_type_max_ptr);
}

TEST_F(StateTest, Not_Equal_States_3d) {
  State<double> state(3, std::numeric_limits<float>::max());

  EXPECT_TRUE(state != *state_type_max_ptr);
}

TEST_F(StateTest, Not_Equal_States_3d_False) {
  State<double> state_a({std::numeric_limits<double>::min(),
    0.0, std::numeric_limits<double>::max()});
  State<double> state_b = state_a;

  EXPECT_FALSE(state_a != state_b);
}

TEST_F(StateTest, Self_Summation_States_3d) {
  EXPECT_EQ((*state_a_ptr += *state_b_ptr), *sum_ptr);
}

TEST_F(StateTest, Self_Subtraction_States_3d) {
  EXPECT_EQ((*state_a_ptr -= *state_b_ptr), *difference_ptr);
}

TEST_F(StateTest, Addition_States_3d) {
  EXPECT_EQ((*state_a_ptr + *state_b_ptr), *sum_ptr);
}

TEST_F(StateTest, Subtraction_States_3d) {
  EXPECT_EQ((*state_a_ptr - *state_b_ptr), *difference_ptr);
}

TEST_F(StateTest, Output_State_3d) {
  std::string state_str = "[1.1, 2.2, 3.3]";

  std::stringstream output;
  output << *state_a_ptr;

  EXPECT_EQ(state_str, output.str());
}
}   //  namespace kd_tree

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
