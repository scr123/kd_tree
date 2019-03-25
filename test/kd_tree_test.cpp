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
 * @file kd_tree_test.cpp
 * @brief Unit tests using Google Test framework to validate class
 *  implementation.
 * @author Sean Crutchlow <sean.GH1231@gmail.com>
 * @version 1.0
 */

#include <gtest/gtest.h>
#include <memory>
#include <vector>

#include "kd_tree/kd_tree.h"

namespace kd_tree {

class KdTest : public ::testing::Test {
 protected:
    KdTree<double> *kd_ptr;

  virtual void SetUp() {
		/*
		 *			K-D Tree:																			                             Dimension:	  Depth:
		 *					 		        [1.2, 3.4, 5.7]															               ||   0		     1
		 *							           /         \
		 *				   [0.5, 11.1, 5.6]			[6.4, 7.9, 8.1]											             ||   1		     2
		 *					/						                  /			 \
		 *	  [0.3, 9.2, 6.1]		   [11.5, 1.3, 2.7]			[5.8, 10.1, 50.6]							     ||   2		     3
		 *				 	 		           / \			 			              /
		 *				   [11.2, 4.2, 0.7][6.6, 7.7, 9.1]        [7.1, 9.9, 0.3]    				     ||   0		     4
		 *				 	 		    		       /\			    				             /\
		 *  			 	 	  [1.8, 3.2, 14.7][10.0, 1.1, 5.3][7.0, 11.0, 5.0][11.6, 13.5, 4.2]	 ||   1		     5
		 *					 	 		    											                       \
		 * 																		   	                     [8.9, 16.2, 7.2]    ||   2		     6
     *                                                                   \
     *                                                              [8.9, 16.2, 7.2]   ||   0        7
		 */

    /// Insert all nodes for K-D Tree in vector of states
    std::vector<State<double>> nodes{
      {1.2, 3.4, 5.7}, {6.4, 7.9, 8.1}, {11.5, 1.3, 2.7},
      {5.8, 10.1, 50.6}, {7.1, 9.9, 0.3}, {6.6, 7.7, 9.1},
      {1.8, 3.2, 14.7}, {7.0, 11.0, 5.0}, {11.6, 13.5, 4.2},
      {11.2, 4.2, 0.7}, {8.9, 16.2, 7.2}, {10.0, 1.1, 5.3},
      {0.5, 11.1, 5.6}, {0.3, 9.2, 6.1}, {8.9, 16.2, 7.2}};

    /// Use constructor with State vector argument to build K-D Tree
    kd_ptr = new KdTree<double>(nodes);
  }

  virtual void TearDown() {
    delete kd_ptr;
  }
};

TEST_F(KdTest, Find_Depth) {
  EXPECT_EQ(7, kd_ptr->FindDepth());
}

TEST_F(KdTest, Search_One_Match) {
  Node<State<double>> target({6.6, 7.7, 9.1});
  Node<State<double>> target_lc({1.8, 3.2, 14.7});
  Node<State<double>> target_rc({10.0, 1.1, 5.3});

  std::vector<std::shared_ptr<
    Node<State<double>>>> vec = kd_ptr->Search(target);

  /// Check that search has found a single match
  bool all_nodes_found = (vec.size() == 1
    && *vec[0].get() == target
    && *vec[0]->GetLeftChild() == target_lc
    && *vec[0]->GetRightChild() == target_rc);

  EXPECT_TRUE(all_nodes_found);
}

TEST_F(KdTest, Search_Two_Matches) {
  Node<State<double>> target({8.9, 16.2, 7.2});

  std::vector<std::shared_ptr<
    Node<State<double>>>> vec = kd_ptr->Search(target);

  /// Check that search has found two matches
  bool all_nodes_found = (vec.size() == 2
    && *vec[0].get() == target && *vec[1].get() == target
    && *vec[0]->GetRightChild().get() == *vec[1].get());

  EXPECT_TRUE(all_nodes_found);
}

TEST_F(KdTest, Find_Min_0) {
  Node<State<double>> min_dimension_0({0.3, 9.2, 6.1});

  EXPECT_EQ(min_dimension_0, *kd_ptr->FindMin(kd_ptr->GetRoot(), 0));
}

TEST_F(KdTest, Find_Min_1) {
  Node<State<double>> min_dimension_1({10.0, 1.1, 5.3});

  EXPECT_EQ(min_dimension_1, *kd_ptr->FindMin(kd_ptr->GetRoot(), 1));
}

TEST_F(KdTest, Find_Min_2) {
  Node<State<double>> min_dimension_2({7.1, 9.9, 0.3});

  EXPECT_EQ(min_dimension_2, *kd_ptr->FindMin(kd_ptr->GetRoot(), 2));
}

TEST_F(KdTest, Find_Nearest_Root_Case) {
  Node<State<double>> target_node({6.6, 7.7, 9.1});
  Node<State<double>> nearest_node({1.2, 3.4, 5.7});

  EXPECT_EQ(nearest_node.GetData(),
    kd_ptr->FindNearest(target_node)->GetData());
}

TEST_F(KdTest, Find_Nearest) {
  Node<State<double>> target_node({7.1, 9.9, 0.3});
  Node<State<double>> nearest_node({11.6, 13.5, 4.2});

  EXPECT_EQ(nearest_node.GetData(),
    kd_ptr->FindNearest(target_node)->GetData());
}

TEST_F(KdTest, Delete_Node) {
  Node<State<double>> target_node({6.6, 7.7, 9.1});
  Node<State<double>> replacement({10.0, 1.1, 5.3});
  Node<State<double>> left_child({1.8, 3.2, 14.7});

  /// Search for node to be deleted
  std::shared_ptr<Node<State<double>>>
    node_ptr = kd_ptr->Search(target_node).at(0);

  /// Delete node
  kd_ptr->Delete(node_ptr);

  /// Search for node that is expected to take place of deleted node
  std::shared_ptr<Node<State<double>>>
    replacement_ptr = kd_ptr->Search(replacement).at(0);

  /// Search for left child of node replacement
  std::shared_ptr<Node<State<double>>>
    left_child_ptr = kd_ptr->Search(left_child).at(0);

  EXPECT_EQ(replacement_ptr->GetLeftChild().get(), left_child_ptr.get());
}

TEST_F(KdTest, Delete_Nearest_Node_Which_Is_Root) {
  Node<State<double>> target_node({1.2, 3.4, 5.7});
  Node<State<double>> root_replacement({1.8, 3.2, 14.7});

  /// Search for nearest node... root of tree in this case
  std::shared_ptr<Node<State<double>>>
    node_ptr = kd_ptr->FindNearest(target_node);

  /// Delete node
  kd_ptr->Delete(node_ptr);

  EXPECT_EQ(root_replacement.GetData(), kd_ptr->GetRoot()->GetData());
}

TEST_F(KdTest, Delete_Nearest_Node_Which_Has_Same_Contents) {
  Node<State<double>> target_node({8.9, 16.2, 7.2});
  Node<State<double>> parent_node({11.6, 13.5, 4.2});

  std::shared_ptr<Node<State<double>>>
    nearest_ptr = kd_ptr->FindNearest(target_node);

  kd_ptr->Delete(nearest_ptr);

  std::shared_ptr<Node<State<double>>>
    parent_ptr = kd_ptr->Search(parent_node).at(0);

  std::shared_ptr<Node<State<double>>>
    replacement_ptr = kd_ptr->Search(target_node).at(0);

  /// Check new depth of tree and right child of parent for replacement
  bool is_correct = (kd_ptr->FindDepth() == 6
      && parent_ptr->GetRightChild() == replacement_ptr);

  EXPECT_TRUE(is_correct);
}

TEST_F(KdTest, KD_Tree_Is_Equal) {
  EXPECT_TRUE(*kd_ptr == *kd_ptr);
}

TEST_F(KdTest, KD_Tree_Is_Equal_Tree_Copy_Constructor) {
  KdTree<double> kd_tree_copy(*kd_ptr);

  EXPECT_TRUE(kd_tree_copy == *kd_ptr);
}

TEST_F(KdTest, KD_Tree_Is_Equal_Tree_Assignment) {
  KdTree<double> kd_tree_copy = *kd_ptr;

  EXPECT_TRUE(kd_tree_copy == *kd_ptr);
}

TEST_F(KdTest, KD_Tree_Not_Equal_Different_Depth) {
  KdTree<double> kd_tree_copy(*kd_ptr);

  kd_tree_copy.Insert(Node<State<double>>({9.1, 16.2, 8.4}));

  EXPECT_TRUE(kd_tree_copy != *kd_ptr);
}

TEST_F(KdTest, KD_Tree_Roots_Is_Deep_Copy) {
  KdTree<double> kd_tree_copy = *kd_ptr;

  /**
   * Root addresses in memory should not be the same since 
   * a deep copy was performed.
   */
  EXPECT_NE(kd_tree_copy.GetRoot().get(), kd_ptr->GetRoot().get());
}

TEST_F(KdTest, KD_Tree_Not_Equal_Different_Root) {
  KdTree<double> kd_tree_copy = *kd_ptr;

  kd_tree_copy.GetRoot()->SetData(State<double>({1.2, 3.4, 5.66666666}));

  EXPECT_TRUE(kd_tree_copy != *kd_ptr);
}
}   //  namespace kd_tree

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
