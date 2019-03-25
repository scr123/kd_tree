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
 * @file kd_tree.h
 * @brief KdTree class providing basic functionality of a K-D Tree data structure.
 * @author Sean Crutchlow <sean.GH1231@gmail.com>
 * @version 1.0
 */

#ifndef KD_TREE_H_
#define KD_TREE_H_

#include <algorithm>
#include <climits>
#include <functional>
#include <iostream>
#include <memory>
#include <vector>

#include "kd_tree/state.h"
#include "kd_tree/node.h"

namespace kd_tree {
/**
 * @brief      Class for kd tree.
 *
 * @tparam     T     C++ Fundamental Types
 */
template<typename T>
class KdTree {
	/**
	 * Node of State<T> type
	 */
  typedef Node<State<T>> node_t;

	/**
	 * Smart pointer typedef of node_t
	 */
  typedef std::shared_ptr<node_t> node_ptr;

 public:
	/**
	 * @brief      Constructs the object.
	 */
  KdTree() : is_root_(true), node_length_(0) {}

	/**
	 * @brief      Constructs the object.
	 *
	 * @param[in]  _kd   KdTree to be copied
	 */
  KdTree(const KdTree& _kd) : is_root_(true), node_length_(0) {
    *this = _kd;
  }

	/**
	 * @brief      Constructs the object.
	 *
	 * @param[in]  _states  The states
	 */
  explicit KdTree(const std::vector<State<T>>& _states) : is_root_(true),
    node_length_(0) {
    /// Insert all states into K-D tree
    for (auto it : _states) {
      Node<State<T>> node(it);
      this->Insert(node);
    }
  }

	/**
	 * @brief      Direct assignment operator.
	 *
	 * @param[in]  _other  The other
	 *
	 * @return     KdTree for assignment.
	 */
  KdTree& operator=(const KdTree& _other) {
    /// Assignment by right-hand side to left-hand side copy
    Copy(this, _other.GetRoot());
    return *this;
  }

	/**
	 * @brief      Equal to operator.
	 *
	 * @param      _other  The other
	 *
	 * @return     True if equal, False otherwise.
	 */
  bool operator==(KdTree const& _other) const {
    /// Check depth of tree before comparing node data.
    if (this->FindDepth() != _other.FindDepth())
      return false;

    /// Check equality of all nodes.
    return IsEqualR(this->GetRoot(), _other.GetRoot());
  }

	/**
	 * @brief      Not equal to operator.
	 *
	 * @param      _other  The other
	 *
	 * @return     True if not equal, False otherwise.
	 */
  bool operator!=(KdTree const& _other) const {
    return !(*this == _other);
  }

	/**
	 * @brief      Gets the root.
	 *
	 * @return     The root.
	 */
  node_ptr const& GetRoot() const {
    return this->root_;
  }

  /**
   * @brief      Inserts nodes into KdTree.
   *
   * @param      _kd    KdTree to be modified
   * @param[in]  _root  The root
   */
  void Copy(KdTree* _kd, node_ptr _root) {
    if (_root.get() == nullptr)
      return;

    /// Insert root node for K-D Tree on left-hand side
    _kd->Insert(*_root);

    /// Copy node_ptrs recursively starting with root children
    CopyR(_kd->GetRoot(), _root);
  }

	/**
	 * @brief      Inserts node.
	 *
	 * @param[in]  _node  The node
	 */
  void Insert(const node_t& _node) {
    /// Allocate memory for shared pointer of node to be inserted.
    node_ptr shared_node = std::make_shared<node_t>(_node);
    InsertR(this->GetRoot(), shared_node, 0);
  }

	/**
	 * @brief      Deletes node.
	 *
	 * @param[in]  _node  The node
	 */
  void Delete(node_ptr _node) {
    DeleteR(this->GetRoot(), _node, 0);
  }

	/**
	 * @brief      Finds max depth of KdTree.
	 *
	 * @return     Max depth.
	 */
  size_t FindDepth() const {
    return FindDepthR(this->GetRoot());
  }

  /**
   * @brief      Searches for node matches.
   *
   * @param[in]  _node  The node
   *
   * @return     Vector of node_ptrs containing matches.
   */
  std::vector<node_ptr> Search(const node_t& _node) {
    std::vector<node_ptr> vec;
    SearchR(this->GetRoot(), _node, &vec, 0);

    return vec;
  }

	/**
	 * @brief      Finds node with minimum value for specified dimension.
	 *
	 * @param[in]  _node       The node
	 * @param[in]  _dimension  The dimension
	 *
	 * @return     node_ptr to min node.
	 */
  node_ptr FindMin(node_ptr _node, unsigned _dimension) {
    return FindMinR(_node, _dimension, 0);
  }

	/**
	 * @brief      Finds nearest node based on Euclidian distance.
	 *
	 * @param[in]  _node  The node
	 *
	 * @return     node_ptr to the nearest node.
	 */
  node_ptr FindNearest(const node_t& _node) {
    /// Find nearest node
    node_ptr nearest = FindNearestR(this->GetRoot(), _node, 0);

    /// Calculate distance between current node and target node
    T nearest_distance = CalcDist(*nearest, _node);

    /// Calculate distance between root node of tree and target node
    T root_distance = CalcDist(*this->GetRoot(), _node);

    /// Return node with distance that is closest to target node
    return (nearest_distance < root_distance ? nearest : this->GetRoot());
  }

	/**
	 * @brief      Prints KdTree in 2D (C.C.W. +90 degrees).
	 */
  void PrintTree() {
    std::cout << std::endl << "K-D Tree:" << std::endl;
    PrintTreeR(this->GetRoot(), 0);
  }

 private:
  /**
   * @brief      Copies nodes from _rhs to _lhs recursively.
   *
   * @param[in]  _rhs  The right hand side
   * @param[in]  _rhs  The right hand side
   */
  void CopyR(node_ptr _lhs, node_ptr _rhs) {
    if (_rhs.get() == nullptr)
      return;

    /// Node level assignment
    *_lhs = *_rhs;

    /// Traverse K-D Trees
    CopyR(_lhs->GetLeftChild(), _rhs->GetLeftChild());
    CopyR(_lhs->GetRightChild(), _rhs->GetRightChild());
  }

	/**
	 * @brief      Determines if equal recursively.
	 *
	 * @param[in]  _lhs  The left hand side
	 * @param[in]  _rhs  The right hand side
	 *
	 * @return     True if equal, False otherwise.
	 */
  bool IsEqualR(node_ptr _lhs, node_ptr _rhs) const {
    /// If both sides are nullptr, are considered equal
    if (_lhs.get() == nullptr && _rhs.get() == nullptr) {
      return true;
    /// If only one side is nullptr, is not equal
    } else if (_lhs.get() == nullptr || _rhs.get() == nullptr) {
      return false;
    /// If contents of node_ptrs are the same, nodes are equal
    } else if (*_lhs != *_rhs) {
      return false;
    } else {
      /// Traverse K-D Trees
      IsEqualR(_lhs->GetLeftChild(), _rhs->GetLeftChild());
      IsEqualR(_lhs->GetRightChild(), _rhs->GetRightChild());

      return true;
    }
  }

	/**
	 * @brief      Inserts node recursively.
	 *
	 * @param[in]  _root   The root
	 * @param[in]  _node   The node
	 * @param[in]  _depth  The depth
	 *
	 * @return     node_ptr to next _root.
	 */
  node_ptr InsertR(node_ptr _root, node_ptr _node, unsigned _depth) {
    if (_root.get() == nullptr) {
      _root = _node;

      /// Logic to insert first node as root for private class member
      if (is_root_) {
        root_ = _root;

        /// Use size of root data as expected size for all nodes in tree
        node_length_ = _root->GetData().size();
        is_root_ = false;
      }

      return _root;
    }
    /// Find dimension of inspection for given depth
    unsigned curr_dimension = (_depth % this->GetRoot()->GetData().size());

    /**
     * Set node as child according to K-D Tree algorithm
     */
    /// Dimension under inspection for node being inserted if less than
    /// state element
    if (_node->GetData().operator[](curr_dimension)
      < _root->GetData().operator[](curr_dimension)) {
      _root->SetLeftChild(
        InsertR(_root->GetLeftChild(), _node, (_depth + 1)));
    /// Dimension under inspection for node being inserted if greater than
    /// state element
    } else {
      _root->SetRightChild(
        InsertR(_root->GetRightChild(), _node, (_depth + 1)));
    }

    return _root;
  }

	/**
	 * @brief      Deletes node recursively.
	 *
	 * @param[in]  _root   The root
	 * @param[in]  _node   The node
	 * @param[in]  _depth  The depth
	 *
	 * @return     node_ptr to next _root.
	 */
  node_ptr DeleteR(node_ptr _root, node_ptr _node, unsigned _depth) {
    if (_root.get() == nullptr)
      return nullptr;

    /// Find dimension of inspection for given depth
    unsigned curr_dimension = (_depth % node_length_);

    /// Check for same memory address of node that is to be deleted
    if (_root.get() == _node.get()) {
      /// See if right child is elgible for replacing deleted node
      if (_root->GetRightChild() != nullptr) {
        node_ptr min = FindMin(_root->GetRightChild(), curr_dimension);
        _root->SetData(min->GetData());
        _root->SetRightChild(
          DeleteR(_root->GetRightChild(), min, (_depth + 1)));
      /// See if left child is elgible for replacing deleted node
      } else if (_root->GetLeftChild() != nullptr) {
        node_ptr min = FindMin(_root->GetLeftChild(), curr_dimension);
        _root->SetData(min->GetData());
        _root->SetRightChild(
          DeleteR(_root->GetLeftChild(), min, (_depth + 1)));
        _root->SetLeftChild(nullptr);
      /// Set target node to nullptr and reset the shared_ptr
      } else {
        _root = nullptr;
        _root.reset();

        return nullptr;
      }

      return _root;
    }

    /// If memory address for _node and _root is not the same, traverse tree
    if (_node->GetData().operator[](curr_dimension)
      < (_root->GetData()).operator[](curr_dimension)) {
      _root->SetLeftChild(
        DeleteR(_root->GetLeftChild(), _node, (_depth + 1)));
    } else {
      _root->SetRightChild(
        DeleteR(_root->GetRightChild(), _node, (_depth + 1)));
    }

    return _root;
  }

	/**
	 * @brief      Finds max depth recursively.
	 *
	 * @param[in]  _root  The root
	 *
	 * @return     Max depth.
	 */
  size_t FindDepthR(node_ptr _root) const {
    if (_root.get() == nullptr)
      return 0;

    /// Find depth of left and right sub-trees
    unsigned left_subtree_depth = FindDepthR(_root->GetLeftChild());
    unsigned right_subtree_depth = FindDepthR(_root->GetRightChild());

    /// Return max depth between the two sub-trees adding +1 for root depth
    return (std::max(left_subtree_depth, right_subtree_depth) + 1);
  }

  /**
   * @brief      Searches for node matches recursively.
   *
   * @param[in]  _root   The root
   * @param[in]  _node   The node
   * @param      _vec    The vector
   * @param[in]  _depth  The depth
   */
  void SearchR(node_ptr _root, const node_t& _node,
    std::vector<node_ptr>* _vec,
    unsigned _depth) {
    if (_root.get() == nullptr)
      return;

    /// When a match is found, push_back into vector
    if (*_root == _node) {
      _vec->push_back(_root);
    }

    /// Find dimension of inspection for given depth
    unsigned curr_dimension = (_depth % node_length_);

    /// Traverse tree
    if (_node.GetData().operator[](curr_dimension)
      < _root->GetData().operator[](curr_dimension)) {
      return SearchR(_root->GetLeftChild(), _node, _vec, (_depth + 1));
    }

    return SearchR(_root->GetRightChild(), _node, _vec, (_depth + 1));
  }

	/**
	 * @brief      Finds node containing minimum value for specified dimension
   *             recursively.
	 *
	 * @param[in]  _root       The root
	 * @param[in]  _dimension  The dimension
	 * @param[in]  _depth      The depth
	 *
	 * @return     node_ptr to next _root.
	 */
  node_ptr FindMinR(node_ptr _root, unsigned _dimension, unsigned _depth) {
    /**
     * Assert that dimension of inspection is within the number of dimensions
     * for a node.
     */
    assert(_dimension < node_length_);

    if (_root.get() == nullptr)
      return _root;

    /// Find dimension of inspection for given depth
    unsigned curr_dimension = (_depth % node_length_);

    /// Find state containing dimension with lowest value
    if (curr_dimension == _dimension) {
      if (_root->GetLeftChild().get() == nullptr)
        return _root;
      return FindMinR(_root->GetLeftChild(), _dimension, (_depth + 1));
    }

    /**
     * Between two nodes, return node with lowest value for specified
     * dimension.
     */
    return GetMinNode(_root,
                      FindMinR(_root->GetLeftChild(),
                        _dimension,
                        (_depth + 1)),
                      FindMinR(_root->GetRightChild(),
                        _dimension,
                        (_depth + 1)),
                      _dimension);
  }

	/**
	 * @brief      Gets the minimum node.
	 *
	 * @param[in]  _curr         The curr
	 * @param[in]  _left_child   The left child
	 * @param[in]  _right_child  The right child
	 * @param[in]  _dimension    The dimension
	 *
	 * @return     The minimum node.
	 */
  node_ptr GetMinNode(node_ptr _curr,
    node_ptr _left_child,
    node_ptr _right_child,
    unsigned _dimension) {

    /// Temporary node_ptr set to parent of left and right children
    node_ptr tmp = _curr;

    /// Check which node contains a smaller for specified dimension
    if ((_left_child.get() != nullptr)
      && ((_left_child->GetData().operator[](_dimension))
        < (_curr->GetData().operator[](_dimension)))) {
      tmp = _left_child;
    }

    if ((_right_child.get() != nullptr)
      && ((_right_child->GetData().operator[](_dimension))
        < (_curr->GetData().operator[](_dimension)))) {
      tmp = _right_child;
    }

    return tmp;
  }

	/**
	 * @brief      Finds nearest node based on Euclidian distance recursively.
	 *
	 * @param[in]  _root   The root
	 * @param[in]  _node   The node
	 * @param[in]  _depth  The depth
	 *
	 * @return     node_ptr to next _root.
	 */
  node_ptr FindNearestR(node_ptr _root, const node_t& _node, unsigned _depth) {
    if (_root.get() == nullptr)
      return _root;

    /// Find dimension of inspection for given depth
    unsigned curr_dimension = (_depth % node_length_);

    /// Temporary node_ptr
    node_ptr curr;

    /// Traverse tree
    if (_node.GetData().operator[](curr_dimension)
      < _root->GetData().operator[](curr_dimension)) {
      curr = FindNearestR(_root->GetLeftChild(), _node, (_depth + 1));
    } else {
      curr = FindNearestR(_root->GetRightChild(), _node, (_depth + 1));
    }

    if (curr.get() == nullptr)
      return _root;

    return curr;
  }

	/**
	 * @brief      Calculates the distance.
	 *
	 * @param[in]  _lhs  The left hand side
	 * @param[in]  _rhs  The right hand side
	 *
	 * @return     The distance.
	 */
  T CalcDist(node_t _lhs, node_t _rhs) {
    auto diff = _lhs.GetData() - _rhs.GetData();
    T sum = 0;

    /// Euclidian distance
    for (auto it : diff.GetState())
      sum += std::pow(it, 2.0);

    return std::sqrt(sum);
  }

	/**
	 * @brief      Prints left and right branches of KdTree recursively.
	 *
	 * @param[in]  _root    The root
	 * @param[in]  _spaces  The spaces
	 */
  void PrintTreeR(node_ptr _root, unsigned _spaces) {
    if (_root.get() != nullptr) {
      static unsigned offset = this->GetRoot()->GetData().size();

      PrintTreeR(_root->GetRightChild(), _spaces + (offset * 8));

      std::cout << std::setw(_spaces) << " ";
      std::cout << _root->GetData() << std::endl;

      PrintTreeR(_root->GetLeftChild(), _spaces + (offset * 8));
    }
  }

  /// Root node of K-D Tree
  node_ptr root_;

  /// Flag for assigning root node of K-D Tree
  bool is_root_;

  /// Number of dimensions in each node's data
  size_t node_length_;
};
}  // namespace kd_tree

#endif  // KD_TREE_H_
