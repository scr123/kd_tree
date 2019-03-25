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
 * @file node.h
 * @brief Node class providing a template for a node object.
 * @author Sean Crutchlow <sean.GH1231@gmail.com>
 * @version 1.0
 */

#ifndef NODE_H_
#define NODE_H_

#include <cmath>
#include <memory>
#include <ostream>

namespace kd_tree {
/**
 * @brief      Class for node.
 *
 * @tparam     T     Type for data within node.
 */
template<typename T>
class Node{
    /**
     * Smart pointer typedef of node
     */
    typedef std::shared_ptr<Node> node_ptr;

 public:
    /**
     * @brief      Constructs the object.
     *
     * @param[in]  _data  The data
     */
    explicit Node(const T _data) : data_(_data) {}

    /**
     * @brief      Constructs the object.
     *
     * @param[in]  _node  The node
     */
    Node(const Node& _node) {
      *this = _node;
    }

    /**
     * @brief      Direct assignment operator.
     *
     * @param[in]  _other  The other
     *
     * @return     Node for assignment.
     */
    Node& operator=(const Node& _other) {
      this->data_ = _other.data_;

      if (_other.GetLeftChild().get() != nullptr)
        this->left_child_ = std::make_shared<Node>(*_other.GetLeftChild());

      if (_other.GetRightChild().get() != nullptr)
        this->right_child_ = std::make_shared<Node>(*_other.GetRightChild());

      return *this;
    }

    /**
     * @brief      Equal to operator.
     *
     * @param      _other  The other
     *
     * @return     True if equal, False otherwise.
     */
    bool operator==(Node const& _other) const {
      return this->data_ == _other.GetData();
    }

    /**
     * @brief      Not equal to operator.
     *
     * @param      _other  The other
     *
     * @return     True if not equal, False otherwise.
     */
    bool operator!=(Node const& _other) const {
        return !(*this == _other);
    }

    /**
     * @brief      Gets the data.
     *
     * @return     The data.
     */
    T const& GetData() const {
        return this->data_;
    }

    /**
     * @brief      Gets the left child.
     *
     * @return     The left child.
     */
    node_ptr const& GetLeftChild() const {
        return this->left_child_;
    }

    /**
     * @brief      Gets the right child.
     *
     * @return     The right child.
     */
    node_ptr const& GetRightChild() const {
        return this->right_child_;
    }

    /**
     * @brief      Sets the data.
     *
     * @param[in]  _data  The data
     */
    void SetData(T const _data) {
        data_ = _data;
    }

    /**
     * @brief      Sets the left child.
     *
     * @param[in]  _node  The node
     */
    void SetLeftChild(node_ptr _node) {
        left_child_ = _node;
    }

    /**
     * @brief      Sets the right child.
     *
     * @param[in]  _node  The node
     */
    void SetRightChild(node_ptr _node) {
        right_child_ = _node;
    }

 private:
    T data_;
    node_ptr left_child_, right_child_;
};

/**
 * @brief      Output operator.
 *
 * @param      _stream  The stream
 * @param[in]  _node    The node
 *
 * @tparam     T        Type for data within node.
 *
 * @return     Output stream.
 */
template <typename T>
std::ostream& operator<<(std::ostream& _stream, const Node<T>& _node) {
    _stream << "Node = [Data: (" << _node.GetData() << ")";

    _stream << "   Left Child: (";
    if (_node.GetLeftChild() != nullptr)
        _stream << _node.GetLeftChild()->GetData();
    _stream << ")";

    _stream << "   Right Child: (";
    if (_node.GetRightChild() != nullptr)
        _stream << _node.GetRightChild()->GetData();
    _stream << ")";

    _stream << "]";

    return _stream;
}
}   // namespace kd_tree

#endif  // NODE_H_
