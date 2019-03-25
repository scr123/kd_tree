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
 * @file state.h
 * @brief State class providing a template for a Nx1 state vector.
 * @author Sean Crutchlow <sean.GH1231@gmail.com>
 * @version 1.0
 */

#ifndef STATE_H_
#define STATE_H_

#include <cmath>
#include <cassert>
#include <initializer_list>
#include <ostream>
#include <vector>

namespace kd_tree {
/**
 * @brief      Class for state.
 *
 * @tparam     T     C++ Fundamental Types
 */
template<typename T>
class State{
 public:
  /**
   * @brief      Constructs the object.
   */
  State() {}

  /**
   * @brief      Constructs the object.
   *
   * @param[in]  _state  The state
   */
  State(const std::initializer_list<T>& _state) : state_(_state) {}

  /**
   * @brief      Constructs the object.
   *
   * @param[in]  _state  The state
   */
  State(const State& _state) : state_(_state.GetState()) {}

  /**
   * @brief      Constructs the object.
   *
   * @param[in]  _n    The size
   * @param[in]  _val  The value
   */
  State(const unsigned _n, const T _val) {
    std::vector<T> vec(_n, _val);
    state_ = vec;
  }

  /**
   * @brief      Subscript operator.
   *
   * @param[in]  _i    Index for accesssor reference
   *
   * @return     Reference to index.
   */
  const T& operator[](int const _i) const {
    return state_.at(_i);
  }

  /**
   * @brief      Equal to operator.
   *
   * @param      _other  The other
   *
   * @return     True if equal, False otherwise.
   */
  bool operator==(State const& _other) const {
    if (this->size() != _other.size())
      return false;

    /// Check MISRA float equality for all state elements
    for (int i = 0; i < this->size(); i++) {
      if (std::fabs(static_cast<float>(this->state_[i] - _other[i])) > 1e-8)
        return false;
    }

    return true;
  }

  /**
   * @brief      Not equal to operator.
   *
   * @param      _other   The other
   *
   * @return     True if not equal, False otherwise.
   */
  bool operator!=(State const& _other) const {
    return !(*this == _other);
  }

  /**
   * @brief      Assignment by sum operator.
   *
   * @param      _other  The other
   *
   * @return     State for assignment.
   */
  State operator+=(State const& _other) {
    assert(this->size() == _other.size());

    for (int i = 0; i < this->size(); ++i)
      this->state_[i] += _other[i];

    return *this;
  }

  /**
   * @brief      Assignment by difference operator.
   *
   * @param      _other  The other
   *
   * @return     State for assignment.
   */
  State operator-=(State const& _other) {
    assert(this->size() == _other.size());

    for (int i = 0; i < this->size(); ++i)
      this->state_[i] -= _other[i];

    return *this;
  }

  friend State operator+(State const& _lhs, State const& _rhs) {
    State State(_lhs);

    State += _rhs;

    return State;
  }

  friend State operator-(State const& _lhs, State const& _rhs) {
    State state(_lhs);

    state -= _rhs;

    return state;
  }

  /**
   * @brief      Gets size.
   *
   * @return     Size of state vector.
   */
  const size_t size() const {
    return this->state_.size();
  }

/**
 * @brief      Gets the state.
 *
 * @return     The state.
 */
  const std::vector<T> GetState() const {
    return this->state_;
  }

 private:
  std::vector<T> state_;
};

/**
 * @brief      Output operator.
 *
 * @param      _stream  The stream
 * @param[in]  _node    The state
 *
 * @tparam     T        Type for data within state.
 *
 * @return     Output stream.
 */
template<typename T>
std::ostream& operator<<(std::ostream& _stream, const State<T>& _state) {
  _stream << "[";
  size_t size = _state.size();

  for (int i = 0; i < size; i++) {
    _stream << _state[i];
    if (i < (size - 1))
      _stream  << ", ";
  }
  _stream << "]";

  return _stream;
}
}  // namespace kd_tree

#endif  // STATE_H_