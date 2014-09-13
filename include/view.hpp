/*
 * harmont - c++ opengl wrapper library
 *
 * Written in 2014 by Richard Vock
 *
 * To the extent possible under law, the author(s) have dedicated all copyright and related and neighboring rights to this software to the public domain worldwide.
 * This software is distributed without any warranty.
 *
 * You should have received a copy of the CC0 Public Domain Dedication along with this software.
 * If not, see <http://creativecommons.org/publicdomain/zero/1.0/>.
 *
 */

#ifndef HARMONT_VIEW_HPP_
#define HARMONT_VIEW_HPP_

#include "common.hpp"

namespace harmont {

static Eigen::Matrix4f ortho(float left, float right, float bottom, float top, float zNear, float zFar);

static Eigen::Matrix4f ortho(float left, float right, float bottom, float top);

static Eigen::Matrix4f perspective(float fovy, float aspect, float zNear, float zFar);

static Eigen::Matrix4f frustum( float left, float right, float bottom, float top, float nearVal, float farVal );

template <class U>
static Eigen::Vector3f project(const Eigen::Vector3f& obj, const Eigen::Matrix4f& model, const Eigen::Matrix4f& proj, const Eigen::Matrix<U,4,1>& viewport);

template <class U>
static Eigen::Vector3f unproject(const Eigen::Vector3f& win, const Eigen::Matrix4f& model, const Eigen::Matrix4f& proj, const Eigen::Matrix<U,4,1>& viewport);

} // harmont

#endif /* HARMONT_VIEW_HPP_ */
