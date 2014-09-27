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

#include <view.hpp>

using namespace Eigen;

namespace harmont {

Matrix4f ortho(float left, float right, float bottom, float top, float zNear, float zFar) {
	Matrix4f result = Matrix4f::Identity();
	result(0,0) = 2.f / (right - left);
	result(1,1) = 2.f / (top - bottom);
	result(2,2) = - 2.f / (zFar - zNear);
	result(0,3) = - (right + left) / (right - left);
	result(1,3) = - (top + bottom) / (top - bottom);
	result(2,3) = - (zFar + zNear) / (zFar - zNear);
	return result;
}

Matrix4f ortho(float left, float right, float bottom, float top) {
	Matrix4f result = Matrix4f::Identity();
	result(0,0) = 2.f / (right - left);
	result(1,1) = 2.f / (top - bottom);
	result(2,2) = - 1.f;
	result(0,3) = - (right + left) / (right - left);
	result(1,3) = - (top + bottom) / (top - bottom);
	return result;
}

Matrix4f perspective(float fovy, float aspect,	float near, float far) {
    //float f = 1.f / std::tan(fovy * static_cast<float>(M_PI) / 360.f);
	//Matrix4f result = Matrix4f::Zero();
	//result(0,0) = f / aspect;
	//result(1,1) = f;
	//result(2,2) = (far + near) / (near - far);
	//result(3,2) = - 1.f;
	//result(2,3) = (2.f * far * near) / (near - far);
	//return result;
    float range = static_cast<float>(std::tan((fovy / 2.f) * M_PI / 180.f) * near);	
    float left = -range * aspect;
    float right = range * aspect;
    float bottom = -range;
    float top = range;

    Matrix4f result = Matrix4f::Zero();
    result(0,0) = (2.f * near) / (right - left);
    result(1,1) = (2.f * near) / (top - bottom);
    result(2,2) = - (far + near) / (far - near);
    result(3,2) = - 1.f;
    result(2,3) = - (2.f * far * near) / (far - near);
    return result;
}

Matrix4f frustum( float left, float right, float bottom, float top, float nearVal, float farVal ) {
	Matrix4f result = Matrix4f::Zero();
	result(0,0) = (2.f * nearVal) / (right - left);
	result(1,1) = (2.f * nearVal) / (top - bottom);
	result(2,0) = (right + left) / (right - left);
	result(2,1) = (top + bottom) / (top - bottom);
	result(2,2) = -(farVal + nearVal) / (farVal - nearVal);
	result(3,2) = -1.f;
	result(2,3) = -(2.f * farVal * nearVal) / (farVal - nearVal);
	return result;
}

template <class U>
Vector3f project(const Vector3f& obj, const Matrix4f& model, const Matrix4f& proj, const Matrix<U,4,1>& viewport) {
	Vector4f tmp(obj[0], obj[1], obj[2], 1.f);
	tmp = model * tmp;
	tmp = proj * tmp;

	tmp /= tmp[3];
	tmp = tmp * 0.5f + Vector4f(0.5f,0.5f,0.5f,0.5f);
	tmp[0] = tmp[0] * static_cast<float>(viewport[2]) + static_cast<float>(viewport[0]);
	tmp[1] = tmp[1] * static_cast<float>(viewport[3]) + static_cast<float>(viewport[1]);

	return Vector3f(tmp[0], tmp[1], tmp[2]);
}

template <class U>
Vector3f unproject(const Vector3f& win, const Matrix4f& model, const Matrix4f& proj, const Matrix<U,4,1>& viewport) {
	Matrix4f inverse = (proj * model).inverse();

	Vector4f tmp(win[0], win[1], win[2], 1.f);
	tmp[0] = (tmp[0] - static_cast<float>(viewport[0])) / static_cast<float>(viewport[2]);
	tmp[1] = (tmp[1] - static_cast<float>(viewport[1])) / static_cast<float>(viewport[3]);
	tmp = tmp * 2.f - Vector4f(1.f, 1.f, 1.f, 1.f);

	Vector4f obj = inverse * tmp;
	obj /= obj[3];

	return Vector3f(obj[0], obj[1], obj[2]);
}

} // harmont


template Vector3f harmont::project<int>(const Vector3f& obj, const Matrix4f& model, const Matrix4f& proj, const Matrix<int,4,1>& viewport);
template Vector3f harmont::project<unsigned int>(const Vector3f& obj, const Matrix4f& model, const Matrix4f& proj, const Matrix<unsigned int,4,1>& viewport);
template Vector3f harmont::project<float>(const Vector3f& obj, const Matrix4f& model, const Matrix4f& proj, const Matrix<float,4,1>& viewport);
template Vector3f harmont::project<double>(const Vector3f& obj, const Matrix4f& model, const Matrix4f& proj, const Matrix<double,4,1>& viewport);
template Vector3f harmont::unproject<int>(const Vector3f& win, const Matrix4f& model, const Matrix4f& proj, const Matrix<int,4,1>& viewport);
template Vector3f harmont::unproject<unsigned int>(const Vector3f& win, const Matrix4f& model, const Matrix4f& proj, const Matrix<unsigned int,4,1>& viewport);
template Vector3f harmont::unproject<float>(const Vector3f& win, const Matrix4f& model, const Matrix4f& proj, const Matrix<float,4,1>& viewport);
template Vector3f harmont::unproject<double>(const Vector3f& win, const Matrix4f& model, const Matrix4f& proj, const Matrix<double,4,1>& viewport);
