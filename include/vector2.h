/*! \file vector2.h Contains the Vector2 class; a two-dimensional vector, and some operations on a vector. */

#ifndef __VECTOR_H__
#define __VECTOR_H__

#include <iostream>
#include <cmath>

namespace RVO {
  /*! Defines a two-dimensional vector and its operations. */
  class Vector2 {

  private:
    /*! The x-component of the vector.向量的x分量 */
    float _x;
    /*! The y-component of the vector.向量的y分量 */
    float _y;

  public:
    /*! \returns The x-component of the vector.返回x分量 */
    inline float x() const { return _x; }
    /*! \returns The y-component of the vector.返回y分量 */
    inline float y() const { return _y; }

    /*! Constructs a null vector.构造一个空的向量 */
    inline Vector2() { _x = 0; _y = 0; }
    /*! Copy constructor.复制构造函数
        \param q The vector to be copied into the new vector. 要复制到新的向量中的向量*/
    inline Vector2(const Vector2& q) { _x = q.x(); _y = q.y(); }
    /*! Constructor.构造函数
        \param x The x-component of the new vector.
        \param y The y-component of the new vector. */
    inline Vector2(float x, float y) { _x = x; _y = y; }

    /*! Unary minus.一元减号
        \returns The negation of the vector. 返回向量的对立面 */
    inline Vector2 operator-() const { return Vector2(-_x, -_y); }
    /*! Unary plus.
        \returns A reference to the vector.对向量的引用  */
    inline const Vector2& operator+() const { return (*this); }

    /*! Dot product.数量积（点积）
        \param q The right hand side vector右边向量
        \returns The dot product of the lhs vector and the rhs vector. 
		返回：lhs向量和rhs向量的点积*/
    inline float operator*(const Vector2& q) const { return _x * q.x() + _y * q.y(); }
    /*! Scalar product.标量积
        \param a The right hand side scalar右边标量
        \returns The scalar product of the lhs vector and the rhs scalar.返回向量和标量的标量积  */
    inline Vector2 operator*(float a) const { return Vector2(_x * a, _y * a); }
    /*! Scalar division.标量除
        \param a The right hand side scalar
        \returns The scalar division of the lhs vector and the rhs scalar.  */
    inline Vector2 operator/(float a) const { float inv_a = 1.0f/a; return Vector2(_x * inv_a, _y * inv_a); }
    /*! Vector addition.向量加法
        \param q The right hand side vector
        \returns The sum of the lhs vector and the rhs vector.  */
    inline Vector2 operator+(const Vector2& q) const { return Vector2(_x + q.x(), _y + q.y()); }
    /*! Vector subtraction.向量减法
        \param q The right hand side vector
        \returns The vector difference of the lhs vector and the rhs vector.  */
    inline Vector2 operator-(const Vector2& q) const { return Vector2(_x - q.x(), _y - q.y()); }

    /*! Vector equality.向量相等
        \param q The right hand side vector
        \returns True if the lhs vector and the rhs vector are equal. False otherwise.  */
    inline bool operator==(const Vector2& q) const { return (_x == q.x() && _y == q.y()); }
    /*! Vector inequality.向量不等于
        \param q The right hand side vector
        \returns True if the lhs vector and the rhs vector are not equal. False otherwise.
		不想等返回true*/
    inline bool operator!=(const Vector2& q) const { return (_x != q.x() || _y != q.y()); }

    /*! The operator multiplies the vector by a scalar.向量与标量的算符乘法
        \param a The scalar
        \returns A reference to the vector. 返回对向量的引用 */
    inline Vector2& operator*=(float a) { _x *= a; _y *= a; return *this; }
    /*! The operator divides the vector by a scalar.向量与标量的算符除法
        \param a The scalar
        \returns A reference to the vector.返回向量的引用  */
    inline Vector2& operator/=(float a) { float inv_a = 1.0f/a; _x *= inv_a; _y *= inv_a; return *this; }
    /*! The operator adds an rhs vector to the vector.向量与向量的算符加
        \param q The right hand side vector
        \returns A reference to the vector.返回向量的引用  */
    inline Vector2& operator+=(const Vector2& q) { _x += q.x(); _y += q.y(); return *this; }
    /*! The operator subtracts an rhs vector from the vector.向量与向量的算符减法
        \param q The right hand side vector
        \returns A reference to the vector.返回向量的引用  */
    inline Vector2& operator-=(const Vector2& q) { _x -= q.x(); _y -= q.y(); return *this; }
  };
}

/*! Scalar multiplication.标量乘法
    \param a The left hand side scalar左边标量
    \param q The right hand side vector右边向量
    \returns The scalar multiplication of the lhs scalar and the rhs vector.  */
inline RVO::Vector2 operator*(float a, const RVO::Vector2& q) { return RVO::Vector2(a * q.x(), a * q.y()); }

/*! Writes a vector to the standard output.将向量写到标准输出
    \param os The output stream输出流
    \param q The vector向量
    \returns The standard output.返回标准输出  */
inline std::ostream& operator<<(std::ostream& os, const RVO::Vector2& q) {
  //os << "(" << q.x() << "," << q.y() << ")";
  os << q.x() << " " << q.y();
  return os;
}

/*! \param q A vector一个向量
    \returns The squared absolute value of the vector.向量平方的绝对值  */
inline float absSq(const RVO::Vector2& q) { return q*q; }
/*! \param q A vector
    \returns The absolute value of the vector.返回向量的绝对值  */
inline float abs(const RVO::Vector2& q) { return std::sqrt(absSq(q)); }
/*! \param q A vector
    \returns The normalized vector.返回归一化向量  */
inline RVO::Vector2 norm(const RVO::Vector2& q) { return q / abs(q); }
/*! \param p A point
    \param q A point
    \returns The normal vector to the line segment pq.返回线段pq的法向量 */
inline RVO::Vector2 normal(const RVO::Vector2& p, const RVO::Vector2& q) { return norm(RVO::Vector2(q.y() - p.y(), -(q.x() - p.x()))); }
/*! \param q A vector
    \returns The angle the vector makes with the positive x-axis. Is in the range [-PI, PI].
	返回与x轴正方向的向量角。范围【-PI，PI】*/
inline float atan(const RVO::Vector2& q) { return std::atan2(q.y(), q.x()); }
/*! \param p A vector
    \param q A vector
    \returns Returns the determinant of the 2x2 matrix formed by using p as the upper row and q as the lower row.
	返回2×2矩阵的行列式，p作为上面一行，q作为下面一行*/
inline float det(const RVO::Vector2& p, const RVO::Vector2& q) { return p.x()*q.y() - p.y()*q.x(); }

#endif
