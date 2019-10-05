/*! \file vector2.h Contains the Vector2 class; a two-dimensional vector, and some operations on a vector. */

#ifndef __VECTOR_H__
#define __VECTOR_H__

#include <iostream>
#include <cmath>

namespace RVO {
  /*! Defines a two-dimensional vector and its operations. */
  class Vector2 {

  private:
    /*! The x-component of the vector.������x���� */
    float _x;
    /*! The y-component of the vector.������y���� */
    float _y;

  public:
    /*! \returns The x-component of the vector.����x���� */
    inline float x() const { return _x; }
    /*! \returns The y-component of the vector.����y���� */
    inline float y() const { return _y; }

    /*! Constructs a null vector.����һ���յ����� */
    inline Vector2() { _x = 0; _y = 0; }
    /*! Copy constructor.���ƹ��캯��
        \param q The vector to be copied into the new vector. Ҫ���Ƶ��µ������е�����*/
    inline Vector2(const Vector2& q) { _x = q.x(); _y = q.y(); }
    /*! Constructor.���캯��
        \param x The x-component of the new vector.
        \param y The y-component of the new vector. */
    inline Vector2(float x, float y) { _x = x; _y = y; }

    /*! Unary minus.һԪ����
        \returns The negation of the vector. ���������Ķ����� */
    inline Vector2 operator-() const { return Vector2(-_x, -_y); }
    /*! Unary plus.
        \returns A reference to the vector.������������  */
    inline const Vector2& operator+() const { return (*this); }

    /*! Dot product.�������������
        \param q The right hand side vector�ұ�����
        \returns The dot product of the lhs vector and the rhs vector. 
		���أ�lhs������rhs�����ĵ��*/
    inline float operator*(const Vector2& q) const { return _x * q.x() + _y * q.y(); }
    /*! Scalar product.������
        \param a The right hand side scalar�ұ߱���
        \returns The scalar product of the lhs vector and the rhs scalar.���������ͱ����ı�����  */
    inline Vector2 operator*(float a) const { return Vector2(_x * a, _y * a); }
    /*! Scalar division.������
        \param a The right hand side scalar
        \returns The scalar division of the lhs vector and the rhs scalar.  */
    inline Vector2 operator/(float a) const { float inv_a = 1.0f/a; return Vector2(_x * inv_a, _y * inv_a); }
    /*! Vector addition.�����ӷ�
        \param q The right hand side vector
        \returns The sum of the lhs vector and the rhs vector.  */
    inline Vector2 operator+(const Vector2& q) const { return Vector2(_x + q.x(), _y + q.y()); }
    /*! Vector subtraction.��������
        \param q The right hand side vector
        \returns The vector difference of the lhs vector and the rhs vector.  */
    inline Vector2 operator-(const Vector2& q) const { return Vector2(_x - q.x(), _y - q.y()); }

    /*! Vector equality.�������
        \param q The right hand side vector
        \returns True if the lhs vector and the rhs vector are equal. False otherwise.  */
    inline bool operator==(const Vector2& q) const { return (_x == q.x() && _y == q.y()); }
    /*! Vector inequality.����������
        \param q The right hand side vector
        \returns True if the lhs vector and the rhs vector are not equal. False otherwise.
		����ȷ���true*/
    inline bool operator!=(const Vector2& q) const { return (_x != q.x() || _y != q.y()); }

    /*! The operator multiplies the vector by a scalar.���������������˷�
        \param a The scalar
        \returns A reference to the vector. ���ض����������� */
    inline Vector2& operator*=(float a) { _x *= a; _y *= a; return *this; }
    /*! The operator divides the vector by a scalar.������������������
        \param a The scalar
        \returns A reference to the vector.��������������  */
    inline Vector2& operator/=(float a) { float inv_a = 1.0f/a; _x *= inv_a; _y *= inv_a; return *this; }
    /*! The operator adds an rhs vector to the vector.�����������������
        \param q The right hand side vector
        \returns A reference to the vector.��������������  */
    inline Vector2& operator+=(const Vector2& q) { _x += q.x(); _y += q.y(); return *this; }
    /*! The operator subtracts an rhs vector from the vector.�������������������
        \param q The right hand side vector
        \returns A reference to the vector.��������������  */
    inline Vector2& operator-=(const Vector2& q) { _x -= q.x(); _y -= q.y(); return *this; }
  };
}

/*! Scalar multiplication.�����˷�
    \param a The left hand side scalar��߱���
    \param q The right hand side vector�ұ�����
    \returns The scalar multiplication of the lhs scalar and the rhs vector.  */
inline RVO::Vector2 operator*(float a, const RVO::Vector2& q) { return RVO::Vector2(a * q.x(), a * q.y()); }

/*! Writes a vector to the standard output.������д����׼���
    \param os The output stream�����
    \param q The vector����
    \returns The standard output.���ر�׼���  */
inline std::ostream& operator<<(std::ostream& os, const RVO::Vector2& q) {
  //os << "(" << q.x() << "," << q.y() << ")";
  os << q.x() << " " << q.y();
  return os;
}

/*! \param q A vectorһ������
    \returns The squared absolute value of the vector.����ƽ���ľ���ֵ  */
inline float absSq(const RVO::Vector2& q) { return q*q; }
/*! \param q A vector
    \returns The absolute value of the vector.���������ľ���ֵ  */
inline float abs(const RVO::Vector2& q) { return std::sqrt(absSq(q)); }
/*! \param q A vector
    \returns The normalized vector.���ع�һ������  */
inline RVO::Vector2 norm(const RVO::Vector2& q) { return q / abs(q); }
/*! \param p A point
    \param q A point
    \returns The normal vector to the line segment pq.�����߶�pq�ķ����� */
inline RVO::Vector2 normal(const RVO::Vector2& p, const RVO::Vector2& q) { return norm(RVO::Vector2(q.y() - p.y(), -(q.x() - p.x()))); }
/*! \param q A vector
    \returns The angle the vector makes with the positive x-axis. Is in the range [-PI, PI].
	������x��������������ǡ���Χ��-PI��PI��*/
inline float atan(const RVO::Vector2& q) { return std::atan2(q.y(), q.x()); }
/*! \param p A vector
    \param q A vector
    \returns Returns the determinant of the 2x2 matrix formed by using p as the upper row and q as the lower row.
	����2��2���������ʽ��p��Ϊ����һ�У�q��Ϊ����һ��*/
inline float det(const RVO::Vector2& p, const RVO::Vector2& q) { return p.x()*q.y() - p.y()*q.x(); }

#endif
