/* FortuneAlgorithm
 * Copyright (C) 2018 Pierre Vigier
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "Vector2.h"
// STL
#include <cmath>

Vector2::Vector2(double x, double y) : x(x), y(y)
{

}

// Unary operators

Vector2 Vector2::operator-() const
{
    return Vector2(-x, -y);
}

Vector2& Vector2::operator+=(const Vector2& other)
{
    x += other.x;
    y += other.y;
    return *this;
}

Vector2& Vector2::operator-=(const Vector2& other)
{
    x -= other.x;
    y -= other.y;
    return *this;
}

Vector2& Vector2::operator*=(double t)  
{
    x *= t;
    y *= t;
    return *this; 
}

// Other operations

Vector2 Vector2::getOrthogonal() const
{
    return Vector2(-y, x);
}

double Vector2::dot(const Vector2& other) const
{
    return x * other.x + y * other.y;
}

double Vector2::getNorm() const
{
    return std::sqrt(x * x + y * y);
}

double Vector2::getDistance(const Vector2& other) const
{
    return (*this - other).getNorm();
}

double Vector2::getDet(const Vector2& other) const
{
    return x * other.y - y * other.x;
}

Vector2 operator+(Vector2 lhs, const Vector2& rhs)
{
    lhs += rhs;
    return lhs;
}

Vector2 operator-(Vector2 lhs, const Vector2& rhs) 
{
    lhs -= rhs;
    return lhs;
}

Vector2 operator*(double t, Vector2 vec)
{
    vec *= t;
    return vec;
}

Vector2 operator*(Vector2 vec, double t)
{
    return t * vec;
}

std::ostream& operator<<(std::ostream& os, const Vector2& vec)
{
    os << "(" << vec.x << ", " << vec.y << ")";
    return os;
}

