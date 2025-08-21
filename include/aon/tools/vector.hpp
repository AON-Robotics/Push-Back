/**
 * \file vector.hpp
 *
 * \brief Contains implementation of 2d physical vector.
 *
 * \details Can be used as cartesian coordinate and polar coordinate, making
 *      sums, substractions, products, and divisions between scalars and vectors
 *      and vectors and vectors much easier.
 *
 * \warning NOT to be confused with C++'s `vector` containers. This is why this
 *     class is inside the `aon` namespace.
 *
 */
#ifndef AON_TOOLS_VECTOR_HPP_
#define AON_TOOLS_VECTOR_HPP_

#include <cmath>
#include <string>
#include <iostream>

namespace aon {

// ============================================================================
//      _             _
//     /_\  _ _  __ _| |___
//    / _ \| ' \/ _` | / -_)
//   /_/ \_\_||_\__, |_\___|
//              |___/
// ============================================================================
//
/**
 * \class Angle
 *
 * \brief Class that will make working with different angle units much easier
 *
 * \code{.cpp}

int main(){
  aon::Angle a1 = aon::Angle().SetDegrees(-135);
  aon::Angle a2 = aon::Angle().SetRadians(1);

  std::cout << a1 + a2 << std::endl;              // -77.70 deg
  std::cout << a1 - a2 << std::endl;              // -192.29 deg
  std::cout << (a1 == a2) << std::endl;           // false
  std::cout << (a1 < a2) << std::endl;            // true
  std::cout << (a1 > a2) << std::endl;            // false
  std::cout << int(a2) << std::endl;              // -57
  std::cout << float(a2) << std::endl;            // -57.2958
  std::cout << double(a2) << std::endl;           // -57.2958

  return 0;
}

 * \endcode
 */
class Angle {
 private:
  //> Stored angle in units of degrees
  double degrees = 0;
  //> Stored angle in units of radians
  double radians = 0;

 public:
  /**
   * \brief Set the angle in units of degrees.
   *
   * \param degrees Angle in units of degrees
   *
   * \returns This angle object so it can be initialized as it is created. See
   *     example.
   *
   * \code{.cpp}

int main(){
  aon::Angle direction = aon::Angle().SetDegrees(45);
  std::cout << direction << std::endl; // 45.00 deg

  return 0;
}

   * \endcode
   */
  Angle SetDegrees(double degrees) {
    this->degrees = degrees;
    radians = M_PI * degrees / 180.0;

    return (*this);
  }

  /**
   * \brief Set the angle in units of radians.
   *
   * \param radians Angle in units of radians
   *
   * \returns This angle object so it can be initialized as it is created. See
   *     example.
   *
   * \code{.cpp}

int main(){
  aon::Angle direction = aon::Angle().SetRadians(M_PI / 4.0);
  std::cout << direction << std::endl; // 45.00 deg

  return 0;
}

   * \endcode
   */
  Angle SetRadians(double radians) {
    this->radians = radians;
    degrees = 180.0 * radians / M_PI;

    return (*this);
  }

  /**
   * \brief Set the angle using 2d cartesian point.
   *
   * \param x X component of cartesian point
   * \param y Y component of cartesian point
   *
   * \returns This angle object so it can be initialized as it is created. See
   *     example.
   *
   * \code {.cpp}

int main(){
  aon::Angle direction = aon::Angle().SetPos(2, 4);
  std::cout << direction << std::endl; // 63.43 deg

  return 0;
}

  * \endcode
  */
  Angle SetPos(double x, double y) { return SetRadians(std::atan2(y, x)); }

  //> Retrieve angle in units of degrees.
  double GetDegrees() { return degrees; }

  //> Retrieve angle in units of radians.
  double GetRadians() { return radians; }

  /**
   * \brief Convert object to string using precision of 2 decimal points.
   *
   * \returns Angle string in degrees with suffix of " deg".
   *
   * \code{.cpp}

int main(){
  aon::Angle direction = aon::Angle().SetDegrees(135);

  std::cout << std::string(direction) << std::endl; // 135.00 deg

  return 0;
}

   * \endcode
   */
  operator std::string() {
    // How many digits do we want to retrieve.
    const int precision = 2;

    std::string str = std::to_string(GetDegrees());

    // Finds period separating decimal part and extracts substring from index 0
    // up to "precision" units to the right of the period.
    str = str.substr(0, str.find(".") + precision + 1);

    return str + " deg";
  }

  /**
   * \brief Adds Angle object and returns new one.
   *
   * \param rhs Right hand side
   *
   * \returns New Angle object with sum of angles.
   *
   * \note There is no need to literally call this method. See the example.
   *
   * \code{.cpp}

int main(){
  aon::Angle angle1 = aon::Angle().SetDegrees(45);
  aon::Angle angle2 = aon::Angle().SetDegrees(25);

  std::cout << angle1+ angle2) << std::endl; // 70.00 deg

  return 0;
}

   * \endcode
   */
  Angle operator+(Angle rhs) {
    return Angle().SetDegrees(GetDegrees() + rhs.GetDegrees());
  }

  /**
   * \brief Substract Angle objects and return a new one.
   *
   * \param rhs Right hand side
   *
   * \returns New Angle object with difference of angles.
   *
   * \note There is no need to literally call this method. See the example.
   *
   * \code{.cpp}

int main(){
  aon::Angle angle1 = aon::Angle().SetDegrees(45);
  aon::Angle angle2 = aon::Angle().SetDegrees(25);

  std::cout << angle1- angle2) << std::endl; // 20.00 deg

  return 0;
}

  * \endcode
  */
  Angle operator-(Angle rhs) {
    return Angle().SetDegrees(GetDegrees() - rhs.GetDegrees());
  }

  /**
   * \brief Add another Angle object to this one and store it here.
   *
   * \param rhs Right hand side
   *
   * \returns This Angle object so operation can be chained.
   *
   * \note There is no need to literally call this method. See the example.
   *
   * \code{.cpp}

int main(){
  aon::Angle direction = aon::Angle().SetDegrees(40);
  direction += aon::Angle().SetDegrees(50);

  std::cout << direction << std::endl; // 90.00 deg

  return 0;
}

  * \endcode
  */
  Angle &operator+=(Angle rhs) {
    (*this) = (*this) + rhs;
    return *this;
  }

  /**
   * \brief Substract another Angle object to this one and store it here.
   *
   * \param rhs Right hand side
   *
   * \returns This Angle object so operation can be chained.
   *
   * \note There is no need to literally call this method. See the example.
   *
   * \code{.cpp}

int main(){
  aon::Angle direction = aon::Angle().SetDegrees(40);
  direction -= aon::Angle().SetDegrees(50);

  std::cout << direction << std::endl; // -10.00 deg

  return 0;
}

  * \endcode
  */
  Angle &operator-=(Angle rhs) {
    (*this) = (*this) - rhs;
    return *this;
  }

  //> "Equals" comparison operator for Angle objects.
  bool operator==(Angle &rhs) { return (GetDegrees() == rhs.GetDegrees()); }
  //> "Less Than" comparison operator for Angle objects.
  bool operator<(Angle &rhs) { return (GetDegrees() < rhs.GetDegrees()); }
  //> "Not Equals" comparison operator for Angle objects.
  bool operator!=(Angle &rhs) { return (!(*this == rhs)); }
  //> "Less than or Equal to" comparison operator for Angle objects.
  bool operator<=(Angle &rhs) { return (*this == rhs || *this < rhs); }
  //> "Greater than" comparison operator for Angle objects.
  bool operator>(Angle &rhs) { return (!(*this <= rhs)); }
  //> "Greater than or Equal to" comparison operator for Angle objects.
  bool operator>=(Angle &rhs) { return (!(*this < rhs)); }

  //> Integer conversion operator for Angle object. Returns degrees.
  operator int() { return static_cast<int>(GetDegrees()); }
  //> Floating point conversion operator for Angle object. Returns degrees.
  operator float() { return static_cast<float>(GetDegrees()); }
  //> Double conversion operator for Angle object. Returns degrees.
  operator double() { return GetDegrees(); }
};

/**
 * \brief Operator overload for simple use of Angle with std::cout
 *
 * \param out Output stream (usually std::cout)
 * \param direction Angle object we want to print out
 *
 * \returns Updated output stream.
 *
 * \see Angle
 */
std::ostream &operator<<(std::ostream &out, Angle direction) {
  return out << std::string(direction);
}

// ============================================================================
//   __   __      _
//   \ \ / /__ __| |_ ___ _ _
//    \ V / -_) _|  _/ _ \ '_|
//     \_/\___\__|\__\___/_|
//
// ============================================================================

/**
 * \class Vector
 *
 * \brief Represents a 2d physical vector (quantity that hase magnitude and
 *     direction).
 *
 * \details Represents vector both in cartesian and polar coordinates.
 *     Therefore, the literal magnitude and directions are stored as are its "x"
 *     component and "y" component.
 * \code{.cpp}
int main(){
  aon::Vector v1 = aon::Vector().SetPos(3, 4);
  aon::Vector v2 = aon::Vector().SetMagnitude(5).SetDegrees(90);

  std::cout << v1 << std::endl;            // MAG: 5.0, ANG: 53.13 deg
  std::cout << v2 << std::endl;            // MAG: 5.0, ANG: 90.00 deg
  std::cout << v1 + v2 << std::endl;       // MAG: 9.4, ANG: 71.56 deg
  std::cout << v1 - v2 << std::endl;       // MAG: 3.1, ANG: -18.43 deg
  std::cout << v1.Dot(v2) << std::endl;    // 20
  std::cout << (v1 == v2) << std::endl;    // false
  std::cout << (v1 != v2) << std::endl;    // true

  return 0;
}
 * \endcode
 */
class Vector {
 private:
  //> X component of 2d vector.
  double x = 0;
  //> Y component of 2d vector.
  double y = 0;
  //> Magnitude of the vector.
  double magnitude = 0;
  //> Direction of the vector.
  Angle *direction = new Angle();

 public:
  /**
   * \brief Set the X component and update magnitude and direction.
   *
   * \param x The new value for the X component
   *
   * \returns This object so it can be initialized as it is created. See
   *     example.
   *
   * \code{.cpp}

int main(){
  aon::Vector vector = aon::Vector().SetX(3);

  std::cout << vector << std::endl;   // MAG: 3.0, ANG: 0.00 deg

  return 0;
}

   * \endcode
   */
  Vector SetX(double x) {
    this->x = x;
    magnitude = std::hypot(x, y);
    direction->SetRadians(std::atan2(y, x));

    return *this;
  }

  /**
   * \brief Set the Y component and update magnitude and direction.
   *
   * \param y The new value for the Y component
   *
   * \returns This object so it can be initialized as it is created. See
   *     example.
   *
   * \code{.cpp}

int main(){
  aon::Vector vector = aon::Vector().SetY(4);

  std::cout << vector << std::endl;   // MAG: 4.0, ANG: 90.00 deg

  return 0;
}

   * \endcode
   */
  Vector SetY(double y) {
    this->y = y;
    magnitude = std::hypot(x, y);
    direction->SetRadians(std::atan2(y, x));

    return *this;
  }

  /**
   * \brief Set both X and Y components and update magnitude and direction.
   *
   * \param x The new value for the X component
   * \param y The new value for the Y component
   *
   * \returns This object so it can be initialized as it is created. See
   *     example.
   *
   *
   * \code{.cpp}

int main(){
  // Equivalent to `aon::Vector vector = aon::Vector().SetX(3).SetY(4);`
  aon::Vector vector = aon::Vector().SetPos(3, 4);

  std::cout << vector << std::endl;   // MAG: 5.0, ANG: 53.13 deg

  return 0;
}

   * \endcode
   */
  Vector SetPosition(double x, double y) {
    this->x = x;
    this->y = y;
    magnitude = std::hypot(x, y);
    direction->SetRadians(std::atan2(y, x));

    return *this;
  }

  //> Shorthand for Vector::SetPosition()
  Vector SetPos(double x, double y) { return SetPosition(x, y); }

  /**
   * \brief Set the vector's magnitude and update the X and Y components.
   *
   * \param magnitude The vector's new magnitude
   *
   * \returns This object so it can be initialized as it is created. See
   *     example.
   *
   * \code{.cpp}

int main(){
  aon::Vector vector = aon::Vector().SetMagnitude(6);

  std::cout << vector << std::endl;   // MAG: 6.0, ANG: 0.00 deg

  return 0;
}

   * \endcode
   */
  Vector SetMagnitude(double magnitude) {
    this->magnitude = abs(magnitude);
    x = magnitude * std::cos(direction->GetRadians());
    y = magnitude * std::sin(direction->GetRadians());

    return *this;
  }

  //> Shorthand for Vector::SetMagnitude()
  Vector SetMag(double value) { return SetMagnitude(value); }

  /**
   * \brief Set the vector's direction and update the X and Y components.
   *
   * \param direction The new direction as an Angle object's pointer
   *
   * \returns This object so it can be initialized as it is created. See
   *     example.
   *
   * \note This method is mainly intended for internal use, but it's still
   *     public for convenience.
   *
   * \code{.cpp}

int main(){
  aon::Angle direction = aon::Angle().SetDegrees(30);
  aon::Vector vector = aon::Vector().SetDirection(&direction);

  std::cout << vector << std::endl;   // MAG: 0.0, ANG: 30.00 deg

  return 0;
}

   * \endcode
   */
  Vector SetDirection(Angle *direction) {
    this->direction = new Angle();
    this->direction->SetDegrees(direction->GetDegrees());

    x = magnitude * std::cos(direction->GetRadians());
    y = magnitude * std::sin(direction->GetRadians());

    return *this;
  }

  /**
   * \brief Set the vector's direction and update the X and Y components.
   *
   * \param degrees The new direction in units of degrees
   *
   * \returns This object so it can be initialized as it is created. See
   *     example.
   *
   * \code{.cpp}

int main(){
  aon::Vector vector = aon::Vector().SetDegrees(60);

  std::cout << vector << std::endl;   // MAG: 0.0, ANG: 60.00 deg

  return 0;
}

   * \endcode
   */
  Vector SetDegrees(double degrees) {
    Angle *angle = new Angle();
    angle->SetDegrees(degrees);
    SetDirection(angle);

    return *this;
  }

  /**
   * \brief Set the vector's direction and update the X and Y components.
   *
   * \param radians The new direction in units of radians
   *
   * \returns This object so it can be initialized as it is created. See
   *     example.
   *
   * \code{.cpp}

int main(){
  aon::Vector vector = aon::Vector().SetRadians(2.0);

  std::cout << vector << std::endl;   // MAG: 0.0, ANG: 114.59 deg

  return 0;
}

   * \endcode
   */
  Vector SetRadians(double radians) {
    Angle *angle = new Angle();
    angle->SetRadians(radians);
    SetDirection(angle);

    return *this;
  }

  //> Retrieve X component of vector.
  double GetX() { return x; }
  //> Retrieve Y component of vector.
  double GetY() { return y; }
  //> Retrieve magnitude of vector.
  double GetMagnitude() { return magnitude; }
  //> Retrieve direction of vector in degrees.
  double GetDegrees() { return direction->GetDegrees(); }
  //> Retrieve direction of vector in radians.
  double GetRadians() { return direction->GetRadians(); }

  /**
   * \brief Normalize the vector and return a copy.
   *
   * \returns Vector with magnitude 1, but the same direction as this one.
   *
   * \code{.cpp}

int main(){
  aon::Vector vector = aon::Vector().SetPos(3, 4);

  std::cout << vector.Normalize() << std::endl;   // MAG: 1.0, ANG: 53.13 deg

  return 0;
}

   * \endcode
   */
  Vector Normalize() {
    return Vector().SetMagnitude(1).SetDegrees(direction->GetDegrees());
  }

  /**
   * \brief Calculate the dot product of this vector and the right hand side.
   *
   * \details This dot product, or scalar product, is geometrically interpreted
   *      as the magnitude of the projection of this vector over vector `rhs`.
   *
   * \param rhs Right hand side (i.e. the "destination" of the projection)
   *
   * \returns The scalar product of this vector over `rhs`.
   *
   * \code{.cpp}

int main(){
  aon::Vector v1 = aon::Vector().SetPos(3, 4);
  aon::Vector v2 = aon::Vector().SetPos(5, 12);

  std::cout << v1.Dot(v2) << std::endl;   // 3*5 + 4*12 = 63

  return 0;
}

   * \endcode
   *
   */
  double Dot(Vector rhs) { return (GetX() * rhs.GetX() + GetY() * rhs.GetY()); }

  /**
   * \brief Uses dot product identity to calculate the angle form target object to parameter
   * 
   * \param v The vector whose angle relative to the target vector we want to find
   * 
   * \returns The angle from the target vector to the parameter one in \b degrees
   * 
   * \note The acos() function returns an angle in \b radians, conversion to \b degrees is necessary
   */
  double getAngleTo(Vector v){ return (acos(this->Dot(v) / (this->GetMagnitude() * v.GetMagnitude())) * 180 / M_PI);}

  /**
   * \brief Term by term addition of vector components.
   *
   * \param rhs Right hand side
   *
   * \returns New Vector object
   *
   * \note There is no need to literally call this method. See the example.
   *
   * \code{.cpp}

int main(){
  aon::Vector v1 = aon::Vector().SetPos(3, 4);
  aon::Vector v2 = aon::Vector().SetPos(5, 12);

  std::cout << v1 + v2 << std::endl;   // MAG: 17.8, ANG: 63.43 deg

  return 0;
}

   * \endcode
   */
  Vector operator+(Vector rhs) {
    return Vector().SetPosition(x + rhs.GetX(), y + rhs.GetY());
  }

  /**
   * \brief Term by term substraction of vector components.
   *
   * \param rhs Right hand side
   *
   * \returns New Vector object
   *
   * \note There is no need to literally call this method. See the example.
   *
   * \code{.cpp}

int main(){
  aon::Vector v1 = aon::Vector().SetPos(3, 4);
  aon::Vector v2 = aon::Vector().SetPos(5, 12);

  std::cout << v1 - v2 << std::endl;   // MAG: 8.2, ANG: -104.03 deg

  return 0;
}

   * \endcode
   */
  Vector operator-(Vector rhs) {
    return Vector().SetPosition(x - rhs.GetX(), y - rhs.GetY());
  }

  /**
   * \brief Vector product, but treating vectors as polar coordinates.
   *
   * \details The vector product or cross product is not appropriate for this
   *     class since it's result appears in the 3rd dimension and we're just
   *     focusing on 2 dimensions. Therefore, I chose to implement this product
   *     as a product of polar coordinates, where the magnitudes are multiplied
   *     and the angles are added.
   *
   * \param rhs Right hand side
   *
   * \returns New Vector object
   *
   * \note There is no need to literally call this method. See the example.
   *
   * \code{.cpp}

int main(){
  aon::Vector v1 = aon::Vector().SetPos(3, 4);
  aon::Vector v2 = aon::Vector().SetPos(5, 12);

  std::cout << v1 * v2 << std::endl;   // MAG: 65.0, ANG: 120.51

  return 0;
}

   * \endcode
   */
  Vector operator*(Vector rhs) {
    return Vector()
        .SetMagnitude(magnitude * rhs.GetMagnitude())
        .SetDegrees(GetDegrees() + rhs.GetDegrees());
  }

  /**
   * \brief Multiply vector by scalar. Only affects magnitude.
   *
   * \param scalar Scalar multiple
   *
   * \returns New Vector object
   *
   * \note There is no need to literally call this method. See the example.
   *
   * \warning There will be errors if the scalar is anything other than a double
   *
   * \code{.cpp}

int main(){
  aon::Vector vector = aon::Vector().SetPos(10, 10);

  std::cout << vector * 2.0 << std::endl;   // MAG: 28.2, ANG: 45.00 deg

  return 0;
}

   * \endcode
   */
  Vector operator*(double scalar) {
    return Vector().SetMagnitude(scalar * magnitude).SetDegrees(GetDegrees());
  }

  /**
   * \brief Vector division, but treating vectors as polar coordinates.
   *
   * \details Since the product operator is being interpeted as multiplying
   *     polar coordinates, I only saw it fitting that they should be able to be
   *     divided too.
   *
   * \param rhs Right hand side
   *
   * \returns New Vector object
   *
   * \note There is no need to literally call this method. See the example.
   *
   * \code{.cpp}

int main(){
  aon::Vector v1 = aon::Vector().SetPos(3, 4);
  aon::Vector v2 = aon::Vector().SetPos(5, 12);

  std::cout << v2 / v1 << std::endl;   // MAG: 2.6, ANG: 14.25 deg

  return 0;
}

   * \endcode
   */
  Vector operator/(Vector rhs) {
    return Vector()
        .SetMagnitude(magnitude / rhs.GetMagnitude())
        .SetDegrees(GetDegrees() - rhs.GetDegrees());
  }

  /**
   * \brief Divide vector by scalar. Only affects magnitude.
   *
   * \param scalar Scalar divisor
   *
   * \returns New Vector object
   *
   * \note There is no need to literally call this method. See the example.
   *
   * \warning There will be errors if the scalar is anything other than a double
   *
   * \code{.cpp}

int main(){
  aon::Vector vector = aon::Vector().SetPos(10, 10);

  std::cout << vector / 2.0 << std::endl;   // MAG: 7.0, ANG: 45.00 deg

  return 0;
}

   * \endcode
   */
  Vector operator/(double scalar) {
    return Vector().SetMagnitude(magnitude / scalar).SetDegrees(GetDegrees());
  }

  //> Compound addition operator derived from the simple addition operator.
  Vector &operator+=(Vector rhs) {
    (*this) = (*this) + rhs;
    return *this;
  }

  //> Compound substraction operator derived from the difference operator.
  Vector &operator-=(Vector rhs) {
    (*this) = (*this) - rhs;
    return *this;
  }

  //> Compound product operator interpreting vector as polar point.
  Vector &operator*=(Vector rhs) {
    (*this) = (*this) * rhs;
    return *this;
  }

  //> Compound product operator to amplify its magnitude.
  Vector &operator*=(double scalar) {
    (*this) = (*this) * scalar;
    return *this;
  }

  //> Compound division operator interpreting vector as polar point.
  Vector &operator/=(Vector rhs) {
    (*this) = (*this) / rhs;
    return *this;
  }

  //> "Equals" comparison operator for Vector objects.
  bool operator==(Vector &rhs) { return (x == rhs.GetX() && y == rhs.GetY()); }
  //> "Not Equals" comparison operator for Vector objects.
  bool operator!=(Vector &rhs) { return (!(*this == rhs)); }

  /**
   * \brief Convert object to string.
   *
   * \returns Vector string with magnitude and direction in degrees.
   *
   * \code{.cpp}

int main(){
  aon::Vector vector = aon::Vector().SetPos(8, 15);

  std::cout << std::string(vector) << std::endl;   // MAG: 17.0, ANG: 61.92 deg

  return 0;
}

   * \endcode
   */
  operator std::string() {
    // How many digits do we want to retrieve.
    const int precision = 1;

    std::string str = std::to_string(GetMagnitude());

    // Finds period separating decimal part and extracts substring from index 0
    // up to "precision" units to the right of the period.
    str = str.substr(0, str.find(".") + precision + 1);

    return "" + str + " ∠ " + std::string(*direction);
  }

  //> Integer conversion operator for Vector object. Returns magnitude.
  operator int() { return static_cast<int>(magnitude); }
  //> Float conversion operator for Vector object. Returns magnitude.
  operator float() { return static_cast<float>(magnitude); }
  //> Double conversion operator for Vector object. Returns magnitude.
  operator double() { return magnitude; }
};

/**
 * \brief Operator overload for simple use of Vector with std::cout
 *
 * \param out Output stream (usually std::cout)
 * \param v Vector object we want to print out
 *
 * \returns Updated output stream.
 *
 * \see Vector
 */
std::ostream &operator<<(std::ostream &out, Vector v) {
  return out << std::string(v);
}

//> Extends the vector's scalar mutliplication so it's commutative.
Vector operator*(double scalar, Vector vector) { return vector * scalar; }

}  // namespace aon

#endif  // AON_TOOLS_VECTOR_HPP_
