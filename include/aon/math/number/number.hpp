#pragma once
#include <stdexcept>
#include <cmath>

enum class UnitType { NONE, LENGTH, ORIENTATION };

enum class LengthUnits { NONE, INCHES, METERS, CENTIMETERS, MILLIMETERS };

enum class OrientationUnits { NONE, RADIANS, DEGREES };

class Number {
 private:
  double value;
  UnitType category = UnitType::NONE;
  LengthUnits lUnit = LengthUnits::NONE;
  OrientationUnits oUnit = OrientationUnits::NONE;

  // --- conversion helpers ---
  static double toMeters(double v, LengthUnits unit) {
    switch (unit) {
      case LengthUnits::INCHES:
        return v * 0.0254;
      case LengthUnits::CENTIMETERS:
        return v * 0.01;
      case LengthUnits::MILLIMETERS:
        return v * 0.001;
      case LengthUnits::METERS:
        return v;
    }
    throw std::invalid_argument("Unknown LengthUnits");
  }

  static double fromMeters(double v, LengthUnits unit) {
    switch (unit) {
      case LengthUnits::INCHES:
        return v / 0.0254;
      case LengthUnits::CENTIMETERS:
        return v / 0.01;
      case LengthUnits::MILLIMETERS:
        return v / 0.001;
      case LengthUnits::METERS:
        return v;
    }
    throw std::invalid_argument("Unknown LengthUnits");
  }

  static double toRadians(double v, OrientationUnits unit) {
    switch (unit) {
      case OrientationUnits::DEGREES:
        return v * M_PI / 180.0;
      case OrientationUnits::RADIANS:
        return v;
    }
    throw std::invalid_argument("Unknown OrientationUnits");
  }

  static double fromRadians(double v, OrientationUnits unit) {
    switch (unit) {
      case OrientationUnits::DEGREES:
        return v * 180.0 / M_PI;
      case OrientationUnits::RADIANS:
        return v;
    }
    throw std::invalid_argument("Unknown OrientationUnits");
  }

 public:
  // constructors
  Number(double v, LengthUnits unit)
      : value(v), category(UnitType::LENGTH), lUnit(unit) {}

  Number(double v, OrientationUnits unit)
      : value(v), category(UnitType::ORIENTATION), oUnit(unit) {}

  Number(double v = 0) : value(v) {}

  // getters
  double as(LengthUnits unit) const {
    if (category != UnitType::LENGTH)
      throw std::logic_error("Not a length number");
    return fromMeters(value, unit);
  }

  double as(OrientationUnits unit) const {
    if (category != UnitType::ORIENTATION)
      throw std::logic_error("Not an orientation number");
    return fromRadians(value, unit);
  }

  // operators
  Number operator+(const Number &other) {
    if (category != other.category)
      throw std::logic_error("Cannot add different unit categories");
    value = value + other.value;
    return *this;
  }

  Number operator-(const Number &other) {
    if (category != other.category) {
      throw std::logic_error("Cannot subtract different unit categories");
    }
    value = value - other.value;
    return *this;
  }
};
