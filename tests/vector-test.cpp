#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdlib>
#include <iostream>

#include "aon/tools/vector.hpp"

#define CHECK(expression)                                                     \
  do {                                                                        \
    if (!(expression)) {                                                      \
      std::cerr << __FILE__ << ':' << __LINE__ << " CHECK failed: "           \
                << #expression << '\n';                                       \
      std::exit(1);                                                           \
    }                                                                         \
  } while (false)

namespace {

constexpr double kTolerance = 1e-9;

void checkNear(double actual, double expected,
               double tolerance = kTolerance) {
  if (std::abs(actual - expected) > tolerance) {
    std::cerr << "expected " << expected << ", got " << actual << '\n';
    std::exit(1);
  }
}

void defaultVectorIsZero() {
  aon::Vector vector;
  checkNear(vector.GetX(), 0.0);
  checkNear(vector.GetY(), 0.0);
  checkNear(vector.GetMagnitude(), 0.0);
  checkNear(vector.GetDegrees(), 0.0);
  checkNear(vector.GetRadians(), 0.0);
}

void cartesianSettersPreserveCurrentConversions() {
  aon::Vector vector = aon::Vector().SetPosition(3.0, 4.0);
  checkNear(vector.GetX(), 3.0);
  checkNear(vector.GetY(), 4.0);
  checkNear(vector.GetMagnitude(), 5.0);
  checkNear(vector.GetDegrees(), 53.13010235415598);

  vector.SetX(0.0);
  checkNear(vector.GetMagnitude(), 4.0);
  checkNear(vector.GetDegrees(), 90.0);

  vector.SetY(-4.0);
  checkNear(vector.GetMagnitude(), 4.0);
  checkNear(vector.GetDegrees(), -90.0);
}

void polarSettersPreserveCurrentConversions() {
  aon::Vector vector = aon::Vector().SetMagnitude(5.0).SetDegrees(90.0);
  checkNear(vector.GetX(), 0.0);
  checkNear(vector.GetY(), 5.0);
  checkNear(vector.GetMagnitude(), 5.0);
  checkNear(vector.GetRadians(), M_PI / 2.0);

  vector.SetRadians(M_PI);
  checkNear(vector.GetX(), -5.0);
  checkNear(vector.GetY(), 0.0);
  checkNear(vector.GetDegrees(), 180.0);

  aon::Vector negative = aon::Vector().SetMagnitude(-2.0);
  checkNear(negative.GetMagnitude(), 2.0);
  checkNear(negative.GetX(), -2.0);
  checkNear(negative.GetY(), 0.0);
}

void extendedAnglesRemainUnwrapped() {
  aon::Vector vector = aon::Vector().SetMagnitude(1.0).SetDegrees(450.0);
  checkNear(vector.GetDegrees(), 450.0);
  checkNear(vector.GetRadians(), 5.0 * M_PI / 2.0);
  checkNear(vector.GetX(), 0.0);
  checkNear(vector.GetY(), 1.0);

  vector.SetRadians(-3.0 * M_PI);
  checkNear(vector.GetRadians(), -3.0 * M_PI);
  checkNear(vector.GetDegrees(), -540.0);
  checkNear(vector.GetX(), -1.0);
  checkNear(vector.GetY(), 0.0);
}

void setDirectionCopiesRequiredNonOwningInput() {
  aon::Angle angle = aon::Angle().SetDegrees(30.0);
  aon::Vector vector = aon::Vector().SetMagnitude(2.0).SetDirection(&angle);
  angle.SetDegrees(60.0);

  checkNear(vector.GetDegrees(), 30.0);
  checkNear(vector.GetX(), std::sqrt(3.0));
  checkNear(vector.GetY(), 1.0);
}

void copyMutationDoesNotChangeOriginalDirection() {
  aon::Vector original = aon::Vector().SetPosition(3.0, 4.0);
  aon::Vector copy = original;
  copy.SetPosition(0.0, 5.0);

  checkNear(original.GetX(), 3.0);
  checkNear(original.GetY(), 4.0);
  checkNear(original.GetMagnitude(), 5.0);
  checkNear(original.GetDegrees(), 53.13010235415598);
  checkNear(copy.GetDegrees(), 90.0);

  aon::Vector assigned;
  assigned = original;
  assigned.SetPosition(-5.0, 0.0);
  checkNear(original.GetDegrees(), 53.13010235415598);
  checkNear(assigned.GetDegrees(), 180.0);
}

void arithmeticRetainsExistingMeanings() {
  aon::Vector first = aon::Vector().SetPosition(3.0, 4.0);
  aon::Vector second = aon::Vector().SetPosition(5.0, 12.0);

  aon::Vector sum = first + second;
  checkNear(sum.GetX(), 8.0);
  checkNear(sum.GetY(), 16.0);

  aon::Vector difference = second - first;
  checkNear(difference.GetX(), 2.0);
  checkNear(difference.GetY(), 8.0);

  checkNear(first.Dot(second), 63.0);

  aon::Vector normalized = first.Normalize();
  checkNear(normalized.GetMagnitude(), 1.0);
  checkNear(normalized.GetDegrees(), 53.13010235415598);

  aon::Vector scaled = first * 2.0;
  checkNear(scaled.GetMagnitude(), 10.0);
  checkNear(scaled.GetDegrees(), 53.13010235415598);

  aon::Vector divided = second / first;
  // Preserve the current host result from Vector's unqualified abs call.
  // Correcting its truncation belongs to a separate numeric-behavior change.
  checkNear(divided.GetMagnitude(), 2.0);
  checkNear(divided.GetDegrees(),
            67.38013505195957 - 53.13010235415598);
}

}  // namespace

int main() {
  defaultVectorIsZero();
  cartesianSettersPreserveCurrentConversions();
  polarSettersPreserveCurrentConversions();
  extendedAnglesRemainUnwrapped();
  setDirectionCopiesRequiredNonOwningInput();
  copyMutationDoesNotChangeOriginalDirection();
  arithmeticRetainsExistingMeanings();
  std::cout << "vector tests passed\n";
  return 0;
}
