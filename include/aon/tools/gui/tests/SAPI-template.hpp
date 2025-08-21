#include "../base-gui.hpp"

#ifndef AON_TOOLS_GUI_TESTS_SAPI_TEMPLATE_HPP_
#define AON_TOOLS_GUI_TESTS_SAPI_TEMPLATE_HPP_

namespace aon {
namespace tests {
class AONDisplay {
 public:
  void initiliaze_simplified_gui() {
    rectangle_draw_test();
    circle_draw_test();
  }

 private:
  // DRAW RECTANGLE
  void rectangle_draw_test() {
    int start_time = pros::millis();
    int counter = 0;
    double test, test2;
    while (pros::millis() - start_time < 10 * 1000) {
      pros::screen::set_pen(COLOR_RED);
      // pros::screen::draw_rect(1,1,480,200);
      pros::screen::fill_rect(5, 5, 240, 200);
      pros::screen::set_pen(COLOR_DARK_SEA_GREEN);
      pros::screen::print(TEXT_MEDIUM, 1, "%d", counter);
      std::cout << counter << std::endl;
      counter++;
    }
    pros::screen::erase();
  }
  // DRAW CIRCLE
  void circle_draw_test() {
    int start_time = pros::millis();
    int counter = 0;
    double test, test2;
    while (pros::millis() - start_time < 10 * 1000) {
      pros::screen::set_pen(COLOR_DARK_SEA_GREEN);
      pros::screen::draw_circle(240, 200, 100);
      counter++;
    }
    pros::screen::erase();
  }
};
};  // namespace tests
};  // namespace aon

#endif  // AON_TOOLS_GUI_TESTS_SAPI_TEMPLATE_HPP_
