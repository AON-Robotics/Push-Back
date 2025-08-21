#include "../base-gui.hpp"

#ifndef AON_TOOLS_GUI_TESTS_SAPI_RECTANGLE_HPP_
#define AON_TOOLS_GUI_TESTS_SAPI_RECTANGLE_HPP_

namespace aon {

namespace gui::tests {
// Code from
// https://stackoverflow.com/questions/3018313/algorithm-to-convert-rgb-to-hsv-and-hsv-to-rgb-in-range-0-255-for-both
typedef struct RgbColor {
  unsigned char r;
  unsigned char g;
  unsigned char b;
} RgbColor;

typedef struct HsvColor {
  unsigned char h;
  unsigned char s;
  unsigned char v;
} HsvColor;

RgbColor HsvToRgb(HsvColor hsv) {
  RgbColor rgb;
  unsigned char region, remainder, p, q, t;

  if (hsv.s == 0) {
    rgb.r = hsv.v;
    rgb.g = hsv.v;
    rgb.b = hsv.v;
    return rgb;
  }

  region = hsv.h / 43;
  remainder = (hsv.h - (region * 43)) * 6;

  p = (hsv.v * (255 - hsv.s)) >> 8;
  q = (hsv.v * (255 - ((hsv.s * remainder) >> 8))) >> 8;
  t = (hsv.v * (255 - ((hsv.s * (255 - remainder)) >> 8))) >> 8;

  switch (region) {
    case 0:
      rgb.r = hsv.v;
      rgb.g = t;
      rgb.b = p;
      break;
    case 1:
      rgb.r = q;
      rgb.g = hsv.v;
      rgb.b = p;
      break;
    case 2:
      rgb.r = p;
      rgb.g = hsv.v;
      rgb.b = t;
      break;
    case 3:
      rgb.r = p;
      rgb.g = q;
      rgb.b = hsv.v;
      break;
    case 4:
      rgb.r = t;
      rgb.g = p;
      rgb.b = hsv.v;
      break;
    default:
      rgb.r = hsv.v;
      rgb.g = p;
      rgb.b = q;
      break;
  }

  return rgb;
}

static int counter_t = 0;
static void rectangle_draw_sapi() {
  pros::screen::set_pen(RGB2COLOR(178, 215, 50));
  pros::screen::fill_rect(0, 0, 480, 240);

  pros::screen::set_pen(RGB2COLOR(252, 204, 26));
  for (int i = 0; i < 480; i += 30) {
    for (int j = 0; j < 240; j += 30) {
      pros::screen::fill_circle(i, j, 10);
    }
  }

  pros::screen::set_pen(RGB2COLOR(194, 20, 96));
  pros::screen::print(TEXT_MEDIUM, 1, "Number: %d", counter_t++);

  pros::screen::set_pen(RGB2COLOR(252, 96, 10));
  pros::screen::fill_circle(480 / 2, 240 / 2, 40);

  pros::delay(10);
}
static int color_value = 0;
static void morphing_circle() {
  HsvColor color_change;
  color_change.h = color_value % 359;
  color_change.s = 128;
  color_change.v = 255;

  RgbColor result = HsvToRgb(color_change);
  pros::screen::set_pen(RGB2COLOR(result.r, result.g, result.b));
  pros::screen::fill_circle(480 / 2, 240 / 2, 40);
  color_value++;
  pros::delay(10);
}
};  // namespace gui::tests
};  // namespace aon

#endif  // AON_TOOLS_GUI_TESTS_SAPI_RECTANGLE_HPP_
