#include "../../../../../include/aon/tools/gui/gui-debug.hpp"
#include "../../../../../include/aon/constants.hpp"
#include <cmath>
#include <cstdio>

namespace aon {

// ============================================================================
// Field Mapper — layout constants
// ============================================================================

// Field view: 200×200 px square, anchored top-left with padding
static constexpr int  FM_FX1    = 5;    // field view left edge
static constexpr int  FM_FY1    = 28;   // field view top edge
static constexpr int  FM_FSIZE  = 200;  // pixel width/height of the field square
static constexpr int  FM_FX2    = FM_FX1  + FM_FSIZE;
static constexpr int  FM_FY2    = FM_FY1  + FM_FSIZE;

// Data panel sits to the right of the field view
static constexpr int  FM_DX     = FM_FX2 + 8;  // data panel left edge (x=213)
static constexpr int  FM_DW     = BRAIN_SCREEN_WIDTH - FM_DX - 4; // ~263 px

// Half-field size in inches (6 tiles × TILE_WIDTH / 2)
static constexpr double FIELD_HALF = 6.0 * TILE_WIDTH / 2.0; // ≈ 70.866 in

// ── Coordinate helpers ──────────────────────────────────────────────────────

static int fieldToScreenX(double x) {
  return FM_FX1 + static_cast<int>((x + FIELD_HALF) / (2.0 * FIELD_HALF) * FM_FSIZE);
}

static int fieldToScreenY(double y) {
  // y increases upward on field, downward on screen → flip
  return FM_FY2 - static_cast<int>((y + FIELD_HALF) / (2.0 * FIELD_HALF) * FM_FSIZE);
}

// ── Arc computation ──────────────────────────────────────────────────────────

static void computeArc(const GuiDebug::MapPoint* buf, int start, int end,
                       GuiDebug::ArcResult& out) {
  out = {};
  if (start < 0 || end <= start) return;

  // Arc length = sum of segment distances
  double arcLen = 0.0;
  for (int i = start + 1; i <= end; ++i) {
    double dx = buf[i].x - buf[i - 1].x;
    double dy = buf[i].y - buf[i - 1].y;
    arcLen += std::sqrt(dx * dx + dy * dy);
  }

  // Chord from start to end points
  double cx = buf[end].x - buf[start].x;
  double cy = buf[end].y - buf[start].y;
  double chord = std::sqrt(cx * cx + cy * cy);

  // Heading change (radians → degrees), unwrapped to [0, 360)
  double dTheta = buf[end].theta - buf[start].theta;
  // Normalise to (−π, π]
  while (dTheta >  M_PI) dTheta -= 2.0 * M_PI;
  while (dTheta < -M_PI) dTheta += 2.0 * M_PI;
  double dDeg = std::fabs(dTheta) * (180.0 / M_PI);

  // Radius from arc-length / angle (circular arc assumption)
  double radius = 0.0;
  if (std::fabs(dTheta) > 0.01) {  // > ~0.57° before assuming straight
    radius = arcLen / std::fabs(dTheta);
  }

  out.arcLength    = arcLen;
  out.chordLength  = chord;
  out.deltaHeading = dDeg;
  out.radius       = radius;
  out.valid        = true;
}

// ============================================================================
// Display
// ============================================================================

void GuiDebug::DisplayFieldMapper() {
  pros::screen::set_eraser(COLOR_BLACK);
  pros::screen::erase();

  // ── Header bar ────────────────────────────────────────────────────────────
  // [BACK]  FIELD MAPPER  [CLEAR]
  pros::screen::set_eraser(COLOR_DARK_GRAY);
  pros::screen::erase_rect(5, 2, 70, 24);
  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_SMALL, 14, 8, "BACK");

  pros::screen::set_eraser(COLOR_RED);
  pros::screen::erase_rect(BRAIN_SCREEN_WIDTH - 75, 2, BRAIN_SCREEN_WIDTH - 5, 24);
  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_SMALL, BRAIN_SCREEN_WIDTH - 63, 8, "CLEAR");

  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_MEDIUM, BRAIN_SCREEN_WIDTH / 2 - 55, 5, "FIELD MAPPER");

  // ── Field view ────────────────────────────────────────────────────────────
  // Outer border
  pros::screen::set_eraser(COLOR_DARK_GRAY);
  pros::screen::erase_rect(FM_FX1 - 1, FM_FY1 - 1, FM_FX2 + 1, FM_FY2 + 1);
  // Field background
  pros::screen::set_eraser(0x1A1A1A);
  pros::screen::erase_rect(FM_FX1, FM_FY1, FM_FX2, FM_FY2);

  // Tile grid (6 tiles → 7 lines in each direction)
  pros::screen::set_pen(0x333333);
  for (int i = 0; i <= 6; ++i) {
    int px = FM_FX1 + i * FM_FSIZE / 6;
    int py = FM_FY1 + i * FM_FSIZE / 6;
    pros::screen::draw_line(px, FM_FY1, px, FM_FY2);
    pros::screen::draw_line(FM_FX1, py, FM_FX2, py);
  }

  // Field origin cross (faint)
  pros::screen::set_pen(0x444444);
  int ox = fieldToScreenX(0.0);
  int oy = fieldToScreenY(0.0);
  pros::screen::draw_line(ox - 5, oy, ox + 5, oy);
  pros::screen::draw_line(ox, oy - 5, ox, oy + 5);

  // Path trace
  if (mapBufferCount > 1) {
    // Draw arc segment (between arcStartIndex and end) highlighted in yellow
    for (int i = 1; i < mapBufferCount; ++i) {
      int x0 = fieldToScreenX(mapBuffer[i - 1].x);
      int y0 = fieldToScreenY(mapBuffer[i - 1].y);
      int x1 = fieldToScreenX(mapBuffer[i].x);
      int y1 = fieldToScreenY(mapBuffer[i].y);

      bool inArc = (arcStartIndex >= 0 && i > arcStartIndex);
      pros::screen::set_pen(inArc ? COLOR_YELLOW : COLOR_CYAN);
      pros::screen::draw_line(x0, y0, x1, y1);
    }

    // Arc start marker (green dot)
    if (arcStartIndex >= 0 && arcStartIndex < mapBufferCount) {
      int ax = fieldToScreenX(mapBuffer[arcStartIndex].x);
      int ay = fieldToScreenY(mapBuffer[arcStartIndex].y);
      pros::screen::set_pen(COLOR_GREEN);
      pros::screen::draw_pixel(ax, ay);
      pros::screen::draw_pixel(ax + 1, ay);
      pros::screen::draw_pixel(ax, ay + 1);
      pros::screen::draw_pixel(ax + 1, ay + 1);
    }
  }

  // Robot position + heading arrow (most recent point)
  if (mapBufferCount > 0) {
    const auto& cur = mapBuffer[mapBufferCount - 1];
    int rx = fieldToScreenX(cur.x);
    int ry = fieldToScreenY(cur.y);

    // Heading arrow: theta=0 points in +y direction on field → upward on screen
    double cosH = std::cos(cur.theta);
    double sinH = std::sin(cur.theta);
    int arrowLen = 10;
    // +y on field = -y on screen; +x on field = +x on screen
    int ex = rx + static_cast<int>( sinH * arrowLen);
    int ey = ry - static_cast<int>( cosH * arrowLen);

    pros::screen::set_pen(COLOR_WHITE);
    pros::screen::draw_line(rx, ry, ex, ey);
    // Dot at robot center
    pros::screen::draw_pixel(rx, ry);
    pros::screen::draw_pixel(rx + 1, ry);
    pros::screen::draw_pixel(rx, ry + 1);
    pros::screen::draw_pixel(rx + 1, ry + 1);
  }

  // ── Data panel ────────────────────────────────────────────────────────────
  const int dx = FM_DX;
  int dy = FM_FY1;
  const int rowH = 22;

  // Current pose values (from last map point, or zeros)
  double curX = 0.0, curY = 0.0, curHdgDeg = 0.0;
  if (mapBufferCount > 0) {
    curX     = mapBuffer[mapBufferCount - 1].x;
    curY     = mapBuffer[mapBufferCount - 1].y;
    curHdgDeg = mapBuffer[mapBufferCount - 1].theta * (180.0 / M_PI);
  }

  char buf[40];
  pros::screen::set_pen(COLOR_LIGHT_GRAY);
  pros::screen::print(pros::E_TEXT_SMALL, dx, dy, "X:");
  pros::screen::set_pen(COLOR_GREEN);
  snprintf(buf, sizeof(buf), "%.2f\"", curX);
  pros::screen::print(pros::E_TEXT_SMALL, dx + 20, dy, buf);
  dy += rowH;

  pros::screen::set_pen(COLOR_LIGHT_GRAY);
  pros::screen::print(pros::E_TEXT_SMALL, dx, dy, "Y:");
  pros::screen::set_pen(COLOR_GREEN);
  snprintf(buf, sizeof(buf), "%.2f\"", curY);
  pros::screen::print(pros::E_TEXT_SMALL, dx + 20, dy, buf);
  dy += rowH;

  pros::screen::set_pen(COLOR_LIGHT_GRAY);
  pros::screen::print(pros::E_TEXT_SMALL, dx, dy, "H:");
  pros::screen::set_pen(COLOR_GREEN);
  snprintf(buf, sizeof(buf), "%.1f" "\xb0", curHdgDeg);
  pros::screen::print(pros::E_TEXT_SMALL, dx + 20, dy, buf);
  dy += rowH;

  pros::screen::set_pen(COLOR_LIGHT_GRAY);
  pros::screen::print(pros::E_TEXT_SMALL, dx, dy, "D:");
  pros::screen::set_pen(COLOR_GREEN);
  snprintf(buf, sizeof(buf), "%.2f\"", mapTotalDist);
  pros::screen::print(pros::E_TEXT_SMALL, dx + 20, dy, buf);
  dy += rowH;

  // Separator
  pros::screen::set_eraser(0x333333);
  pros::screen::erase_rect(dx, dy, dx + FM_DW, dy + 1);
  dy += 6;

  // ── Arc results ──────────────────────────────────────────────────────────
  if (arcMeasured && arcResult.valid) {
    pros::screen::set_pen(COLOR_YELLOW);
    pros::screen::print(pros::E_TEXT_SMALL, dx, dy, "ARC:");
    dy += rowH;

    // Radius + ΔHdg on the same row
    pros::screen::set_pen(COLOR_LIGHT_GRAY);
    pros::screen::print(pros::E_TEXT_SMALL, dx, dy, "Radius:");
    pros::screen::set_pen(COLOR_YELLOW);
    if (arcResult.radius > 0.01) {
      snprintf(buf, sizeof(buf), "%.2f\"", arcResult.radius);
    } else {
      snprintf(buf, sizeof(buf), "straight");
    }
    pros::screen::print(pros::E_TEXT_SMALL, dx + 56, dy, buf);
    pros::screen::set_pen(COLOR_LIGHT_GRAY);
    pros::screen::print(pros::E_TEXT_SMALL, dx + 130, dy, "\xce\x94Hdg:");
    pros::screen::set_pen(COLOR_YELLOW);
    snprintf(buf, sizeof(buf), "%.1f" "\xb0", arcResult.deltaHeading);
    pros::screen::print(pros::E_TEXT_SMALL, dx + 170, dy, buf);
    dy += rowH;

    pros::screen::set_pen(COLOR_LIGHT_GRAY);
    pros::screen::print(pros::E_TEXT_SMALL, dx, dy, "ArcLen:");
    pros::screen::set_pen(COLOR_YELLOW);
    snprintf(buf, sizeof(buf), "%.2f\"", arcResult.arcLength);
    pros::screen::print(pros::E_TEXT_SMALL, dx + 56, dy, buf);
    dy += rowH;

    pros::screen::set_pen(COLOR_LIGHT_GRAY);
    pros::screen::print(pros::E_TEXT_SMALL, dx, dy, "Chord:");
    pros::screen::set_pen(COLOR_YELLOW);
    snprintf(buf, sizeof(buf), "%.2f\"", arcResult.chordLength);
    pros::screen::print(pros::E_TEXT_SMALL, dx + 48, dy, buf);
    dy += rowH;
  } else {
    pros::screen::set_pen(0x555555);
    pros::screen::print(pros::E_TEXT_SMALL, dx, dy, "ARC: --");
    dy += rowH * 4;  // reserve same space
  }

  // ── Arc control buttons ──────────────────────────────────────────────────
  const int btnY1 = BRAIN_SCREEN_HEIGHT - 35;
  const int btnY2 = BRAIN_SCREEN_HEIGHT - 5;
  const int halfW = (FM_DW - 6) / 2;

  // [MARK S]
  uint32_t markSColor = (arcStartIndex >= 0) ? COLOR_DARK_GREEN : 0x005500;
  pros::screen::set_eraser(markSColor);
  pros::screen::erase_rect(dx, btnY1, dx + halfW, btnY2);
  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_SMALL, dx + 4, btnY1 + 8,
                      (arcStartIndex >= 0) ? "S SET" : "MARK S");

  // [MARK E]
  uint32_t markEColor = arcMeasured ? COLOR_DARK_BLUE : 0x000055;
  pros::screen::set_eraser(markEColor);
  pros::screen::erase_rect(dx + halfW + 6, btnY1, dx + FM_DW, btnY2);
  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_SMALL, dx + halfW + 10, btnY1 + 8, "MARK E");
}

// ============================================================================
// Touch Handler
// ============================================================================

void GuiDebug::HandleFieldMapperTouch() {
  pros::screen_touch_status_s_t touch = pros::screen::touch_status();
  if (touch.touch_status <= 0) return;

  static uint32_t lastTouchMs = 0;
  uint32_t now = pros::millis();
  if (now - lastTouchMs < 300) return;
  lastTouchMs = now;

  int tx = touch.x;
  int ty = touch.y;

  // ── BACK button (top-left) ────────────────────────────────────────────────
  if (tx >= 5 && tx <= 70 && ty >= 2 && ty <= 24) {
    DisplayDebugMenu();
    currentScreen = DebugMenu;
    return;
  }

  // ── CLEAR button (top-right) ──────────────────────────────────────────────
  if (tx >= BRAIN_SCREEN_WIDTH - 75 && tx <= BRAIN_SCREEN_WIDTH - 5 &&
      ty >= 2 && ty <= 24) {
    ClearMapPath();
    DisplayFieldMapper();
    return;
  }

  // ── MARK START button ─────────────────────────────────────────────────────
  const int btnY1 = BRAIN_SCREEN_HEIGHT - 35;
  const int btnY2 = BRAIN_SCREEN_HEIGHT - 5;
  const int halfW = (FM_DW - 6) / 2;
  const int markSX2 = FM_DX + halfW;
  const int markEX1 = FM_DX + halfW + 6;
  const int markEX2 = FM_DX + FM_DW;

  if (tx >= FM_DX && tx <= markSX2 && ty >= btnY1 && ty <= btnY2) {
    // Set arc start to the most recent path point
    arcStartIndex = (mapBufferCount > 0) ? mapBufferCount - 1 : 0;
    arcMeasured   = false;
    arcResult     = {};
    DisplayFieldMapper();
    return;
  }

  // ── MARK END button ───────────────────────────────────────────────────────
  if (tx >= markEX1 && tx <= markEX2 && ty >= btnY1 && ty <= btnY2) {
    if (arcStartIndex >= 0 && mapBufferCount > arcStartIndex + 1) {
      computeArc(mapBuffer, arcStartIndex, mapBufferCount - 1, arcResult);
      arcMeasured = arcResult.valid;
    }
    DisplayFieldMapper();
    return;
  }
}

}  // namespace aon
