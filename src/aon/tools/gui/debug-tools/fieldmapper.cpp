#include "../../../../../include/aon/tools/gui/gui-debug.hpp"
#include "../../../../../include/aon/constants.hpp"
#include <cmath>
#include <cstdio>

namespace aon {
// ============================================================================
// Field Mapper — color constants
// ============================================================================

static constexpr uint32_t COLOR_FIELD_BG       = 0x1A1A1A; // field background
static constexpr uint32_t COLOR_GRID           = 0x333333; // tile grid lines
static constexpr uint32_t COLOR_ORIGIN         = 0x444444; // field origin cross
static constexpr uint32_t COLOR_LABEL_DIM      = 0x555555; // placeholder / no-data text
static constexpr uint32_t COLOR_LABEL_HINT     = 0x888888; // SELECT mode hint text
static constexpr uint32_t COLOR_BTN_DISPLACE   = 0x005566; // DISPLACE button
static constexpr uint32_t COLOR_BTN_ARC_IDLE   = 0x005500; // ARC/MARK START inactive
static constexpr uint32_t COLOR_BTN_END_IDLE   = 0x000055; // MARK END inactive

// ============================================================================
// Field Mapper — layout constants
// ============================================================================

// Field view: 200×200 px square, anchored top-left with padding
static constexpr int  FIELDX1    = 5;    // field view left edge
static constexpr int  FIELDY1    = 36;   // field view top edge (below 28px header)
static constexpr int  FSIZE  = 200;  // pixel width/height of the field square
static constexpr int  FIELDX2    = FIELDX1  + FSIZE;
static constexpr int  FIELDY2    = FIELDY1  + FSIZE;

// Data panel sits to the right of the field view
static constexpr int  displayX     = FIELDX2 + 8;  // data panel left edge (x=213)
static constexpr int  displayW     = BRAIN_SCREEN_WIDTH - displayX - 4; // ~263 px

// Half-field size in inches (6 tiles × TILE_WIDTH / 2)
static constexpr double FIELD_HALF = 6.0 * TILE_WIDTH / 2.0; // ≈ 70.866 in

// ── Coordinate helpers ──────────────────────────────────────────────────────

static int fieldToScreenX(double x) {
  return FIELDX1 + static_cast<int>((x + FIELD_HALF) / (2.0 * FIELD_HALF) * FSIZE);
}

static int fieldToScreenY(double y) {
  // y increases upward on field, downward on screen → flip
  return FIELDY2 - static_cast<int>((y + FIELD_HALF) / (2.0 * FIELD_HALF) * FSIZE);
}

// ── Arc computation ──────────────────────────────────────────────────────────

static void computeArc(const Pose* buf, int start, int end,
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
  out.innerRadius  = (radius > 0.01) ? radius - (DRIVE_WIDTH / 2.0) : 0.0;
  out.outerRadius  = (radius > 0.01) ? radius + (DRIVE_WIDTH / 2.0) : 0.0;
  out.valid        = true;
}

// ============================================================================
// DISPLAY FUNCTIONS
// ============================================================================

void GuiDebug::DisplayFieldMapper() {
  pros::screen::set_eraser(COLOR_BLACK);
  pros::screen::erase();

  // Header
  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_MEDIUM, BRAIN_SCREEN_WIDTH / 2 - 55, 10, "FIELD MAPPER");

  // BACK button
  const int backX1 = 10, backY1 = 6, backX2 = backX1 + 80, backY2 = backY1 + 28;
  pros::screen::set_eraser(COLOR_DARK_GRAY);
  pros::screen::erase_rect(backX1, backY1, backX2, backY2);
  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_MEDIUM, backX1 + 10, backY1 + 6, "BACK");

  // CLEAR button
  const int clearX1 = BRAIN_SCREEN_WIDTH - 90, clearY1 = 6, clearX2 = BRAIN_SCREEN_WIDTH - 10, clearY2 = clearY1 + 28;
  pros::screen::set_eraser(COLOR_RED);
  pros::screen::erase_rect(clearX1, clearY1, clearX2, clearY2);
  pros::screen::set_pen(COLOR_WHITE);
  pros::screen::print(pros::E_TEXT_MEDIUM, clearX1 + 10, clearY1 + 6, "CLEAR");

  // ── Field view ────────────────────────────────────────────────────────────
  // Outer border
  pros::screen::set_eraser(COLOR_DARK_GRAY);
  pros::screen::erase_rect(FIELDX1 - 1, FIELDY1 - 1, FIELDX2 + 1, FIELDY2 + 1);
  // Field background
  pros::screen::set_eraser(COLOR_FIELD_BG);
  pros::screen::erase_rect(FIELDX1, FIELDY1, FIELDX2, FIELDY2);

  // Tile grid (6 tiles → 7 lines in each direction)
  pros::screen::set_pen(COLOR_GRID);
  for (int i = 0; i <= 6; ++i) {
    int px = FIELDX1 + i * FSIZE / 6;
    int py = FIELDY1 + i * FSIZE / 6;
    pros::screen::draw_line(px, FIELDY1, px, FIELDY2);
    pros::screen::draw_line(FIELDX1, py, FIELDX2, py);
  }

  // Field origin cross (faint)
  pros::screen::set_pen(COLOR_ORIGIN);
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
    // +y on field = -y on screen; +x on field = +x on screen
    const int arrowLen = 10;
    int arrowTipX = rx + static_cast<int>(std::sin(cur.theta) * arrowLen);
    int arrowTipY = ry - static_cast<int>(std::cos(cur.theta) * arrowLen);

    pros::screen::set_pen(COLOR_WHITE);
    pros::screen::draw_line(rx, ry, arrowTipX, arrowTipY);
    // Dot at robot center
    pros::screen::draw_pixel(rx, ry);
    pros::screen::draw_pixel(rx + 1, ry);
    pros::screen::draw_pixel(rx, ry + 1);
    pros::screen::draw_pixel(rx + 1, ry + 1);
  }

  // ── Data panel ────────────────────────────────────────────────────────────
  const int dx = displayX;
  int dy = FIELDY1;
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
  snprintf(buf, sizeof(buf), "%.2f\"", mapTotalDist); // cumulative path distance in inches
  pros::screen::print(pros::E_TEXT_SMALL, dx + 20, dy, buf);
  dy += rowH;

  // Separator
  pros::screen::set_eraser(COLOR_GRID);
  pros::screen::erase_rect(dx, dy, dx + displayW, dy + 1);
  dy += 6;

  // ── Stats panel ──────────────────────────────────────────────────────────
  if (mapMode == MapMode::DISPLACEMENT) {
    pros::screen::set_pen(COLOR_CYAN);
    pros::screen::print(pros::E_TEXT_SMALL, dx, dy, "DISPLACE:");
    dy += rowH;

    if (arcStartIndex >= 0 && dispEndIndex > arcStartIndex) {
      const auto& startPose = mapBuffer[arcStartIndex];
      const auto& endPose   = mapBuffer[dispEndIndex];
      double deltaX     = endPose.x - startPose.x;
      double deltaY     = endPose.y - startPose.y;
      double deltaTheta = endPose.theta - startPose.theta;
      while (deltaTheta >  M_PI) deltaTheta -= 2.0 * M_PI;
      while (deltaTheta < -M_PI) deltaTheta += 2.0 * M_PI;
      double distance = std::sqrt(deltaX * deltaX + deltaY * deltaY);

      pros::screen::set_pen(COLOR_LIGHT_GRAY);
      pros::screen::print(pros::E_TEXT_SMALL, dx, dy, "X:");
      pros::screen::set_pen(COLOR_CYAN);
      snprintf(buf, sizeof(buf), "%.2f\"", deltaX);
      pros::screen::print(pros::E_TEXT_SMALL, dx + 16, dy, buf);
      pros::screen::set_pen(COLOR_LIGHT_GRAY);
      pros::screen::print(pros::E_TEXT_SMALL, dx + 130, dy, "Hdg:");
      pros::screen::set_pen(COLOR_CYAN);
      snprintf(buf, sizeof(buf), "%.1f" "\xb0", deltaTheta * (180.0 / M_PI));
      pros::screen::print(pros::E_TEXT_SMALL, dx + 158, dy, buf);
      dy += rowH;

      pros::screen::set_pen(COLOR_LIGHT_GRAY);
      pros::screen::print(pros::E_TEXT_SMALL, dx, dy, "Y:");
      pros::screen::set_pen(COLOR_CYAN);
      snprintf(buf, sizeof(buf), "%.2f\"", deltaY);
      pros::screen::print(pros::E_TEXT_SMALL, dx + 16, dy, buf);
      pros::screen::set_pen(COLOR_LIGHT_GRAY);
      pros::screen::print(pros::E_TEXT_SMALL, dx + 130, dy, "Hyp:");
      pros::screen::set_pen(COLOR_CYAN);
      snprintf(buf, sizeof(buf), "%.2f\"", distance);
      pros::screen::print(pros::E_TEXT_SMALL, dx + 158, dy, buf);
      dy += rowH;

      dy += rowH * 2; // padding to match row count
    } else if (arcStartIndex >= 0) {
      pros::screen::set_pen(COLOR_LABEL_DIM);
      pros::screen::print(pros::E_TEXT_SMALL, dx, dy, "-- mark end --");
      dy += rowH * 3;
    } else {
      pros::screen::set_pen(COLOR_LABEL_DIM);
      pros::screen::print(pros::E_TEXT_SMALL, dx, dy, "-- mark start --");
      dy += rowH * 3;
    }
  } else if (mapMode == MapMode::ARC) {
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
      pros::screen::print(pros::E_TEXT_SMALL, dx + 130, dy, "Hdg:");
      pros::screen::set_pen(COLOR_YELLOW);
      snprintf(buf, sizeof(buf), "%.1f" "\xb0", arcResult.deltaHeading);
      pros::screen::print(pros::E_TEXT_SMALL, dx + 158, dy, buf);
      dy += rowH;

      pros::screen::set_pen(COLOR_LIGHT_GRAY);
      pros::screen::print(pros::E_TEXT_SMALL, dx, dy, "ArcLen:");
      pros::screen::set_pen(COLOR_YELLOW);
      snprintf(buf, sizeof(buf), "%.2f\"", arcResult.arcLength);//Arc Length
      pros::screen::print(pros::E_TEXT_SMALL, dx + 56, dy, buf);
      if (arcResult.radius > 0.01) {
        pros::screen::set_pen(COLOR_LIGHT_GRAY);
        pros::screen::print(pros::E_TEXT_SMALL, dx + 130, dy, "In:");
        pros::screen::set_pen(COLOR_YELLOW);
        snprintf(buf, sizeof(buf), "%.2f\"", arcResult.innerRadius);
        pros::screen::print(pros::E_TEXT_SMALL, dx + 150, dy, buf);
      }
      dy += rowH;

      pros::screen::set_pen(COLOR_LIGHT_GRAY);
      pros::screen::print(pros::E_TEXT_SMALL, dx, dy, "Chord:");
      pros::screen::set_pen(COLOR_YELLOW);
      snprintf(buf, sizeof(buf), "%.2f\"", arcResult.chordLength);
      pros::screen::print(pros::E_TEXT_SMALL, dx + 48, dy, buf);
      if (arcResult.radius > 0.01) {
        pros::screen::set_pen(COLOR_LIGHT_GRAY);
        pros::screen::print(pros::E_TEXT_SMALL, dx + 130, dy, "Out:");
        pros::screen::set_pen(COLOR_YELLOW);
        snprintf(buf, sizeof(buf), "%.2f\"", arcResult.outerRadius);
        pros::screen::print(pros::E_TEXT_SMALL, dx + 154, dy, buf);
      }
      dy += rowH;
    } else {
      pros::screen::set_pen(COLOR_LABEL_DIM);
      pros::screen::print(pros::E_TEXT_SMALL, dx, dy, "ARC: --");
      dy += rowH * 4;
    }
  } else {
    // SELECT mode — prompt
    pros::screen::set_pen(COLOR_LABEL_HINT);
    pros::screen::print(pros::E_TEXT_SMALL, dx, dy, "Select mode below:");
    dy += rowH * 4;
  }

  // ── Bottom buttons ────────────────────────────────────────────────────────
  const int btnY1 = BRAIN_SCREEN_HEIGHT - 35;
  const int btnY2 = BRAIN_SCREEN_HEIGHT - 5;
  const int halfW = (displayW - 6) / 2;

  if (mapMode == MapMode::SELECT) {
    // [DISPLACE]  [ARC MEAS]
    pros::screen::set_eraser(COLOR_BTN_DISPLACE);
    pros::screen::erase_rect(dx, btnY1, dx + halfW, btnY2);
    pros::screen::set_pen(COLOR_WHITE);
    pros::screen::print(pros::E_TEXT_SMALL, dx + 4, btnY1 + 8, "DISPLACE");

    pros::screen::set_eraser(COLOR_BTN_ARC_IDLE);
    pros::screen::erase_rect(dx + halfW + 6, btnY1, dx + displayW, btnY2);
    pros::screen::set_pen(COLOR_WHITE);
    pros::screen::print(pros::E_TEXT_SMALL, dx + halfW + 10, btnY1 + 8, "ARC MEAS");
  } else if (mapMode == MapMode::DISPLACEMENT) {
    // [MARK START]  [MARK END]
    uint32_t startColor = (arcStartIndex >= 0) ? COLOR_DARK_GREEN : COLOR_BTN_ARC_IDLE;
    pros::screen::set_eraser(startColor);
    pros::screen::erase_rect(dx, btnY1, dx + halfW, btnY2);
    pros::screen::set_pen(COLOR_WHITE);
    pros::screen::print(pros::E_TEXT_SMALL, dx + 4, btnY1 + 8,
                        (arcStartIndex >= 0) ? "START SET" : "MARK START");

    uint32_t endColor = (dispEndIndex >= 0) ? COLOR_DARK_BLUE : COLOR_BTN_END_IDLE;
    pros::screen::set_eraser(endColor);
    pros::screen::erase_rect(dx + halfW + 6, btnY1, dx + displayW, btnY2);
    pros::screen::set_pen(COLOR_WHITE);
    pros::screen::print(pros::E_TEXT_SMALL, dx + halfW + 10, btnY1 + 8,
                        (dispEndIndex >= 0) ? "END SET" : "MARK END");
  } else {
    // ARC mode — [MARK START]  [MARK END]
    uint32_t markSColor = (arcStartIndex >= 0) ? COLOR_DARK_GREEN : COLOR_BTN_ARC_IDLE;
    pros::screen::set_eraser(markSColor);
    pros::screen::erase_rect(dx, btnY1, dx + halfW, btnY2);
    pros::screen::set_pen(COLOR_WHITE);
    pros::screen::print(pros::E_TEXT_SMALL, dx + 4, btnY1 + 8,
                        (arcStartIndex >= 0) ? "START SET" : "MARK START");

    uint32_t markEColor = arcMeasured ? COLOR_DARK_BLUE : COLOR_BTN_END_IDLE;
    pros::screen::set_eraser(markEColor);
    pros::screen::erase_rect(dx + halfW + 6, btnY1, dx + displayW, btnY2);
    pros::screen::set_pen(COLOR_WHITE);
    pros::screen::print(pros::E_TEXT_SMALL, dx + halfW + 10, btnY1 + 8, "MARK END");
  }
}

// ============================================================================
// TOUCH HANDLERS
// ============================================================================

void GuiDebug::HandleFieldMapperTouch() {
  const auto touch = pros::screen::touch_status();
  if (touch.touch_status <= 0) return;

  int x = touch.x;
  int y = touch.y;

  // BACK button
  {
    const int backX1 = 10, backY1 = 6, backX2 = backX1 + 80, backY2 = backY1 + 28;
    if (x >= backX1 && x <= backX2 && y >= backY1 && y <= backY2) {
      DisplayDebugMenu();
      currentScreen = DebugMenu;
      pros::delay(300);
      return;
    }
  }

  // CLEAR button
  {
    const int clearX1 = BRAIN_SCREEN_WIDTH - 90, clearY1 = 6;
    const int clearX2 = BRAIN_SCREEN_WIDTH - 10, clearY2 = clearY1 + 28;
    if (x >= clearX1 && x <= clearX2 && y >= clearY1 && y <= clearY2) {
      ClearMapPath();
      DisplayFieldMapper();
      pros::delay(300);
      return;
    }
  }

  // Mode / Mark buttons (bottom of data panel)
  {
    const int btnY1 = BRAIN_SCREEN_HEIGHT - 35;
    const int btnY2 = BRAIN_SCREEN_HEIGHT - 5;
    const int halfW = (displayW - 6) / 2;
    const int startBtnX2 = displayX + halfW;          // right edge of start button
    const int endBtnX1   = displayX + halfW + 6;       // left edge of end button
    const int endBtnX2   = displayX + displayW;  // right edge of end button

    if (x >= displayX && x <= startBtnX2 && y >= btnY1 && y <= btnY2) {
      if (mapMode == MapMode::SELECT) {
        // Enter DISPLACEMENT mode
        mapMode = MapMode::DISPLACEMENT;
        arcStartIndex = -1;
        dispEndIndex  = -1;
      } else if (mapMode == MapMode::DISPLACEMENT) {
        // MARK START
        arcStartIndex = (mapBufferCount > 0) ? mapBufferCount - 1 : 0;
        dispEndIndex  = -1;
      } else if (mapMode == MapMode::ARC) {
        // MARK START
        arcStartIndex = (mapBufferCount > 0) ? mapBufferCount - 1 : 0;
        arcMeasured   = false;
        arcResult     = {};
      }
      DisplayFieldMapper();
      pros::delay(300);
      return;
    }

    if (x >= endBtnX1 && x <= endBtnX2 && y >= btnY1 && y <= btnY2) {
      if (mapMode == MapMode::SELECT) {
        // Enter ARC mode
        mapMode = MapMode::ARC;
        arcStartIndex = -1;
        arcMeasured   = false;
        arcResult     = {};
      } else if (mapMode == MapMode::DISPLACEMENT) {
        // MARK END for displacement
        if (arcStartIndex >= 0 && mapBufferCount > arcStartIndex) {
          dispEndIndex = mapBufferCount - 1;
        }
      } else if (mapMode == MapMode::ARC) {
        // MARK END — compute arc
        if (arcStartIndex >= 0 && mapBufferCount > arcStartIndex + 1) {
          computeArc(mapBuffer, arcStartIndex, mapBufferCount - 1, arcResult);
          arcMeasured = arcResult.valid;
        }
      }
      DisplayFieldMapper();
      pros::delay(300);
      return;
    }
  }
}

}  // namespace aon
