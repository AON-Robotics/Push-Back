#pragma once
#ifndef AON_TOOLS_GUI_HPP_
#define AON_TOOLS_GUI_HPP_

#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <functional>
#include "../../../api.h"
#include "aon/constants.hpp"
#include "../function-reader.hpp"
#include "../gui-image-generator/gui-images.hpp"

namespace aon {
  class Gui;
  // Owning GUI instance. Created once in gui.cpp; type selected by the
  // TESTING_AUTONOMOUS flag. Access members via gui->method().
  extern std::unique_ptr<Gui> gui;
  
  namespace routines {
    int RedRoutine();
    int BlueRoutine();
  }
    
  // Add other auton routine declarations as needed


// Forward declaration of FunctionReader for autonomous routines
extern std::unique_ptr<FunctionReader<int>> autonomousReader;

// GUI screen states
enum GuiScreen {
  MainMenu,
  AutonMenu,
  RedAutons,
  BlueAutons,
  SkillAutons,
  DebugMenu,
  RegisteredFunctions,
  AutonRunner,
  VARS,
  DATA,
  LiveGraph,
};

// Auton selection system
enum Alliance { Red, Blue, Skills };

struct AutonOption {
  const char* name;
  int (*routine)();
};

// Constants
static constexpr int autonOptionsCount = 3;

// Base Gui class - handles core GUI functionality without debug features
class Gui {
public:
  // Core auton selection: store the selected auton as an instance
  AutonOption selectedAuton = {"None", nullptr};
  std::string selectedAutonName = "None";
  // Optional invoker for debug-registered autons (used by GuiDebug)
  std::function<int()> selectedAutonInvoker = nullptr;
  // Runtime auton state (present here so callers referencing aon::gui compile)
  bool autonRunning = false;
  bool autonCompleted = false;
  
  // Preselected auton indices (1-3, 0 = none)
  int selectedRedAut = 0;
  int selectedBlueAut = 0;
  int selectedSkill = 0;

  // Screen management
  GuiScreen currentScreen = MainMenu;
  GuiScreen previousScreen = MainMenu;

  // TODO: move this to be only parameter based
  // Auton routines for each alliance
  AutonOption redAutonOptions[autonOptionsCount] = {
    {"Red AUT1", aon::routines::RedRoutine},
    {"Red AUT2", aon::routines::RedRoutine},
    {"Red AUT3", aon::routines::RedRoutine},
  };
  
  AutonOption blueAutonOptions[autonOptionsCount] = {
    {"Blue AUT1", aon::routines::BlueRoutine},
    {"Blue AUT2", aon::routines::BlueRoutine},
    {"Blue AUT3", aon::routines::BlueRoutine},
  };
  
  AutonOption skillsAutonOptions[autonOptionsCount] = {
    {"Skills AUT1", aon::routines::RedRoutine},
    {"Skills AUT2", aon::routines::RedRoutine},
    {"Skills AUT3", aon::routines::RedRoutine},
  };


  // Main initialization method (does not start the GUI loop task)
  virtual void initialize();

  // Screen display methods
  virtual void displayMainMenu();
  virtual void displayAutonMenu();
  virtual void displayRedAutonMenu();
  virtual void displayBlueAutonMenu();
  virtual void displaySkillsMenu();

  // Touch handler methods
  virtual void handleMainMenuTouch(const pros::screen_touch_status_s_t& touchStatus);
  virtual void handleAutonMenuTouch();
  virtual void handleRedAutonMenuTouch();
  virtual void handleBlueAutonMenuTouch();
  virtual void handleSkillsMenuTouch();

  // Debug-related APIs (no-op defaults). These are implemented fully in
  // `GuiDebug`. Declaring them here lets user code call them whether the
  // concrete GUI is `Gui` or `GuiDebug`.
  virtual void setVariableRegister(const std::function<void()>& /*Register*/ ) {}
  virtual void variableChanger(double& /*variableRef*/, const std::string& /*name*/) {}
  virtual void setTestRegister(const std::function<void()>& /*Register*/ ) {}
  virtual void registerTestFunction(int (* /*func*/)(), const std::string& /*name*/) {}
  virtual void registerTestFunction(const std::function<int()>& /*func*/, const std::string& /*name*/) {}
  virtual void registerTestFunction(void (* /*func*/)(), const std::string& /*name*/) {}
  virtual void setGraphDataProviders(std::function<double()> /*getX*/, std::function<double()> /*getY*/) {}
  virtual void registerDataEntry(const std::string& /*name*/, std::function<double()> /*getter*/) {}
  virtual void setDataRegister(const std::function<void()>& /*Register*/) {}
  virtual void registerResetHandler(const std::string& /*name*/, const std::function<void()>& /*cb*/) {}
  virtual void invokeResetHandler() {}

  // Auton selection helper
  void selectAutonByList(Alliance alliance, int index1Based);
  // Invoke the currently selected auton. `GuiDebug` overrides this to
  // prefer debug-registered invokers; base `Gui` calls the normal
  // `selectedAutonRoutine` (or returns 0).
  virtual int invokeSelectedAuton();
  
protected:
  // Helper methods
  void applyPreselectedAuton();
  int displayInitializationMessage();
  
  // Virtual method for GUI loop - can be extended by derived classes
  virtual void mainLoop();
};

}

#endif  // AON_TOOLS_GUI_HPP_
