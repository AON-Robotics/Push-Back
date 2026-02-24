#pragma once
#ifndef AON_TOOLS_GUI_V2_HPP_
#define AON_TOOLS_GUI_V2_HPP_

#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <functional>
#include "../../../api.h"
#include "../function-reader.hpp"
#include "../gui-image-generator/gui-images.hpp"
#include "../../odometry/odometry.hpp"

namespace aon {
  class Gui;
  // Global GUI reference. The concrete instance is defined in the
  // implementation file; change the implementation type there only.
  extern Gui& gui;
  
  int RedRingsRoutine();
  int BlueRingsRoutine();
    
  // Add other auton routine declarations as needed


// namespace aon {


// Forward declaration of FunctionReader for autonomous routines
extern std::unique_ptr<FunctionReader<int>> AutonomousReader;

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
static constexpr int AutonOptionsCount = 3;

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
  GuiScreen CurrentScreen = MainMenu;
  GuiScreen PreviousScreen = MainMenu;
  bool debugEnabled = false;

  // Persistent GUI loop task handle
  std::unique_ptr<pros::Task> guiLoopTask{nullptr};

  // Auton routines for each alliance
  AutonOption RedAutonOptions[AutonOptionsCount] = {
    {"Red AUT1", aon::RedRingsRoutine},
    {"Red AUT2", aon::RedRingsRoutine},
    {"Red AUT3", aon::RedRingsRoutine},
  };
  
  AutonOption BlueAutonOptions[AutonOptionsCount] = {
    {"Blue AUT1", aon::BlueRingsRoutine},
    {"Blue AUT2", aon::BlueRingsRoutine},
    {"Blue AUT3", aon::BlueRingsRoutine},
  };
  
  AutonOption SkillsAutonOptions[AutonOptionsCount] = {
    {"Skills AUT1", aon::RedRingsRoutine},
    {"Skills AUT2", aon::RedRingsRoutine},
    {"Skills AUT3", aon::RedRingsRoutine},
  };

  // Constructor
  Gui() : debugEnabled(false) {}

  // Main initialization method
  virtual void Initialize();

  // Screen display methods
  virtual void DisplayMainMenu();
  virtual void DisplayAutonMenu();
  virtual void DisplayRedAutonMenu();
  virtual void DisplayBlueAutonMenu();
  virtual void DisplaySkillsMenu();

  // Touch handler methods
  virtual void HandleMainMenuTouch(const pros::screen_touch_status_s_t& touchStatus);
  virtual void HandleAutonMenuTouch();
  virtual void HandleRedAutonMenuTouch();
  virtual void HandleBlueAutonMenuTouch();
  virtual void HandleSkillsMenuTouch();

  // Debug-related APIs (no-op defaults). These are implemented fully in
  // `GuiDebug`. Declaring them here lets user code call them whether the
  // concrete GUI is `Gui` or `GuiDebug`.
  virtual void SetVariableRegister(const std::function<void()>& /*Register*/ ) {}
  virtual void VariableChanger(double& /*variableRef*/, const std::string& /*name*/) {}
  virtual void SetTestRegister(const std::function<void()>& /*Register*/ ) {}
  virtual void RegisterTestFunction(int (* /*func*/)(), const std::string& /*name*/) {}
  virtual void RegisterTestFunction(const std::function<int()>& /*func*/, const std::string& /*name*/) {}
  virtual void RegisterTestFunction(void (* /*func*/)(), const std::string& /*name*/) {}
  virtual void SetGraphDataProviders(std::function<double()> /*getX*/, std::function<double()> /*getY*/) {}
  virtual void RegisterDataEntry(const std::string& /*name*/, std::function<double()> /*getter*/) {}
  virtual void SetDataRegister(const std::function<void()>& /*Register*/) {}

  // Auton selection helper
  void SelectAutonByList(Alliance alliance, int index1Based);
  // Invoke the currently selected auton. `GuiDebug` overrides this to
  // prefer debug-registered invokers; base `Gui` calls the normal
  // `selectedAutonRoutine` (or returns 0).
  virtual int InvokeSelectedAuton();
  
protected:
  // Helper methods
  void ApplyPreselectedAuton();
  int DisplayInitializationMessage();
  
  // Virtual method for GUI loop - can be extended by derived classes
  virtual void RunGuiLoop();
};


  void InitializeGui();
}

#endif  // AON_TOOLS_GUI_V2_HPP_
