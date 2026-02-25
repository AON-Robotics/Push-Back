
#ifndef AON_TOOLS_GUI_V2_DEBUG_HPP_
#define AON_TOOLS_GUI_V2_DEBUG_HPP_
#include "gui-v2.hpp"
#include <map>
#include <string>


namespace aon {

// Debug-specific GUI class - extends Gui with debug functionality
class GuiDebug : public Gui {
public:
  // Container to store test/auton functions and their names
  std::vector<std::pair<std::string, std::function<int()>>> testFunctions;
  
  // Optional lazy register that user code can set to seed tests on demand
  std::function<void()> testRegister = nullptr;

  // Variable changer registry: name + pointer to double to modify live
  struct VariableEntry {
    std::string name;
    double* ptr;
  };
  std::vector<VariableEntry> variableEntries;
  std::function<void()> variableRegister = nullptr;

  // Data viewer registry: name + getter for live data display
  struct DataEntry {
    std::string name;
    std::function<double()> getter;
  };
  std::vector<DataEntry> dataEntries;
  std::function<void()> dataRegister = nullptr;

  // Live graph data buffer
  struct GraphPoint {
    double x;
    double y;
  };
  static constexpr int GRAPH_BUFFER_SIZE = 300;
  GraphPoint graphBuffer[GRAPH_BUFFER_SIZE] = {};
  int graphBufferIndex = 0;
  double graphMinX = -10.0, graphMaxX = 10.0;
  double graphMinY = -10.0, graphMaxY = 10.0;
  bool graphAutoScale = true;

  // User provides data by setting these callbacks
  std::function<double()> graphGetX = nullptr;
  std::function<double()> graphGetY = nullptr;

  // Named reset handler registry; user registers handlers and picks active one
  std::map<std::string, std::function<void()>> resetHandlers;
  std::string activeResetHandlerName;

  // Register a named reset handler (overwrites existing with same name)
  void RegisterResetHandler(const std::string& name, const std::function<void()>& cb) { resetHandlers[name] = cb; activeResetHandlerName = name; }

  // Invoke the active reset handler (no-op if not set or not found)
  void InvokeResetHandler() { auto it = resetHandlers.find(activeResetHandlerName); if (it != resetHandlers.end()) it->second(); }

  // Constructor
  GuiDebug() = default; // Default to use TESTING_AUTONOMOUS for conditional display
  virtual ~GuiDebug() = default;

  // Override Initialize to include debug setup
  virtual void Initialize() override;

  // Override DisplayMainMenu to support conditional button sizing
  virtual void DisplayMainMenu() override;

  // Debug menu display methods (delegated to subsystems)
  void DisplayDebugMenu();
  void DisplayRegisteredAutonsMenu();
  void DisplayDebugMenu2();
  void DisplayDebugMenu3();
  void DisplayDebugMenu4();
  void DisplayLiveGraph();

  // Override main menu touch handler to handle DEBUG button
  virtual void HandleMainMenuTouch(const pros::screen_touch_status_s_t& touchStatus) override;

  // Debug touch handler methods (delegated to subsystems)
  void HandleDebugMenuTouch();
  void HandleRegisteredAutonsMenuTouch();
  void HandleDebugMenu2Touch();
  void HandleDebugMenu3Touch();
  void HandleDebugMenu4Touch();
  void HandleLiveGraphTouch();
  void DisplayVariablesMenu();

  // API: register a variable to be editable in Debug Menu 3
  void VariableChanger(double& variableRef, const std::string& name);

  // Allow user code to provide a register
  void SetVariableRegister(const std::function<void()>& Register);

  // API: register a data entry for the Data screen
  void RegisterDataEntry(const std::string& name, std::function<double()> getter) override;
  void SetDataRegister(const std::function<void()>& Register) override;

  // Allow user code to provide a register that calls registerTestFunction(...)
  void SetTestRegister(const std::function<void()>& Register);

  // Register test functions
  void RegisterTestFunction(int (*func)(), const std::string& name);
  void RegisterTestFunction(const std::function<int()>& func, const std::string& name);
  void RegisterTestFunction(void (*func)(), const std::string& name);

  // Allow user code to set data providers for live graph
  void SetGraphDataProviders(std::function<double()> getX, std::function<double()> getY);

  // Add a new data point to the graph buffer
  void AddGraphPoint(double x, double y);

protected:
  // Override GUI loop to include debug screens
  virtual void RunGuiLoop() override;

  // Internal helper to add unique entries by name
  void AddTestFunctionInternal(const std::string& name, std::function<int()> fn);

  // Helper to invoke selected auton
  int InvokeSelectedAuton();
};

}  // namespace aon

#endif  // AON_TOOLS_GUI_V2_DEBUG_HPP_
