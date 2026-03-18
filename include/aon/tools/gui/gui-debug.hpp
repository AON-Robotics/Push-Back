
#ifndef AON_TOOLS_GUI_DEBUG_HPP_
#define AON_TOOLS_GUI_DEBUG_HPP_
#include "gui.hpp"
#include <map>
#include <string>
#include <type_traits>


namespace aon {

// Debug-specific GUI class - extends Gui with debug functionality
class GuiDebug : public Gui {
public:
  // Container to store test/auton functions and their names
  std::vector<std::pair<std::string, std::function<int()>>> testFunctions;
  
  // Optional lazy register that user code can set to seed tests on demand
  std::function<void()> testRegister = nullptr;

  // Variable changer registry: type-erased so any T supporting + and - can be stored
  struct VariableEntry {
    std::string name;
    std::function<double()> get;         // read current value as double
    std::function<void(double)> apply;   // add a delta to the variable
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
  void registerResetHandler(const std::string& name, const std::function<void()>& handler) { resetHandlers[name] = handler; activeResetHandlerName = name; }

  // Invoke the active reset handler (no-op if not set or not found)
  void invokeResetHandler() { auto it = resetHandlers.find(activeResetHandlerName); if (it != resetHandlers.end()) it->second(); }

  // Constructor
  GuiDebug() = default; // Default to use TESTING_AUTONOMOUS for conditional display
  virtual ~GuiDebug() = default;

  // Override initialize to include debug setup
  virtual void initialize() override;

  // Override displayMainMenu to support conditional button sizing
  virtual void displayMainMenu() override;

  // Debug menu display methods (delegated to subsystems)
  void DisplayDebugMenu();
  void DisplayRegisteredAutonsMenu();
  void DisplayAutonRunner();
  void DisplayLiveGraph();
  void DisplayVariablesMenu();
  void DisplayDataMenu();

  // Override main menu touch handler to handle DEBUG button
  virtual void handleMainMenuTouch(const pros::screen_touch_status_s_t& touchStatus) override;

  // Debug touch handler methods (delegated to subsystems)
  void HandleDebugMenuTouch();
  void HandleRegisteredAutonsMenuTouch();
  void HandleAutonRunnerTouch();
  void HandleLiveGraphTouch();
  void HandleVariablesMenuTouch();
  void HandleDataMenuTouch();

  // API: register a variable to be editable in Debug Menu 3.
  // T must support + and - (detected via std::void_t).
  template <
    typename T,
    typename = std::void_t<
      decltype(std::declval<T>() + std::declval<T>()),
      decltype(std::declval<T>() - std::declval<T>())
    >
  >
  void variableChanger(T& variableRef, const std::string& name) {
    for (const auto& e : variableEntries) {
      if (e.name == name) return;
    }
    variableEntries.push_back({
      name,
      [&variableRef]() -> double { return static_cast<double>(variableRef); },
      [&variableRef](double delta) { variableRef += static_cast<T>(delta); }
    });
  }

  // Allow user code to provide a register
  void setVariableRegister(const std::function<void()>& Register);

  // API: register a data entry for the Data screen
  void registerDataEntry(const std::string& name, std::function<double()> getter) override;
  void setDataRegister(const std::function<void()>& Register) override;

  // Allow user code to provide a register that calls registerTestFunction(...)
  void setTestRegister(const std::function<void()>& Register);

  // Register test functions
  void registerTestFunction(int (*func)(), const std::string& name);
  void registerTestFunction(const std::function<int()>& func, const std::string& name);
  void registerTestFunction(void (*func)(), const std::string& name);

  // Allow user code to set data providers for live graph
  void setGraphDataProviders(std::function<double()> getX, std::function<double()> getY);

  // Add a new data point to the graph buffer
  void AddGraphPoint(double x, double y);

protected:
  // Override GUI loop to include debug screens
  virtual void mainLoop() override;

  // Internal helper to add unique entries by name
  void AddTestFunctionInternal(const std::string& name, std::function<int()> fn);

  // Helper to invoke selected auton
  int invokeSelectedAuton();
};

}  // namespace aon

#endif  // AON_TOOLS_GUI_DEBUG_HPP_
