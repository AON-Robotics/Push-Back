
#ifndef AON_TOOLS_GUI_DEBUG_HPP_
#define AON_TOOLS_GUI_DEBUG_HPP_
#include "gui.hpp"
#include "aon/math/pose.hpp"
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

  // ── Field Mapper ──────────────────────────────────────────────────────────

  static constexpr int MAP_BUFFER_SIZE = 600;
  Pose mapBuffer[MAP_BUFFER_SIZE] = {};
  int      mapBufferCount  = 0;   // number of valid points (0..MAP_BUFFER_SIZE)
  double   mapTotalDist    = 0.0; // cumulative distance traveled (inches)

  // Arc measurement state
  enum class MapMode { SELECT, DISPLACEMENT, ARC };
  MapMode mapMode = MapMode::SELECT;

  int  arcStartIndex = -1;  // index into mapBuffer where arc/disp start was set; -1 = not set
  int  dispEndIndex  = -1;  // index into mapBuffer where disp end was set; -1 = not set
  bool arcMeasured   = false;
  struct ArcResult {
    double radius;       // inches — robot center (pass directly to driveInArc)
    double innerRadius;  // inches — inner drive wheel
    double outerRadius;  // inches — outer drive wheel
    double arcLength;    // inches
    double chordLength;  // inches
    double deltaHeading; // degrees
    bool   valid = false;
  } arcResult = {};

  // User provides live pose via this callback
  std::function<Pose()> mapGetPose = nullptr;

  // Register the pose provider (called once in globals/opcontrol setup)
  void setMapDataProvider(std::function<Pose()> getPose) override;

  // Append a new pose sample; computes running distance
  void AddMapPoint(double x, double y, double theta);  // theta in degrees (matches Pose)

  // Erase all recorded path data and reset arc state
  void ClearMapPath();

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
  void DisplayFieldMapper();

  // Override main menu touch handler to handle DEBUG button
  virtual void handleMainMenuTouch(const pros::screen_touch_status_s_t& touchStatus) override;

  // Debug touch handler methods (delegated to subsystems)
  void HandleDebugMenuTouch();
  void HandleRegisteredAutonsMenuTouch();
  void HandleAutonRunnerTouch();
  void HandleLiveGraphTouch();
  void HandleVariablesMenuTouch();
  void HandleDataMenuTouch();
  void HandleFieldMapperTouch();

  // Override the type-erased virtual; called by the template in the base class
  void variableChangerImpl(const std::string& name,std::function<double()> get,std::function<void(double)> apply) override {
    for (const auto& e : variableEntries) {
      if (e.name == name) return;
    }
    variableEntries.push_back({name, std::move(get), std::move(apply)});
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
