"""在 SUMO 底部状态栏显示仿真时间相对真实时间的倍率。

使用最近约两秒的 steady_clock 时间窗口，每半秒更新；无步进时显示 0x。
只观察 GUI 已收到的步进事件，不控制时钟或额外推进仿真。
"""
from pathlib import Path

WINDOW_SOURCE = Path('gui/GUIApplicationWindow.cpp')
SPEED_LABEL_CODE = '''class SimulationSpeedLabel : public FXLabel {
    FXDECLARE(SimulationSpeedLabel)
protected:
    SimulationSpeedLabel() = default;
private:
    using Clock = std::chrono::steady_clock;
    std::deque<std::pair<Clock::time_point, SUMOTime>> samples;
    Clock::time_point lastStep{};
    Clock::time_point lastDisplay{};
public:
    enum { ID_REFRESH = FXLabel::ID_LAST };
    explicit SimulationSpeedLabel(FXComposite* parent) :
        FXLabel(parent, "Sim/real: --", nullptr,
                FRAME_SUNKEN | FRAME_THICK | JUSTIFY_CENTER_X | JUSTIFY_CENTER_Y |
                LAYOUT_FIX_WIDTH | LAYOUT_CENTER_Y, 0, 0, 145, 0, 3, 3, 2, 2) {
        setTipText("Simulation seconds per real second, measured over the last 2 seconds.");
        getApp()->addTimeout(this, ID_REFRESH, 500);
    }
    ~SimulationSpeedLabel() override { getApp()->removeTimeout(this, ID_REFRESH); }
    void reset(bool stopped = false) {
        samples.clear();
        lastStep = Clock::time_point{};
        lastDisplay = Clock::time_point{};
        setText(stopped ? "Sim/real: 0.00x" : "Sim/real: --");
    }
    void step(SUMOTime simTime) {
        const auto now = Clock::now();
        if (!samples.empty() && simTime < samples.back().second) reset();
        if (!samples.empty() && simTime == samples.back().second) return;
        samples.emplace_back(now, simTime);
        lastStep = now;
        while (samples.size() > 1 && now - samples.front().first > std::chrono::seconds(2)) {
            samples.pop_front();
        }
        if (samples.size() < 2 || now - lastDisplay < std::chrono::milliseconds(500)) return;
        const double elapsed = std::chrono::duration<double>(now - samples.front().first).count();
        if (elapsed < 0.5) return;
        const double factor = STEPS2TIME(simTime - samples.front().second) / elapsed;
        std::ostringstream value;
        value << "Sim/real: " << std::fixed << std::setprecision(2) << factor << 'x';
        setText(value.str().c_str());
        lastDisplay = now;
    }
    long onTimer(FXObject*, FXSelector, void*) {
        if (lastStep != Clock::time_point{} &&
            Clock::now() - lastStep > std::chrono::milliseconds(1200)) reset(true);
        getApp()->addTimeout(this, ID_REFRESH, 500);
        return 1;
    }
};
FXDEFMAP(SimulationSpeedLabel) SimulationSpeedLabelMap[] = {
    FXMAPFUNC(SEL_TIMEOUT, SimulationSpeedLabel::ID_REFRESH, SimulationSpeedLabel::onTimer),
};
FXIMPLEMENT(SimulationSpeedLabel, FXLabel, SimulationSpeedLabelMap, ARRAYNUMBER(SimulationSpeedLabelMap))
static SimulationSpeedLabel* simulationSpeedLabel = nullptr;

'''


def source_patches():
    return {WINDOW_SOURCE: [
        ('#include "GUIApplicationWindow.h"',
         '#include <chrono>\n#include <deque>\n#include <iomanip>\n#include <sstream>\n\n#include "GUIApplicationWindow.h"'),
        ('std::mt19937 GUIApplicationWindow::myGamingRNG;',
         SPEED_LABEL_CODE + 'std::mt19937 GUIApplicationWindow::myGamingRNG;'),
        ('myCartesianCoordinate = GUIDesigns::buildFXLabel(myCartesianFrame, TL("N/A"), "", TL("Network coordinate"), nullptr, LAYOUT_CENTER_Y);',
         '''myCartesianCoordinate = GUIDesigns::buildFXLabel(myCartesianFrame, TL("N/A"), "", TL("Network coordinate"), nullptr, LAYOUT_CENTER_Y);
        simulationSpeedLabel = new SimulationSpeedLabel(myStatusbar);'''),
        ('myRunThread->resume();\n    getApp()->forceRefresh();',
         '''myRunThread->resume();
    if (simulationSpeedLabel != nullptr) simulationSpeedLabel->reset();
    getApp()->forceRefresh();'''),
        ('myRunThread->stop();\n    getApp()->forceRefresh();',
         '''myRunThread->stop();
    if (simulationSpeedLabel != nullptr) simulationSpeedLabel->reset(true);
    getApp()->forceRefresh();'''),
        ('myAmLoading = false;\n    GUIEvent_SimulationLoaded* ec',
         '''myAmLoading = false;
    if (simulationSpeedLabel != nullptr) simulationSpeedLabel->reset();
    GUIEvent_SimulationLoaded* ec'''),
        ('updateTimeLCD(myRunThread->getNet().getCurrentTimeStep());\n    const int running',
         '''updateTimeLCD(myRunThread->getNet().getCurrentTimeStep());
    if (simulationSpeedLabel != nullptr) simulationSpeedLabel->step(myRunThread->getNet().getCurrentTimeStep());
    const int running'''),
    ]}
