"""在 SUMO 底部双坐标状态栏为每辆 Lincoln 创建持续跟踪按钮。

本模块只提供 GUI 源码修改，由 build_sumo_gui.py 合并编译。
每半秒读取一次本地车辆列表，仅识别 CoBridge 标记的 Lincoln 代理；
点击后调用 SUMO 原有视图跟踪接口，不请求 TraCI，也不推进仿真。
"""
from pathlib import Path

WINDOW_SOURCE = Path('gui/GUIApplicationWindow.cpp')
TRACKING_INCLUDES = '''#include <guisim/GUIVehicleControl.h>
#include <guisim/GUIBaseVehicle.h>
#include <utils/vehicle/SUMOVehicle.h>
#include <utils/gui/windows/GUISUMOAbstractView.h>

'''
TRACKING_STATUS_CODE = '''// 仅复制车辆名称和图元 ID，不在刷新之间保留车辆指针。
class LincolnTrackBar : public FXHorizontalFrame {
    FXDECLARE(LincolnTrackBar)
protected:
    LincolnTrackBar() = default;
private:
    GUIApplicationWindow* window = nullptr;
    std::map<std::string, FXToggleButton*> buttons;
    using Fleet = std::map<std::string, GUIGlID>;

    // 与 GUIVehicleControl 的增删操作使用同一把锁，并保证异常时解锁。
    struct VehicleListLock {
        GUIVehicleControl& control;
        explicit VehicleListLock(GUIVehicleControl& value) : control(value) { control.secureVehicles(); }
        ~VehicleListLock() { control.releaseVehicles(); }
    };

    Fleet collectLincolns() const {
        Fleet fleet;
        if (!window->getRunner() || !window->getRunner()->networkAvailable()) return fleet;
        auto* control = dynamic_cast<GUIVehicleControl*>(&window->getRunner()->getNet().getVehicleControl());
        if (!control) return fleet;
        VehicleListLock lock(*control);
        for (auto it = control->loadedVehBegin(); it != control->loadedVehEnd(); ++it) {
            const SUMOVehicle* vehicle = it->second;
            if (!vehicle->isOnRoad() && !vehicle->isParking()) continue;
            if (vehicle->getParameter().getParameter("autodrive.lincoln.owner", "").find("lincoln:") != 0) continue;
            const auto* object = dynamic_cast<const GUIBaseVehicle*>(vehicle);
            if (object) fleet.emplace(vehicle->getID(), object->getGlID());
        }
        return fleet;
    }

    void refreshButtons(const Fleet& fleet) {
        bool changed = false;
        for (auto it = buttons.begin(); it != buttons.end();) {
            if (fleet.count(it->first)) { ++it; continue; }
            delete it->second;
            it = buttons.erase(it);
            changed = true;
        }
        for (const auto& vehicle : fleet) {
            if (buttons.count(vehicle.first)) continue;
            auto* button = new FXToggleButton(this, vehicle.first.c_str(), "Stop tracking", nullptr, nullptr, this, ID_TRACK,
                                       TOGGLEBUTTON_NORMAL | TOGGLEBUTTON_KEEPSTATE | LAYOUT_CENTER_Y | LAYOUT_FIX_WIDTH, 0, 0, 105);
            button->setTipText("Center and continuously track this Lincoln in the active view.");
            button->create();
            buttons.emplace(vehicle.first, button);
            changed = true;
        }
        if (changed) { recalc(); getParent()->recalc(); }
        updateTrackingState(fleet);
    }

    void updateTrackingState(const Fleet& fleet) {
        const auto* view = window->getActiveView();
        for (const auto& item : buttons) {
            const auto vehicle = fleet.find(item.first);
            const bool tracking = view && vehicle != fleet.end()
                                  && view->getTrackedID() == vehicle->second;
            item.second->setState(tracking);
            item.second->setTipText((tracking ? "Stop tracking " + item.first
                                             : "Center and track " + item.first).c_str());
        }
    }

public:
    enum { ID_REFRESH = FXHorizontalFrame::ID_LAST, ID_TRACK };
    explicit LincolnTrackBar(FXComposite* parent, GUIApplicationWindow* application) :
        FXHorizontalFrame(parent, LAYOUT_RIGHT | LAYOUT_CENTER_Y, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0),
        window(application) {
        getApp()->addTimeout(this, ID_REFRESH, 500);
    }
    ~LincolnTrackBar() override { getApp()->removeTimeout(this, ID_REFRESH); }

    long onRefresh(FXObject*, FXSelector, void*) {
        refreshButtons(collectLincolns());
        getApp()->addTimeout(this, ID_REFRESH, 500);
        return 1;
    }

    long onTrack(FXObject* sender, FXSelector, void*) {
        // 点击时重新校验；已删除的车辆不会使用上次刷新的旧 ID。
        const Fleet fleet = collectLincolns();
        auto* view = window->getActiveView();
        if (!view) return 1;
        for (const auto& button : buttons) {
            if (button.second != sender) continue;
            const auto vehicle = fleet.find(button.first);
            if (vehicle == fleet.end()) return 1;
            const bool alreadyTracking = view->getTrackedID() == vehicle->second;
            view->stopTrack();
            if (!alreadyTracking) {
                view->startTrack(vehicle->second);
                view->centerTo(vehicle->second, false);
            }
            view->update();
            updateTrackingState(fleet);
            break;
        }
        return 1;
    }

};
FXDEFMAP(LincolnTrackBar) LincolnTrackBarMap[] = {
    FXMAPFUNC(SEL_TIMEOUT, LincolnTrackBar::ID_REFRESH, LincolnTrackBar::onRefresh),
    FXMAPFUNC(SEL_COMMAND, LincolnTrackBar::ID_TRACK, LincolnTrackBar::onTrack),
};
FXIMPLEMENT(LincolnTrackBar, FXHorizontalFrame, LincolnTrackBarMap, ARRAYNUMBER(LincolnTrackBarMap))

'''


def source_patches():
    """添加状态栏跟踪按钮，复用 SUMO 的车辆生命周期与原生视图跟踪。"""
    return {WINDOW_SOURCE: [
        ('#include "GUIApplicationWindow.h"',
         TRACKING_INCLUDES + '#include "GUIApplicationWindow.h"'),
        ('std::mt19937 GUIApplicationWindow::myGamingRNG;',
         TRACKING_STATUS_CODE + 'std::mt19937 GUIApplicationWindow::myGamingRNG;'),
        ('myCartesianFrame = new FXHorizontalFrame(myStatusbar, GUIDesignHorizontalFrameStatusBar);',
         'new LincolnTrackBar(myStatusbar, this);\n        myCartesianFrame = new FXHorizontalFrame(myStatusbar, GUIDesignHorizontalFrameStatusBar);'),
    ]}
