"""SUMO Lincoln 右键导航：菜单、地图选点和异步 CoBridge 请求。

复用现有 GUI 构建器；左键 tracking 不变。选点状态、鼠标释放和 Esc
独立处理。socket 在后台限时通信，FOX 控件只在 GUI 线程访问。
"""
import json
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parents[4]
BRIDGE_SOCKET_FILE = PROJECT_DIR / '.runtime' / 'cobridge' / 'control.sock'
WINDOW_SOURCE = Path('gui/GUIApplicationWindow.cpp')
VIEW_SOURCE = Path('utils/gui/windows/GUISUMOAbstractView.cpp')
INCLUDES = '''#include <future>
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <foreign/nlohmann/json.hpp>
#include <utils/options/OptionsCont.h>
'''
NAVIGATION_CODE = r'''
using NavigationJson = nlohmann::json;
static const char* navigationSocketDefault = __BRIDGE_SOCKET__;

static NavigationJson requestLincolnNavigation(NavigationJson command) {
    const char* configured = std::getenv("AUTODRIVE_BRIDGE_SOCKET");
    const std::string path = configured && *configured ? configured : navigationSocketDefault;
    struct SocketOwner {
        int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
        ~SocketOwner() { if (fd >= 0) ::close(fd); }
    } connection;
    if (connection.fd < 0) throw std::runtime_error("Cannot create navigation socket");
    timeval timeout{2, 0};
    setsockopt(connection.fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(connection.fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    if (path.size() >= sizeof(address.sun_path)) throw std::runtime_error("Bridge socket path too long");
    std::strcpy(address.sun_path, path.c_str());
    if (::connect(connection.fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0)
        throw std::runtime_error("CoBridge unavailable; start or restart run_cobridge");
    const std::string payload = command.dump() + '\n';
    size_t sent = 0;
    while (sent < payload.size()) {
        const auto size = ::send(connection.fd, payload.data() + sent, payload.size() - sent, MSG_NOSIGNAL);
        if (size <= 0) throw std::runtime_error("Navigation send timeout");
        sent += size;
    }
    std::string response;
    char buffer[4096];
    while (response.find('\n') == std::string::npos && response.size() < 65536) {
        const auto size = ::recv(connection.fd, buffer, sizeof(buffer), 0);
        if (size <= 0) throw std::runtime_error("Navigation reply timeout");
        response.append(buffer, size);
    }
    const auto reply = NavigationJson::parse(response);
    if (!reply.value("ok", false)) throw std::runtime_error(reply.value("error", "Navigation rejected"));
    return reply.at("result");
}

class LincolnNavigationUI : public FXHorizontalFrame {
    FXDECLARE(LincolnNavigationUI)
protected:
    LincolnNavigationUI() = default;
private:
    GUIApplicationWindow* window = nullptr;
    GUISUMOAbstractView* pickingView = nullptr;
    GUISUMOAbstractView* consumedView = nullptr;
    NavigationJson picking;
    NavigationJson active;
    std::map<std::string, NavigationJson> jobs;
    std::map<std::string, std::pair<std::string, std::string>> states;
    NavigationJson queued;
    std::future<NavigationJson> pending;
    std::string state;
    std::string detail;
    int pollCountdown = 0;
    size_t pollIndex = 0;
    std::string pendingName;

    void launch(NavigationJson command) {
        if (pending.valid()) { queued = command; return; }
        pendingName = command.value("sumo_id", "");
        pending = std::async(std::launch::async, requestLincolnNavigation, std::move(command));
    }
public:
    enum { ID_REFRESH = FXHorizontalFrame::ID_LAST };
    LincolnNavigationUI(FXComposite* parent, GUIApplicationWindow* application) :
        FXHorizontalFrame(parent), window(application) {
        hide();
        getApp()->addTimeout(this, ID_REFRESH, 250);
    }
    ~LincolnNavigationUI() override { getApp()->removeTimeout(this, ID_REFRESH); }

    void beginSelection(const std::string& name) {
        auto* view = window->getActiveView();
        if (!view || !window->getRunner()->networkAvailable()) return;
        auto* control = dynamic_cast<GUIVehicleControl*>(&window->getRunner()->getNet().getVehicleControl());
        if (!control) return;
        std::string owner;
        control->secureVehicles();
        auto* vehicle = control->getVehicle(name);
        if (vehicle) owner = vehicle->getParameter().getParameter("autodrive.lincoln.owner", "");
        control->releaseVehicles();
        if (owner.find("lincoln:") != 0) return;
        if (jobs.count(name) && jobs[name].value("owner_key", "") != owner) {
            jobs.erase(name);
            states.erase(name);
        }
        char* resolved = ::realpath(OptionsCont::getOptions().getString("net-file").c_str(), nullptr);
        if (!resolved) { window->setStatusBarText("Cannot resolve active SUMO network"); return; }
        const std::string networkFile(resolved);
        std::free(resolved);
        picking = {{"sumo_id", name}, {"owner_key", owner}, {"network_file", networkFile}};
        pickingView = view;
        view->setFocus();
        window->setStatusBarText(name + ": Select destination on map; Esc to cancel");
    }

    bool click(GUISUMOAbstractView* view, double x, double y) {
        if (view != pickingView || picking.is_null()) return false;
        auto command = picking;
        command["action"] = "lincoln.navigate";
        command["x"] = x;
        command["y"] = y;
        active = picking;
        jobs[active.value("sumo_id", "")] = active;
        picking = nullptr;
        pickingView = nullptr;
        consumedView = view;
        state = "planning";
        detail = "Waiting for CoBridge";
        states[active.value("sumo_id", "")] = {state, detail};
        window->setStatusBarText(detail);
        launch(command);
        return true;
    }
    bool release(GUISUMOAbstractView* view) {
        if (view != consumedView) return false;
        consumedView = nullptr;
        return true;
    }
    bool cancelSelection(GUISUMOAbstractView* view) {
        if (view != pickingView) return false;
        pickingView = nullptr;
        picking = nullptr;
        window->setStatusBarText("Destination selection cancelled");
        return true;
    }
    std::string hint(const std::string& name) const {
        if (!picking.is_null() && picking.value("sumo_id", "") == name) return "\nSelecting destination";
        const auto found = states.find(name);
        if (found != states.end()) return "\nNavigation: " + found->second.first + "\n" + found->second.second;
        return "\nRight click: Navigate here";
    }
    long onRefresh(FXObject*, FXSelector, void*) {
        if (pending.valid() && pending.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            try {
                const auto response = pending.get();
                state = response.value("state", "error");
                detail = response.value("detail", "");
            } catch (const std::exception& error) {
                state = "error";
                detail = error.what();
                if (std::any_of(detail.begin(), detail.end(), [](unsigned char ch) { return ch > 127; }))
                    detail = "Navigation rejected; restart CoBridge/Lincoln or check their logs.";
            }
            states[pendingName] = {state, detail};
            window->setStatusBarText("Navigation " + state + ": " + detail);
            pollCountdown = 4;
        }
        if (!queued.is_null() && !pending.valid()) {
            auto command = queued; queued = nullptr; launch(command);
        } else if (!jobs.empty() && !pending.valid() && --pollCountdown <= 0) {
            for (size_t count = 0; count < jobs.size(); ++count) {
                auto job = jobs.begin();
                std::advance(job, pollIndex++ % jobs.size());
                const auto status = states[job->first].first;
                if (status != "planning" && status != "navigating") continue;
                auto command = job->second;
                command["action"] = "lincoln.navigation.status";
                launch(command);
                break;
            }
            pollCountdown = 4;
        }
        getApp()->addTimeout(this, ID_REFRESH, 250);
        return 1;
    }
};
FXDEFMAP(LincolnNavigationUI) LincolnNavigationUIMap[] = {
    FXMAPFUNC(SEL_TIMEOUT, LincolnNavigationUI::ID_REFRESH, LincolnNavigationUI::onRefresh),
};
FXIMPLEMENT(LincolnNavigationUI, FXHorizontalFrame, LincolnNavigationUIMap, ARRAYNUMBER(LincolnNavigationUIMap))
static LincolnNavigationUI* lincolnNavigationUI = nullptr;

bool autodriveNavigationClick(GUISUMOAbstractView* view, double x, double y) {
    return lincolnNavigationUI && lincolnNavigationUI->click(view, x, y);
}
bool autodriveNavigationRelease(GUISUMOAbstractView* view) {
    return lincolnNavigationUI && lincolnNavigationUI->release(view);
}
bool autodriveNavigationEscape(GUISUMOAbstractView* view) {
    return lincolnNavigationUI && lincolnNavigationUI->cancelSelection(view);
}

class LincolnNavigationButton : public FXToggleButton {
    FXDECLARE(LincolnNavigationButton)
protected:
    LincolnNavigationButton() = default;
private:
    std::string vehicleName;
public:
    enum { ID_NAVIGATE = FXToggleButton::ID_LAST };
    LincolnNavigationButton(FXComposite* parent, const char* name, FXObject* target, FXSelector selector) :
        FXToggleButton(parent, name, "Stop tracking", nullptr, nullptr, target, selector,
                       TOGGLEBUTTON_NORMAL | TOGGLEBUTTON_KEEPSTATE | LAYOUT_CENTER_Y | LAYOUT_FIX_WIDTH,
                       0, 0, 105), vehicleName(name) { }
    long onRightMenu(FXObject*, FXSelector, void* data) {
        auto* event = static_cast<FXEvent*>(data);
        FXMenuPane menu(this);
        new FXMenuCommand(&menu, "Navigate here", nullptr, this, ID_NAVIGATE);
        menu.create();
        menu.popup(nullptr, event->root_x, event->root_y);
        getApp()->runModalWhileShown(&menu);
        return 1;
    }
    long onNavigate(FXObject*, FXSelector, void*) {
        if (lincolnNavigationUI) lincolnNavigationUI->beginSelection(vehicleName);
        return 1;
    }
};
FXDEFMAP(LincolnNavigationButton) LincolnNavigationButtonMap[] = {
    FXMAPFUNC(SEL_RIGHTBUTTONPRESS, 0, LincolnNavigationButton::onRightMenu),
    FXMAPFUNC(SEL_COMMAND, LincolnNavigationButton::ID_NAVIGATE, LincolnNavigationButton::onNavigate),
};
FXIMPLEMENT(LincolnNavigationButton, FXToggleButton, LincolnNavigationButtonMap, ARRAYNUMBER(LincolnNavigationButtonMap))
'''


def source_patches():
    code = NAVIGATION_CODE.replace('__BRIDGE_SOCKET__', json.dumps(str(BRIDGE_SOCKET_FILE)))
    declarations = '''extern bool autodriveNavigationClick(GUISUMOAbstractView*, double, double);
extern bool autodriveNavigationRelease(GUISUMOAbstractView*);
extern bool autodriveNavigationEscape(GUISUMOAbstractView*);
'''
    return {
        WINDOW_SOURCE: [
            ('#include "GUIApplicationWindow.h"', INCLUDES + '#include "GUIApplicationWindow.h"'),
            ('// 仅复制车辆名称和图元 ID，不在刷新之间保留车辆指针。',
             code + '\n// 仅复制车辆名称和图元 ID，不在刷新之间保留车辆指针。'),
            ('''new FXToggleButton(this, vehicle.first.c_str(), "Stop tracking", nullptr, nullptr, this, ID_TRACK,
                                       TOGGLEBUTTON_NORMAL | TOGGLEBUTTON_KEEPSTATE | LAYOUT_CENTER_Y | LAYOUT_FIX_WIDTH, 0, 0, 105)''',
             'new LincolnNavigationButton(this, vehicle.first.c_str(), this, ID_TRACK)'),
            ('''item.second->setTipText((tracking ? "Stop tracking " + item.first
                                             : "Center and track " + item.first).c_str());''',
             '''item.second->setTipText(((tracking ? "Stop tracking " + item.first
                                             : "Center and track " + item.first)
                        + (lincolnNavigationUI ? lincolnNavigationUI->hint(item.first) : "")).c_str());'''),
            ('myCartesianCoordinate = GUIDesigns::buildFXLabel(myCartesianFrame, TL("N/A"), "", TL("Network coordinate"), nullptr, LAYOUT_CENTER_Y);',
             '''myCartesianCoordinate = GUIDesigns::buildFXLabel(myCartesianFrame, TL("N/A"), "", TL("Network coordinate"), nullptr, LAYOUT_CENTER_Y);
        lincolnNavigationUI = new LincolnNavigationUI(myStatusbar, this);'''),
        ],
        VIEW_SOURCE: [
            ('#include "GUISUMOAbstractView.h"',
             '#include "GUISUMOAbstractView.h"\n' + declarations),
            ('''    FXEvent* e = (FXEvent*) ptr;
    // check whether the selection-mode is activated''',
             '''    FXEvent* e = (FXEvent*) ptr;
    const Position navigationPosition = screenPos2NetPos(e->win_x, e->win_y);
    if (autodriveNavigationClick(this, navigationPosition.x(), navigationPosition.y())) return 1;
    // check whether the selection-mode is activated'''),
            ('GUISUMOAbstractView::onLeftBtnRelease(FXObject*, FXSelector, void* ptr) {',
             '''GUISUMOAbstractView::onLeftBtnRelease(FXObject*, FXSelector, void* ptr) {
    if (autodriveNavigationRelease(this)) return 1;'''),
            ('''GUISUMOAbstractView::onKeyPress(FXObject* o, FXSelector sel, void* ptr) {
    const FXEvent* e = (FXEvent*) ptr;''',
             '''GUISUMOAbstractView::onKeyPress(FXObject* o, FXSelector sel, void* ptr) {
    const FXEvent* e = (FXEvent*) ptr;
    if (e->code == FX::KEY_Escape && autodriveNavigationEscape(this)) return 1;'''),
        ],
    }
