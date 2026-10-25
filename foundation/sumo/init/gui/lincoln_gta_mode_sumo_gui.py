"""为 SUMO 的 Lincoln 右键菜单添加独立的 GTA Mode 信号灯开关。

本文件提供 C++ 源码补丁，由 build_sumo_gui.py 在导航功能之后合并。
复用导航的后台 socket 请求函数与车辆身份标记；不修改 tracking 行为。
菜单勾选使用 Lincoln 已执行的确认状态，等待期间禁用再次点击。
"""
from pathlib import Path

WINDOW_SOURCE = Path('gui/GUIApplicationWindow.cpp')
GTA_CODE = r'''
class LincolnGtaModeUI : public FXHorizontalFrame {
    FXDECLARE(LincolnGtaModeUI)
protected:
    LincolnGtaModeUI() = default;
private:
    GUIApplicationWindow* window = nullptr;
    NavigationJson selection;
    NavigationJson pendingIdentity;
    std::future<NavigationJson> pending;
    bool confirmed = false;
    bool ready = false;
    bool failed = false;
    std::chrono::steady_clock::time_point deadline;

    NavigationJson vehicleIdentity(const std::string& name) const {
        if (!window->getRunner() || !window->getRunner()->networkAvailable())
            throw std::runtime_error("No active SUMO network");
        auto* control = dynamic_cast<GUIVehicleControl*>(&window->getRunner()->getNet().getVehicleControl());
        if (!control) throw std::runtime_error("Vehicle control unavailable");
        struct VehicleLock {
            GUIVehicleControl& control;
            explicit VehicleLock(GUIVehicleControl& value) : control(value) { control.secureVehicles(); }
            ~VehicleLock() { control.releaseVehicles(); }
        };
        std::string owner;
        {
            VehicleLock lock(*control);
            const auto* vehicle = control->getVehicle(name);
            if (vehicle) owner = vehicle->getParameter().getParameter("autodrive.lincoln.owner", "");
        }
        if (owner.find("lincoln:") != 0) throw std::runtime_error("Lincoln no longer available");
        char* resolved = ::realpath(OptionsCont::getOptions().getString("net-file").c_str(), nullptr);
        if (!resolved) throw std::runtime_error("Cannot resolve active SUMO network");
        const std::string networkFile(resolved);
        std::free(resolved);
        return {{"sumo_id", name}, {"owner_key", owner}, {"network_file", networkFile}};
    }

    void launch(const char* action) {
        auto command = selection;
        command["action"] = action;
        command["enabled"] = !confirmed;
        pendingIdentity = selection;
        pending = std::async(std::launch::async, requestLincolnNavigation, std::move(command));
    }

    void showError(std::string message) {
        if (std::any_of(message.begin(), message.end(), [](unsigned char ch) { return ch > 127; }))
            message = "Request failed; restart CoBridge/Lincoln or check their logs.";
        window->setStatusBarText("GTA Mode: " + message);
        failed = true;
        ready = false;
    }

public:
    enum { ID_REFRESH = FXHorizontalFrame::ID_LAST };
    LincolnGtaModeUI(FXComposite* parent, GUIApplicationWindow* application) :
        FXHorizontalFrame(parent), window(application) {
        hide();
        getApp()->addTimeout(this, ID_REFRESH, 250);
    }
    ~LincolnGtaModeUI() override { getApp()->removeTimeout(this, ID_REFRESH); }

    void openMenu(const std::string& name) {
        ready = false;
        failed = false;
        confirmed = false;
        selection = nullptr;
        deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
        try {
            selection = vehicleIdentity(name);
            window->setStatusBarText(name + ": Reading GTA Mode...");
            if (!pending.valid()) launch("lincoln.gta.status");
        } catch (const std::exception& error) { showError(error.what()); }
    }

    bool isConfirmed() const { return confirmed; }
    bool canToggle() const { return ready && !pending.valid(); }

    void toggle() {
        if (!canToggle()) return;
        try {
            if (vehicleIdentity(selection.at("sumo_id")) != selection)
                throw std::runtime_error("Vehicle identity changed; reopen the menu");
            ready = false;
            deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
            window->setStatusBarText(selection.value("sumo_id", "") + ": Applying GTA Mode...");
            launch("lincoln.gta.set");
        } catch (const std::exception& error) { showError(error.what()); }
    }

    long onRefresh(FXObject*, FXSelector, void*) {
        if (pending.valid() && pending.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            try {
                const auto response = pending.get();
                if (pendingIdentity == selection) {
                    confirmed = response.at("enabled").get<bool>();
                    ready = !response.value("pending", false);
                    const auto error = response.value("error", "");
                    if (!error.empty()) showError(error);
                    else if (ready) window->setStatusBarText(selection.value("sumo_id", "")
                        + (confirmed ? ": GTA Mode ON (ignore traffic lights)" : ": GTA Mode OFF (obey traffic lights)"));
                }
            } catch (const std::exception& error) {
                if (pendingIdentity == selection) showError(error.what());
            }
        }
        if (!selection.is_null() && !pending.valid() && !ready && !failed) {
            if (std::chrono::steady_clock::now() >= deadline)
                showError("Confirmation timed out; reopen menu to read current state.");
            else launch("lincoln.gta.status");
        }
        getApp()->addTimeout(this, ID_REFRESH, 250);
        return 1;
    }
};
FXDEFMAP(LincolnGtaModeUI) LincolnGtaModeUIMap[] = {
    FXMAPFUNC(SEL_TIMEOUT, LincolnGtaModeUI::ID_REFRESH, LincolnGtaModeUI::onRefresh),
};
FXIMPLEMENT(LincolnGtaModeUI, FXHorizontalFrame, LincolnGtaModeUIMap, ARRAYNUMBER(LincolnGtaModeUIMap))
static LincolnGtaModeUI* lincolnGtaModeUI = nullptr;

'''
MENU_HANDLERS = '''    long onGtaMode(FXObject*, FXSelector, void*) {
        if (lincolnGtaModeUI) lincolnGtaModeUI->toggle();
        return 1;
    }
    long onUpdateGtaMode(FXObject* sender, FXSelector, void*) {
        auto* item = static_cast<FXMenuCheck*>(sender);
        item->setCheck(lincolnGtaModeUI && lincolnGtaModeUI->isConfirmed());
        if (lincolnGtaModeUI && lincolnGtaModeUI->canToggle()) item->enable();
        else item->disable();
        return 1;
    }
'''


def source_patches():
    """扩展已生成的导航右键菜单，GTA 代码全部保留在本模块。"""
    return {WINDOW_SOURCE: [
        ('class LincolnNavigationButton : public FXToggleButton {',
         GTA_CODE + 'class LincolnNavigationButton : public FXToggleButton {'),
        ('enum { ID_NAVIGATE = FXToggleButton::ID_LAST };',
         'enum { ID_NAVIGATE = FXToggleButton::ID_LAST, ID_GTA_MODE };'),
        ('new FXMenuCommand(&menu, "Navigate here", nullptr, this, ID_NAVIGATE);',
         '''new FXMenuCommand(&menu, "Navigate here", nullptr, this, ID_NAVIGATE);
        if (lincolnGtaModeUI) lincolnGtaModeUI->openMenu(vehicleName);
        auto* gtaItem = new FXMenuCheck(&menu, "GTA Mode", this, ID_GTA_MODE);
        gtaItem->disable();'''),
        ('    long onNavigate(FXObject*, FXSelector, void*) {',
         MENU_HANDLERS + '    long onNavigate(FXObject*, FXSelector, void*) {'),
        ('FXDEFMAP(LincolnNavigationButton) LincolnNavigationButtonMap[] = {',
         '''FXDEFMAP(LincolnNavigationButton) LincolnNavigationButtonMap[] = {
    FXMAPFUNC(SEL_COMMAND, LincolnNavigationButton::ID_GTA_MODE, LincolnNavigationButton::onGtaMode),
    FXMAPFUNC(SEL_UPDATE, LincolnNavigationButton::ID_GTA_MODE, LincolnNavigationButton::onUpdateGtaMode),'''),
        ('lincolnNavigationUI = new LincolnNavigationUI(myStatusbar, this);',
         '''lincolnNavigationUI = new LincolnNavigationUI(myStatusbar, this);
        lincolnGtaModeUI = new LincolnGtaModeUI(myStatusbar, this);'''),
    ]}
