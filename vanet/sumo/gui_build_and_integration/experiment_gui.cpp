class VanetExperimentUI : public FXHorizontalFrame {
    FXDECLARE(VanetExperimentUI)
protected:
    VanetExperimentUI() = default;
private:
    GUIApplicationWindow* window = nullptr;
    FXComboBox* group = nullptr;
    FXLabel* status = nullptr;
    std::future<NavigationJson> pending;
    NavigationJson queuedOperation;
    bool hasQueuedOperation = false;
    bool pendingIsOperation = false;
    bool userOperationPending = false;
    std::string statusText;
    std::string statusDetail;
    bool available = false;
    bool running = false;
    bool prepared = false;
    bool emergency = false;
    int countdown = 0;

    void showStatus(const std::string& text, const std::string& detail) {
        if (text != statusText) {
            status->setText(text.c_str());
            statusText = text;
        }
        if (detail != statusDetail) {
            status->setTipText(detail.c_str());
            if (!detail.empty()) window->setStatusBarText("VANET: " + detail);
            statusDetail = detail;
        }
    }

    void dispatch(NavigationJson command, bool operation) {
        try {
            pending = std::async(std::launch::async, requestLincolnNavigation, std::move(command));
            pendingIsOperation = operation;
        } catch (const std::exception& error) {
            available = false;
            if (operation) userOperationPending = false;
            showStatus("VANET: request failed", error.what());
        }
    }

    void request(const char* action) {
        const bool operation = std::string(action) != "vanet.status";
        if (userOperationPending || (!operation && pending.valid())) return;
        const char* groups[] = {"normal", "emergency", "cooperative"};
        char* resolved = ::realpath(OptionsCont::getOptions().getString("net-file").c_str(), nullptr);
        if (!resolved) {
            available = false;
            showStatus("VANET: network unavailable", "Unable to resolve the current SUMO network file.");
            return;
        }
        const std::string networkFile(resolved); std::free(resolved);
        NavigationJson command = {{"action", action}, {"source", "sumo_gui"},
            {"group", groups[group->getCurrentItem()]}, {"network_file", networkFile}};
        if (operation) userOperationPending = true;
        // 查询期间保持按钮可用；点击的操作在查询结束后优先提交，不丢弃点击。
        if (pending.valid()) {
            queuedOperation = std::move(command);
            hasQueuedOperation = true;
            return;
        }
        dispatch(std::move(command), operation);
    }
public:
    enum { ID_REFRESH = FXHorizontalFrame::ID_LAST, ID_RESET, ID_PREPARE, ID_START, ID_CANCEL, ID_GROUP };
    VanetExperimentUI(FXComposite* parent, GUIApplicationWindow* application) :
        FXHorizontalFrame(parent, LAYOUT_SIDE_BOTTOM | LAYOUT_FILL_X), window(application) {
        new FXLabel(this, "VANET");
        group = new FXComboBox(this, 14, this, ID_GROUP, COMBOBOX_STATIC | FRAME_SUNKEN | FRAME_THICK);
        group->appendItem("1 Normal"); group->appendItem("2 Emergency"); group->appendItem("3 V2V + RSU");
        group->setNumVisible(3); group->setCurrentItem(0);
        new FXButton(this, "Reset scene", nullptr, this, ID_RESET);
        new FXButton(this, "Prepare / Enable", nullptr, this, ID_PREPARE);
        new FXButton(this, "Start experiment", nullptr, this, ID_START);
        new FXButton(this, "Cancel emergency", nullptr, this, ID_CANCEL);
        status = new FXLabel(this, "Connecting...");
        status->setTipText("Select group, reset scene, prepare, then start. Right-click Lincoln to change destination.");
        getApp()->addTimeout(this, ID_REFRESH, 250);
    }
    ~VanetExperimentUI() override { getApp()->removeTimeout(this, ID_REFRESH); }
    long onReset(FXObject*, FXSelector, void*) { request("vanet.reset"); return 1; }
    long onPrepare(FXObject*, FXSelector, void*) { request("vanet.prepare"); return 1; }
    long onStart(FXObject*, FXSelector, void*) { request("vanet.start"); return 1; }
    long onCancel(FXObject*, FXSelector, void*) { request("vanet.cancel"); return 1; }
    long onUpdate(FXObject* sender, FXSelector selector, void*) {
        const auto id = FXSELID(selector);
        bool enabled = available && !userOperationPending;
        if (id == ID_START) enabled = enabled && prepared;
        if (id == ID_RESET || id == ID_GROUP) enabled = enabled && !running && !emergency;
        if (id == ID_PREPARE) enabled = enabled && !running;
        if (id == ID_CANCEL) enabled = enabled && emergency;
        sender->handle(this, FXSEL(SEL_COMMAND, enabled ? ID_ENABLE : ID_DISABLE), nullptr);
        return 1;
    }
    long onRefresh(FXObject*, FXSelector, void*) {
        if (pending.valid() && pending.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            const bool completedOperation = pendingIsOperation;
            try {
                const auto response = pending.get();
                available = response.value("enabled", false);
                const auto phase = response.value("phase", "unavailable");
                running = phase == "running";
                prepared = response.value("prepared", false);
                emergency = response.value("emergency", false);
                const auto detail = response.value("detail", "");
                showStatus("VANET: " + phase + " / " + response.value("group", ""), detail);
            } catch (const std::exception& error) {
                available = false;
                showStatus("VANET: request failed", error.what());
            }
            if (completedOperation) userOperationPending = false;
            if (hasQueuedOperation) {
                hasQueuedOperation = false;
                dispatch(std::move(queuedOperation), true);
            }
            countdown = 2;
        }
        if (!pending.valid() && --countdown <= 0) request("vanet.status");
        getApp()->addTimeout(this, ID_REFRESH, 250);
        return 1;
    }
};
FXDEFMAP(VanetExperimentUI) VanetExperimentUIMap[] = {
    FXMAPFUNC(SEL_TIMEOUT, VanetExperimentUI::ID_REFRESH, VanetExperimentUI::onRefresh),
    FXMAPFUNC(SEL_COMMAND, VanetExperimentUI::ID_RESET, VanetExperimentUI::onReset),
    FXMAPFUNC(SEL_COMMAND, VanetExperimentUI::ID_PREPARE, VanetExperimentUI::onPrepare),
    FXMAPFUNC(SEL_COMMAND, VanetExperimentUI::ID_START, VanetExperimentUI::onStart),
    FXMAPFUNC(SEL_COMMAND, VanetExperimentUI::ID_CANCEL, VanetExperimentUI::onCancel),
    FXMAPFUNCS(SEL_UPDATE, VanetExperimentUI::ID_RESET, VanetExperimentUI::ID_GROUP, VanetExperimentUI::onUpdate),
};
FXIMPLEMENT(VanetExperimentUI, FXHorizontalFrame, VanetExperimentUIMap, ARRAYNUMBER(VanetExperimentUIMap))
