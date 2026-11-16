#pragma once
#include <QObject>
#include <QPointer>
#include <QPoint>
#include <QTimer>
#include <QGraphicsObject>
#include <QGraphicsView>
#include <QAction>
#include <QToolBar>
#include <QToolButton>
#include <nlohmann/json.hpp>

namespace omnetpp { class cObject; }

// GUI-only camera control. Never advances or reads the simulation clock.
class QtLincolnFollow : public QObject {
    QTimer timer;
    QPointer<QGraphicsView> view;
    QPointer<QGraphicsObject> item;
    QPointer<QAction> action;
    QPointer<QToolButton> button;
    const omnetpp::cObject* target = nullptr;
    bool enabled = true;
    bool pressed = false;
    QPoint pressPosition;
    void discoverView();
    void detach();
    bool eventFilter(QObject* watched, QEvent* event) override;
public:
    QtLincolnFollow();
    ~QtLincolnFollow() override;
    void setTarget(const omnetpp::cObject* value);
    void setEnabled(bool value);
    void updateView();
    void attach(QGraphicsView* viewer, QToolBar* toolbar, QGraphicsObject* graphic);
    nlohmann::json status() const;
};
