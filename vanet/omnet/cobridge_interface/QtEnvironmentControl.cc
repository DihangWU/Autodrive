#include "QtEnvironmentControl.h"
#include <QApplication>
#include <QAction>
#include <QTimer>
#include <QPixmap>

void startQtEnvironment() {
    if(!qApp)return;
    auto* timer=new QTimer(qApp);
    QObject::connect(timer,&QTimer::timeout,[timer]() {
        for(auto* window:QApplication::topLevelWidgets()) {
            auto* action=window->findChild<QAction*>("actionFastRun");
            if(action&&action->isEnabled()) {
                timer->stop();timer->deleteLater();
                action->trigger();return;
            }
        }
    });
    timer->start(100);
}
void stopQtEnvironment() {
    if(qApp)QTimer::singleShot(100,qApp,&QCoreApplication::quit);
}
bool saveQtEnvironment(const std::string& path) {
    if(!qApp)return false;
    for(auto* window:QApplication::topLevelWidgets())
        if(window->findChild<QAction*>("actionFastRun"))return window->grab().save(QString::fromStdString(path));
    return false;
}
