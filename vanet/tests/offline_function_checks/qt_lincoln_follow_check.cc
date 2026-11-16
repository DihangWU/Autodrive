#include "QtLincolnFollow.h"
#include <QApplication>
#include <QGraphicsScene>
#include <QVBoxLayout>
#include <QMouseEvent>
#include <QPainter>
#include <QToolButton>
#include <cassert>

class CarItem : public QGraphicsObject {
public:
    QRectF boundingRect() const override {return {-3,-2,6,4};}
    void paint(QPainter* painter,const QStyleOptionGraphicsItem*,QWidget*) override {
        painter->drawRect(boundingRect());
    }
};

static void mouse(QWidget* viewport,QEvent::Type type,QPoint position,Qt::MouseButton button,Qt::MouseButtons buttons) {
    QMouseEvent event(type,position,button,buttons,Qt::NoModifier);
    QApplication::sendEvent(viewport,&event);
}

int main(int argc,char** argv) {
    QApplication app(argc,argv);
    QWidget window;auto* layout=new QVBoxLayout(&window);
    auto* toolbar=new QToolBar();auto* view=new QGraphicsView();
    layout->addWidget(toolbar);layout->addWidget(view);window.resize(700,500);
    QGraphicsScene scene;view->setScene(&scene);view->setSceneRect(0,0,1200,900);
    view->setDragMode(QGraphicsView::ScrollHandDrag);
    auto* car=new CarItem();scene.addItem(car);car->setPos(2,3);
    window.show();app.processEvents();
    QtLincolnFollow follow;follow.attach(view,toolbar,car);follow.updateView();
    auto centered=[&]() {assert(follow.status().at("center_error_pixels").get<double>()<=3.);};
    centered();
    auto* action=toolbar->findChild<QAction*>("vanetFollowLincoln");assert(action&&action->isChecked());
    auto* button=view->findChild<QToolButton*>("vanetFollowLincolnButton");
    assert(button&&button->isVisible()&&button->toolButtonStyle()==Qt::ToolButtonTextOnly);
    assert(button->text()==action->text()&&button->width()>70);
    car->setPos(900,700);follow.updateView();centered();
    assert(follow.status().at("control_visible").get<bool>());
    view->scale(2.,2.);auto transform=view->transform();follow.updateView();centered();
    assert(view->transform()==transform);
    // A selection click must not turn following off.
    mouse(view->viewport(),QEvent::MouseButtonPress,{100,100},Qt::LeftButton,Qt::LeftButton);
    mouse(view->viewport(),QEvent::MouseButtonRelease,{100,100},Qt::LeftButton,Qt::NoButton);
    assert(action->isChecked());
    mouse(view->viewport(),QEvent::MouseButtonPress,{100,100},Qt::LeftButton,Qt::LeftButton);
    mouse(view->viewport(),QEvent::MouseMove,{150,130},Qt::NoButton,Qt::LeftButton);
    mouse(view->viewport(),QEvent::MouseButtonRelease,{150,130},Qt::LeftButton,Qt::NoButton);
    assert(!action->isChecked());
    assert(follow.status().at("control_visible").get<bool>());
    auto centre=view->mapToScene(view->viewport()->rect().center());
    car->setPos(300,200);follow.updateView();assert(view->mapToScene(view->viewport()->rect().center())==centre);
    action->trigger();centered();assert(action->isChecked());assert(view->transform()==transform);
    // Relayout/reset can delete the graphics object; the next update must be safe.
    delete car;follow.updateView();assert(!follow.status().at("attached").get<bool>());
    car=new CarItem();scene.addItem(car);car->setPos(30,40);
    follow.attach(view,toolbar,car);follow.updateView();centered();
    delete view;follow.updateView();assert(!follow.status().at("attached").get<bool>());
}
