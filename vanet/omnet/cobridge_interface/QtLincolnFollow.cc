#include "QtLincolnFollow.h"
#include <omnetpp/cobject.h>
#include <QApplication>
#include <QGraphicsScene>
#include <QMouseEvent>
#include <QKeyEvent>
#include <QWheelEvent>
#include <QScrollBar>
#include <QSignalBlocker>
#include <QToolButton>
#include <QVariant>
#include <QFontMetrics>
#include <cmath>

// OMNeT++ 6.1 Qtenv exposes each module/figure as a cObject* in item data
// slot 1 (src/qtenv/graphicsitems.h, modulecanvasviewer.cc). Using that metadata
// avoids private Qtenv class linkage, zoom conversions and Cmdenv dependencies.
Q_DECLARE_METATYPE(omnetpp::cObject*)
constexpr int QTENV_OBJECT_ITEM_ROLE = 1;

QtLincolnFollow::QtLincolnFollow() {
    QObject::connect(&timer, &QTimer::timeout, this, [this]() { updateView(); });
    timer.start(50);
}
QtLincolnFollow::~QtLincolnFollow() { detach(); }

void QtLincolnFollow::detach() {
    if(view) {
        view->viewport()->removeEventFilter(this);
        view->horizontalScrollBar()->removeEventFilter(this);
        view->verticalScrollBar()->removeEventFilter(this);
    }
    if(button)delete button;
    if(action)delete action;
    action=nullptr;view=nullptr;item=nullptr;pressed=false;
}

void QtLincolnFollow::setTarget(const omnetpp::cObject* value) {
    if(target!=value) {target=value;item=nullptr;}
}

void QtLincolnFollow::attach(QGraphicsView* viewer,QToolBar* toolbar,QGraphicsObject* graphic) {
    if(view!=viewer||!action||!button) {
        detach();view=viewer;
        action=new QAction(QString::fromUtf8("跟随 Lincoln"),toolbar);
        action->setObjectName("vanetFollowLincoln");
        action->setCheckable(true);action->setChecked(enabled);
        action->setToolTip(QString::fromUtf8("保持 Lincoln 居中并保留缩放；拖动地图暂停，再次点击恢复。"));
        // Qtenv's icon-only toolbar elides text actions. A viewport overlay
        // keeps this control readable and independent of map zoom/scrolling.
        // Parent to the view, not its viewport: QGraphicsView scrolling also
        // moves viewport children, which would scroll a resume button offscreen.
        button=new QToolButton(view);
        button->setObjectName("vanetFollowLincolnButton");
        button->setDefaultAction(action);button->setToolButtonStyle(Qt::ToolButtonTextOnly);
        button->setText(action->text());
        QFontMetrics metrics(button->font());
        button->setFixedSize(metrics.horizontalAdvance(action->text())+24,metrics.height()+12);
        button->move(8,8);button->show();button->raise();
        QObject::connect(action,&QAction::toggled,this,[this](bool checked) {setEnabled(checked);});
        view->viewport()->installEventFilter(this);
        view->horizontalScrollBar()->installEventFilter(this);
        view->verticalScrollBar()->installEventFilter(this);
    }
    item=graphic;
}

void QtLincolnFollow::discoverView() {
    if(!qApp||!target)return;
    for(auto* widget:QApplication::allWidgets()) {
        auto* candidate=qobject_cast<QGraphicsView*>(widget);
        if(!candidate||!candidate->isVisible()||!candidate->scene()
                ||!candidate->inherits("omnetpp::qtenv::ModuleCanvasViewer"))continue;
        QWidget* inspector=candidate->parentWidget();
        while(inspector&&!inspector->inherits("omnetpp::qtenv::ModuleInspector"))inspector=inspector->parentWidget();
        auto* toolbar=inspector?inspector->findChild<QToolBar*>():nullptr;
        if(!toolbar)continue;
        for(auto* graphic:candidate->scene()->items()) {
            if(graphic->data(QTENV_OBJECT_ITEM_ROLE).value<omnetpp::cObject*>()==target) {
                if(auto* object=graphic->toGraphicsObject()) {attach(candidate,toolbar,object);return;}
            }
        }
    }
}

void QtLincolnFollow::setEnabled(bool value) {
    enabled=value;
    if(action) {QSignalBlocker blocker(action);action->setChecked(value);}
    if(value)updateView();
}

void QtLincolnFollow::updateView() {
    if(!view||!item||!button||item->scene()!=view->scene())discoverView();
    if(!enabled||pressed||!view||!item||!view->isVisible()||item->scene()!=view->scene())return;
    auto point=item->mapToScene(QPointF());
    if(!std::isfinite(point.x())||!std::isfinite(point.y()))return;
    // Permit centering near map boundaries, including Lincoln's western start.
    // Coordinates already include Qtenv's zoom; never call scale()/fitInView().
    auto visible=view->mapToScene(view->viewport()->rect()).boundingRect();
    QRectF required(point.x()-visible.width()/2.-2,point.y()-visible.height()/2.-2,
                    visible.width()+4,visible.height()+4);
    if(!view->sceneRect().contains(required))view->setSceneRect(view->sceneRect().united(required));
    if((view->mapToScene(view->viewport()->rect().center())-point).manhattanLength()>1.)view->centerOn(point);
}

bool QtLincolnFollow::eventFilter(QObject* watched,QEvent* event) {
    if(!view)return false;
    if(watched==view->viewport()) {
        if(event->type()==QEvent::MouseButtonPress) {
            auto* mouse=static_cast<QMouseEvent*>(event);
            pressed=(mouse->button()==Qt::LeftButton||mouse->button()==Qt::MiddleButton)
                &&!(mouse->modifiers()&(Qt::ControlModifier|Qt::ShiftModifier));
            pressPosition=mouse->pos();
        } else if(event->type()==QEvent::MouseMove&&pressed) {
            auto* mouse=static_cast<QMouseEvent*>(event);
            if((mouse->pos()-pressPosition).manhattanLength()>=QApplication::startDragDistance())setEnabled(false);
        } else if(event->type()==QEvent::MouseButtonRelease||event->type()==QEvent::UngrabMouse)pressed=false;
        else if(event->type()==QEvent::Wheel) {
            if(!(static_cast<QWheelEvent*>(event)->modifiers()&Qt::ControlModifier))setEnabled(false);
        } else if(event->type()==QEvent::KeyPress) {
            int key=static_cast<QKeyEvent*>(event)->key();
            if(key==Qt::Key_Left||key==Qt::Key_Right||key==Qt::Key_Up||key==Qt::Key_Down
                    ||key==Qt::Key_PageUp||key==Qt::Key_PageDown||key==Qt::Key_Home||key==Qt::Key_End)setEnabled(false);
        }
    } else if(event->type()==QEvent::MouseButtonPress||event->type()==QEvent::Wheel||event->type()==QEvent::KeyPress)
        setEnabled(false);
    return false; // Keep Qtenv's native pan/zoom/selection behavior.
}

nlohmann::json QtLincolnFollow::status() const {
    nlohmann::json value={{"enabled",enabled},{"attached",bool(view&&item)},
                          {"control_visible",bool(button&&button->isVisible()&&view&&view->rect().contains(button->geometry()))}};
    if(view&&item) {
        auto center=view->mapToScene(view->viewport()->rect().center());auto point=item->mapToScene(QPointF());
        value["center"]={center.x(),center.y()};value["target"]={point.x(),point.y()};
        value["center_error_pixels"]=std::hypot(view->mapFromScene(point).x()-view->viewport()->rect().center().x(),
                                              view->mapFromScene(point).y()-view->viewport()->rect().center().y());
    }
    return value;
}
