#pragma once
#include <QMouseEvent>
#include <QGLWidget>
#ifdef WITH_ANTTWEAKBAR
    #include "AntTweakBar.h"
#endif

/// QGLWidget support for AntTweakBar (event forwarding)
class AntTweakBarEventFilter : public QObject{
private:
    QGLWidget* parent;
public:
    AntTweakBarEventFilter(QGLWidget* parent) : QObject(parent){
        this->parent = parent;
        // all move events detected, not just when clicked
        parent->setMouseTracking(true);
    }

    /// Forward events to AntTweakBar
    bool eventFilter(QObject* obj, QEvent* event){
#ifdef WITH_ANTTWEAKBAR
        bool event_on_atb = false;
        if (obj == parent) {
            QMouseEvent* me = dynamic_cast<QMouseEvent*>(event);
            QKeyEvent* ke = static_cast<QKeyEvent*>(event);

            TwMouseButtonID mouse_button;
            if(me!=NULL){
                if(me->button()==Qt::LeftButton) mouse_button = TW_MOUSE_LEFT;
                if(me->button()==Qt::RightButton) mouse_button = TW_MOUSE_RIGHT;
                if(me->button()==Qt::MiddleButton) mouse_button = TW_MOUSE_MIDDLE;
            }

            switch(event->type()){
            case QEvent::Resize:
                event_on_atb = TwWindowSize(parent->width()*2, parent->height()*2);
                // qDebug() << "Resize" F<< status;
                break;
            case QEvent::KeyRelease:
                event_on_atb = TwKeyPressed(ke->key(), TW_KMOD_NONE);
                // qDebug() << "Key" << status;
                break;
            case QEvent::MouseMove:
                event_on_atb = TwMouseMotion(me->pos().x(), me->pos().y());
                // qDebug() << "Move" << status << e->pos();
                break;
            case QEvent::MouseButtonPress:
                event_on_atb = TwMouseButton(TW_MOUSE_PRESSED, mouse_button);
                // qDebug() << "Press" << status;
                break;
            case QEvent::MouseButtonRelease:
                event_on_atb = TwMouseButton(TW_MOUSE_RELEASED, mouse_button);
                // qDebug() << "Release" << status;
                return true;
            default:
                break;
            }
            if(event_on_atb == true){
                parent->update();
                return true;
            }
            else
                return false; // Super::eventFilter(obj, event);
        } else {
            // pass the event on to the parent class
            return false; // Super::eventFilter(obj, event);
        }
#else
        return false;
#endif
    }
};
