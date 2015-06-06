#pragma once
namespace Convert{
    inline Vector2i tr(QPoint p1){ return Vector2i(p1.x(), p1.y()); }
    inline QPoint tr(Vector2i p1){ return QPoint(p1.x(), p1.y()); }
    inline BBox2i tr(QRect rect){ return BBox2i(tr(rect.topLeft()), tr(rect.bottomRight())); }
    inline QRect tr(BBox2i rect){ return QRect(tr(rect.corner(BBox2i::TopLeft)), tr(rect.corner(BBox2i::BottomRight))); }
}