#pragma once
#include <QGLFormat>
class OpenGL32Format : public QGLFormat {
public:
    OpenGL32Format() {
        setVersion(3,2);
        setProfile(QGLFormat::CoreProfile);
#if 0
        /// Enable Anti-Aliasing
        setSampleBuffers(true);
        setAlpha(true);
#else
        setSampleBuffers(false); ///< no anti-aliasing
        setAlpha(false);
#endif
        setSwapInterval(0);
    }
};
