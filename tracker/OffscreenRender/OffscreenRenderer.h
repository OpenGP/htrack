#pragma once
#include "tracker/ForwardDeclarations.h"
#include "tracker/Types.h"

class OffscreenRenderer{
protected:
    Camera* camera = NULL;
    Cylinders *cylinders;
public:
    CustomFrameBuffer* fb = NULL; ///< TODO: private

public:
    void init(Camera *camera, Cylinders* cylinders);
    void render_offscreen(bool reinit=false);
};

