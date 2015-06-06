///--- Global Forward Declaration of classes

/// @note keep this super-lightweight (no includes here)
#pragma once

/// Forward Declares
struct DataFrame;
class Cloud;
class Camera;
class Cylinders;
class Detector;
class DataStream;
class Worker;
class SkeletonSerializer;
class Tracker;
class SolutionStream;
class DetectionStream;
class FindFingers;
class SolutionQueue;
class HandFinder;
class DetectionStream;
class OffscreenRenderer;
class TrivialDetector;
class Skeleton; ///< Legacy
class ICP; ///< Legacy

class Sensor;
class SensorOpenNI;
class SensorSoftKin;

/// UI/OpenGL
class TwSettings;
class QGLWidget;
class ColorTexture8UC3;
class DepthTexture16UC1;

/// Rendering
class DebugRenderer;
class CloudRenderer;
class Cylinders_renderer;
class CustomFrameBuffer;
class QuadRenderer;
class KinectDataRenderer;

/// Externally Defined
namespace cv{ class Mat; }
namespace cudax{class CudaHelper;}
