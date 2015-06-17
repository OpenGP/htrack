#pragma once
#include <QObject>

struct DataFrame;
class Camera;

class Sensor{
protected:
    bool initialized;
    const Camera * camera;

public:   
    Sensor(Camera* camera): initialized(false), camera(camera) {}
    virtual ~Sensor(){}
    virtual bool spin_wait_for_data(float timeout_seconds) = 0;
    virtual bool fetch_streams(DataFrame& frame) = 0;
    virtual void start() = 0;
    virtual void stop() = 0;
private:
    virtual int initialize() = 0;
};

class SensorOpenNI : public Sensor{
public:
    SensorOpenNI(Camera* camera);
    virtual ~SensorOpenNI();
    bool spin_wait_for_data(float timeout_seconds);
    bool fetch_streams(DataFrame& frame);
    void start(){}
    void stop(){}
private:
    int initialize();
};

class SensorSoftKin : public Sensor{
public:
    SensorSoftKin(Camera* camera);
    virtual ~SensorSoftKin();
    bool spin_wait_for_data(float timeout_seconds);
    bool fetch_streams(DataFrame& frame);
    void start(); ///< calls initialize
    void stop();
private:
    int initialize();
};

class SensorDepthSenseGrabber : public Sensor{
public:
    SensorDepthSenseGrabber(Camera* camera);
    virtual ~SensorDepthSenseGrabber();
    bool spin_wait_for_data(float timeout_seconds);
    bool fetch_streams(DataFrame& frame);
    void start(); ///< calls initialize
    void stop();
private:
    int initialize();
};

class SensorRealSense : public Sensor {
public:
	SensorRealSense(Camera* camera);
	virtual ~SensorRealSense();
    bool spin_wait_for_data(float timeout_seconds);
	bool fetch_streams(DataFrame& frame);
	void start(); ///< calls initialize
	void stop();
private:
	int initialize();
};

