#include "DebugRenderer.h"

struct Points : public CloudRenderer{
    Points(Matrix_3xN& points) {
        this->init(); ///< compile shaders
        setup(&points, NULL);
    }
    Points(const std::vector<Vector3>& points, const std::vector<Vector3>& colors){
        this->init();
        Matrix_3xN _points(3,points.size());
        for(int i=0; i<points.size(); i++)
            _points.col(i) = points[i];
        Matrix_3xN _colors(3,colors.size());
        for(int i=0; i<colors.size(); i++)
            _colors.col(i) = colors[i];
        setup(&_points, &_colors);
    }
    Points(const std::vector<Vector3>& points, Vector3 color){
        this->init(); ///< compile shaders
        Matrix_3xN data(3,points.size());
        for(int i=0; i<points.size(); i++)
            data.col(i) = points[i];
        Matrix_3xN colors(3,points.size());
        for(int i=0; i<3; i++)
            colors.row(i).array().setConstant(color(i));
        setup(&data, &colors);
    }
};

struct Segments : public Segment_renderer{
    Segments(const std::vector<pair<Vector3, Vector3>>& segments, const std::vector<Vector3>& colors){
        this->init();
        Matrix_3xN _segments(3,segments.size()*2);
        for(int i=0; i<segments.size(); i++){
            _segments.col(i*2) = segments[i].first;
            _segments.col(i*2+1) = segments[i].second;
        }
        Matrix_3xN _colors(3,colors.size()*2);
        for(int i=0; i<colors.size(); i++){
            _colors.col(i*2) = colors[i];
            _colors.col(i*2+1) = colors[i];
        }
        setup(&_segments, &_colors);
    }
    Segments(const std::vector<pair<Vector3, Vector3>>& segments, Vector3 color){
        this->init(); ///< compile shaders
        Matrix_3xN _segments(3,segments.size()*2);
        for(int i=0; i<segments.size(); i++){
            _segments.col(i*2) = segments[i].first;
            _segments.col(i*2+1) = segments[i].second;
        }
        Matrix_3xN _colors(3, segments.size()*2);
        for(int i=0; i<3; i++)
            _colors.row(i).array().setConstant(color(i));
        setup(&_segments, &_colors);
    }
};

void DebugRenderer::add_points(const std::vector<Vector3> &points, Vector3 color){ add( new Points(points, color) ); }

void DebugRenderer::add_points(const std::vector<Vector3> &points, const std::vector<Vector3> &colors){ add( new Points(points, colors) ); }

void DebugRenderer::add_segments(const std::vector<pair<Vector3, Vector3> > &segments, const std::vector<Vector3> &colors){ add( new Segments(segments, colors) ); }

void DebugRenderer::add_segments(const std::vector<pair<Vector3, Vector3> > &segments, Vector3 color){ add( new Segments(segments, color) ); }
