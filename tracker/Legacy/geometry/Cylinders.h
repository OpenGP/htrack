#pragma once

#include <map>
#include <string>
#include "Joint.h"
#include "Skeleton.h"

//===========================================================================//

struct Segment
{
	Joint *joint;       // associated joint in skeleton
	float length;       // length of the cylinder segment
	float radius1;      // radius at the start/base of the segment
	float radius2;      // radius at the end/top of the segment
    float radius1_y; 
	float radius2_y; 
    Segment(Joint *j) : joint(j) {}
	float distanceTo(const Vec3f &point, Vec3f &nearestPoint, Vec3f &normal);
};

//===========================================================================//

class Cylinders
{
	public:

		Cylinders(Skeleton *skeleton, bool hand = true);
		~Cylinders();

		void update() {
			float s = skeleton->getScaleMagnitude();
			if(scale != s) {
				for(size_t i = 0; i < segments.size(); ++i) {
					segments[i].radius1 *= s / scale;
					segments[i].radius2 *= s / scale;
					segments[i].length  *= s / scale;
				}
				scale = s;
			}
		}

		float nearest(const Vec3f &point,
			Vec3f &nearestPoint, Vec3f &normal,
            std::string &joint);

		void transform(const Mat4f &matrix) {
			skeleton->transform(matrix);
		}

		Skeleton *getSkeleton() {
			return skeleton;
		}

		std::vector<Segment>& getSegments() {
			return segments;
		}

        void setSegments(vector<Segment> segments) {
            for (int i = 0; i < segments.size(); ++i) {
                this->segments[i].radius1 = segments[i].radius1;
                this->segments[i].radius2 = segments[i].radius2;
                this->segments[i].radius1_y = segments[i].radius1_y;
                this->segments[i].radius2_y = segments[i].radius2_y;
                this->segments[i].length = segments[i].length;
            }
        }

        Segment &getSegmentByID(int id) {
            static Segment nullseg(NULL);
            if(ids.find(id) == ids.end()) {
                return nullseg;
            } else {
                return segments[ids[id]];
            }
        }

        ///--- reload segments parameters
		void handSegments();
        void recomputeLengths();

        float *serialize();

	private:

		Skeleton *skeleton;
		std::vector<Segment> segments;
        std::map<int, int> ids;
		float scale;
        float *serial_ptr;

};
