#include "Skeleton_IO.h"

#include <fstream>
#include <iostream>
#include "tracker/Legacy/geometry/Skeleton.h"
using namespace std;

//===========================================================================//

void Skeleton_IO::read(const std::string &filename)
{
	_skeleton = new Skeleton();
	
	ifstream is(filename.c_str());

	while(!is.eof())
	{
		string line;
		getline(is, line);
		stringstream s(line);

		if(is.eof() || !line.compare("end_of_skeleton"))
			break;

		while(!s.eof())
		{
			string name;
			s >> name;

			string parent;
			s >> parent;

			Mat4f m;
			s >> m(0,0) >> m(1,0) >> m(2,0) >> m(3,0) 
			  >> m(0,1) >> m(1,1) >> m(2,1) >> m(3,1) 
			  >> m(0,2) >> m(1,2) >> m(2,2) >> m(3,2) 
			  >> m(0,3) >> m(1,3) >> m(2,3) >> m(3,3);

			Joint *j = new Joint(name);
			j->setTransformation(m);

			if(!name.compare("root"))
			{
				_skeleton->setRoot(j);
			}
			else
			{
				Joint *p = _skeleton->getJoint(parent);

				if(p)
				{
					j->setParent(p);
					p->addChild(j);
				}

				_skeleton->addJoint(j);
			}
		}
	}

	_skeleton->getRoot()->update();
	_skeleton->setInitialTransformations();

	is.close();
}

//===========================================================================//

void Skeleton_IO::write(const std::string &filename)
{
	ofstream of(filename.c_str());

	for(Joint *j : _skeleton->getJoints_())
	{
		of << j->getName() << " ";

		if(j->getParent() != NULL)
			of << j->getParent()->getName() << " ";
		else
			of << "root ";

		const Mat4f &m = _skeleton->getInitialTransformation(j->getName());
		of << m(0,0) << " " << m(1,0) << " " << m(2,0) << " " << m(3,0) << " "
		   << m(0,1) << " " << m(1,1) << " " << m(2,1) << " " << m(3,1) << " "
		   << m(0,2) << " " << m(1,2) << " " << m(2,2) << " " << m(3,2) << " "
		   << m(0,3) << " " << m(1,3) << " " << m(2,3) << " " << m(3,3) << endl;
	}

    of << "end_of_skeleton" << endl;

	of.close();
}

//===========================================================================//

