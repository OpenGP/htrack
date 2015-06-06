#ifndef POSTUREFILE_H
#define POSTUREFILE_H

#include <string>
#include <vector>
#include <fstream>
#include <sstream>

class PostureFile
{
	public:

		PostureFile(const std::string &filename, bool ignorePose = false)
		{
			std::ifstream is(filename.c_str());

			if(!is.is_open())
			{
				std::cout << "[PostureFile] error opening specified file "
				          << filename << std::endl;
				return;
			}

			while(!is.eof())
			{
				std::string line;
				std::getline(is, line);
				std::stringstream s(line);

				if(is.eof())
					return;

				std::vector<float> posture;
				int index = 0;

				while(!s.eof())
				{
					float value;
					s >> value;

					if(!ignorePose || index > 5)
						posture.push_back(value);
					else
						posture.push_back(0.0f);

					++index;
				}

				postures.push_back(posture);
			}
		}

		std::vector< std::vector<float> > postures;

};

#endif

