#include <iostream>
#include <fstream>
#include <vector>
#include <liblas/liblas.hpp>
#include "ColorMask.h"
#include "ospray/common/OSPCommon.h"
#include "ParticleModel.h"
// embree
#include "common/sys/filename.h"

namespace ospray {
	namespace las {
		void importModel(ParticleModel *model, const embree::FileName &fileName){
			std::ifstream fin(fileName.c_str(), std::ios::in | std::ios::binary);
			liblas::ReaderFactory rf;
			liblas::Reader reader = rf.CreateWithStream(fin);

			const liblas::Header &header = reader.GetHeader();
			std::cout << "Reading LIDAR data set from " << fileName
				<< " with " << header.GetPointRecordsCount()
				<< " points\n";
			//Offsets we need to apply to position the points properly
			const vec3f scale = vec3f(header.GetScaleX(), header.GetScaleY(), header.GetScaleZ());
			const vec3f offset = vec3f(header.GetOffsetX(), header.GetOffsetY(), header.GetOffsetZ());
			std::cout << "scale: " << scale << ", offset: " << offset << std::endl;

			const float inv_max_color = 1.f / 65536.f;
			//Unfortunately the libLAS designers prefer Java style iterators
			while (reader.ReadNextPoint()){
				const liblas::Point &lasp = reader.GetPoint();
				const liblas::Color lasc = lasp.GetColor();
				const vec3f p = vec3f(lasp.GetX(), lasp.GetY(), lasp.GetZ()) * scale + offset;
				const vec3f c = vec3f(lasc.GetRed() * inv_max_color, lasc.GetGreen() * inv_max_color,
						lasc.GetBlue() * inv_max_color);
				uint32_t col_masked = 0;
				SET_RED(col_masked, static_cast<int>(c.x * 255));
				SET_GREEN(col_masked, static_cast<int>(c.y * 255));
				SET_BLUE(col_masked, static_cast<int>(c.z * 255));
				model->position.push_back(p);
				model->addAttribute("color", *reinterpret_cast<float*>(&col_masked));
			}
		}
	}
}

