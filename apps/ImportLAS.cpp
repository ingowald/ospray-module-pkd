#include <iostream>
#include <fstream>
#include <vector>
#include <limits>
#include <lasreader.hpp>
#include "ColorMask.h"
#include "ospray/common/OSPCommon.h"
#include "ParticleModel.h"
// embree
#include "common/sys/filename.h"

namespace ospray {
	namespace las {
		enum LIDAR_CLASSIFICATION {
			CREATED = 0,
			UNCLASSIFIED,
			GROUND,
			LOW_VEGETATION,
			MEDIUM_VEGETATION,
			HIGH_VEGETATION,
			BUILDING,
			NOISE,
			MODEL_KEY_POINT,
			WATER,
			OVERLAP_POINT,
			RESERVED
		};
		LIDAR_CLASSIFICATION classify_point(uint8_t class_attrib){
			assert(class_attrib < 32);
			switch (class_attrib){
				case 0: return CREATED;
				case 1: return UNCLASSIFIED;
				case 2: return GROUND;
				case 3: return LOW_VEGETATION;
				case 4: return MEDIUM_VEGETATION;
				case 5: return HIGH_VEGETATION;
				case 6: return BUILDING;
				case 7: return NOISE;
				case 8: return MODEL_KEY_POINT;
				case 9: return WATER;
				case 10: return RESERVED;
				case 11: return RESERVED;
				case 12: return OVERLAP_POINT;
				default: return RESERVED;
			}
		}
		void importModel(ParticleModel *model, const embree::FileName &fileName){
			LASreadOpener read_opener;
			read_opener.set_file_name(fileName.c_str());
			LASreader *reader = read_opener.open();
			if (!reader){
				std::cout << "ImportLAS Error: Failed to open: " << fileName << ", skipping\n";
				return;
			}

			bool has_color = reader->header.point_data_format == 2
				|| reader->header.point_data_format == 3
				|| reader->header.point_data_format == 5;

			std::cout << "LiDAR file '" << fileName
				<< "' contains " << reader->npoints << " points "
				<< (has_color ? "with" : "without") << " color attributes\n"
				<< "min: ( " << reader->get_min_x()
				<< ", " << reader->get_min_y()
				<< ", " << reader->get_min_z() << " )\n"
				<< "max: ( " << reader->get_max_x()
				<< ", " << reader->get_max_y()
				<< ", " << reader->get_max_z() << " )\n";

			const vec3f min_pt(reader->get_min_x(), reader->get_min_y(), reader->get_min_z());
			const vec3f max_pt(reader->get_max_x(), reader->get_max_y(), reader->get_max_z());
			model->lidar_current_bounds.extend(min_pt);
			model->lidar_current_bounds.extend(max_pt);

			int num_noise = 0;
			const float inv_max_color = 1.0f / std::numeric_limits<uint16_t>::max();
			while (reader->read_point()){
				// Points classified as low point are noise and should be discarded
				if (classify_point(reader->point.get_classification()) == NOISE){
					++num_noise;
					continue;
				}
				reader->point.compute_coordinates();
				// The scale/offset is done for us now (right?)
				const vec3f p = vec3f(reader->point.coordinates[0], reader->point.coordinates[1],
						reader->point.coordinates[2]);
				const uint16_t *rgba = reader->point.get_rgb();
				vec3f c;
				if (has_color){
					c = vec3f(rgba[0] * inv_max_color, rgba[1] * inv_max_color, rgba[2] * inv_max_color);
				}
				else {
					c = vec3f(1.0);
				}
				uint32_t col_masked = 0;
				SET_RED(col_masked, static_cast<int>(c.x * 255));
				SET_GREEN(col_masked, static_cast<int>(c.y * 255));
				SET_BLUE(col_masked, static_cast<int>(c.z * 255));
				model->position.push_back(p);
				model->addAttribute("color", *reinterpret_cast<float*>(&col_masked));
			}
			std::cout << "Discarded " << num_noise << " noise classified points\n";
			reader->close();
			delete reader;
		}
	}
}

