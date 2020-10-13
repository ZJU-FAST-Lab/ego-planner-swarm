#ifndef _DYNAMIC_OBS_H
#define _DYNAMIC_OBS_H
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
using namespace std;

class DynamicObs             
{ 
	public:
		DynamicObs(pcl::PointXYZ center, double width, double height, double resolution) \
		: _width(width), _height(height), _resolution(resolution)
		{
			_widNum = ceil(_width/_resolution);
			_heiNum = ceil(_height/_resolution);
			// _cloudMap_ptr = new pcl::PointCloud<pcl::PointXYZ>();
			// _cloudMap_ptr = new pcl::PointCloud<pcl::PointXYZ>;
			update(center);
		}

		~DynamicObs()
		{
			// delete _cloudMap_ptr;
		}

		void update(pcl::PointXYZ center) {
			if(_cloudMap_ptr.size() > 0) {
				_cloudMap_ptr.clear();
			}
			_center_pos.x = center.x;
			_center_pos.y = center.y;
			_center_pos.z = center.z;

			pcl::PointXYZ pt_random;
			for (int r = -_widNum/2.0; r < _widNum/2.0; r++) 
				for (int s = -_widNum/2.0; s < _widNum/2.0; s++) 
					for (int t = 0; t < _heiNum; t++) {
					pt_random.x = _center_pos.x + (r+0.5) * _resolution + 1e-2;
					pt_random.y = _center_pos.y + (s+0.5) * _resolution + 1e-2;
					pt_random.z = _center_pos.z + (t+0.5) * _resolution + 1e-2;
					_cloudMap_ptr.points.push_back(pt_random);
				}
			_cloudMap_ptr.width = _cloudMap_ptr.points.size();
			_cloudMap_ptr.height = 1;
			_cloudMap_ptr.is_dense = true;
		}

		pcl::PointCloud<pcl::PointXYZ> _cloudMap_ptr;
		// typedef unique_ptr<DynamicObs> Ptr;
	private:
		double _width, _height;
		int _widNum, _heiNum;
		pcl::PointXYZ _center_pos;
		double _resolution;
};

#endif