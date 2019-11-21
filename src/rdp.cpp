#include "rdp.h"

namespace Aibee
{

std::vector<int> RDP::run(const std::vector<Eigen::Vector3d>& data, 
			  			  int start_idx, int end_idx, const double& epsilon)
{
	std::vector<int> result;
	assert(start_idx >= 0);
	assert(end_idx < data.size());
	if(start_idx >= end_idx)
		return std::vector<int>();

	double max_dist = 0.;
	int max_idx = -1;
	Eigen::Vector3d line = data[end_idx] - data[start_idx];
	double line_dist = line.norm();
	for(int i = start_idx + 1; i < end_idx; ++i)
	{
		Eigen::Vector3d to_point = data[i] - data[start_idx];
		double distance = to_point.cross(line).norm() / line_dist;
		if(distance > max_dist){
			max_dist = distance;
			max_idx = i;
		}
	}
	std::cout<<"max distance = "<<max_dist<<", max idx = "<<max_idx<<std::endl;
	if(max_dist > epsilon){
		result = run(data, start_idx, max_idx, epsilon);
		std::vector<int> res1 = run(data, max_idx, end_idx, epsilon);
		result.insert(result.end(), res1.begin()+1, res1.end());
	}
	else{
		result = {start_idx, end_idx};
	}
	//sort(result.begin(), result.end());
	
	//std::cout<<"idx = "<<start_idx <<"  "<<end_idx<<" size = "<<result.size()<<std::endl;
	return result;
}















}