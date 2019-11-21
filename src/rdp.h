#pragma once

#include <vector>
#include <assert.h>
#include <Eigen/Dense>
#include <iostream>

namespace Aibee
{
class RDP
{
public:
    RDP(){}

    std::vector<int> run(const std::vector<Eigen::Vector3d>& data, 
			            int start_idx, int end_idx, const double& epsilon);

};



}