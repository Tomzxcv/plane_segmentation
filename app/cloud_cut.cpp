//
// Created by lhm on 19-8-22.
//
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include "plane_segmentation.h"
#include "rdp.h"

using namespace std;
using namespace cv;


int main(int argc, char **argv)
{
    if(argc != 4)
    {
        std::cout<<"usage : ./cloud_cut basic_path config_file point_cloud_file"<<std::endl;
        return -1;
    }

    std::string basic_path = argv[1];
    std::string config_file = argv[2];
    std::string cloud_file = argv[3];

    std::shared_ptr<Aibee::Config> config = std::make_shared<Aibee::Config>();
    config->load(config_file);
    Aibee::PlaneSegmentation plane_seg(config);

    //cut point cloud by ring
    plane_seg.loadPointCloud(basic_path + "/" + cloud_file);

    plane_seg.cutPointCloud(-1);

    plane_seg.sortPointCloud(-1);

    std::string cmd = "mkdir -p " + basic_path + "/debug";
    int a = system(cmd.c_str());

    Aibee::RDP rdp;
    for(int i=0; i<16; i++){
        std::string file_name = basic_path + "/debug/cut-" + std::to_string(i) + ".ply";
        plane_seg.savePointCloud(file_name, i);

        //rdp test
        std::vector<Eigen::Vector3d> data;
        plane_seg.get_point(i, data);
        std::vector<int> res = rdp.run(data, 0, data.size()-1, 0.04);
        std::cout<<"data size = "<<data.size()<<std::endl;
        std::cout<<"rdp = "<<i<<" res size = "<<res.size()<<std::endl;
        for(int x = 0; x<res.size(); x++)
            std::cout<<"-----"<<data[res[x]].transpose()<<std::endl;
    }

    plane_seg.savePointCloud(basic_path + "/debug/cut-cloud.ply", -1);

    // std::string cut_file_name;
    // for(int i=0; i<16; i++){
    //     cut_file_name = "/home/lhm/aibee_workspace/auto_calibration/data_test/cut_result" + std::to_string(i) + ".txt";
    //     plane_seg.showCutResult(cut_file_name, i);
    // }

    return 0;
}