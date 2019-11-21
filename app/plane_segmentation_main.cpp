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

extern const std::vector<Eigen::Vector3i> g_vColors;

template <typename T>
void swap(T* a, T* b)
{
    T tmp = *a;
    *a = *b;
    *b = tmp;
    return;
}

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        std::cout<<"usage : ./plane_segmentation_main basic_path config_file cloud_file"<<std::endl;
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

    std::vector<std::vector<Eigen::Vector3d>> vv_cutPoints(16);
    Aibee::RDP rdp;
    for(int i=0; i<16; i++){
        //rdp
        std::vector<Eigen::Vector3d> data;
        plane_seg.get_point(i, data);
        std::vector<int> res = rdp.run(data, 0, data.size()-1, 0.04);
        std::cout<<"data size = "<<data.size()<<std::endl;
        std::cout<<"rdp = "<<i<<" res size = "<<res.size()<<std::endl;
        for(auto idx : res)
            vv_cutPoints[i].push_back(std::move(data[idx]));
    }

    // std::string cut_file_name;
    // for(int i=0; i<16; i++){
    //     cut_file_name = "/home/lhm/aibee_workspace/auto_calibration/data_test/cut_result" + std::to_string(i) + ".txt";
    //     plane_seg.showCutResult(cut_file_name, i);
    // }

    //std::string file_name = basic_path + "/debug/rdp_result";
    std::list<Aibee::PointStruct> points;
    std::vector<std::vector<Aibee::PointStruct>> point_clusteK_output;
    plane_seg.loadCutResult(vv_cutPoints, points);
    //plane_seg.loadCutResult(file_name, points);

    plane_seg.clusteByk(points, point_clusteK_output, false);

    std::vector<std::vector<Eigen::Vector3d>> point_plane_output;
    plane_seg.clusteByPointCloud(point_clusteK_output, point_plane_output, basic_path + "/debug");

    std::vector<Eigen::Vector4d> planes;
    int idx_ = 0;
    for(auto iter_ = point_plane_output.begin(); iter_ != point_plane_output.end();)
    {
        std::cout<<"id = "<<idx_++<<std::endl;
        Eigen::Vector4d plane_;
        bool isOK = plane_seg.fit_plane_opt(*iter_, plane_);
        if(!isOK){
            iter_ = point_plane_output.erase(iter_);
            continue;
        }
        if(plane_[2] < 0){
            plane_ = -plane_;
        }
        planes.push_back(std::move(plane_));
        iter_++;
    }
    std::cout<<"after fiter planes size = "<<point_plane_output.size()<<std::endl;

    //merge planes
    assert(point_plane_output.size() == planes.size());
    const int plane_size = point_plane_output.size();
    for(int p1 = 0; p1 < plane_size; ++p1){
        if(point_plane_output[p1].size() == 0 || planes[p1] == Eigen::Vector4d::Identity())
                continue;

        for(int p2 = p1+1; p2 < plane_size; ++p2){
            
            if(point_plane_output[p2].size() == 0 || planes[p2] == Eigen::Vector4d::Identity())
                continue;

            Eigen::Vector4d plane_tmp1 = planes[p1];
            Eigen::Vector4d plane_tmp2 = planes[p2];
            plane_tmp1 /= plane_tmp1.norm();
            plane_tmp2 /= plane_tmp2.norm();

            double merge_score = std::abs(plane_tmp1.dot(plane_tmp2));
            std::cout<<"merge score : "<<p1<<" "<<p2<<" => "<<merge_score<<std::endl;

            if(merge_score > config->_min_merge_score)
            {
                std::vector<Eigen::Vector3d> new_planes;
                Eigen::Vector4d plane_;
                new_planes.insert(new_planes.end(), point_plane_output[p1].begin(), point_plane_output[p1].end());
                new_planes.insert(new_planes.end(), point_plane_output[p2].begin(), point_plane_output[p2].end());
                bool isOK = plane_seg.fit_plane_opt(new_planes, plane_);
                if(isOK)
                {
                    std::cout<<"merge plane success!"<<std::endl;
                    std::cout<<"id  = "<<p1<<" plane = "<<planes[p1].transpose()<<std::endl;
                    std::cout<<"id  = "<<p2<<" plane = "<<planes[p2].transpose()<<std::endl;
                    std::cout<<"new plane = "<<plane_.transpose()<<std::endl;

                    point_plane_output[p1].swap(new_planes);
                    point_plane_output[p2].clear();
                    planes[p1] = plane_;
                    planes[p2] = Eigen::Vector4d::Identity();
                }
                std::cout<<"try to merge planes, success = "<<(int)isOK<<std::endl;
            }
        }
    }

    //find tag planes
    int show_idx = 0;
    double best_score = 3.;
    std::vector<int> tag_plane_ids{-1, -1, -1};
    for(int a1 = 0; a1 < planes.size(); ++a1){
        for(int a2 = a1 + 1; a2 < planes.size(); ++a2){
            for(int a3 = a2 + 1; a3 < planes.size(); ++a3){
                Eigen::Vector3d normal_1(planes[a1][0], planes[a1][1], planes[a1][2]);
                Eigen::Vector3d normal_2(planes[a2][0], planes[a2][1], planes[a2][2]);
                Eigen::Vector3d normal_3(planes[a3][0], planes[a3][1], planes[a3][2]);
                double d1 = std::abs(normal_1.dot(normal_2));
                double d2 = std::abs(normal_1.dot(normal_3));
                double d3 = std::abs(normal_2.dot(normal_3));
                double score = d1 + d2 + d3;
                if(score < best_score)
                {
                    best_score = score;
                    tag_plane_ids = std::vector<int>{a1, a2, a3};
                }
                std::cout<<show_idx<<", dot_score1 = "<<d1<<", dot_score2 = "<<d2<<", dot_score3 = "<<d3<<std::endl;
                std::cout<<show_idx++<<", idx1 = "<<a1<<", idx2 = "<<a2<<", idx3 = "<<a3<<std::endl;
                std::cout<<"score = "<<score<<std::endl;
            }
        }
    }

    std::cout<<"find tag planes best score = "<<best_score<<std::endl;
    std::cout<<"plane ids : "<<tag_plane_ids[0]<<","<<tag_plane_ids[1]<<","<<tag_plane_ids[2]<<std::endl;
    for(int i=0; i<tag_plane_ids.size(); ++i){
        std::cout<<i<<", plane function = "<<planes[tag_plane_ids[i]].transpose()<<std::endl;
    }

    //Correspondence point cloud to image planes
    //find plane 0
    for(int i_ = 1; i_ < tag_plane_ids.size(); ++i_)
    {
        const Eigen::Vector4d& plane_curr = planes[tag_plane_ids[i_]];
        const Eigen::Vector4d& plane_last = planes[tag_plane_ids[i_-1]];
        if(abs(plane_curr[2]) < abs(plane_last[2]))
        {
            swap(&tag_plane_ids[i_], &tag_plane_ids[i_ - 1]);
        }
    }
    //計算coss product
    const Eigen::Vector3d n1 = planes[tag_plane_ids[0]].block(0, 0, 3, 1);
    const Eigen::Vector3d n2 = planes[tag_plane_ids[1]].block(0, 0, 3, 1);
    Eigen::Vector3d cross = n1.cross(n2);
    if(cross[2] < 0)
    {
        swap(&tag_plane_ids[0], &tag_plane_ids[1]);
    }

    std::cout<<"final plane : "<<std::endl;
    for(int i=0; i<tag_plane_ids.size(); ++i)
    {
        std::cout<<"plane id = "<<i<<", idx = "<<tag_plane_ids[i]<<std::endl;
    }

    pcl::PointCloud<pcl::PointXYZRGB> cloud_out;
    pcl::PLYWriter wt;
    for(int i_ = 0; i_ < tag_plane_ids.size(); ++i_)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud_tmp;
        const Eigen::Vector3i& color = g_vColors[i_];
        const int tag_id = tag_plane_ids[i_];
        pcl::PointXYZRGB pc;
        pcl::PointXYZ pc_tmp;
        for(auto pt_ : point_plane_output[tag_id])
        {
            pc.x = pt_[0];
            pc.y = pt_[1];
            pc.z = pt_[2];
            pc.r = color[0];
            pc.g = color[1];
            pc.b = color[2];
            cloud_out.points.push_back(pc);

            pc_tmp.x = pt_[0];
            pc_tmp.y = pt_[1];
            pc_tmp.z = pt_[2];
            cloud_tmp.push_back(pc_tmp);
        }
        wt.write(basic_path + "/debug/tag_cloud-" + std::to_string(i_) + ".ply", cloud_tmp);
    }
    wt.write(basic_path + "/debug/tag_cloud.ply", cloud_out);

    return 0;
}