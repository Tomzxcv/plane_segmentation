#pragma once

#include <iostream>
#include <Eigen/Core>
#include <vector>
#include <ceres/ceres.h>
#include <glog/logging.h>
#include <unordered_set>
#include <unordered_map>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <yaml-cpp/yaml.h>
#include "clustering.h"

extern const std::vector<Eigen::Vector3i> g_vColors;

namespace Aibee{

#define RING_SIZE  16
#define LINE_MIN_DISTANCE 0.2
#define LINE_MIN_POINT_SIZE 100


struct PointStruct
{
    PointStruct(const Eigen::Vector3d& p){
        _point = p;
        _angle = 0.;
        _line_angle = 0.;
        _k = 0.;
        _cluster = -1;
    }

    Eigen::Vector3d _point;
    double _k;
    double _angle;
    double _line_angle;
    int _cluster;
    int _plane_idx;
};

struct ExponentialResidual
{
    ExponentialResidual(double x, double y) : _x(x), _y(y){}

    template <typename T>
    bool operator()(const T* const k, const T* const b, T* residual) const
    {
        residual[0] = _y - (k[0] * _x + b[0]);
        return true;
    }

private:
    const double _x;
    const double _y;
};

class ExtricCalibCostFunctionPlaneSelf
{
public:
    static ceres::CostFunction* CreateAutoDiffCostFunction(
        const double scale_factor, const Eigen::Vector3d& point, const double dist)
        {
            return new ceres::AutoDiffCostFunction<ExtricCalibCostFunctionPlaneSelf, 1, 3>(
                new ExtricCalibCostFunctionPlaneSelf(scale_factor, point, dist));
        }

        template <typename T>
        bool operator()(const T* const plane, T* residual)const{
            residual[0] = (T)scale_factor_*(point_.cast<T>()(0, 0) * plane[0] +
                                                                               point_.cast<T>()(1, 0) * plane[1] + 
                                                                               point_.cast<T>()(2, 0) * plane[2] - (T)dist_ )/
                                                                               sqrt(plane[0] * plane[0] + plane[1] * plane[1] + plane[2] * plane[2]);
            return true;
        }


private:
    ExtricCalibCostFunctionPlaneSelf(const double scale_factor, const Eigen::Vector3d& point, const double dist):
    scale_factor_(scale_factor), point_(point), dist_(dist){}

    const double scale_factor_;
    const Eigen::Vector3d point_;
    const double dist_;
};

struct Config
{
    bool load(const std::string& file_name)
    {
        std:;cout<<"load config file name : "<<file_name<<std::endl;
        YAML::Node config = YAML::LoadFile(file_name);
        
        if(config["cut_angle_start"])   _cut_angle1 = config["cut_angle_start"].as<double>();
        else _cut_angle1 = 180.;

        if(config["cut_angle_end"])   _cut_angle2 = config["cut_angle_end"].as<double>();
        else _cut_angle2 = 360.;

        if(config["clusteByAngle_radius"])   _clusteByAngle_radius = config["clusteByAngle_radius"].as<double>();
        else _clusteByAngle_radius = 2.;

        if(config["clusteByAngle_minPts"])   _clusteByAngle_minPts = config["clusteByAngle_minPts"].as<int>();
        else _clusteByAngle_minPts = 200;

         if(config["clusteByPoints_radius"])   _clusteByPoints_radius = config["clusteByPoints_radius"].as<double>();
        else _clusteByPoints_radius = 0.3;

        if(config["clusteByPoints_minPts"])   _clusteByPoints_minPts = config["clusteByPoints_minPts"].as<int>();
        else _clusteByPoints_minPts = 100;

        if(config["min_merge_score"])   _min_merge_score = config["min_merge_score"].as<double>();
        else _min_merge_score = 10000.;

        std::cout<<"cut angle = "<<_cut_angle1<<", "<<_cut_angle2<<std::endl;
        std::cout<<"clusteByAngle : radius = "<<_clusteByAngle_radius<<", minPts = "<<_clusteByAngle_minPts<<std::endl;
        std::cout<<"clusteByPoints : radius = "<<_clusteByPoints_radius<<", minPts = "<<_clusteByPoints_minPts<<std::endl;
        std::cout<<"min_merge_score = "<<_min_merge_score<<std::endl;
        return true;
    }

    double _cut_angle1;
    double _cut_angle2;
    double _clusteByAngle_radius;
    int _clusteByAngle_minPts;
    double _clusteByPoints_radius;
    int _clusteByPoints_minPts;
    double _min_merge_score;
};

class PlaneSegmentation
{
public:
    PlaneSegmentation(std::shared_ptr<Config> config);

    std::shared_ptr<Config> _config;

    //load cloud points
    bool loadPointCloud(const std::string& file_name);

    //cut point cloud by angle, ring_idx = -1 ,for all ring numbers point cloud
    void cutPointCloud(int ring_idx);

    //sort point cloud by angle
    void sortPointCloud(int ring_idx);

    //save point cloud
    void savePointCloud(const std::string& out_file_name, int ring_idx);

    bool clusteByk(std::list<PointStruct>& point_input, 
                    std::vector<std::vector<PointStruct>>& point_output, bool is_show = true);

    void clusteByPointCloud(std::vector<std::vector<PointStruct>>& point_input,
                                                        std::vector<std::vector<Eigen::Vector3d>>& point_plane_output,
                                                        const std::string save_path);

    void showCutResult(const std::string& cut_file_name, const int ring_idx);

    //void loadCutResult(const std::string& cut_file_name, std::list<PointStruct>& point_output);

    void loadCutResult(const std::vector<std::vector<Eigen::Vector3d>>& vv_cutPoints, 
                        std::list<PointStruct>& point_output);

    void curve_fitting(std::vector<PointStruct>& data, double& k, double& b);

    bool fit_plane_eigen(const std::vector<Eigen::Vector3d>& points_3d,
                                                Eigen::Vector4d& plane);

    bool fit_plane_opt(const std::vector<Eigen::Vector3d>& points_3d,
                                                 Eigen::Vector4d& plane);

    void get_point(const int ring_idx, std::vector<Eigen::Vector3d>& data);

private:
    std::vector<std::list<PointStruct>> _vvPoints;

    double getAngle(const std::vector<double>& a1, const std::vector<double>& a2);

    bool loadFileTxt(const string& file_name, std::vector<Eigen::Vector3d>& points);

    double compute_distance(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2);
};

}