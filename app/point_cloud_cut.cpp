//
// Created by lhm on 19-8-22.
//
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>

using namespace std;
using namespace cv;

struct PointCloud
{
    PointCloud(double angle, int ring, const Eigen::Vector3f& pt):_angle(angle),_point(pt), _ring(ring){}

    double _angle;
    Eigen::Vector3f _point;
    int _ring;
};

bool compare_pc(const PointCloud& p1, const PointCloud& p2)
{
    return p1._angle < p2._angle;
}

double getAngle(const std::vector<double>& a1, const std::vector<double>& a2)
{
    assert(a1.size() == a2.size());
    double normal_a1 = 0., normal_a2 = 0.;
    double cross = 0.;
    for(int i=0; i<a1.size(); i++)
    {
        cross += a1[i] * a2[i];
        normal_a1 += a1[i] * a1[i];
        normal_a2 += a2[i] * a2[i];
    }
    normal_a1 = sqrt(normal_a1);
    normal_a2 = sqrt(normal_a2);
    double angle = cross / (normal_a1 * normal_a2);
    return acos(angle) * 180 /M_PI;
}
int main(int argc, char **argv)
{

    vector<double> cut_angle{180,360};
    sort(cut_angle.begin(), cut_angle.end());
  
    string basic_path = "/home/lhm/aibee_workspace/auto_calibration/data/";

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    vector<pcl::PointCloud<pcl::PointXYZI>> v_cloud_out (16);
    pcl::PLYReader Reader;
    Reader.read(basic_path + "spinning.ply", *cloud);

    std::vector<double> x_axes{1., 0};
    vector<std::vector<PointCloud>> vv_SortPoints(16, vector<PointCloud>());
    for(int i=0; i<cloud->points.size(); i++)
    {
        double x = cloud->points[i].x;
        double y = cloud->points[i].y;
        vector<double> pt{x, y};
        double angle = getAngle(x_axes, pt);
        if(y < 0)
        {
            angle = 360 - angle;
        }
        
        if(angle >= cut_angle[0] && angle <= cut_angle[1])
        {
            //cloud_out->points.push_back(cloud->points[i]);
            Eigen::Vector3f point(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
            int ring  = cloud->points[i].intensity;
            vv_SortPoints[ring].push_back(move(PointCloud(angle, ring, point)));
        }
    }

    pcl::PLYWriter writer;
    for(int i=0; i<16; i++){
        sort(vv_SortPoints[i].begin(), vv_SortPoints[i].end(), compare_pc);

        for(auto pt_i : vv_SortPoints[i])
        {
            pcl::PointXYZI pt;
            pt.x = pt_i._point[0];
            pt.y = pt_i._point[1];
            pt.z = pt_i._point[2];
            pt.intensity = pt_i._ring;
            v_cloud_out[i].points.push_back(pt);
        }
        writer.write(basic_path + "spinning_cut_sort-" + std::to_string(i) + ".ply", v_cloud_out[i]);
    }
    cout<<"point cloud cut success!"<<endl;

    return 0;
}