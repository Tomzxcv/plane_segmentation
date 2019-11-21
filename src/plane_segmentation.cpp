#include "plane_segmentation.h"


const std::vector<Eigen::Vector3i> g_vColors{
    Eigen::Vector3i (255,0,0),
    Eigen::Vector3i (0,255,0),
    Eigen::Vector3i (0,0,255),
    Eigen::Vector3i (255,255,0),
    Eigen::Vector3i (255,0,255),
    Eigen::Vector3i (0,255,255),
    Eigen::Vector3i (255,255,255),
    Eigen::Vector3i (255,222,173),
    Eigen::Vector3i (0, 0, 0),
    Eigen::Vector3i (0, 0, 128),
    Eigen::Vector3i (135, 206, 235),
    Eigen::Vector3i (85, 107, 47),
    Eigen::Vector3i (34, 139, 34),
    Eigen::Vector3i (139, 69, 19),
    Eigen::Vector3i (240, 128, 128),
    Eigen::Vector3i (138, 43, 226)
};

namespace Aibee
{

bool compare_point(const PointStruct& first, const PointStruct& second)
{
	return first._angle < second._angle;
}

PlaneSegmentation::PlaneSegmentation(std::shared_ptr<Config> config)
{
	_vvPoints.resize(RING_SIZE);
    _config = config;
}

bool PlaneSegmentation::loadPointCloud(const std::string& file_name)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PLYReader reader;
	reader.read(file_name, *cloud);

	int ring_idx;
	for(auto pt : cloud->points)
	{
		ring_idx = pt.intensity;
		if(ring_idx >= RING_SIZE){
			std::cerr<<"point cloud ring error, ring = "<<ring_idx<<std::endl;
			continue;
		}
		_vvPoints[ring_idx].push_back(PointStruct(Eigen::Vector3d(pt.x, pt.y, pt.z)));
	}
	std::cout<<"load point cloud in file "<<file_name<<std::endl;
	for(int i=0; i<_vvPoints.size(); i++){
		std::cout<<"ring "<<i<<",  point size = "<<_vvPoints[i].size()<<std::endl;
	}
	return true;
}

void PlaneSegmentation::cutPointCloud(int ring_idx)
{
	int start_idx,end_idx;
	if(ring_idx == -1)
	{
		start_idx =0;
		end_idx = RING_SIZE;
	}
	else
	{
		start_idx = ring_idx;
		end_idx = start_idx+1;
	}
	
	std::vector<double> x_axes{1., 0};
    double angle1 = _config->_cut_angle1;
    double angle2 = _config->_cut_angle2;
	std::vector<double> cut_angle{angle1, angle2};
	sort(cut_angle.begin(), cut_angle.end());
	for(int ring_=start_idx; ring_<end_idx; ring_++)
	{
		for(auto iter = _vvPoints[ring_].begin(); iter != _vvPoints[ring_].end(); )
		{
			std::vector<double> pt{iter->_point[0], iter->_point[1]};
			double angle = getAngle(x_axes, pt);
			if(iter->_point[1] < 0)
			{
				angle = 360 - angle;
			}
			if(angle >= cut_angle[0] && angle <= cut_angle[1]){
				iter->_angle = angle;
				iter++;
			}
			else
				iter = _vvPoints[ring_].erase(iter);
		}
	}
	std::cout<<"cut point cloud!"<<std::endl;
	for(int i=0; i<_vvPoints.size(); i++){
		std::cout<<"ring "<<i<<",  point size = "<<_vvPoints[i].size()<<std::endl;
	}
	return;
}


void PlaneSegmentation::sortPointCloud(int ring_idx)
{
	int start_idx,end_idx;
	if(ring_idx == -1)
	{
		start_idx =0;
		end_idx = RING_SIZE;
	}
	else
	{
		start_idx = ring_idx;
		end_idx = start_idx+1;
	}
	for(int ring_=start_idx; ring_<end_idx; ring_++)
	{
		_vvPoints[ring_].sort(compare_point);
	}
}

double PlaneSegmentation::getAngle(const std::vector<double>& a1, const std::vector<double>& a2)
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

void PlaneSegmentation::savePointCloud(const std::string& out_file_name, int ring_idx)
{
	int start_idx,end_idx;
	if(ring_idx == -1)
	{
		start_idx =0;
		end_idx = RING_SIZE;
	}
	else
	{
		start_idx = ring_idx;
		end_idx = start_idx+1;
	}
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
	for(int i=start_idx; i<end_idx; i++)
	{
		for(auto iter = _vvPoints[i].begin(); iter != _vvPoints[i].end(); iter++){
			pcl::PointXYZI cd;
			cd.x = iter->_point[0];
			cd.y = iter->_point[1];
			cd.z = iter->_point[2];
			cd.intensity = i;
			cloud->points.push_back(cd);
		}
	}
	pcl::PLYWriter writer;
	writer.write(out_file_name, *cloud);
	std::cout<<"save "<<cloud->points.size()<<" points in file "<<out_file_name<<std::endl;
}

bool PlaneSegmentation::loadFileTxt(const string& file_name, vector<Eigen::Vector3d>& points)
{
    ifstream is;
    is.open(file_name);
    if(!is.is_open())
    {
        cerr<<"can't open file "<<file_name<<endl;
        return false;
    }

    string line;
    while (getline(is, line))
    {
        double x,y,z;
        istringstream iss(line);
        iss >> x;
        iss >> y;
        iss >> z;
        
        points.push_back(Eigen::Vector3d(x,y,z));

    }
    //cout<<"find "<<points.size()<<" points in file "<<file_name<<endl;
    return true;
}


void PlaneSegmentation::loadCutResult(/*const std::string& cut_file_name,*/ 
                        const std::vector<std::vector<Eigen::Vector3d>>& vv_cutPoints,
                        std::list<PointStruct>& point_output)
{
     //load cut result
    // std::vector<std::vector<Eigen::Vector3d>> vv_cutPoints;
    // for(int i=0; i<16; i++)
    // {
    //     vector<Eigen::Vector3d> cut_points;
    //     string file_name = cut_file_name + to_string(i) + ".txt";
    //     loadFileTxt(file_name, cut_points);
    //     vv_cutPoints.push_back(std::move(cut_points));

    //     cout<<i<<" -->> load " <<vv_cutPoints[i].size()<<" points in file "<<file_name<<endl;
    // }

    

    std::unordered_map<double, std::list<PointStruct>> m_angle_points;
	for(int ring_i=0; ring_i<16; ++ring_i)
    {
        const vector<Eigen::Vector3d>& cut_point = vv_cutPoints[ring_i];
        assert(cut_point.size() >= 2);
        int cut_idx = 1;
        double k = (cut_point[1][1] - cut_point[0][1])/(cut_point[1][0] - cut_point[0][0]);
        double b = cut_point[1][1] - k * cut_point[1][0];
        double angle = atan(k);
        angle = angle*180/M_PI;
        double line_distance =  compute_distance(cut_point[1], cut_point[0]);
        std::vector<PointStruct> last_line_points;
        for(auto iter = _vvPoints[ring_i].begin(); iter != _vvPoints[ring_i].end(); iter++)
        {
            const Eigen::Vector3d& p = iter->_point;
            double distance = compute_distance(cut_point[cut_idx],  p);
            if(distance < 0.0001)
            {
                cut_idx++;
                //TODO
                if(last_line_points.size() > LINE_MIN_POINT_SIZE)
                {
                    double opt_k = k;
                    double opt_b = b;
                    curve_fitting(last_line_points, opt_k, opt_b);
                    double opt_angle = atan(opt_k);
                    opt_angle = opt_angle*180/M_PI;
                    for(int pt_i = 0; pt_i<last_line_points.size(); ++pt_i)
                    {
                        last_line_points[pt_i]._line_angle = opt_angle;
                        m_angle_points[opt_angle].push_back(last_line_points[pt_i]);
                    }
                }
                last_line_points.clear();
                last_line_points = std::vector<PointStruct>();

                if(cut_idx >= cut_point.size())
                {
                    //cout<<"cut_idx : "<<cut_idx<<endl;
                    break;
                }
                line_distance = compute_distance(cut_point[cut_idx], cut_point[cut_idx-1]);
                k = (cut_point[cut_idx][1] - cut_point[cut_idx-1][1])/(cut_point[cut_idx][0] - cut_point[cut_idx-1][0]);
                b = cut_point[cut_idx][1] - k * cut_point[cut_idx][0];
                angle = atan(k);
                angle = angle*180/M_PI;
            }
            else
            {
                if(line_distance < LINE_MIN_DISTANCE)
                    continue;

                PointStruct out_pt(p);
                out_pt._line_angle = angle;
                last_line_points.push_back(std::move(out_pt));

                //last_line_points.push_back(out_pt);
                //m_angle_points[angle].push_back(std::move(out_pt));
            }
        }//for(auto iter = _vvPoints[i].begin(); iter != _vvPoints[i].end(); )
    }//for(int i=0; i<16; i++)

    std::cout<<"find different angle size = "<<m_angle_points.size()<<std::endl;
    for(auto iter = m_angle_points.begin(); iter != m_angle_points.end(); iter++)
    {
        std::cout<<"angle = "<<iter->first<<", point size = "<<iter->second.size()<<std::endl;
        if(iter->second.size() < LINE_MIN_POINT_SIZE)
            continue;
        
        point_output.insert(point_output.end(), iter->second.begin(), iter->second.end());
    }
    std::cout<<"find all point size = "<<point_output.size()<<endl;
    return;
}

bool PlaneSegmentation::clusteByk(std::list<PointStruct>& point_input, 
                            std::vector<std::vector<PointStruct>>& point_output, bool is_show)
{
	 //clustering according to k_angle
    std::vector<ClusterPoint> cluster_point;

    for(auto point_i : point_input)
    {
        std::vector<double> data{point_i._line_angle};
        cluster_point.push_back(ClusterPoint(data));
    }
    
    //n: file number; eps: radius; minPts: min points size
    double radius = _config->_clusteByAngle_radius;
    int minPts = _config->_clusteByAngle_minPts;
    DBCAN dbScan(-1, radius,  minPts, cluster_point);
    dbScan.run();

	//compute cluster size
    int cluster_size = -1;
    std::unordered_map<double, int> m_angle_custerId;
    for(auto p : dbScan.points)
    {
        if(p.cluster > cluster_size)
            cluster_size = p.cluster;

        m_angle_custerId[p._data[0]] = p.cluster;
    }
    cout<<"clusteByk : find cluster size = "<<++cluster_size<<endl;

    if(cluster_size < 1)
    {
        std::cerr<<"can't find cluster in clusteByk, cluster size = "<<cluster_size<<std::endl;
        return false;
    }
    point_output.resize(cluster_size);
    for(auto point_i : point_input)
    {
        double line_angle = point_i._line_angle;
        int cluster_idx = m_angle_custerId[line_angle];
        if(cluster_idx < 0)
            continue;

        point_output[cluster_idx].push_back(point_i);
    }

    if(is_show){
        //show point cloud in each cluster 
        int cluster_idx = 0;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr show_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        for(int i=0; i<point_output.size(); i++)
        {
            std::cout<<"clusteByk : cluster id = "<<i<<", points size = "<<point_output[i].size()<<std::endl;
            pcl::visualization::PCLVisualizer viewer("cloud viewer");
            show_cloud->points.clear();
            
            for(auto pt_ : point_output[i]){
                    pcl::PointXYZRGB point;
                    point.x = pt_._point[0];
                    point.y = pt_._point[1];
                    point.z = pt_._point[2];
                    point.r = 255;
                    point.g = 0;
                    point.b = 0;
                    show_cloud->points.push_back(point);
            }
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(show_cloud);
            viewer.setBackgroundColor(255,255,255);
            viewer.addPointCloud<pcl::PointXYZRGB>(show_cloud, rgb, "points");
            viewer.spin();

            // pcl::PLYWriter wt;
            // wt.write("cut-debug-" + to_string(i) + ".ply", *show_cloud);
        }
    }
	return true;
}

void PlaneSegmentation::clusteByPointCloud(std::vector<std::vector<PointStruct>>& point_input,
                                                        std::vector<std::vector<Eigen::Vector3d>>& point_plane_output,
                                                        const std::string save_path)
{
    for(int cluster_id = 0; cluster_id < point_input.size(); cluster_id++)
    {
        //clustering according to k_angle
        std::vector<ClusterPoint> cluster_point;
        for(const auto& pt : point_input[cluster_id])
        {
            std::vector<double> data{pt._point[0], pt._point[1], pt._point[2]};
            cluster_point.push_back(ClusterPoint(data));
        }
        std::cout<<"clusteByPointCloud : id = "<<cluster_id<<", points size = "<<cluster_point.size()<<std::endl;
        //n: file number; eps: radious; minPts: min points size
        double radius = _config->_clusteByPoints_radius;
        int minPts = _config->_clusteByPoints_minPts;
        DBCAN dbScan(-1, radius,  minPts, cluster_point);
        dbScan.run();

        //compute cluster size
        int cluster_size = -1;
        for(auto p : dbScan.points)
        {
            if(p.cluster > cluster_size)
                cluster_size = p.cluster;
        }
        cout<<"clusteByPointCloud : find cluster size = "<<++cluster_size<<endl;

        //show point cloud in each cluster
        vector<vector<Eigen::Vector3d>> vs_clusterId_cloud(cluster_size);
        for(auto p : dbScan.points)
        {
            if(p.cluster < 0)
                continue;
            
            Eigen::Vector3d pt_cloud;
            pt_cloud[0] = p._data[0];
            pt_cloud[1] = p._data[1];
            pt_cloud[2] = p._data[2];
            vs_clusterId_cloud[p.cluster].push_back(pt_cloud);
        }

        for(int tmp = 0; tmp<cluster_size; tmp++)
        {
            point_plane_output.push_back(std::move(vs_clusterId_cloud[tmp]));
        }
    }
    std::cout<<"find "<<point_plane_output.size()<<" planes!"<<std::endl;

    pcl::PointCloud<pcl::PointXYZRGB> cloud_out;
    for(int i=0; i<point_plane_output.size(); i++)
    {
        std::cout<<"plane "<<i<<"  point size = "<<point_plane_output[i].size()<<std::endl;
        //cloud_out.points.clear();
        int color_idx = i % g_vColors.size();
        pcl::PointXYZRGB pt_color;
        for(auto p : point_plane_output[i])
        {
            pt_color.x = p[0];
            pt_color.y = p[1];
            pt_color.z = p[2];
            pt_color.r =  g_vColors[color_idx][0];
            pt_color.g = g_vColors[color_idx][1];
            pt_color.b = g_vColors[color_idx][2];
            cloud_out.points.push_back(pt_color);
        }
        // pcl::PLYWriter wt;
        // wt.write("plane-debug-" + to_string(i) + ".ply", cloud_out);
    }
    //std::cout<<"find plane cloud points size = "<<cloud_out.points.size()<<std::endl;
    pcl::PLYWriter writer;
    writer.write(save_path + "/plane_cloud.ply", cloud_out);

    return;
}

void PlaneSegmentation::curve_fitting(std::vector<PointStruct>& data, double& k, double& b)
{
    std::cout<<"Initial k = "<<k<<", b = "<<b<<std::endl;
    ceres::Problem problem;
    for(int i=0; i<data.size(); ++i)
    {
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<ExponentialResidual, 1, 1, 1>(
                new ExponentialResidual(data[i]._point[0], data[i]._point[1])), NULL, &k, &b);
    }

    ceres::Solver::Options options;
    options.max_num_iterations = 25;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout<<summary.BriefReport()<<std::endl;
    std::cout<<"Final k = "<<k<<", b = "<<b<<std::endl;
    return;
}


void PlaneSegmentation::showCutResult(const std::string& cut_file_name, const int ring_idx)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::visualization::PCLVisualizer viewer("cloud viewer");
    // pcl::PLYReader Reader;
    // Reader.read(file_name_cut, *cloud);

    vector<Eigen::Vector3d> cut_points;
    loadFileTxt(cut_file_name, cut_points);

    assert (cut_points.size() > 1);
    int cut_idx = 1;
    int color_idx = 0;
    for(auto& pt : _vvPoints[ring_idx])
    {
        pcl::PointXYZRGB pt_color;
        Eigen::Vector3d cut = cut_points[cut_idx];
        double distance = pow(cut[0] - pt._point[0], 2) + pow(cut[1] - pt._point[1], 2) + pow(cut[2] - pt._point[2], 2);
        distance = sqrt(distance);

        if(distance < 0.0001)
        {
                if(++cut_idx >= cut_points.size())
                    cerr<<"cut index error, cut_idx = "<<cut_idx<<", cut_points size = "<<cut_points.size()<<endl;

                color_idx = (++color_idx) % g_vColors.size(); 
                cout<<"color idx = "<<color_idx<<endl;
        }
        else
        {
            pt_color.x = pt._point[0];
            pt_color.y = pt._point[1];
            pt_color.z = pt._point[2];
            pt_color.r = g_vColors[color_idx][0];
            pt_color.g = g_vColors[color_idx][1];
            pt_color.b = g_vColors[color_idx][2];
            cloud->points.push_back(pt_color);
        }
    }
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer.setBackgroundColor(255,255,255);
    viewer.addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "points");
    viewer.spin();

    pcl::PLYWriter writer;
    writer.write(cut_file_name + ".ply", *cloud);
    return;
}

double PlaneSegmentation::compute_distance(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2)
{
    double distance = pow(v1[0] - v2[0], 2) + pow(v1[1]-v2[1], 2) + pow(v1[2] - v2[2], 2);
    distance = sqrt(distance);
    return distance;
}

bool PlaneSegmentation::fit_plane_eigen(const std::vector<Eigen::Vector3d>& points_3d,
                                                Eigen::Vector4d& plane)
{
    if(points_3d.size() < 3)
        return false;

    Eigen::MatrixXd A(points_3d.size(), 3);
    Eigen::VectorXd b = Eigen::VectorXd::Constant(points_3d.size(), 1.);
    for(int i=0; i<points_3d.size(); ++i)
    {
        A.block(i, 0, 1, 3) = points_3d[i].transpose();
    }
    Eigen::JacobiSVD<Eigen::MatrixXd> solve_A(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Vector3d plane_normal = solve_A.solve(b);
    double norm = plane_normal.norm();
    plane.block(0, 0, 3, 1) = plane_normal / norm;
    plane(3, 0) = 1. / norm;
    //std::cout<<"fit_plane_eigen find plane = "<<plane.transpose()<<std::endl;
    return true;
}

bool PlaneSegmentation::fit_plane_opt(const std::vector<Eigen::Vector3d>& points_input,
                                                                                Eigen::Vector4d& plane)
{
    if(points_input.size() < 300)
    {
        std::cout<<"fit_plane_opt : points 3d too few, points 3d size = "<<points_input.size()<<std::endl;
        return false;
    }
    //RANSAC, to find inliers
    const int point_size = points_input.size();
    const int max_iteration = 100;
    const double inlier_ratio_thresh = 0.85;
    const double min_error_thresh = 0.014;

    std::vector<Eigen::Vector3d> best_inliers_points;
    int best_inliers_size = 0;
    Eigen::Vector4d best_plane;

    int current_iteration = 0;
    std::srand((unsigned)time(NULL));
    while(current_iteration < max_iteration)
    {
        //choose init points for fit plane
        std::vector<Eigen::Vector3d> points_data;
        Eigen::Vector4d plane;
        for(int i=0; i<10; ++i)
        {
            int rand_idx = std::rand() % point_size;
            points_data.push_back(points_input[rand_idx]);
        }
        //compute plane and count inliers
        int inliers = 0;
        std::vector<Eigen::Vector3d> inliers_points;
        fit_plane_eigen(points_data, plane);
        for(const auto& pt_ : points_input)
        {
            Eigen::Vector4d p_tmp;
            p_tmp.block(0, 0, 3, 1) = pt_;
            p_tmp(3, 0) = -1.;
            double distance = std::abs(p_tmp.dot(plane));
            if(distance < min_error_thresh)
            {
                inliers_points.push_back(pt_);
                inliers++;
            }
        }
        if(inliers > best_inliers_size)
        {
            best_inliers_size = inliers;
            best_inliers_points.swap(inliers_points);
            best_plane = plane;

            if((double)inliers/(double)point_size > inlier_ratio_thresh)
            {
                current_iteration += 10;
            }
        }
        current_iteration++;
    }

    std::cout<<"point size = "<<point_size<<",  best inliers size = "<<best_inliers_size<<std::endl;
    std::cout<<"before optimize plane = "<<best_plane.transpose()<<std::endl;
    //optimization
    ceres::Problem::Options problem_options;
    ceres::Problem problem(problem_options);
    problem.AddParameterBlock(best_plane.data(), 3, nullptr);

    best_inliers_points.clear();
    for(auto pt_ : points_input)
    {
        // Eigen::Vector4d p_tmp;
        // p_tmp.block(0, 0, 3, 1) = pt_;
        // p_tmp(3, 0) = -1.;
        problem.AddResidualBlock(ExtricCalibCostFunctionPlaneSelf::CreateAutoDiffCostFunction(
            1.0, pt_, best_plane[3]), new ceres::HuberLoss(0.025), best_plane.data());
        
        best_inliers_points.push_back(pt_);
    }
    ceres::Solver::Summary summary;
    ceres::Solver::Options solver_options;
    solver_options.linear_solver_type = ceres::DENSE_QR;
    solver_options.max_num_iterations = 200;
    solver_options.num_threads = 4;
    solver_options.function_tolerance = 1e-24;
    solver_options.parameter_tolerance = 1e-24;
    solver_options.gradient_tolerance = 1e-24;
    ceres::Solve(solver_options, &problem, &summary);
    Eigen::Vector3d best_normal = best_plane.block(0, 0, 3, 1);
    plane = best_plane / best_normal.norm();

    std::cout<<"after  optimize plane = "<<plane.transpose()<<std::endl;
    return true;
}

void PlaneSegmentation::get_point(const int ring_idx, 
                    std::vector<Eigen::Vector3d>& data)
{
    for(auto iter = _vvPoints[ring_idx].cbegin(); 
        iter != _vvPoints[ring_idx].cend(); iter++){
        data.push_back(iter->_point);
    }
    return;
}

}
