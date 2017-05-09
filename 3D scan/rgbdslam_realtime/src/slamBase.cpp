#include "slamBase.h"

int PCDtoPLYconvertor(string &input_filename ,string &output_filename)
{
    pcl::PCLPointCloud2 cloud;
    if (pcl::io::loadPCDFile(input_filename , cloud) < 0)
    {
        cout << "Error: cannot load the PCD file!!!"<< endl;
        return -1;
    }

    pcl::PLYWriter writer;
    writer.write(output_filename, cloud, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(),true,true);
    return 0;
}

PointCloud::Ptr image2PointCloud(cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& camera)
{
    // 最终返回的点云对象
    PointCloud::Ptr cloud (new PointCloud);
    int temp;
    // 首先是遍历深度图
    for (int m = 0; m < depth.rows; m+=2)
        for (int n = 0; n < depth.cols; n+=2)
        {
            // 获取深度图中(m,n)处的值
            ushort d = depth.ptr<ushort>(m)[n];
            // d 可能没有值，若如此，跳过此点
            if (d == 0)
                continue;
            // d 存在值，则向点云增加一个点
            temp = d;
            PointT p;

            // 计算这个点的空间坐标
            p.z = double(d) / camera.scale;
            p.x = (n - camera.cx) * p.z / camera.fx;
            p.y = (m - camera.cy) * p.z / camera.fy;
            
            // 从rgb图像中获取它的颜色
            // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
            p.b = rgb.ptr<uchar>(m)[n*3];
            p.g = rgb.ptr<uchar>(m)[n*3+1];
            p.r = rgb.ptr<uchar>(m)[n*3+2];

            // ---------------  for realsense
            //p.y = -p.y;
            //p.x = -p.x;
            // --------------------------------
            
            // 把p加入到点云中
            cloud->points.push_back(p);
        }
    // save point cloud
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;

    // normaly, the number of the value will not be larger than 4
//#ifdef DEBUG
    cout << BLUE"distance : " << temp;
    cout << " | point : " << cloud->points[1].x << " " << cloud->points[1].y << " " << cloud->points[1].z << endl;
//#endif
    return cloud;
}

cv::Point3f point2dTo3d(cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera)
{
    cv::Point3f p; // 3D 点
    p.z = double(point.z) / camera.scale;
    p.x = (point.x - camera.cx) * p.z / camera.fx;
    p.y = (point.y - camera.cy) * p.z / camera.fy;
    return p;
}

// computeKeyPointsAndDesp, get keypoints and descriptor
void computeKeyPointsAndDesp(FRAME& frame, string detector, string descriptor)
{
    cv::Ptr<cv::FeatureDetector> _detector;
    cv::Ptr<cv::DescriptorExtractor> _descriptor;

    _detector = cv::FeatureDetector::create(detector.c_str());
    _descriptor = cv::DescriptorExtractor::create(descriptor.c_str());

    if (!_detector || !_descriptor)
    {
        cerr << "Unknown detector or discriptor type !" << detector << "," << descriptor << endl;
        return;
    }

    _detector->detect(frame.rgb, frame.kp);
    _descriptor->compute(frame.rgb, frame.kp, frame.desp);
    return;
}

// estimateMotion 计算两个帧之间的运动
// 输入：帧1和帧2
// 输出：rvec 和 tvec，分别表示R旋转矩阵和t平移向量
RESULT_OF_PNP estimateMotion(FRAME& frame1, FRAME& frame2, CAMERA_INTRINSIC_PARAMETERS& camera)
{
    static ParameterReader pd;

    // 进行描述子的匹配工作，并用matches储存匹配结果
    vector< cv::DMatch > matches;
    cv::BFMatcher matcher;
    matcher.match(frame1.desp, frame2.desp, matches);
/*    
    cv::Mat img_matches; 
    cv::drawMatches(frame1.rgb, 
                    frame1.kp, 
                    frame2.rgb, 
                    frame2.kp,
                    matches, 
                    img_matches, 
                    cv::Scalar::all(-1), 
                    cv::Scalar::all(-1),  
                    vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    imshow("ORB Features", img_matches);  
    cv::waitKey();
*/
    RESULT_OF_PNP result;
    vector< cv::DMatch > goodMatches;
    double minDis = 9999;
    double good_match_threshold = atof( pd.getData( "good_match_threshold" ).c_str() );
    for(size_t i = 0;i < matches.size(); i++)
    {
        if (matches[i].distance < minDis)
            minDis = matches[i].distance;
    }
    
    cout << "min distance is : " << minDis << endl;
    if (minDis < 10) 
        minDis = 10;

    for (size_t i = 0; i < matches.size(); i++)
    {
        if (matches[i].distance < good_match_threshold * minDis)
            goodMatches.push_back(matches[i]);
    }

    cout << "matches number is : " << matches.size() << ", good matches number is : " << goodMatches.size() << endl;

    if (goodMatches.size() <= 5) 
    {
        result.inliers = -1;
        return result;
    }
    // 第一个帧的三维点
    vector<cv::Point3f> pts_obj;
    // 第二个帧的图像点
    vector< cv::Point2f > pts_img;

    // 对于ORB中match上的点进行重新的2d到3d的反投影，构建稀疏点云，用来估计位姿
    for (size_t i = 0; i < goodMatches.size(); i++)
    {
        // query 是第一个, train 是第二个
        cv::Point2f p = frame1.kp[goodMatches[i].queryIdx].pt;

        // 获取d是要小心！x是向右的，y是向下的，所以y才是行，x是列！
        ushort d = frame1.depth.ptr<ushort>(int(p.y))[int(p.x)];
        if (d == 0)
            continue;
        pts_img.push_back(cv::Point2f(frame2.kp[goodMatches[i].trainIdx].pt));

        // 将(u,v,d)转成(x,y,z)
        cv::Point3f pt(p.x, p.y, d);
        cv::Point3f pd = point2dTo3d(pt, camera);
        pts_obj.push_back(pd);
    }

    if (pts_obj.size() == 0 || pts_img.size() == 0)
    {
        result.inliers = -1;
        return result;
    }

    double camera_matrix_data[3][3] = 
    {
        {camera.fx,    0,     camera.cx},
        {0,       camera.fy,  camera.cy},
        {0,            0,         1}
    };

    // 构建相机矩阵
    cv::Mat cameraMatrix( 3, 3, CV_64F, camera_matrix_data);
    cv::Mat rvec, tvec, inliers;

    // 求解pnp，这是最关键的一步
    cv::solvePnPRansac(pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 1.0, 100, inliers);

    result.rvec = rvec;
    result.tvec = tvec;
    result.inliers = inliers.rows;

    cout << "inliers number is : " << result.inliers << endl;
    return result;
}


// cvMat2Eigen
// 将平移向量和旋转矩阵转换成变换矩阵
Eigen::Isometry3d cvMat2Eigen(cv::Mat& rvec, cv::Mat& tvec)
{
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    Eigen::Matrix3d r;
    for (int i = 0;i < 3; i++)
        for (int j = 0;j < 3; j++) 
            r(i,j) = R.at<double>(i,j);
  
    // 将平移向量和旋转矩阵转换成变换矩阵
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    Eigen::AngleAxisd angle(r);
    T = angle;
    T(0,3) = tvec.at<double>(0,0); 
    T(1,3) = tvec.at<double>(1,0); 
    T(2,3) = tvec.at<double>(2,0);
    return T;
}

// joinPointCloud 
// 输入：原始点云，新来的帧以及它的位姿
// 输出：将新来帧加到原始帧后的图像
PointCloud::Ptr joinPointCloud(PointCloud::Ptr original, FRAME& newFrame, Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMETERS& camera ) 
{
    PointCloud::Ptr newCloud = image2PointCloud( newFrame.rgb, newFrame.depth, camera );

    // 合并点云
    PointCloud::Ptr output (new PointCloud());
    pcl::transformPointCloud( *original, *output, T.matrix() );

    *newCloud += *output;

    // Voxel grid 滤波降采样，参数从文件中获取
    static pcl::VoxelGrid<PointT> voxel;
    static ParameterReader pd;
    double gridsize = atof( pd.getData("voxel_grid").c_str() );
    voxel.setLeafSize( gridsize, gridsize, gridsize );
    voxel.setInputCloud( newCloud );
    PointCloud::Ptr tmp( new PointCloud() );
    voxel.filter( *tmp );
    // tmp就是滤波之后的结果
    return tmp;
}

// normal transformation
double normofTransform(cv::Mat rvec, cv::Mat tvec)
{
    return fabs(min(cv::norm(rvec), 2*M_PI-cv::norm(rvec)))+fabs(cv::norm(tvec));
}

CHECK_RESULT checkKeyframes(CAMERA_INTRINSIC_PARAMETERS camera, FRAME& f1, FRAME& f2, g2o::SparseOptimizer& opti, bool is_loops)
{
    // 读取参数
    static ParameterReader pd;
    static int min_inliers = atoi( pd.getData("min_inliers").c_str() );
    static double max_norm = atof( pd.getData("max_norm").c_str() );
    static double keyframe_threshold = atof( pd.getData("keyframe_threshold").c_str() );
    static double max_norm_lp = atof( pd.getData("max_norm_lp").c_str() );
    //static CAMERA_INTRINSIC_PARAMETERS camera = getDefaultCamera();

    RESULT_OF_PNP result = estimateMotion(f1, f2, camera);
    if (result.inliers < min_inliers)   //inliers不够，放弃该帧
        return NOT_MATCHED;

    // 计算运动范围是否太大，这个约束是很强的
    double norm = normofTransform(result.rvec, result.tvec);
    cout << BLUE"Transform normal value is : " << norm;
    cout << RESET" " << endl;;

    if(is_loops == false) 
    {
        if (norm >= max_norm)
            return TOO_FAR_AWAY;   // too far away, may be error
    }
    else  // loop detect
    {
        if (norm >= max_norm_lp)
            return TOO_FAR_AWAY;   // too far away, may be error
    }

    // too adjacent frame
    if (norm <= keyframe_threshold)
        return TOO_CLOSE;

    // 向g2o中增加这个顶点与上一帧联系的边
    // 顶点只需设定id即可
    if (is_loops == false)
    {
        g2o::VertexSE3 *v = new g2o::VertexSE3();
        v->setId(f2.frameID);
        v->setEstimate(Eigen::Isometry3d::Identity());
        opti.addVertex(v);
    }

    // 边部分
    g2o::EdgeSE3* edge = new g2o::EdgeSE3();
    // 连接此边的两个顶点id
    edge->setVertex(0, opti.vertex(f1.frameID));
    edge->setVertex(1, opti.vertex(f2.frameID));
    edge->setRobustKernel(new g2o::RobustKernelHuber());
    // 信息矩阵
    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6,6 >::Identity();

    // 信息矩阵是协方差矩阵的逆，表示我们对边的精度的预先估计
    // 因为pose为6D的，信息矩阵是6*6的阵，假设位置和角度的估计精度均为0.1且互相独立
    // 那么协方差则为对角为0.01的矩阵，信息阵则为100的矩阵
    information(0,0) = information(1,1) = information(2,2) = 100;
    information(3,3) = information(4,4) = information(5,5) = 100;

    // 也可以将角度设大一些，表示对角度的估计更加准确
    edge->setInformation(information);

    // 边的估计即是pnp求解之结果，就是得到一两帧之间的变换矩阵
    Eigen::Isometry3d T = cvMat2Eigen(result.rvec, result.tvec);
    edge->setMeasurement(T.inverse());

    // 将此边加入图中
    opti.addEdge(edge);
    return KEYFRAME;
}

void checkNearbyLoops(CAMERA_INTRINSIC_PARAMETERS camera, vector<FRAME>& frames, FRAME& currFrame, g2o::SparseOptimizer& opti)
{
    static ParameterReader pd;
    static int nearby_loops = atoi(pd.getData("nearby_loops").c_str());

    // 就是把currFrame和 frames里末尾几个测一遍
    if (frames.size() <= nearby_loops)
    {
        // no enough keyframes, check everyone
        for (size_t i = 0; i < frames.size(); i++)
            checkKeyframes(camera, frames[i], currFrame, opti, true);
    }
    else
    {
        // check the nearest ones
        for (size_t i = frames.size()-nearby_loops;i < frames.size(); i++)
            checkKeyframes(camera, frames[i], currFrame, opti, true);
    }
}

void checkRandomLoops(CAMERA_INTRINSIC_PARAMETERS camera, vector<FRAME>& frames, FRAME& currFrame, g2o::SparseOptimizer& opti)
{
    static ParameterReader pd;
    static int random_loops = atoi( pd.getData("random_loops").c_str() );
    srand( (unsigned int) time(NULL) );
    // 随机取一些帧进行检测
    if ( frames.size() <= random_loops)
    {
        // no enough keyframes, check everyone
        for (size_t i = 0;i < frames.size(); i++)
            checkKeyframes(camera, frames[i], currFrame, opti, true);
    }
    else
    {
        // randomly check loops
        for (int i = 0;i < random_loops; i++)
        {
            int index = rand()%frames.size();
            checkKeyframes(camera, frames[index], currFrame, opti, true);
        }
    }
}

int initRealsense(rs::context & ctx, rs::device ** dev)
{
    printf("There are %d connected RealSense devices.\n", ctx.get_device_count());
    if(ctx.get_device_count() == 0) 
        return EXIT_FAILURE;
    (*dev) = ctx.get_device(0);
    printf("\nUsing device 0, an %s\n", (*dev)->get_name());
    printf("    Serial number: %s\n", (*dev)->get_serial());
    printf("    Firmware version: %s\n", (*dev)->get_firmware_version());

    // Configure depth and color to run with the device's preferred settings
    (*dev)->enable_stream(rs::stream::depth, rs::preset::best_quality);
    (*dev)->enable_stream(rs::stream::color, rs::preset::best_quality);
    (*dev)->start();

    return EXIT_SUCCESS;
}

void readfromrealsense(rs::device * dev, cv::Mat& rgb_2, cv::Mat& depth)
{
    dev->wait_for_frames();

    // Retrieve camera parameters for mapping between depth and color
    rs::intrinsics depth_intrin = dev->get_stream_intrinsics(rs::stream::depth_aligned_to_color);
    rs::intrinsics color_intrin = dev->get_stream_intrinsics(rs::stream::rectified_color);
    float scale = dev->get_depth_scale();

    cv::Mat rgb(color_intrin.height, color_intrin.width, CV_8UC3, (uchar *)dev->get_frame_data(rs::stream::rectified_color));
    depth = cv::Mat(depth_intrin.height, depth_intrin.width, CV_16U,(uchar *)dev->get_frame_data(rs::stream::depth_aligned_to_color));
    cvtColor(rgb, rgb_2, cv::COLOR_BGR2RGB);
}

void readFromFile(cv::Mat& rgb_2, cv::Mat& depth)
{
    ParameterReader pd_;
    int startIndex = atoi(pd_.getData("start_index").c_str());
    int endIndex   = atoi(pd_.getData("end_index").c_str());

    static int currentIndex = startIndex;

    string rgbDir   = pd_.getData("rgb_dir");
    string depthDir = pd_.getData("depth_dir");

    string rgbExt   = pd_.getData("rgb_extension");
    string depthExt = pd_.getData("depth_extension");

    stringstream ss;
    ss << rgbDir << currentIndex << rgbExt;
    string filename;
    ss >> filename;
    rgb_2 = cv::imread(filename);

    ss.clear();
    filename.clear();
    ss << depthDir << currentIndex << depthExt;
    ss >> filename;

    depth = cv::imread(filename, -1);

    currentIndex++;
}






