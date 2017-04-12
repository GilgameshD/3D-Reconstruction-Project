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

            // 把p加入到点云中
            cloud->points.push_back( p );
        }
    // 设置并保存点云
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;

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
    
    // for debug
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
    
    cout << "min dis = " << minDis << endl;
    if (minDis < 10) 
        minDis = 10;

    for (size_t i = 0; i < matches.size(); i++)
    {
        if (matches[i].distance < good_match_threshold * minDis)
            goodMatches.push_back(matches[i]);
    }

    cout << "good matches: " << goodMatches.size() << endl;

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

    cout << "inliers number : " << result.inliers << endl;
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



