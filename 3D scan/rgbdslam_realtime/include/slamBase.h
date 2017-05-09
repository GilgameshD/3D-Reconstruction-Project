// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

// g2o
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

// realsense
#include <librealsense/rs.hpp>

// thread
#include <thread>
#include <boost/make_shared.hpp>
#include <condition_variable>
#include <mutex>
#include <iomanip>

// STL
#include <fstream>
#include <vector>
#include <map>
#include <iostream>
#include <sstream>
using namespace std;

// 类型定义
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef g2o::BlockSolver_6_3 SlamBlockSolver; 
typedef g2o::LinearSolverEigen< SlamBlockSolver::PoseMatrixType > SlamLinearSolver; 


// 相机内参结构
struct CAMERA_INTRINSIC_PARAMETERS 
{ 
    double cx, cy, fx, fy, scale;
};

// 帧结构
struct FRAME
{
    int frameID;        // 帧ID
    cv::Mat rgb, depth; // 该帧对应的彩色图与深度图
    cv::Mat desp;       // 特征描述子
    vector<cv::KeyPoint> kp; //关键点
};

// PnP 结果
struct RESULT_OF_PNP
{
    cv::Mat rvec, tvec;
    int inliers;
};

enum CHECK_RESULT
{
    NOT_MATCHED = 0, 
    TOO_FAR_AWAY, 
    TOO_CLOSE, 
    KEYFRAME
};

// 参数读取类
// 使用STL内部的map<>类型来作为参数文件的储存方式
class ParameterReader
{
public:
    ParameterReader( string filename = "../parameters.txt" )
    {
        ifstream fin( filename.c_str() );
        if (!fin)
        {
            cerr << "parameter file does not exist." << endl;
            return;
        }
        while(!fin.eof())
        {
            string str;
            getline( fin, str );
            if (str[0] == '#')
            {
                // 以‘＃’开头的是注释
                continue;
            }

            int pos = str.find("=");
            if (pos == -1)
                continue;
            string key = str.substr( 0, pos );
            string value = str.substr( pos+1, str.length() );
            data[key] = value;

            if (!fin.good())
                break;
        }
    }
    // 这是一个可以进行迭代的访问函数
    string getData( string key )
    {
        map<string, string>::iterator iter = data.find(key);
        if (iter == data.end())
        {
            cerr << "Parameter name " << key << " not found!" << endl;
            return string("NOT_FOUND");
        }
        return iter->second;
    }
public:
    map<string, string> data;
};

inline static CAMERA_INTRINSIC_PARAMETERS getDefaultCamera()
{
    ParameterReader pd;
    CAMERA_INTRINSIC_PARAMETERS camera;
    camera.fx = atof( pd.getData( "camera.fx" ).c_str());
    camera.fy = atof( pd.getData( "camera.fy" ).c_str());
    camera.cx = atof( pd.getData( "camera.cx" ).c_str());
    camera.cy = atof( pd.getData( "camera.cy" ).c_str());
    camera.scale = atof( pd.getData( "camera.scale" ).c_str() );
    return camera;
}

// 函数接口
// image2PonitCloud 将rgb图+深度图转换为点云
PointCloud::Ptr image2PointCloud(cv::Mat& rgb, cv::Mat& depth, CAMERA_INTRINSIC_PARAMETERS& camera );

// point2dTo3d 将单个点从图像坐标转换为空间坐标
// input: 3维点Point3f (u,v,d)
cv::Point3f point2dTo3d(cv::Point3f& point, CAMERA_INTRINSIC_PARAMETERS& camera );

// computeKeyPointsAndDesp 同时提取关键点与特征描述子
void computeKeyPointsAndDesp(FRAME& frame, string detector, string descriptor );

// estimateMotion 计算两个帧之间的运动
// 输入：帧1和帧2, 相机内参
RESULT_OF_PNP estimateMotion(FRAME& frame1, FRAME& frame2, CAMERA_INTRINSIC_PARAMETERS& camera );

// cvMat2Eigen
Eigen::Isometry3d cvMat2Eigen( cv::Mat& rvec, cv::Mat& tvec );

// join new point cloud 
PointCloud::Ptr joinPointCloud( PointCloud::Ptr original, FRAME& newFrame, Eigen::Isometry3d T, CAMERA_INTRINSIC_PARAMETERS& camera ) ;

// convert pcd to ply file
int PCDtoPLYconvertor(string & input_filename ,string& output_filename);

// estimate the motion between two frame
double normofTransform(cv::Mat rvec, cv::Mat tvec);

// 检测关键帧，下面的两个检测方法用的这个
CHECK_RESULT checkKeyframes(CAMERA_INTRINSIC_PARAMETERS camera, FRAME& f1, FRAME& f2, g2o::SparseOptimizer& opti, bool is_loops = false);

// 检测近距离的回环
void checkNearbyLoops(CAMERA_INTRINSIC_PARAMETERS camera, vector<FRAME>& frames, FRAME& currFrame, g2o::SparseOptimizer& opti);

// 随机检测回环
void checkRandomLoops(CAMERA_INTRINSIC_PARAMETERS camera, vector<FRAME>& frames, FRAME& currFrame, g2o::SparseOptimizer& opti);

// init realsense 
int initRealsense(rs::context & ctx, rs::device ** dev);

// read from realsense
void readfromrealsense(rs::device * dev, cv::Mat& rgb_2, cv::Mat& depth);

// read from file
void readFromFile(cv::Mat& rgb_2, cv::Mat& depth);

//the following are UBUNTU/LINUX ONLY terminal color
#define RESET "\033[0m"
#define BLACK "\033[30m" /* Black */
#define RED "\033[31m" /* Red */
#define GREEN "\033[32m" /* Green */
#define YELLOW "\033[33m" /* Yellow */
#define BLUE "\033[34m" /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m" /* Cyan */
#define WHITE "\033[37m" /* White */
#define BOLDBLACK "\033[1m\033[30m" /* Bold Black */
#define BOLDRED "\033[1m\033[31m" /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m" /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m" /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m" /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m" /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m" /* Bold White */



