#include <iostream>
#include <fstream>
#include <sstream>
using namespace std;

#include "slamBase.h"

// PCL滤波器的头文件
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>

// 后端图优化的头文件
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

// 把g2o的定义放到前面
typedef g2o::BlockSolver_6_3 SlamBlockSolver; 
typedef g2o::LinearSolverEigen< SlamBlockSolver::PoseMatrixType > SlamLinearSolver; 

// 检测两个帧，结果定义
enum CHECK_RESULT
{
    NOT_MATCHED = 0, 
    TOO_FAR_AWAY, 
    TOO_CLOSE, 
    KEYFRAME
}; 

// 给定index，读取一帧数据
FRAME readFrame(int index, ParameterReader& pd);

// 估计一个运动的大小
double normofTransform(cv::Mat rvec, cv::Mat tvec);

// 检测关键帧，下面的两个检测方法用的这个
CHECK_RESULT checkKeyframes(FRAME& f1, FRAME& f2, g2o::SparseOptimizer& opti, bool is_loops = false);

// 检测近距离的回环
void checkNearbyLoops(vector<FRAME>& frames, FRAME& currFrame, g2o::SparseOptimizer& opti);

// 随机检测回环
void checkRandomLoops(vector<FRAME>& frames, FRAME& currFrame, g2o::SparseOptimizer& opti);

int main(int argc, char** argv)
{
    // 这里是静态变量所以都是同一个文件操作符
    ParameterReader pd;
    int startIndex = atoi(pd.getData("start_index").c_str());
    int endIndex   = atoi(pd.getData("end_index").c_str());

    // 所有的关键帧都放在了这里
    vector<FRAME> keyframes; 
    
    // initialize，读取各个参数
    cout << "Initializing ..." << endl;
    int currIndex = startIndex; // 当前索引为currIndex
    FRAME currFrame = readFrame(currIndex, pd); // 上一帧数据

    string detector = pd.getData("detector");
    string descriptor = pd.getData("descriptor");
    CAMERA_INTRINSIC_PARAMETERS camera = getDefaultCamera();

    // 提取特征点
    computeKeyPointsAndDesp(currFrame, detector, descriptor);
    PointCloud::Ptr cloud = image2PointCloud(currFrame.rgb, currFrame.depth, camera);
    
    // 初始化求解器
    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering( false );
    SlamBlockSolver* blockSolver = new SlamBlockSolver( linearSolver );
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( blockSolver ); // 使用LM算法

    // 定义整个优化器变量
    g2o::SparseOptimizer globalOptimizer;  
    globalOptimizer.setAlgorithm(solver); 

    // 不要输出调试信息
    globalOptimizer.setVerbose( false );
    
    // 向globalOptimizer增加第一个顶点
    g2o::VertexSE3 *v = new g2o::VertexSE3();
    v->setId(currIndex);
    // 估计为单位矩阵， 因为这个是第一个点，不需要变换，
    v->setEstimate(Eigen::Isometry3d::Identity()); 
    v->setFixed(true);  // 设置成固定点
    globalOptimizer.addVertex(v);
    
    // 加入关键帧
    keyframes.push_back(currFrame);
    double keyframe_threshold = atof( pd.getData("keyframe_threshold").c_str() );

    // 开始进行回环检测
    for(currIndex = startIndex+1;currIndex < endIndex;currIndex++)
    {
        cout << "Reading files " << currIndex << endl;
        FRAME currFrame = readFrame(currIndex, pd);         

        computeKeyPointsAndDesp(currFrame, detector, descriptor); 

        CHECK_RESULT result = checkKeyframes(keyframes.back(), currFrame, globalOptimizer); //匹配该帧与keyframes里最后一帧
        switch (result) // 根据匹配结果不同采取不同策略
        {
            case NOT_MATCHED:
                // 没匹配上，直接跳过
                cout << RED"Not enough inliers." << endl;
                break;
            case TOO_FAR_AWAY:
                // 太远了，也直接跳
                cout << RED"Too far away, may be an error." << endl;
                break;
            case TOO_CLOSE:
                // 太近了，可能出错了
                cout << RESET"Too close, not a keyframe" << endl;
                break;
                // 只有在是关键帧的时候才有可能进行回环检测
            case KEYFRAME:
                cout << GREEN"This is a new keyframe" << endl;
                // 不远不近，检测回环
                checkNearbyLoops(keyframes, currFrame, globalOptimizer);
                checkRandomLoops(keyframes, currFrame, globalOptimizer);

                // 加入关键帧
                keyframes.push_back(currFrame);
                break;
        }
    }

    // start optimization
    cout << RESET"optimizing pose graph, vertices: " << globalOptimizer.vertices().size() << endl;
    globalOptimizer.initializeOptimization(); 
    globalOptimizer.optimize(100);         
    cout << "Optimization done." << endl;

    // global point cloud
    cout << "saving the point cloud map..." << endl;
    PointCloud::Ptr output (new PointCloud()); 

    // voxel grid down sampler
    pcl::VoxelGrid<PointT> voxel; 
    double gridsize = 1;
    voxel.setLeafSize(gridsize, gridsize, gridsize);

    // start registration
    for (size_t i = 0; i < keyframes.size(); i++)
    {
        // get one frame from optimizor
        g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(globalOptimizer.vertex(keyframes[i].frameID));
        Eigen::Isometry3d pose = vertex->estimate(); 
        PointCloud::Ptr newCloud = image2PointCloud(keyframes[i].rgb, keyframes[i].depth, camera); //转成点云

        // down sample
        //voxel.setInputCloud(newCloud);
        //voxel.filter(*newCloud);

        // put into global point cloud
        pcl::transformPointCloud(*newCloud, *newCloud, pose.matrix());
        *output += *newCloud;
        newCloud->clear();
    }

    pcl::io::savePCDFile("./result.pcd", *output);

    // convert .pcd to .ply to be visualized
    string input_filename("./result.pcd");
    string output_filename("./pointcloud.ply");
    PCDtoPLYconvertor(input_filename, output_filename);

    cout << "Final map is saved..." << endl;
    return 0;
}

FRAME readFrame(int index, ParameterReader& pd)
{
    FRAME f;
    string rgbDir   = pd.getData("rgb_dir");
    string depthDir = pd.getData("depth_dir");
    
    string rgbExt   = pd.getData("rgb_extension");
    string depthExt = pd.getData("depth_extension");

    stringstream ss;
    ss << rgbDir << index << rgbExt;
    string filename;
    ss >> filename;
    f.rgb = cv::imread(filename);

    ss.clear();
    filename.clear();
    ss << depthDir << index << depthExt;
    ss >> filename;

    f.depth = cv::imread(filename, -1);
    f.frameID = index;
    return f;
}

// normal transformation
double normofTransform(cv::Mat rvec, cv::Mat tvec)
{
    return fabs(min(cv::norm(rvec), 2*M_PI-cv::norm(rvec)))+fabs(cv::norm(tvec));
}

CHECK_RESULT checkKeyframes(FRAME& f1, FRAME& f2, g2o::SparseOptimizer& opti, bool is_loops)
{
    // 读取参数
    static ParameterReader pd;
    static int min_inliers = atoi( pd.getData("min_inliers").c_str() );
    static double max_norm = atof( pd.getData("max_norm").c_str() );
    static double keyframe_threshold = atof( pd.getData("keyframe_threshold").c_str() );
    static double max_norm_lp = atof( pd.getData("max_norm_lp").c_str() );
    static CAMERA_INTRINSIC_PARAMETERS camera = getDefaultCamera();

    // 比较f1 和 f2
    // 输入：帧1和帧2
    // 输出：rvec 和 tvec，分别表示R旋转矩阵和t平移向量
    RESULT_OF_PNP result = estimateMotion(f1, f2, camera);
    if ( result.inliers < min_inliers ) //inliers不够，放弃该帧
        return NOT_MATCHED;

    // 计算运动范围是否太大
    double norm = normofTransform(result.rvec, result.tvec);
    cout << norm << endl;
    if(is_loops == false)
    {
        if (norm >= max_norm)
            return TOO_FAR_AWAY;   // too far away, may be error
    }
    else
    {
        if (norm >= max_norm_lp)
            return TOO_FAR_AWAY;
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
    edge->setVertex(0, opti.vertex(f1.frameID ));
    edge->setVertex(1, opti.vertex(f2.frameID ));
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

void checkNearbyLoops(vector<FRAME>& frames, FRAME& currFrame, g2o::SparseOptimizer& opti)
{
    static ParameterReader pd;
    static int nearby_loops = atoi(pd.getData("nearby_loops").c_str());
    
    // 就是把currFrame和 frames里末尾几个测一遍
    if (frames.size() <= nearby_loops)
    {
        // no enough keyframes, check everyone
        for (size_t i = 0; i < frames.size(); i++)
            checkKeyframes(frames[i], currFrame, opti, true);
    }
    else
    {
        // check the nearest ones
        for (size_t i = frames.size()-nearby_loops;i < frames.size(); i++)
            checkKeyframes(frames[i], currFrame, opti, true);
    }
}

void checkRandomLoops(vector<FRAME>& frames, FRAME& currFrame, g2o::SparseOptimizer& opti )
{
    static ParameterReader pd;
    static int random_loops = atoi( pd.getData("random_loops").c_str() );
    srand( (unsigned int) time(NULL) );
    // 随机取一些帧进行检测
    if ( frames.size() <= random_loops)
    {
        // no enough keyframes, check everyone
        for (size_t i = 0;i < frames.size(); i++)
            checkKeyframes( frames[i], currFrame, opti, true );
    }
    else
    {
        // randomly check loops
        for (int i = 0;i < random_loops; i++)
        {
            int index = rand()%frames.size();
            checkKeyframes(frames[index], currFrame, opti, true);
        }
    }
}



