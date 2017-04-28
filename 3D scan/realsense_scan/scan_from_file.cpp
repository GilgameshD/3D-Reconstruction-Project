/*
 * This method consists two step, the first step is calculating the transform matrix between every two frames,
 * and the second step is optimizating nearby frames with g2o in order to minimize the error, there are many
 * methods to detect the correspondence frames, but I use the simplest way, which hardcode a parameter.
 */

#include <iostream>
#include <vector>
#include <fstream>
#include <map>

// pcl lib
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h> 
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

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

#define RESET "\033[0m"
#define GREEN "\033[32m"
#define YELLOW "\033[33m"
#define BLUE "\033[34m"

// definitions for pcl
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;
// definitions for g2o
typedef g2o::BlockSolver_6_3 SlamBlockSolver;
typedef g2o::LinearSolverEigen< SlamBlockSolver::PoseMatrixType > SlamLinearSolver;

float camera_factor;
float camera_cx;
float camera_cy;
float camera_fx;
float camera_fy;

// reading parameter file
class ParameterReader
{
public:
    ParameterReader(std::string filename = "../parameters.txt")
    {
        ifstream fin(filename.c_str());
        if (!fin)
        {
            std::cerr << "parameter file does not exist." << std::endl;
            return;
        }
        while(!fin.eof())
        {
            std::string str;
            getline(fin, str);
            if(str[0] == '#') continue; // ignore comments
            int pos = str.find("=");
            if(pos == -1) continue;
            std::string key = str.substr(0, pos);
            std::string value = str.substr(pos+1, str.length());
            data[key] = value;
            if (!fin.good()) break;
        }
    }
    // iterative access
    std::string getData(std::string key)
    {
        std::map<std::string, std::string>::iterator iter = data.find(key);
        if(iter == data.end())
        {
            cerr << "Parameter name " << key << " not found!" << endl;
            return std::string("NOT_FOUND");
        }
        return iter->second;
    }

    std::map<std::string, std::string> data;
};

// convert mat to Isometry3d
Eigen::Isometry3d cvMat2Eigen(cv::Mat& rvec, cv::Mat& tvec)
{
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    Eigen::Matrix3d r;
    for (int i = 0;i < 3; i++)
        for (int j = 0;j < 3; j++)
            r(i,j) = R.at<double>(i,j);

    // 将旋转矩阵转换成四元数
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    Eigen::AngleAxisd angle(r);
    T = angle;
    T(0,3) = tvec.at<double>(0,0);
    T(1,3) = tvec.at<double>(1,0);
    T(2,3) = tvec.at<double>(2,0);
    return T;
}

// convert PLY files to PCD files
int PCDtoPLYconvertor(const std::string &input_filename ,const std::string& output_filename)
{
    pcl::PCLPointCloud2 cloud;
    if (pcl::io::loadPCDFile(input_filename , cloud) < 0)
    {
        std::cout << "Error: cannot load the PCD file!!!"<< std::endl;
        return -1;
    }

    pcl::PLYWriter writer;
    writer.write(output_filename, cloud, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(),true,true);
    return 0;
}

std::string int2str(const int &int_temp)
{
    std::string string_temp;
    std::stringstream stream;
    stream << int_temp;
    string_temp = stream.str(); 
    return string_temp;
}

void generatePointCloud(const cv::Mat rgb, const cv::Mat depth, PointCloud::Ptr cloud)
{
    //cv::imshow("1", rgb);
    //cv::imshow("2", depth);

    // search the whole depth image
    for (int m = 0; m < depth.rows; m++)
    {
        for (int n = 0; n < depth.cols; n++)
        {
            ushort d = depth.ptr<ushort>(m)[n];

            // omit this point
            if (d == 0)
                continue;

            PointT p;
            p.z = float(d) / camera_factor;
            p.x = (n - camera_cx) * p.z / camera_fx;
            p.y = (m - camera_cy) * p.z / camera_fy;
            p.b = rgb.ptr<uchar>(m)[n*3];
            p.g = rgb.ptr<uchar>(m)[n*3+1];
            p.r = rgb.ptr<uchar>(m)[n*3+2];

            cloud->points.push_back(p);
        }
    }
    cloud->height = 1;
    cloud->width = (uint32_t)cloud->points.size();
    cout << "generate one point cloud, and the size = "<< cloud->points.size() << endl;

    //pcl::io::savePCDFile("./pointcloud.pcd", *cloud);
    //std::string input_filename("./pointcloud.pcd");
    //std::string output_filename("./pointcloud.ply");
    //PCDtoPLYconvertor(input_filename , output_filename);
}

void readFromFile(cv::Mat &rgb, cv::Mat &depth, const int iterations)
{
    const std::string rgbName = "../data/rgb_png/" + int2str(iterations) + ".png";
    const std::string depthName = "../data/depth_png/" + int2str(iterations) + ".png";

    rgb = cv::imread(rgbName);
    depth = cv::imread(depthName, -1);

    if(rgb.empty() || depth.empty())
    {
        std::cout << "check the file name, no such a file" << std::endl;
    }
}

// add edges and vertexs to the graph to be optimization
void optimizationWithG2o(g2o::SparseOptimizer &globalOptimizer, std::vector<PointCloud::Ptr> inputSequence, PointCloud::Ptr output)
{
    // start optimization
    std::cout << RESET"optimizing pose graph, vertices: " << globalOptimizer.vertices().size() << std::endl;
    globalOptimizer.initializeOptimization();
    globalOptimizer.optimize(100);
    std::cout << "Optimization done. start to get frames... " << std::endl;

    // start registration
    for(size_t i = 0; i < globalOptimizer.vertices().size(); i++)
    {
        // get one frame from optimizor
        g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(globalOptimizer.vertex(i));
        Eigen::Isometry3d pose = vertex->estimate();
        // put into global point cloud
        pcl::transformPointCloud(*(inputSequence[i]), *(inputSequence[i]), pose.matrix());

        //std::cout << GREEN"Transform matrix after G2O: \n" << pose.matrix() << std::endl;

        // merge all point cloud
        *output += *(inputSequence[i]);
    }
    std::cout << RESET"finish merging all frames, waiting for saving the file..." << std::endl;
}

// put into one edge, it connect two frames
void addOneEdgeBetweenTwoFrames(int frameID_1, int frameID_2, g2o::SparseOptimizer &opti, const Eigen::Matrix4f transformMatrix)
{
    g2o::EdgeSE3* edge = new g2o::EdgeSE3();

    // connect two frames
    edge->setVertex(0, opti.vertex(frameID_1));
    edge->setVertex(1, opti.vertex(frameID_2));
    edge->setRobustKernel(new g2o::RobustKernelHuber());

    // information matrix
    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix< double, 6,6 >::Identity();

    information(0,0) = information(1,1) = information(2,2) = 100;
    information(3,3) = information(4,4) = information(5,5) = 100;

    edge->setInformation(information);

    // convert type: Matrix4f -> Affine -> Mat -> Isometry
    cv::Mat matrixR(3,3,CV_32FC1);
    cv::Mat matrixT(3,1,CV_32FC1);

    for(int i = 0;i < 3; ++i)
        for(int j = 0;j < 3; ++j)
        {
            matrixR.at<float>(i,j) = transformMatrix(i, j);
        }

    for(int i = 0;i < 3; ++i)
        matrixT.at<float>(i,0) = transformMatrix(i,0);

    std::cout << GREEN" matrix R : \n" << matrixR << std::endl;
    std::cout << GREEN" matrix T : \n" << matrixT << std::endl;
    Eigen::Isometry3d T = cvMat2Eigen(matrixR, matrixT);
    edge->setMeasurement(T.inverse());
    opti.addEdge(edge);
}

int main(int argc, char** argv) 
{
    ParameterReader pd;

    // record all point clouds
    std::vector<PointCloud::Ptr> inputSequence;
    std::vector<PointCloud::Ptr> inputSequenceAfterDownSample;
    PointCloud::Ptr output(new PointCloud());
    PointCloud::Ptr outputWithoutG2O(new PointCloud());

    // save the global matrix
    Eigen::Matrix4f globalTransformMatrix = Eigen::Matrix4f::Identity();

    // create a view handle
    boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("icp"));

    // create two views
    int v1; 
    int v2;
    view->createViewPort(0.0, 0.0, 0.5, 1.0, v1); 
    view->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    view->initCameraParameters();

    // get parameters from file
    int startIndex = (int)atof(pd.getData("startIndex").c_str());
    int stopIndex = (int)atof(pd.getData("stopIndex").c_str());
    int maxCorrespondence = (int)atof(pd.getData("maxCorrespondence").c_str());
    int maxIteration = (int)atof(pd.getData("maxIteration" ).c_str());
    double TransformationEpsilon = atof(pd.getData("TransformationEpsilon").c_str());
    double EuclideanFitnessEpsilon = atof(pd.getData("EuclideanFitnessEpsilon").c_str());
    int maxNearbyFrame = (int)atof(pd.getData("maxNearbyFrame" ).c_str());
    int fitnessThreshhold = (int)atof(pd.getData("fitnessThreshhold").c_str());
    float downSampleSize = (float)atof(pd.getData("downSampleSize").c_str());
    float outputGridSize = (float)atof(pd.getData("outputGridSize").c_str());
    camera_factor = (float)atof(pd.getData("camera_factor").c_str());
    camera_cx = (float)atof(pd.getData("camera_cx").c_str());
    camera_cy = (float)atof(pd.getData("camera_cy").c_str());
    camera_fx = (float)atof(pd.getData("camera_fx").c_str());
    camera_fy = (float)atof(pd.getData("camera_fy").c_str());

    // loop closure definition
    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(blockSolver); // use L-M
    g2o::SparseOptimizer globalOptimizer;

    // the whole optimizer
    globalOptimizer.setAlgorithm(solver);

    // don not output messages
    globalOptimizer.setVerbose(false);

    // main loop
    for(int iterations = 0;iterations < stopIndex - startIndex;iterations++)
    {
        view->spinOnce();

        PointCloud::Ptr oneFrame(new PointCloud);
        PointCloud::Ptr oneFrameAfterDownSample(new PointCloud);
        PointCloud::Ptr oneFrameResult(new PointCloud);

        cv::Mat rgb;
        cv::Mat depth;

        // read from real sense
        readFromFile(rgb, depth, iterations+startIndex);
        generatePointCloud(rgb, depth, oneFrame);

        // down sample, using a grid box
        inputSequence.push_back(oneFrame);
        std::cout << YELLOW"before down sample, points number : " << inputSequence[iterations]->points.size();
        pcl::VoxelGrid<PointT> grid;
        grid.setLeafSize(downSampleSize, downSampleSize, downSampleSize);
        grid.setInputCloud(oneFrame);
        grid.filter(*oneFrameAfterDownSample);

        std::cout << ". after down sample, points number : " << oneFrameAfterDownSample->points.size() << std::endl;
        inputSequenceAfterDownSample.push_back(oneFrameAfterDownSample);

        if(iterations == 0)
        {
            // share a same point cloud at the first frame
            oneFrameResult = inputSequence[iterations];

            const std::string namePC = int2str(iterations) + "right";
            const char *nameChar = namePC.c_str();

            view->addPointCloud(inputSequence[iterations], "original", v1);
            view->addPointCloud(oneFrameResult, nameChar, v2);
            *outputWithoutG2O = *oneFrameResult;

            // add the first point to globalOptimizer
            g2o::VertexSE3 *v = new g2o::VertexSE3();
            v->setId(iterations);
            v->setEstimate(Eigen::Isometry3d::Identity());
            v->setFixed(true);
            globalOptimizer.addVertex(v);

            continue;
        }

        // add this frame to the graph, for now I add all frame to it.
        g2o::VertexSE3 *v = new g2o::VertexSE3();
        v->setId(iterations);
        v->setEstimate(Eigen::Isometry3d::Identity());
        globalOptimizer.addVertex(v);

        // start ICP process
        std::cout << RESET"Current vertex number is : " << globalOptimizer.vertices().size()
                  << ". and current edge number is : "  << globalOptimizer.edges().size() << std::endl;
        std::cout << "waiting for ICP process..." << std::endl;
        pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
        // for every frame, we should do several icp with nearby frames to add as much as possible
        for(int i = 0;i < maxNearbyFrame && i < inputSequenceAfterDownSample.size()-1; ++i)
        {
            // use now point cloud to find before
            icp.setInputSource(inputSequenceAfterDownSample[iterations]);     // point cloud now
            icp.setInputTarget(inputSequenceAfterDownSample[iterations-i-1]); // point cloud before
            icp.setMaxCorrespondenceDistance(maxCorrespondence);
            icp.setTransformationEpsilon(TransformationEpsilon);      // difference between two matrix
            icp.setEuclideanFitnessEpsilon(EuclideanFitnessEpsilon);  // MSE
            icp.setMaximumIterations(maxIteration);
            icp.align(*oneFrameResult);

            // if the transform is no accurate, omit it.
            if(icp.getFitnessScore() > fitnessThreshhold)
            {
                std::cout << "Too far, ignore this pair of ICP" << std::endl;
                continue;
            }

            // between this frame and last frame, get a rough transform
            if(i == 0)
            {
                globalTransformMatrix = globalTransformMatrix * icp.getFinalTransformation();
                break;
            }

            // the smaller of the score, the better
            std::cout << RESET"Has conveged : " << icp.hasConverged() << " score : " << icp.getFitnessScore() << std::endl;
            std::cout << GREEN"Matrix : \n" << icp.getFinalTransformation() << std::endl;
            std::cout << RESET"Max correspondence distance : " << icp.getMaxCorrespondenceDistance() << std::endl;
            std::cout << "Max ICP iteration : " << icp.getMaximumIterations() << std::endl;
            std::cout << "[ This is " <<  iterations << " iterations ]" << std::endl;
            std::cout << "-----------------------------------------------------------------------------------------" << std::endl;

            // add an edge
            addOneEdgeBetweenTwoFrames(iterations, iterations-i-1, globalOptimizer, icp.getFinalTransformation());
            std::cout << BLUE"add one edge into the graph, it is between " << iterations-i-1 << " and " << iterations << std::endl;
        }

        // show point cloud
        const std::string namePC = int2str(iterations);
        const char *nameChar = namePC.c_str();

        pcl::transformPointCloud(*(inputSequence[iterations]), *oneFrameResult, globalTransformMatrix);
        view->removePointCloud("original");
        view->addPointCloud(inputSequence[iterations], "original", v1);
        view->addPointCloud(oneFrameResult, nameChar, v2);
        *outputWithoutG2O += *oneFrameResult;
    }

    // global optimization with g2o, and get every frame after the optimization
    optimizationWithG2o(globalOptimizer, inputSequence, output);

    // down sample the final point cloud, this grid size is different
    pcl::VoxelGrid<PointT> grid;
    grid.setLeafSize(outputGridSize, outputGridSize, outputGridSize);
    grid.setInputCloud(output);
    grid.filter(*output);

    pcl::VoxelGrid<PointT> grid_2;
    grid_2.setLeafSize(outputGridSize, outputGridSize, outputGridSize);
    grid_2.setInputCloud(outputWithoutG2O);
    grid_2.filter(*outputWithoutG2O);

    // save the whole point cloud before optimizer and after
    pcl::io::savePCDFile("../result_without_g2o.pcd", *outputWithoutG2O);
    std::string input_filename("../result_without_g2o.pcd");
    std::string output_filename("../result_without_g2o.ply");
    PCDtoPLYconvertor(input_filename, output_filename);
    std::cout << "Finish saving the file without g2o" << std::endl;
    pcl::io::savePCDFile("../result.pcd", *output);
    input_filename = "../result.pcd";
    output_filename = "../result.ply";
    PCDtoPLYconvertor(input_filename, output_filename);
    std::cout << "Finish saving the file with g2o" << std::endl;

    view->spin();
    return 0;
}


