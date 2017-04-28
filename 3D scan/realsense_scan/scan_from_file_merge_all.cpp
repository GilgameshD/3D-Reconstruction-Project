/*
 * This method
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

#define RESET "\033[0m"
#define RED "\033[31m"
#define GREEN "\033[32m"
#define YELLOW "\033[33m"
#define BLUE "\033[34m"


typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// camera intrincs
const float camera_factor = 10000;
const float camera_cx = 321.153;
const float camera_cy = 246.486;
const float camera_fx = 610.098;
const float camera_fy = 610.098;

// 参数读取类
// 使用STL内部的map<>类型来作为参数文件的储存方式
class ParameterReader
{
public:
    ParameterReader(std::string filename = "../parameters.txt" )
    {
        ifstream fin(filename.c_str());
        if (!fin)
        {
            cerr << "parameter file does not exist." << endl;
            return;
        }
        while(!fin.eof())
        {
            std::string str;
            getline(fin, str);
            if (str[0] == '#')
            {
                continue;
            }

            int pos = str.find("=");
            if (pos == -1)
                continue;
            std::string key = str.substr(0, pos);
            std::string value = str.substr(pos+1, str.length());
            data[key] = value;

            if (!fin.good())
                break;
        }
    }
    // iterative access
    std::string getData(std::string key)
    {
        std::map<std::string, std::string>::iterator iter = data.find(key);
        if (iter == data.end())
        {
            cerr << "Parameter name " << key << " not found!" << endl;
            return std::string("NOT_FOUND");
        }
        return iter->second;
    }

    std::map<std::string, std::string> data;
};

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
            // 获取深度图中(m,n)处的值
            ushort d = depth.ptr<ushort>(m)[n];

            // omit this point
            if (d == 0)
                continue;

            // d 存在值，则向点云增加一个点
            PointT p;
            // 计算这个点的空间坐标
            p.z = float(d) / camera_factor;
            p.x = (n - camera_cx) * p.z / camera_fx;
            p.y = (m - camera_cy) * p.z / camera_fy;

            // 从rgb图像中获取它的颜色
            // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
            p.b = rgb.ptr<uchar>(m)[n*3];
            p.g = rgb.ptr<uchar>(m)[n*3+1];
            p.r = rgb.ptr<uchar>(m)[n*3+2];

            // 把p加入到点云中
            cloud->points.push_back(p);
        }
    }
    // 设置并保存点云
    cloud->height = 1;
    cloud->width = (uint32_t)cloud->points.size();
    cout << "generate one point cloud, and the size = "<< cloud->points.size() << endl;

    pcl::io::savePCDFile("./pointcloud.pcd", *cloud);

    std::string input_filename("./pointcloud.pcd");
    std::string output_filename("./pointcloud.ply");

    PCDtoPLYconvertor(input_filename , output_filename);
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

int main(int argc, char** argv) 
{
    ParameterReader pd;

    // record all point clouds
    std::vector<PointCloud::Ptr> inputSequence;
    std::vector<PointCloud::Ptr> inputSequenceAfterDownSample;

    // save point clouds that have been merged, in this way, we may avoid the accurative error after too many tansform
    PointCloud::Ptr allPointCloud(new PointCloud);

    // create a view handle
    boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("icp"));

    // create two views
    int v1; 
    int v2;
    view->createViewPort(0.0, 0.0, 0.5, 1.0, v1); 
    view->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    //view->addCoordinateSystem(1.0);   
    view->initCameraParameters();

    // save the global matrix
    Eigen::Matrix4f globalTransformMatrix = Eigen::Matrix4f::Identity();

    // main loop
    int startIndex = (int)atof( pd.getData( "startIndex" ).c_str());
    int stopIndex = (int)atof( pd.getData( "stopIndex" ).c_str());
    
    for(int iterations = 0;iterations < stopIndex - startIndex;iterations++)
    {
        view->spinOnce();

        PointCloud::Ptr oneFrame(new PointCloud);
        PointCloud::Ptr oneFrameAfterDownSample(new PointCloud);
        PointCloud::Ptr oneFrameResult(new PointCloud);

        cv::Mat rgb;
        cv::Mat depth;

        // read from realsense
        readFromFile(rgb, depth, iterations+startIndex);
        generatePointCloud(rgb, depth, oneFrame);

        // down sample, using a grid box
        inputSequence.push_back(oneFrame);
        std::cout << YELLOW"before down sample, points number : " << inputSequence[iterations]->points.size();

        float downSampleSize = (float)atof(pd.getData("downSampleSize").c_str());
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
            continue;
        }

        // start ICP process
        std::cout << RESET"waiting for ICP process..." << std::endl;
        pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;

        // use now point cloud to find before 
        icp.setInputSource(inputSequenceAfterDownSample[iterations]);   // point cloud before
        icp.setInputTarget(inputSequenceAfterDownSample[iterations-1]); // point cloud now

        // start to dp ICP
        icp.setMaxCorrespondenceDistance(100);
        icp.setTransformationEpsilon(1e-10); 
        icp.setEuclideanFitnessEpsilon(0.01); 
        icp.setMaximumIterations(50); 
        icp.align(*oneFrameResult);

        // it seems like the calculation of global transform matrix is wrong
        globalTransformMatrix = globalTransformMatrix * icp.getFinalTransformation();
        pcl::transformPointCloud(*(inputSequence[iterations]), *oneFrameResult, globalTransformMatrix);
        
        // show point cloud
        const std::string namePC = int2str(iterations);
        const char *nameChar = namePC.c_str();

        view->removePointCloud("original");
        view->addPointCloud(inputSequence[iterations], "original", v1);
        view->addPointCloud(oneFrameResult, nameChar, v2);

        // the smaller of the score, the better
        std::cout << "has conveged : " << icp.hasConverged() << " score : " << icp.getFitnessScore() << std::endl;
        std::cout << GREEN"matrix : \n" << icp.getFinalTransformation() << std::endl;
        std::cout << RESET"Max correspondence distance : " << icp.getMaxCorrespondenceDistance() << std::endl;
        std::cout << "Max iteration : " << icp.getMaximumIterations() << std::endl;
        std::cout << "This is " <<  iterations << " iterations" << std::endl;
        std::cout << "-----------------------------------------------------------------------" << std::endl;
    }
    
    std::cout << "finish all alignment..." << std::endl;
    view->spin();

    return 0;
}


