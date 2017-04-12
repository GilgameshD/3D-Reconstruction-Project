/*
1. 储存的点云用来做ICP拼接得到旋转矩阵，备选方案是2D-3D的pnp方法
2. 储存的rgb用来做ORB特征检测判断相关性，对于相关的点加入到g2o
3. 最后做一遍全局优化，将结果拿出来降采样输出
 */

#include <iostream>
#include <vector>
// pcl lib
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/console/parse.h> 
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointT;
typedef pcl::PointXYZRGB PointRGB;

std::string int2str(const int &int_temp)
{
    std::string string_temp;
    std::stringstream stream;
    stream << int_temp;
    string_temp = stream.str(); 
    return string_temp;
}

void readFromFile(PointT::Ptr oneFrame, int iterations)
{
    const std::string namePC = "../data/" + int2str(iterations) + ".pcd";
    pcl::io::loadPCDFile<PointRGB>(namePC, *oneFrame);
    std::cout << "read one file, and size is : " << oneFrame->points.size() << endl;
}

int main(int argc, char** argv) 
{   
    // record all point clouds
    std::vector<PointT::Ptr> inputSequence;
    std::vector<PointT::Ptr> inputSequenceAfterDownSample;

    // create a view handle
    boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("icp"));
    // create two views
    int v1; 
    int v2;
    view->createViewPort(0.0, 0.0, 0.5, 1.0, v1); 
    view->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    // show coordinate
    //view->addCoordinateSystem(1.0);   
    view->initCameraParameters();

    // record the global matrix
    Eigen::Matrix4f globalTransformMatrix = Eigen::Matrix4f::Identity();

    // main loop
    for(int iterations = 0;iterations < 20;iterations++)
    {
        view->spinOnce(); 

        PointT::Ptr oneFrame(new PointT);
        PointT::Ptr oneFrameAfterDownSample(new PointT);
        PointT::Ptr oneFrameResult(new PointT);

        // read from real sense
        readFromFile(oneFrame, iterations);

        // down sample, using a grid box
        inputSequence.push_back(oneFrame);
        std::cout << "before down sample, points number : " << inputSequence[iterations]->points.size() << std::endl;
        double downSampleSize = 0.004;
        pcl::VoxelGrid<PointRGB> grid;
        grid.setLeafSize(downSampleSize, downSampleSize, downSampleSize);
        grid.setInputCloud(oneFrame);
        grid.filter(*oneFrameAfterDownSample);
        std::cout << "after down sapmle, points number : " << oneFrameAfterDownSample->points.size() << std::endl;
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
        std::cout << "waiting for ICP process..." << std::endl;
        pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp; 

        // use now point cloud to find before 
        icp.setInputSource(inputSequenceAfterDownSample[iterations]);   // point cloud bfore
        icp.setInputTarget(inputSequenceAfterDownSample[iterations-1]); // point cloud now

        // start to dp ICP
        //icp.setMaxCorrespondenceDistance(0.01);  
        icp.setTransformationEpsilon(1e-10); 
        icp.setEuclideanFitnessEpsilon(0.01); 
        icp.setMaximumIterations(50); 
        icp.align(*oneFrameResult);

        // it seems like the calculation of global transdorm matrix is wrong
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
        std::cout << "matrix : \n" << icp.getFinalTransformation() << std::endl;
        std::cout << "Max correspondence distance : " << icp.getMaxCorrespondenceDistance() << std::endl;
        std::cout << "Max iteration : " << icp.getMaximumIterations() << std::endl;
        std::cout << "show point cloud now ..." << std::endl;
    }
    
    std::cout << "finish all alignment..." << std::endl;
    view->spin();

    return 0;
}


