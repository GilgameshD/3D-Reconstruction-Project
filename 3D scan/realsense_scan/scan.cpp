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

#include <librealsense/rs.hpp>


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


void readFromRealSense(rs::device *dev, PointT::Ptr oneFrame)
{
    dev->wait_for_frames();

    // Retrieve our images
    const uint16_t * depth_image = (const uint16_t *)dev->get_frame_data(rs::stream::depth);
    const uint8_t * color_image = (const uint8_t *)dev->get_frame_data(rs::stream::color);

    // Retrieve camera parameters for mapping between depth and color
    rs::intrinsics depth_intrin = dev->get_stream_intrinsics(rs::stream::depth);
    rs::extrinsics depth_to_color = dev->get_extrinsics(rs::stream::depth, rs::stream::color);
    rs::intrinsics color_intrin = dev->get_stream_intrinsics(rs::stream::color);
    float scale = dev->get_depth_scale();

    int index = 0;
    for(int dy = 0;dy < depth_intrin.height; ++dy)
    {
        for(int dx = 0;dx<depth_intrin.width; ++dx)
        {
            // Retrieve the 16-bit depth value and map it into a depth in meters
            uint16_t Z = depth_image[dy * depth_intrin.width + dx];
            float depth_in_meters = Z * scale;

            // Skip over pixels with a depth value of zero, which is used to indicate no data
            if(Z == 0) 
                continue;

            // Map from pixel coordinates in the depth image to pixel coordinates in the color image
            rs::float2 depth_pixel = {(float)dx, (float)dy};
            rs::float3 depth_point = depth_intrin.deproject(depth_pixel, depth_in_meters);
            rs::float3 color_point = depth_to_color.transform(depth_point);
            rs::float2 color_pixel = color_intrin.project(color_point);

            // Use the color from the nearest color pixel, or pure white if this point falls outside the color image
            int X = (int)std::round(color_pixel.x), Y = (int)std::round(color_pixel.y);
            
            uint8_t R, G, B;
            R = *(color_image + (Y * color_intrin.width + X) * 3);
            G = *(color_image + (Y * color_intrin.width + X) * 3+1);
            B = *(color_image + (Y * color_intrin.width + X) * 3+2);

            PointRGB addNewPoint;
            addNewPoint.x = depth_point.x;
            addNewPoint.y = depth_point.y;
            addNewPoint.z = depth_point.z;
            addNewPoint.r = R;
            addNewPoint.g = G;
            addNewPoint.b = B;
            oneFrame->points.push_back(addNewPoint);
            index++;
        }
    }
}

int main(int argc, char** argv) try
{   
    // record all point clouds
    std::vector<PointT::Ptr> inputSequence;

    // create a view handle
    boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("icp"));
    // create two views
    int v1; 
    int v2;
    view->createViewPort(0.0, 0.0, 0.5, 1.0, v1); 
    view->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    // show coordinate
    view->addCoordinateSystem(1.0);   
    view->initCameraParameters();

    // Turn on logging. We can separately enable logging to console or to file, and use different severity filters for each.
    rs::log_to_console(rs::log_severity::warn);

    // Create a context object. This object owns the handles to all connected realsense devices.
    rs::context ctx;
    rs::device *dev;

    if(initRealsense(ctx, &dev) == EXIT_FAILURE) 
    {
        std::cout << "hardware fails ..." << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "start reading from realsense..." << std::endl;

    // main loop
    int iterations = 0; 
    while(!view->wasStopped())
    {
        view->spinOnce(); 

        PointT::Ptr oneFrame(new PointT);
        PointT::Ptr oneFrameResult(new PointT);

        // read from real sense
        readFromRealSense(dev, oneFrame);
        inputSequence.push_back(oneFrame);

        if(iterations < 5)
        {
            // share a same point cloud at the first frame
            oneFrameResult = inputSequence[iterations];

            const std::string namePC = int2str(iterations) + "right";
            const char *nameChar = namePC.c_str();

            view->addPointCloud(inputSequence[iterations], "original", v1);
            view->addPointCloud(oneFrameResult, nameChar, v2);
            iterations++;
            continue;
        }

        // down sample, using a grid box
        std::cout << "before down sample points number : " << inputSequence[iterations]->points.size();
        double downSampleSize = 0.005;
        pcl::VoxelGrid<PointRGB> grid;
        grid.setLeafSize(downSampleSize, downSampleSize, downSampleSize);
        grid.setInputCloud(inputSequence[iterations]);
        grid.filter(*(inputSequence[iterations]));
        std::cout << ", after down sapmle, points number : " << inputSequence[iterations]->points.size() << std::endl;

        // start ICP process
        std::cout << "waiting for ICP process..." << std::endl;
        pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp; 

        // use now point cloud to find before 
        icp.setInputSource(inputSequence[iterations]);   // point cloud bfore
        icp.setInputTarget(inputSequence[iterations-1]); // point cloud now

        // start to dp ICP
        //icp.setMaxCorrespondenceDistance(250);  
        icp.setTransformationEpsilon(1e-10); 
        icp.setEuclideanFitnessEpsilon(0.01); 
        icp.setMaximumIterations(50); 
        icp.align(*oneFrameResult);

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

        iterations++;
    }
    std::cout << "finish all alignment..." << std::endl;
    view->spin();

    return 0;
}
catch(const rs::error & e)
{
    // Method calls against librealsense objects may throw exceptions of type rs::error
    printf("rs::error was thrown when calling %s(%s):\n", e.get_failed_function().c_str(), e.get_failed_args().c_str());
    printf("    %s\n", e.what());
    return EXIT_FAILURE;
}




