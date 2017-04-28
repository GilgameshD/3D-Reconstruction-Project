/*
 * read rgb and depth image and write point cloud files to the disk, using default method provided by intel API
 * but the results seem to have something wrong, the color may has some mistakes
 */

#include <iostream>
#include <vector>
#include <ctime>
#include <string>
#include <cstdio>

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

// realsense
#include <librealsense/rs.hpp>

// OpenCV
#include <opencv2/opencv.hpp> 
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

void readFromRealSense(rs::device *dev, PointT::Ptr oneFrame, cv::Mat &color, cv::Mat &depth)
{
    dev->wait_for_frames();

    // Retrieve our images
    const uint16_t *depth_image = (const uint16_t *)dev->get_frame_data(rs::stream::depth);
    const uint8_t *color_image = (const uint8_t *)dev->get_frame_data(rs::stream::color);

    // Retrieve camera parameters for mapping between depth and color
    rs::intrinsics depth_intrin = dev->get_stream_intrinsics(rs::stream::depth);
    rs::extrinsics depth_to_color = dev->get_extrinsics(rs::stream::depth, rs::stream::color);
    rs::intrinsics color_intrin = dev->get_stream_intrinsics(rs::stream::color);
    float scale = dev->get_depth_scale();
/*
    std::cout << color_intrin.intrin.fx << " " << color_intrin.intrin.fy << " "
              << color_intrin.intrin.ppx << " " << color_intrin.intrin.ppy << std::endl;

    std::cout << depth_intrin.intrin.fx << " " << depth_intrin.intrin.fy << " "
              << depth_intrin.intrin.ppx << " " << depth_intrin.intrin.ppy << endl;
*/
    int index = 0;
    for(int dy = 0;dy < depth_intrin.height; ++dy)
    {
        for(int dx = 0;dx < depth_intrin.width; ++dx)
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
            if(X < 0 || Y < 0 || X >= color_intrin.width || Y >= color_intrin.height)
            {
                R = 255;
                G = 255;
                B = 255;
            }
            else
            {
                R = *(color_image + (Y * color_intrin.width + X) * 3);
                G = *(color_image + (Y * color_intrin.width + X) * 3+1);
                B = *(color_image + (Y * color_intrin.width + X) * 3+2);
            }

            PointRGB addNewPoint;  // point cloud
            addNewPoint.x = depth_point.x;
            addNewPoint.y = depth_point.y;
            addNewPoint.z = depth_point.z;
            addNewPoint.r = R;
            addNewPoint.g = G;
            addNewPoint.b = B;
            oneFrame->points.push_back(addNewPoint);

            // save color and depth image
            color.at<cv::Vec3b>(depth_point.y,depth_point.x)[0] = color_image[(Y * color_intrin.width + X)*3];
            color.at<cv::Vec3b>(depth_point.y,depth_point.x)[1] = color_image[(Y * color_intrin.width + X)*3+1];
            color.at<cv::Vec3b>(depth_point.y,depth_point.x)[2] = color_image[(Y * color_intrin.width + X)*3+2];

            depth.at<cv::Vec3b>(depth_point.y,depth_point.x) = depth_image[dy * depth_intrin.width + dx]%255;
            index++;
        }
    }
}

int main() try
{
    // Turn on logging. We can separately enable logging to console or to file, and use different severity filters for each.
    rs::log_to_console(rs::log_severity::warn);

    // Create a context object. This object owns the handles to all connected realsense devices.
    rs::context ctx;
    rs::device * dev;

    if(initRealsense(ctx, &dev) == EXIT_FAILURE) 
        return EXIT_FAILURE;
    std::cout << "Read from realsense..." << std::endl;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("icp")); 
    view->initCameraParameters();

    int index = 0;
    int iteration = 0;
    while(true)
    {
        view->spinOnce(); 

        PointT::Ptr oneFrame(new PointT);
        cv::Mat color(480,640,CV_32FC3, cv::Scalar(0, 0, 0));
        cv::Mat depth(480,640,CV_32FC1, cv::Scalar(0));

        readFromRealSense(dev, oneFrame, color, depth);
        cv::imwrite("../data/color.jpg", color);
        cv::imwrite("../data/depth.jpg", depth);
        imshow("color", color);

        if(iteration % 1 == 0)
        {
            oneFrame->width = 1;
            oneFrame->height = oneFrame->points.size();
            const std::string namePC = "../data/" + int2str(index) + ".pcd";
            //pcl::PCDWriter writer;  
            //writer.write(namePC, *oneFrame);  
            index++;
            std::cout << "save one image...[" << index << "]" << std::endl;

            //view->removePointCloud("original");
            //view->addPointCloud(oneFrame, "original");
        }

        iteration++;
    }
    
    return EXIT_SUCCESS;
}
catch(const rs::error & e)
{
    // Method calls against librealsense objects may throw exceptions of type rs::error
    printf("rs::error was thrown when calling %s(%s):\n", e.get_failed_function().c_str(), e.get_failed_args().c_str());
    printf("    %s\n", e.what());
    return EXIT_FAILURE;
}
