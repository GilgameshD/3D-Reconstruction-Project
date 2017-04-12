// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

///////////////////////////////////////////////////////
// librealsense tutorial #3 - Point cloud generation //
///////////////////////////////////////////////////////

// First include the librealsense C++ header file
#include <librealsense/rs.hpp>
#include <iostream>

#include <opencv2/opencv.hpp>

int iteration = 0;
int saveNumber = 0;
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


std::string int2str(int &int_temp)
{
    std::string string_temp;
    std::stringstream stream;
    stream << int_temp;
    string_temp = stream.str(); 
    return string_temp;
}


void readfromrealsense(rs::device * dev)
{
    dev->wait_for_frames();

    // Retrieve camera parameters for mapping between depth and color
    rs::intrinsics depth_intrin = dev->get_stream_intrinsics(rs::stream::depth_aligned_to_color);
    rs::intrinsics color_intrin = dev->get_stream_intrinsics(rs::stream::rectified_color);
    float scale = dev->get_depth_scale();

    cv::Mat rgb(color_intrin.height, color_intrin.width, CV_8UC3, (uchar *)dev->get_frame_data(rs::stream::rectified_color));
    cv::Mat depth(depth_intrin.height, depth_intrin.width, CV_16U,(uchar *)dev->get_frame_data(rs::stream::depth_aligned_to_color));

    //std::cout << color_intrin.fx << " " << color_intrin.fy << " " << color_intrin.ppx << " " << color_intrin.ppy << std::endl;
    //std::cout << scale << std::endl;

    if(iteration % 1 == 0)
    {
        const std::string namePC1 = "../data/rgb_png/" + int2str(saveNumber) + ".png";
        const std::string namePC2 = "../data/depth_png/" + int2str(saveNumber) + ".png";

        saveNumber++;
        cvtColor(rgb, rgb, cv::COLOR_BGR2RGB);
        cv::imwrite(namePC1, rgb);
        cv::imwrite(namePC2, depth);
        std::cout << "saved one image : " << namePC1 << std::endl;
    }
    iteration++;
}

int main() try
{
    // Turn on logging. We can separately enable logging to console or to file, and use different severity filters for each.
    rs::log_to_console(rs::log_severity::warn);
    //rs::log_to_file(rs::log_severity::debug, "librealsense.log");

    // Create a context object. This object owns the handles to all connected realsense devices.
    rs::context ctx;
    rs::device * dev;

    if(initRealsense(ctx, &dev) == EXIT_FAILURE) 
        return EXIT_FAILURE;

    std::cout << "Read" << std::endl;
    while(true)
    {
        readfromrealsense(dev);
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
