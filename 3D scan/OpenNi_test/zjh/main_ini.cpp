/*****************************************************************************
*                                                                            *
*  OpenNI 2.x Alpha                                                          *
*  Copyright (C) 2012 PrimeSense Ltd.                                        *
*                                                                            *
*  This file is part of OpenNI.                                              *
*                                                                            *
*  Licensed under the Apache License, Version 2.0 (the "License");           *
*  you may not use this file except in compliance with the License.          *
*  You may obtain a copy of the License at                                   *
*                                                                            *
*      http://www.apache.org/licenses/LICENSE-2.0                            *
*                                                                            *
*  Unless required by applicable law or agreed to in writing, software       *
*  distributed under the License is distributed on an "AS IS" BASIS,         *
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  *
*  See the License for the specific language governing permissions and       *
*  limitations under the License.                                            *
*                                                                            *
*****************************************************************************/
#include <OpenNI.h>
// #include "Viewer.h"
#include <opencv2/opencv.hpp>
#include <time.h>
using namespace cv;
int main(int argc, char** argv)
{
    openni::Status rc = openni::STATUS_OK;

    openni::Device device;
    openni::VideoStream depth, color;
    openni::VideoFrameRef       m_depthFrame;
    openni::VideoFrameRef       m_colorFrame;

    const char* deviceURI = openni::ANY_DEVICE;
    if (argc > 1)
    {
        deviceURI = argv[1];
    }

    rc = openni::OpenNI::initialize();

    printf("After initialization:\n%s\n", openni::OpenNI::getExtendedError());

    rc = device.open(deviceURI);
    if (rc != openni::STATUS_OK)
    {
        printf("SimpleViewer: Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
        openni::OpenNI::shutdown();
        return 1;
    }
 // get sensor info
    const openni::SensorInfo* depthInfo = device.getSensorInfo(openni::SENSOR_DEPTH); 
    const openni::Array<openni::VideoMode> & modesDepth = depthInfo->getSupportedVideoModes();
    for (int i = 0; i < modesDepth.getSize(); i++){
        printf("%i: %ix%i, %i fps, %i format\n", i, modesDepth[i].getResolutionX(), modesDepth[i].getResolutionY(), modesDepth[i].getFps(), modesDepth[i].getPixelFormat()); //PIXEL_FORMAT_DEPTH_1_MM = 100, PIXEL_FORMAT_DEPTH_100_UM
    }
  
    const openni::SensorInfo* colorInfo = device.getSensorInfo(openni::SENSOR_COLOR); 
    const openni::Array<openni::VideoMode> & modesColor = colorInfo->getSupportedVideoModes();
    for (int i = 0; i < modesColor.getSize(); i++){
        printf("%i: %ix%i, %i fps, %i format\n", i, modesColor[i].getResolutionX(), modesColor[i].getResolutionY(), modesColor[i].getFps(), modesColor[i].getPixelFormat());
    }

    rc = depth.create(device, openni::SENSOR_DEPTH);
    if (rc == openni::STATUS_OK)
    {
        int depthMode = 4;
        rc = depth.setVideoMode(modesDepth[depthMode]);
        openni::VideoMode depthVideoMode = color.getVideoMode();
        depthVideoMode.setResolution(1280,1024);
        rc = depth.setVideoMode(depthVideoMode);
        if (openni::STATUS_OK != rc) {
                 printf("error: depth format not supported...\n");
        }
        else{
             printf("set depth video mode %d.\n",depthMode);
        }

        rc = depth.start();
        if (rc != openni::STATUS_OK)
        {
            printf("SimpleViewer: Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
            depth.destroy();
        }
    }
    else
    {
        printf("SimpleViewer: Couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError());
    }

    rc = color.create(device, openni::SENSOR_COLOR);
    if (rc == openni::STATUS_OK)
    {
        int colorMode = 15;
        rc =  color.setVideoMode(modesColor[colorMode]);
        if (openni::STATUS_OK != rc){
            printf("error: color format not supported...\n");
        }
        else{
            printf("set color video mode %d.\n", colorMode);
        }
        rc = color.start();
        if (rc != openni::STATUS_OK)
        {
            printf("SimpleViewer: Couldn't start color stream:\n%s\n", openni::OpenNI::getExtendedError());
            color.destroy();
        }
    }
    else
    {
        printf("SimpleViewer: Couldn't find color stream:\n%s\n", openni::OpenNI::getExtendedError());
    }

    if (!depth.isValid() || !color.isValid())
    {
        printf("SimpleViewer: No valid streams. Exiting\n");
        openni::OpenNI::shutdown();
        return 2;
    }


    
  
     // set stream mode
    // int depthMode = 4;
    // rc = depth.setVideoMode(modesDepth[depthMode]);
    // if (openni::STATUS_OK != rc) {
    //      printf("error: depth format not supported...\n");
    // }
    //  else{
    //      printf("set depth video mode %d.\n",depthMode);
    // }
    // int colorMode = 15;
    // rc =  color.setVideoMode(modesColor[colorMode]);
    // if (openni::STATUS_OK != rc){
    //     printf("error: color format not supported...\n");
    // }
    // else{
    //     printf("set color video mode %d.\n", colorMode);
    // }
    

    //     // set device mode
    device.setDepthColorSyncEnabled(TRUE);
    if (rc != openni::STATUS_OK){
        printf("could not synchronise device\n");
        exit(0);
    }

    openni::CameraSettings* settings = color.getCameraSettings();
    settings->setAutoExposureEnabled(TRUE);
    settings->setAutoWhiteBalanceEnabled(TRUE);
    rc = device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
    if (rc != openni::STATUS_OK){
        printf("could not support image registration Mode.\n");
        exit(0);
    }
    // Mat depthImg, colorImg;
    // while(1){
    //     rc = device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
    //     // openni::VideoMode colorVideoMode = color.getVideoMode();
    //     // printf("color resolution, X: %d, Y: %d\n", colorVideoMode.getResolutionX(), colorVideoMode.getResolutionY());
    //     if(depth.isValid()){
    //         if(depth.readFrame(&m_depthFrame) == openni::STATUS_OK){
    //             printf("read and save depth image\n");
    //             Mat depthImg;
    //             int depthImg_h, depthImg_w;
    //             const uint16_t* depthImgBuf = (const uint16_t*) m_depthFrame.getData();
    //             depthImg_h = m_depthFrame.getHeight(); 
    //             depthImg_w = m_depthFrame.getWidth();
    //             depthImg.create(depthImg_h, depthImg_w, CV_16UC1);
    //             memcpy(depthImg.data, depthImgBuf, depthImg_h  *  depthImg_w * sizeof(uint16_t));
    //             imwrite("depthImage.png", depthImg);
    //         }
    //     }
    //     if(color.isValid()){
    //         if(color.readFrame(&m_colorFrame) == openni::STATUS_OK){
    //             printf("read and save color image\n");
    //             Mat  colorImg;
    //             int colorImg_h, colorImg_w;
    //             const openni::RGB888Pixel* colorImgBuf = (const openni::RGB888Pixel*) m_colorFrame.getData();
    //             colorImg_h = m_colorFrame.getHeight(); 
    //             colorImg_w = m_colorFrame.getWidth();
    //             colorImg.create(colorImg_h, colorImg_w, CV_8UC3);
    //             memcpy(colorImg.data, colorImgBuf, 3* colorImg_h  *  colorImg_w * sizeof(uint8_t));
    //             cvtColor(colorImg, colorImg, CV_RGB2BGR);
    //             imwrite("colorImage.png", colorImg);
    //         }
    //     }
    //     sleep(1);
    // }
   SampleViewer sampleViewer("Simple Viewer", device, depth, color);

   rc = sampleViewer.init(argc, argv);
   if (rc != openni::STATUS_OK)
   {
       openni::OpenNI::shutdown();
       return 3;
  }
   sampleViewer.run();
}
