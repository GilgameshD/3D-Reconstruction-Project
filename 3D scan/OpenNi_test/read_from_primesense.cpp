#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include <iostream>
#include <string>
#include <vector> 

#include <opencv2/photo.hpp>
#include <opencv2/highgui.hpp> 

#include <OpenNI.h> 

typedef unsigned char uint8_t;
// namespace
using namespace std;
using namespace openni;
using namespace cv;
using namespace pcl;

std::string int2str(const int &int_temp)
{
    std::string string_temp;
    std::stringstream stream;
    stream << int_temp;
    string_temp = stream.str(); 
    return string_temp;
}

void CheckOpenNIError(Status result, string status)  
{   
    if(result != STATUS_OK)   
        cerr << status << " Error: " << OpenNI::getExtendedError() << endl;  
} 

int main(int argc, char **argv)
{
	float x = 0.0, y = 0.0, z = 0.0, xx = 0.0;  

	IplImage *test2;
	char filename[20] = {0};

	// point cloud 
	PointCloud<PointXYZRGB> color_cloud;

	// opencv image
	Mat cvBGRImg; 
	Mat cvDepthImg;  

	// OpenNI2 image  
    VideoFrameRef oniDepthImg;  
    VideoFrameRef oniColorImg;
    VideoStream   depth, color;
    Status rc = STATUS_OK;

	namedWindow("depth_window");  
    namedWindow("image_window"); 

	// 初始化OpenNI  
    rc = OpenNI::initialize();
	printf("Start initialization:\n%s", openni::OpenNI::getExtendedError());
	
    // open device    
    Device device;  
    rc = device.open(openni::ANY_DEVICE); 
    if (rc != openni::STATUS_OK)
    {
        printf("Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
        openni::OpenNI::shutdown();
        exit(0);
    }

    int wight = 640;
    int height = 480;
    VideoMode depthMode;
    depthMode.setFps(30);
    depthMode.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
    depthMode.setResolution(wight, height);

    VideoMode colorMode;
    colorMode.setFps(30);
    colorMode.setPixelFormat(openni::PIXEL_FORMAT_RGB888);
    colorMode.setResolution(wight, height);

    // create and start depth stream
    rc = depth.create(device, openni::SENSOR_DEPTH);
    if (rc == openni::STATUS_OK)
    {
        rc = depth.setVideoMode(depthMode);
        if (openni::STATUS_OK != rc) 
        {
            printf("error: depth format not supported...\n");
            exit(0);
        }

        rc = depth.start();
        if (rc != openni::STATUS_OK)
        {
            printf("SimpleViewer: Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
            depth.destroy();
        }
    }
    else
        printf("Couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError());

    // create and start color stream
    rc = color.create(device, openni::SENSOR_COLOR);
    if (rc == openni::STATUS_OK)
    {
        rc =  color.setVideoMode(colorMode);
        if (openni::STATUS_OK != rc)
        {
            printf("error: color format not supported...\n");
            exit(0);
        }

        rc = color.start();
        if (rc != openni::STATUS_OK)
        {
            printf("Couldn't start color stream:\n%s\n", openni::OpenNI::getExtendedError());
            color.destroy();
        }
    }
    else
        printf("Couldn't find color stream:\n%s\n", openni::OpenNI::getExtendedError());

    if (!depth.isValid() || !color.isValid())
    {
        printf("No valid streams. Exiting\n");
        openni::OpenNI::shutdown();
        return 2;
    }

    // set device synchronise
    device.setDepthColorSyncEnabled(TRUE);
    if (rc != openni::STATUS_OK)
    {
        printf("could not synchronise device\n");
        exit(0);
    }

    // alignment the depth image to the color image
    openni::CameraSettings* settings = color.getCameraSettings();
    settings->setAutoExposureEnabled(TRUE);
    settings->setAutoWhiteBalanceEnabled(TRUE);
    rc = device.setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR);
    if (rc != openni::STATUS_OK)
    {
        printf("could not support image registration Mode.\n");
        exit(0);
    }

    cout << "Finish all initialization... " << endl;
    cout << "Start to show images..." << endl;
	int count = 0;
    int file_number = 0;
    Mat depthToBeSaved;
	while(true)
	{
		// read frame  
        if(color.readFrame(&oniColorImg) == STATUS_OK)  
        {  
            // convert data into OpenCV type  
            Mat cvRGBImg(oniColorImg.getHeight(), oniColorImg.getWidth(), CV_8UC3, (void*)oniColorImg.getData());  
            cvtColor(cvRGBImg, cvBGRImg, CV_RGB2BGR);  
            imshow("image_window", cvBGRImg);  
        }  

		if(depth.readFrame(&oniDepthImg) == STATUS_OK)  
        {  
            Mat cvRawImg16U(oniDepthImg.getHeight(), oniDepthImg.getWidth(), CV_16UC1, (void*)oniDepthImg.getData());  
            depthToBeSaved = cvRawImg16U;
            cvRawImg16U.convertTo(cvDepthImg, CV_8U, 255.0/(depth.getMaxPixelValue()));    
            imshow("depth_window", cvDepthImg);  
        } 

		char input = waitKey(1); 
		// quit
        if(input == 'q')      
            break;
		// capture  depth and color data   
        if(count % 2 == 0)
		{
			//get data
			DepthPixel *pDepth = (DepthPixel*)oniDepthImg.getData();
			//create point cloud
			color_cloud.width = oniDepthImg.getWidth();
			color_cloud.height = oniDepthImg.getHeight();
			color_cloud.is_dense = false;
			color_cloud.points.resize(color_cloud.width * color_cloud.height);
            cout << "depth image width = " << color_cloud.width << ", depth image height = " << color_cloud.height << endl;

			IplImage temp11 = (IplImage)cvBGRImg;
			test2 = &temp11;			

			for(int i = 0;i < oniDepthImg.getHeight();i++)
			{
				 for(int j = 0;j < oniDepthImg.getWidth();j++)
				 {
					 float k = i;  
					 float m = j; 
					 xx = pDepth[i*oniDepthImg.getWidth()+j];

					 // the disparity is fixed, so the far the point is, the more error it will get
					 CoordinateConverter::convertDepthToWorld(depth, k, m, xx, &x,&y,&z); 
					 color_cloud[i*color_cloud.width+j].x = x*0.001;
					 color_cloud[i*color_cloud.width+j].y = y*0.001;
					 color_cloud[i*color_cloud.width+j].z = z*0.001;
					 color_cloud[i*color_cloud.width+j].b = (uint8_t)test2->imageData[i*test2->widthStep+j*3+0];
					 color_cloud[i*color_cloud.width+j].g = (uint8_t)test2->imageData[i*test2->widthStep+j*3+1];
					 color_cloud[i*color_cloud.width+j].r = (uint8_t)test2->imageData[i*test2->widthStep+j*3+2];
				 }
	   		 }
			
            string fileName1 = "../data/" + int2str(file_number) + ".ply";
            string fileName2 = "../data/depth_png/" + int2str(file_number) + ".png";
            string fileName3 = "../data/rgb_png/" + int2str(file_number) + ".png";
			//pcl::io::savePLYFileBinary(fileName1, color_cloud);
            imwrite(fileName2, depthToBeSaved);  // !!!! improtant, the image to be shown and saved are not the same.
            imwrite(fileName3, cvBGRImg);

			cout << "The " << int2str(file_number) + " point cloud is saved" << endl;
            file_number++;
		}
        count++;
	}
}

