#include <iostream>
#include <string>
using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

 // 定义点云类型
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud; 

// 相机内参
const double camera_factor = 1;
const double camera_cx = 321.153;
const double camera_cy = 246.486;
const double camera_fx = 610.098;
const double camera_fy = 610.098;

int PCDtoPLYconvertor(string & input_filename ,string& output_filename)
{
    pcl::PCLPointCloud2 cloud;
    if (pcl::io::loadPCDFile(input_filename , cloud) < 0)
    {
        cout << "Error: cannot load the PCD file!!!"<< endl;
        return -1;
    }

    pcl::PLYWriter writer;
    writer.write(output_filename, cloud, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(),true,true);
    return 0;
}

// 主函数 
int main( int argc, char** argv )
{
    // 图像矩阵
    cv::Mat rgb, depth;
    // 使用cv::imread()来读取图像
    rgb = cv::imread( "../data/rgb_png/10.png" );
    // rgb 图像是8UC3的彩色图像
    // depth 是16UC1的单通道图像，注意flags设置-1,表示读取原始数据不做任何修改
    depth = cv::imread( "../data/depth_png/10.png", -1 );
    cv::imshow("1", rgb);
    cv::imshow("2", depth);

    PointCloud::Ptr cloud (new PointCloud);
    
    // 遍历深度图
    for (int m = 0; m < depth.rows; m++)
    {
        for (int n = 0; n < depth.cols; n++)
        {
            // 获取深度图中(m,n)处的值
            ushort d = depth.ptr<ushort>(m)[n];
            
            // d 可能没有值，若如此，跳过此点
            if (d == 0)
                continue;
            // d 存在值，则向点云增加一个点
            PointT p;
            cout << d << endl;
            // 计算这个点的空间坐标
            p.z = double(d) / camera_factor;
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
    cloud->width = cloud->points.size();
    cout<<"point cloud size = "<< cloud->points.size() << endl;
    cloud->is_dense = false;
    pcl::io::savePCDFile("./pointcloud.pcd", *cloud);

    // 清除数据并退出
    cloud->points.clear();
    cout<<"Point cloud saved."<<endl;

    string input_filename("./pointcloud.pcd");
    string output_filename("./pointcloud.ply");

    PCDtoPLYconvertor(input_filename , output_filename);
    cv::waitKey(0);
    return 0;
}