#include "slamBase.h"


int main(int argc, char** argv)
{
    ParameterReader pd;
/*
    // open realsense
    rs::log_to_console(rs::log_severity::warn);
    rs::context ctx;
    rs::device * dev;

    if(initRealsense(ctx, &dev) == EXIT_FAILURE) 
    {
        cout << "Open realsense failure..." << endl;
        return EXIT_FAILURE;
    }
    else
        cout << "Open realsense successfully..." << endl;
*/


    // save all key frames
    vector<FRAME> keyframes; 
    
    // initialize and read one frame
    cout << "Initializing ... Waiting for the first frame..." << endl;
    FRAME currFrame;
    currFrame.frameID = 0;
    int startFrame = 0;
    while(currFrame.rgb.empty() || currFrame.depth.empty() || startFrame < 10)
    {
        //readfromrealsense(dev, currFrame.rgb, currFrame.depth); 
        readFromFile(currFrame.rgb, currFrame.depth);
        startFrame++;
    }
    cout << "Get first frame..." << endl;

    string detector = pd.getData("detector");
    string descriptor = pd.getData("descriptor");
    float downSampleSize = (float)atof(pd.getData("downSampleSize").c_str());
    CAMERA_INTRINSIC_PARAMETERS camera = getDefaultCamera();

    // extractor
    computeKeyPointsAndDesp(currFrame, detector, descriptor);
    PointCloud::Ptr cloud = image2PointCloud(currFrame.rgb, currFrame.depth, camera);

    // g2o optimizer
    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(blockSolver); // use LM
    g2o::SparseOptimizer globalOptimizer;  
    globalOptimizer.setAlgorithm(solver); 
    globalOptimizer.setVerbose(false);
    
    // add first node to globalOptimizer
    g2o::VertexSE3 *v = new g2o::VertexSE3();
    v->setId(0);
    // the first node should be identity and fixed
    v->setEstimate(Eigen::Isometry3d::Identity()); 
    v->setFixed(true);  
    globalOptimizer.addVertex(v);
    
    // add first frame to key frame
    keyframes.push_back(currFrame);
    double keyframe_threshold = atof(pd.getData("keyframe_threshold").c_str());

    // start another thread to show point cloud
    //boost::shared_ptr<ShowPointCloud> showPC = boost::make_shared<ShowPointCloud>(camera);

    // start another thread to do optimization every 5 frames
    //LocalMappingOptimization lpmapping;
    //lpmapping.start();

    boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("windows"));
    view->addCoordinateSystem(1.0); 
    PointCloud::Ptr globalMap(new PointCloud);

    // start to do loop closure
    int generateIndex = 0;
    int index = 1;
    while(1)
    {
        cout << RESET"Reading one frame " << index << endl;
        FRAME currFrame;
        currFrame.frameID = index;
        while(currFrame.rgb.empty() || currFrame.depth.empty())
        {
            //readfromrealsense(dev, currFrame.rgb, currFrame.depth); 
            readFromFile(currFrame.rgb, currFrame.depth);
        }   

        computeKeyPointsAndDesp(currFrame, detector, descriptor); 

        CHECK_RESULT result = checkKeyframes(camera, keyframes.back(), currFrame, globalOptimizer); //匹配该帧与keyframes里最后一帧
        switch(result)
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
                cout << RED"Too close, not a keyframe" << endl;
                break;
                // 只有在是关键帧的时候才有可能进行回环检测
            case KEYFRAME:
                cout << GREEN"This is a new keyframe" << endl;
                // 不远不近，检测回环
                checkNearbyLoops(camera, keyframes, currFrame, globalOptimizer);
                checkRandomLoops(camera, keyframes, currFrame, globalOptimizer);

                // 加入关键帧
                keyframes.push_back(currFrame);

                // local map
                globalOptimizer.initializeOptimization(); 
                globalOptimizer.optimize(10);

                for (; generateIndex < keyframes.size(); generateIndex++)
                {
                    // get one frame from optimizor
                    g2o::VertexSE3* vertex = dynamic_cast<g2o::VertexSE3*>(globalOptimizer.vertex(keyframes[generateIndex].frameID));
                    Eigen::Isometry3d pose = vertex->estimate(); 
                    PointCloud::Ptr newCloud = image2PointCloud(keyframes[generateIndex].rgb, keyframes[generateIndex].depth, camera); //转成点云

                    // put into global point cloud
                    pcl::transformPointCloud(*newCloud, *newCloud, pose.matrix());
                    *globalMap += *newCloud;
                    newCloud->clear();
                    cout << pose.matrix() << endl;
                    cout << "generate one point cloud..." << endl;
                }

                view->removePointCloud("cloud");
                view->addPointCloud( globalMap, "cloud");
                view->spinOnce();
                break;
        }
        index++;

        char key = cv::waitKey(10);
        if(key == 'q')
            break;
        if(key == 'p')
            cv::waitKey();
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
    pcl::VoxelGrid<PointT> grid;
    grid.setLeafSize(downSampleSize, downSampleSize, downSampleSize);
    grid.setInputCloud(output);
    grid.filter(*output);
    pcl::io::savePCDFile("./result_after_downsample.pcd", *output);
    // convert .pcd to .ply to be visualized
    string input_filename("./result.pcd");
    string output_filename("./result.ply");
    PCDtoPLYconvertor(input_filename, output_filename);
    cout << "Finish original point cloud saving..." << endl;

    input_filename = "./result_after_downsample.pcd";
    output_filename = "./result_after_downsample.ply";
    PCDtoPLYconvertor(input_filename, output_filename);

    cout << "Finish down sample point cloud saving..." << endl;
    return EXIT_SUCCESS;
}

