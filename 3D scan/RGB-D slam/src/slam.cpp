#include "slamBase.h"


int main(int argc, char** argv)
{
    ParameterReader pd;
    int startIndex = atoi(pd.getData("start_index").c_str());
    int endIndex   = atoi(pd.getData("end_index").c_str());

    // save all key frames
    vector<FRAME> keyframes; 
    
    // initialize and reading all parameters
    cout << "Initializing ..." << endl;
    int currIndex = startIndex; // 当前索引为currIndex
    FRAME currFrame = readFrame(currIndex, pd); // 上一帧数据
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
    v->setId(currIndex);
    // the first node should be identity and fixed
    v->setEstimate(Eigen::Isometry3d::Identity()); 
    v->setFixed(true);  
    globalOptimizer.addVertex(v);
    
    // add first frame to key frame
    keyframes.push_back(currFrame);
    double keyframe_threshold = atof(pd.getData("keyframe_threshold").c_str());

    // start to do loop closure
    for(currIndex = startIndex+1;currIndex < endIndex;currIndex++)
    {
        cout << RESET"Reading files " << currIndex << endl;
        FRAME currFrame = readFrame(currIndex, pd);         

        computeKeyPointsAndDesp(currFrame, detector, descriptor); 

        CHECK_RESULT result = checkKeyframes(camera, keyframes.back(), currFrame, globalOptimizer); //匹配该帧与keyframes里最后一帧
        switch (result) // 根据匹配结果不同采取不同策略
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
                break;
        }
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
    return 0;
}

