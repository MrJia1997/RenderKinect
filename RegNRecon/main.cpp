#pragma warning(disable: 4996)

#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <fstream>
// Assimp
#include <assimp/cimport.h>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter.h>
#include <pcl/point_representation.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;
// Eigen
#include <Eigen/Dense>
#include <Eigen/Core>
// Boost
#include <boost/algorithm/string.hpp>
#include <boost/make_shared.hpp>
// Go-ICP
#include "jly_goicp.h"
#include "ConfigMap.hpp"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> pclPointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

double maxCorr = 0.2;
int maxIter = 5, iterTimes = 5;
int modelPair = 0;

void insertBeeModels(std::vector<std::string> &fileNames) {
    fileNames.clear();
    for (int i = 1; i <= 8; i++) {
        std::stringstream s;
        s << "bee" << i << ".obj";
        fileNames.push_back(s.str());
    }
}

void insertBunnyModels(std::vector<std::string> &fileNames) {
    fileNames.clear();
    for (int i = 1; i <= 11; i++) {
        std::stringstream s;
        s << "point_cloudbunny_34k.obj_" << std::setw(3) << std::setfill('0') << i << ".pcd"; 
        fileNames.push_back(s.str());
    }
}

void insertDiceModels(std::vector<std::string> &fileNames, std::vector<Eigen::Matrix4f> &transformations) {
    fileNames.clear();
    for (int i = 1; i <= 20; i++) {
        std::stringstream s;
        s << "point_cloud" << std::setw(3) << std::setfill('0') << i << ".pcd";
        fileNames.push_back(s.str());
    }
    ifstream trans;
    trans.open("transLog.txt", ios::in);
    string s;
    for (int i = 0; i < 20; i++) {
        Eigen::Matrix4f tempTrans;
        for (int i = 0; i < 3; i++) {
            trans >> s;
        }
        for (int c = 0; c < 4; c++) {
            for (int r = 0; r < 4; r++) {
                trans >> tempTrans(c, r);
            }
        }
        transformations.push_back(tempTrans);
    }
    trans.close();

    /*for (int i = 0; i < 20; i++){
        cout << "Transform Matrix " << i << ":" << endl;
        cout << transformations[i].matrix() << endl;
    }*/
}

void importPointsFromPCD(std::vector<pclPointCloud::Ptr> &models, const std::string filePath) {
    pclPointCloud::Ptr m(new pclPointCloud);
    pcl::io::loadPCDFile(filePath, *m);
    std::cout << "Read " << m->size() << " vertices" << std::endl;
    models.push_back(m);
    return;
}

void importPointsFromOBJ(std::vector<pclPointCloud::Ptr> &models, const std::string filePath) {
    const struct aiScene* scene = aiImportFile(filePath.c_str(), aiProcessPreset_TargetRealtime_Quality);
    if (scene == NULL) {
        std::cerr << "Could not load mesh from file " << filePath << std::endl;
        exit(-1);
    }
    if (scene->mNumMeshes>1) {
        std::cerr << "Object " << filePath << " consists of more than one mesh." << std::endl;
        exit(-1);
    }
    unsigned numVertices = scene->mMeshes[0]->mNumVertices;
    std::cout << "Read " << numVertices << " vertices" << std::endl;

    // normalize
    Eigen::MatrixXd vertices(3, numVertices);
    for (size_t i = 0; i < numVertices; i++) {
        vertices(0, i) = scene->mMeshes[0]->mVertices[i].x;
        vertices(1, i) = scene->mMeshes[0]->mVertices[i].y;
        vertices(2, i) = scene->mMeshes[0]->mVertices[i].z;
    }
    Eigen::VectorXd maxima, minima, means;
    means = vertices.rowwise().mean();
    vertices.colwise() -= means;
    maxima = vertices.rowwise().maxCoeff();
    minima = vertices.rowwise().minCoeff();
    vertices /= (maxima - minima).maxCoeff();


    // assign value
    pclPointCloud::Ptr m(new pclPointCloud);
    m->width = numVertices;
    m->height = 1;
    m->is_dense = false;
    m->points.resize(m->width * m->height);

    for (size_t i = 0; i < numVertices; i++) {
        m->points[i].x = vertices(0, i);
        m->points[i].y = vertices(1, i);
        m->points[i].z = vertices(2, i);
    }

    models.push_back(m);

    return;
}

void readBunnyInfo(
    std::vector<int> &ids,
    std::vector<Eigen::Matrix4f> &tfs,
    std::vector<std::pair<int, int>> &seq) {
    
    ifstream info;
    info.open("result_bunny_34k.obj.txt", ios::in);
    int size;
    string str;
    info >> size >> str;
    for (int i = 0; i < size; i++) {
        Eigen::Matrix4f tf;
        int id1, id2;
        info >> id1 >> id2 >> str;
        ids.push_back(id1);
        for (int c = 0; c < 4; c++) {
            for (int r = 0; r < 4; r++) {
                info >> tf(c, r);
            }
        }
        tfs.push_back(tf);
        // first scan
        if (id2 == -1)
            continue;
        // following scans
        int cur_id_size = ids.size();
        for (int j = 0; j < cur_id_size; j++) {
            if (ids[j] == id1)
                id1 = j;
            if (ids[j] == id2)
                id2 = j;
        }
        seq.push_back(std::make_pair(id2, id1));
    }

    info.close();
    cout << "Read info finished" << endl;
}

// Align target to source
void pairAlign(const pclPointCloud::Ptr cloud_src, const pclPointCloud::Ptr cloud_tgt,
    pclPointCloud::Ptr output, Eigen::Matrix4f &final_transform)
{
    pclPointCloud::Ptr src(new pcl::PointCloud<pcl::PointXYZ>);
    pclPointCloud::Ptr tgt(new pcl::PointCloud<pcl::PointXYZ>);
    src = cloud_src;
    tgt = cloud_tgt;

    // Compute surface normals and curvature
    PointCloudWithNormals::Ptr points_with_normals_src(new PointCloudWithNormals);
    PointCloudWithNormals::Ptr points_with_normals_tgt(new PointCloudWithNormals);

    pcl::NormalEstimation<PointT, PointNormalT> norm_est;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    norm_est.setSearchMethod(tree);
    norm_est.setKSearch(30);

    norm_est.setInputCloud(src);
    norm_est.compute(*points_with_normals_src);
    pcl::copyPointCloud(*src, *points_with_normals_src);

    norm_est.setInputCloud(tgt);
    norm_est.compute(*points_with_normals_tgt);
    pcl::copyPointCloud(*tgt, *points_with_normals_tgt);

    // Align
    pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
    reg.setTransformationEpsilon(1e-6);
    // Set the maximum distance between two correspondences (src<->tgt) to 10cm
    // Note: adjust this based on the size of your datasets
    reg.setMaxCorrespondenceDistance(maxCorr);
    // Set the point representation
    reg.setInputSource(points_with_normals_src);
    reg.setInputTarget(points_with_normals_tgt);

    // Run the same optimization in a loop and visualize the results
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
    PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
    reg.setMaximumIterations(maxIter);
    for (int i = 0; i < iterTimes; ++i)
    {
        // save cloud for visualization purpose
        points_with_normals_src = reg_result;

        // Estimate
        reg.setInputSource(points_with_normals_src);
        reg.align(*reg_result);

        //accumulate transformation between each Iteration
        Ti = reg.getFinalTransformation() * Ti;

        //if the difference between this transformation and the previous one
        //is smaller than the threshold, refine the process by reducing
        //the maximal correspondence distance
        if (fabs((reg.getLastIncrementalTransformation() - prev).sum()) < reg.getTransformationEpsilon())
            reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance() - 0.001);

        prev = reg.getLastIncrementalTransformation();
    }

    // Get the transformation from target to source
    targetToSource = Ti.inverse();

    //std::cout << targetToSource << std::endl;
    //
    // Transform target back in source frame
    pcl::transformPointCloud(*cloud_tgt, *output, targetToSource);

    //add the source to the transformed target
    *output += *cloud_src;

    final_transform = targetToSource;
}

void configGoICP(std::string configName, GoICP & goicp) {
    // Open and parse the associated config file
    ConfigMap config(configName.c_str());

    goicp.MSEThresh = config.getF("MSEThresh");
    goicp.initNodeRot.a = config.getF("rotMinX");
    goicp.initNodeRot.b = config.getF("rotMinY");
    goicp.initNodeRot.c = config.getF("rotMinZ");
    goicp.initNodeRot.w = config.getF("rotWidth");
    goicp.initNodeTrans.x = config.getF("transMinX");
    goicp.initNodeTrans.y = config.getF("transMinY");
    goicp.initNodeTrans.z = config.getF("transMinZ");
    goicp.initNodeTrans.w = config.getF("transWidth");
    goicp.trimFraction = config.getF("trimFraction");
    // If < 0.1% trimming specified, do no trimming
    if (goicp.trimFraction < 0.001)
    {
        goicp.doTrim = false;
    }
    goicp.dt.SIZE = config.getI("distTransSize");
    goicp.dt.expandFactor = config.getF("distTransExpandFactor");

    std::cout << "CONFIG:" << std::endl;
    config.print();
    //cout << "(doTrim)->(" << goicp.doTrim << ")" << endl;
    std::cout << std::endl;
}

void registrationGoICP(std::vector<pclPointCloud::Ptr> &models, std::string &configName, int dataDownsampled) {
    clock_t st, ed;
    GoICP goicp;
    int NdDownsampled = dataDownsampled;

    configGoICP(configName, goicp);

    POINT3D *pM, *pD;
    pclPointCloud::Ptr m = models[modelPair], d = models[modelPair + 1];
    int nM = m->size(), nD = d->size();
    pM = new POINT3D[nM];
    pD = new POINT3D[nD];
    for (int i = 0; i < nM; i++) {
        pM[i].x = m->points[i].x;
        pM[i].y = m->points[i].y;
        pM[i].z = m->points[i].z;
    }
    for (int i = 0; i < nD; i++) {
        pD[i].x = d->points[i].x;
        pD[i].y = d->points[i].y;
        pD[i].z = d->points[i].z;
    }

    // randomize pD for further subsampling
    auto swapPoint = [](POINT3D &a, POINT3D &b) {
        POINT3D temp = a;
        a = b;
        b = temp;
    };
    for (int i = nD; i > 0; --i)
        swapPoint(pD[i - 1], pD[rand() % i]);



    goicp.pModel = pM;
    goicp.Nm = nM;
    goicp.pData = pD;
    goicp.Nd = nD;

    // Build Distance Transform
    cout << "Building Distance Transform..." << flush;
    st = clock();
    goicp.BuildDT();
    ed = clock();
    std::cout << (double)(ed - st) / CLOCKS_PER_SEC << "s (CPU)" << std::endl;

    // Run GO-ICP
    if (NdDownsampled > 0)
    {
        goicp.Nd = NdDownsampled; // Only use first NdDownsampled data points (assumes data points are randomly ordered)
    }
    std::cout << "Model ID: " << modelPair << " (" << goicp.Nm << "), Data ID: " << modelPair + 1 << " (" << goicp.Nd << ")" << std::endl;
    std::cout << "Registering..." << std::endl;
    st = clock();
    goicp.Register();
    ed = clock();
    double time = (double)(ed - st) / CLOCKS_PER_SEC;
    
    Eigen::Matrix4f tf = Eigen::Matrix4f::Identity();
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            tf(i, j) = goicp.optR.val[i][j];
        }
        tf(i, 3) = goicp.optT.val[i][0]; 
    }
    tf.row(3) = Eigen::Vector4f(0, 0, 0, 1).transpose();
    
    /*std::cout << "Optimal Rotation Matrix:" << std::endl;
    std::cout << goicp.optR << std::endl;
    std::cout << "Optimal Translation Vector:" << endl;
    std::cout << goicp.optT << std::endl;*/

    std::cout << "Optimal Affine Tranformation Matrix:" << endl;
    std::cout << tf << std::endl;
    std::cout << "Finished in " << time << "s (CPU)" << std::endl;

    delete [] pM, pD;
    
    pclPointCloud::Ptr result(new pclPointCloud);

    pcl::transformPointCloud(*d, *result, tf);
    *result += *m;

    std::cout << "Result size: " << result->size() << std::endl;
    std::stringstream ss;
    ss << "../tmp/" << modelPair << ".pcd";
    pcl::io::savePCDFile(ss.str(), *result, true);

    pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
    pclPointCloud::Ptr p_cloud(new pclPointCloud);
    pcl::io::loadPCDFile(ss.str(), *p_cloud);
    viewer.showCloud(p_cloud);

    while (!viewer.wasStopped()) {}

}

void registration(std::vector<pclPointCloud::Ptr> &models, 
    std::vector<Eigen::Matrix4f> &transformations,
    std::vector<std::pair<int, int>> &reg_seq) {
    
    int len = models.size();
    std::vector<Eigen::Matrix4f> global_transform(models.size(), Eigen::Matrix4f::Identity());
    
    pclPointCloud::Ptr result(new pclPointCloud);
    
    // add first scan
    *result += *models[0];
    
    for (auto pair : reg_seq) {
        int src_id = pair.first, tgt_id = pair.second;
       
        // initial alignment
        Eigen::Matrix4f init_align = transformations[src_id] * transformations[tgt_id].inverse();
        pclPointCloud::Ptr temp_tgt(new pclPointCloud), reg_res(new pclPointCloud);
        pcl::transformPointCloud(*models[tgt_id], *temp_tgt, init_align);

        // registration
        Eigen::Matrix4f pair_transform = Eigen::Matrix4f::Identity();
        pairAlign(models[src_id], temp_tgt, reg_res, pair_transform);
        //pcl::transformPointCloud(*temp_tgt, *reg_res, pair_transform);
        
        if (src_id == 0) {
            global_transform[tgt_id] = pair_transform * init_align;
            *result += *reg_res;
        }
        else {
            pclPointCloud::Ptr source_res(new pclPointCloud);
            pcl::transformPointCloud(*reg_res, *source_res, global_transform[src_id]);
            global_transform[tgt_id] = global_transform[src_id] * pair_transform * init_align;
            *result += *source_res;
        }
        
        cout << "Register " << src_id << " and " << tgt_id << std::endl;
        cout << pair_transform.matrix() << std::endl;
    }

    // restore first pose
    pclPointCloud::Ptr result_restored(new pclPointCloud);
    pcl::transformPointCloud(*result, *result_restored, transformations[0].inverse());

    // downsampling
    pclPointCloud::Ptr result_downsampled(new pclPointCloud);
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(result_restored);
    sor.setLeafSize(0.005f, 0.005f, 0.005f);
    sor.filter(*result_downsampled);

    pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
    viewer.showCloud(result_downsampled);
    while (!viewer.wasStopped()) {}

}

void reconstruction() {}


int main(int argc, char **argv) {
    
    std::vector<pclPointCloud::Ptr> models;
    std::string filePath = "../obj_models/multi/";
    std::vector<std::string> fileNames;
    
    int dataDownsampled = 2000;
    std::string configName = "config.txt";

    //cout << "Config file name: ";
    //cin >> configName;
    /*cout << "Data downsampled: ";
    cin >> dataDownsampled;*/
    /*cout << "Model pair: ";
    cin >> modelPair;*/

    //insertBeeModels(fileNames);
    insertBunnyModels(fileNames);
    //insertDiceModels(fileNames, transformations);

    
    // import models
    for (auto s : fileNames) {
        if (boost::algorithm::ends_with(s, ".obj")) {
            importPointsFromOBJ(models, filePath + s);
        }
        else if (boost::algorithm::ends_with(s, ".pcd")) {
            importPointsFromPCD(models, filePath + s);
        }
        else {
            std::cerr << "Cannot open file, please check the file name and path." << std::endl;
            exit(-1);
        }
    }
    
    // read scan information
    std::vector<Eigen::Matrix4f> transformations;
    std::vector<int> scan_indices;
    std::vector<std::pair<int, int>> reg_sequence;
    readBunnyInfo(scan_indices, transformations, reg_sequence);

    // registration
    registration(models, transformations, reg_sequence);
    //registrationGoICP(models, configName, dataDownsampled);

    // reconstruction
    reconstruction();

    return 0;
}