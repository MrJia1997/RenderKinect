#pragma warning(disable: 4996)

#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
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
// Eigen
#include <Eigen/Dense>
#include <Eigen/Core>
// Boost
#include <boost/algorithm/string.hpp>
#include <boost/make_shared.hpp>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

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
    for (int i = 1; i <= 10; i++) {
        std::stringstream s;
        s << "bunny" << std::setw(2) << std::setfill('0') << i << ".obj";
        fileNames.push_back(s.str());
    }
}

void importPointsFromPCD(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &models, const std::string filePath) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr m(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(filePath, *m);
    models.push_back(m);
    return;
}

void importPointsFromOBJ(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &models, const std::string filePath) {
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
    maxima = vertices.rowwise().maxCoeff();
    minima = vertices.rowwise().minCoeff();
    means = vertices.rowwise().mean();
    vertices.colwise() -= means;
    vertices /= (maxima - minima).maxCoeff();


    // assign value
    pcl::PointCloud<pcl::PointXYZ>::Ptr m(new pcl::PointCloud<pcl::PointXYZ>);
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


void pairAlign(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, 
    PointCloud::Ptr output, Eigen::Matrix4f &final_transform)
{
    PointCloud::Ptr src(new pcl::PointCloud<pcl::PointXYZ>);
    PointCloud::Ptr tgt(new pcl::PointCloud<pcl::PointXYZ>);

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
    //reg.setMaxCorrespondenceDistance(0.1);
    // Set the point representation
    reg.setInputSource(points_with_normals_src);
    reg.setInputTarget(points_with_normals_tgt);

    // Run the same optimization in a loop and visualize the results
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
    PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
    reg.setMaximumIterations(2);
    for (int i = 0; i < 30; ++i)
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

    //
    // Transform target back in source frame
    pcl::transformPointCloud(*cloud_tgt, *output, targetToSource);

    //add the source to the transformed target
    *output += *cloud_src;

    final_transform = targetToSource;
}

void registration(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &models) {
    int len = models.size();
    Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity(), pairTransform;
    PointCloud::Ptr result(new PointCloud);
    for (int i = 0; i < len - 1; i++) {
        PointCloud::Ptr temp(new PointCloud);
        pairAlign(models[i], models[i + 1], temp, pairTransform);

        //transform current pair into the global transform
        pcl::transformPointCloud(*temp, *result, GlobalTransform);

        //update the global transform
        GlobalTransform = GlobalTransform * pairTransform;
        

        std::cout << "Result size: " << result->size() << std::endl;
        std::stringstream ss;
        ss << "../tmp/" << i << ".pcd";
        pcl::io::savePCDFile(ss.str(), *result, true);

        pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
        pcl::PointCloud<pcl::PointXYZ>::Ptr p_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile(ss.str(), *p_cloud);
        viewer.showCloud(p_cloud);

        while (!viewer.wasStopped()) {}
    }

}

void reconstruction() {}


int main(int argc, char **argv) {
    
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> models;
    std::string filePath = "../obj_models/multi/";
    std::vector<std::string> fileNames;
    
    //fileNames.push_back("point_cloud001.pcd");
    //fileNames.push_back("point_cloud002.pcd");
    insertBeeModels(fileNames);
    //insertBunnyModels(fileNames);
    
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
    
    // registration
    registration(models);

    // reconstruction
    reconstruction();

    return 0;
}