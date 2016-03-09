#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#define CLOUD_INTENSITY_ONLY

// Define "PointCloud" to be a pcl::PointCloud of pcl::PointXYZRGB points
#if defined(CLOUD_INTENSITY_ONLY)
typedef pcl::PointXYZI PointType;
#else
typedef pcl::PointXYZRGB PointType;
#endif
typedef pcl::PointCloud<PointType> PointCloudPCL;
typedef pcl::PointCloud<PointType>::Ptr PointCloudPCLPtr;
typedef pcl::PointCloud<PointType>::ConstPtr PointCloudConstPtr;

// Define "SurfaceNormals" to be a pcl::PointCloud of pcl::Normal points
typedef pcl::Normal NormalType;
typedef pcl::PointCloud<NormalType> SurfaceNormals;
typedef pcl::PointCloud<NormalType>::Ptr SurfaceNormalsPtr;
typedef pcl::PointCloud<NormalType>::ConstPtr SurfaceNormalsConstPtr;

// Define "SurfaceElements" to be a pcl::PointCloud of pcl::PointNormal points
#if defined(CLOUD_INTENSITY_ONLY)
typedef pcl::PointXYZINormal SurfelType;
#else
typedef pcl::PointXYZRGBNormal SurfelType;
#endif
typedef pcl::PointCloud<SurfelType> SurfaceElements;
typedef pcl::PointCloud<SurfelType>::Ptr SurfaceElementsPtr;
typedef pcl::PointCloud<SurfelType>::ConstPtr SurfaceElementsConstPtr;
/*
// Define "LocalDescriptors" to be a pcl::PointCloud of pcl::FPFHSignature33 points
typedef pcl::FPFHSignature33 LocalDescriptorT;
typedef pcl::PointCloud<LocalDescriptorT> LocalDescriptors;
typedef pcl::PointCloud<LocalDescriptorT>::Ptr LocalDescriptorsPtr;
typedef pcl::PointCloud<LocalDescriptorT>::ConstPtr LocalDescriptorsConstPtr;

// Define "GlobalDescriptors" to be a pcl::PointCloud of pcl::VFHSignature308 points
typedef pcl::VFHSignature308 GlobalDescriptorT;
typedef pcl::PointCloud<GlobalDescriptorT> GlobalDescriptors;
typedef pcl::PointCloud<GlobalDescriptorT>::Ptr GlobalDescriptorsPtr;
typedef pcl::PointCloud<GlobalDescriptorT>::ConstPtr GlobalDescriptorsConstPtr;
*/
