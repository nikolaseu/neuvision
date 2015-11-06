#ifndef Z3D_ZPOINTCLOUD___TYPEDEFS_H
#define Z3D_ZPOINTCLOUD___TYPEDEFS_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


// Define "PointCloud" to be a pcl::PointCloud of pcl::PointXYZRGB points
typedef pcl::PointXYZI PointType; //typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> PointCloudPCL;
typedef pcl::PointCloud<PointType>::Ptr PointCloudPCLPtr;
typedef pcl::PointCloud<PointType>::ConstPtr PointCloudConstPtr;

// Define "SurfaceNormals" to be a pcl::PointCloud of pcl::Normal points
typedef pcl::Normal NormalType;
typedef pcl::PointCloud<NormalType> SurfaceNormals;
typedef pcl::PointCloud<NormalType>::Ptr SurfaceNormalsPtr;
typedef pcl::PointCloud<NormalType>::ConstPtr SurfaceNormalsConstPtr;

// Define "SurfaceElements" to be a pcl::PointCloud of pcl::PointNormal points
typedef pcl::PointXYZINormal SurfelType; //typedef pcl::PointXYZRGBNormal SurfelType;
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

#endif // Z3D_ZPOINTCLOUD___TYPEDEFS_H
