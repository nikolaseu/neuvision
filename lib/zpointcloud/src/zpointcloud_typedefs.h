/* * Z3D - A structured light 3D scanner
 * Copyright (C) 2013-2016 Nicolas Ulrich <nikolaseu@gmail.com>
 *
 * This file is part of Z3D.
 *
 * Z3D is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Z3D is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Z3D.  If not, see <http://www.gnu.org/licenses/>.
 */

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
