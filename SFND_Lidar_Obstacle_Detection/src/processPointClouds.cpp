// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"

#include <unordered_set>

// #include "kdtree.h"
//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
  typename pcl::PointCloud<PointT>::Ptr cloud_region (new pcl::PointCloud<PointT>);
    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
	pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (filterRes, filterRes, filterRes);
  sor.filter (*cloud_filtered);
  
  pcl::CropBox<PointT> region(true);
  region.setMin(minPoint);
  region.setMax(maxPoint);
  region.setInputCloud(cloud_filtered);
  region.filter(*cloud_region);
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_region;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
	pcl::ExtractIndices<PointT> extract;
  typename pcl::PointCloud<PointT>::Ptr obstacles (new pcl::PointCloud<PointT> ()), segmentedPlane (new pcl::PointCloud<PointT> ());
  for (int index: inliers->indices)
  {
    segmentedPlane->points.push_back(cloud->points[index]);
    }
  extract.setInputCloud (cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter (*obstacles);
  
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacles, segmentedPlane);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
  pcl::SACSegmentation<PointT> seg;
  pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};
    // TODO:: Fill in this function to find inliers for the cloud.
  
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(maxIterations);
  seg.setDistanceThreshold(distanceThreshold);
  
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);
  
  if (inliers -> indices.size() == 0)
  {
    std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane2(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
// 	float a = (y1-y2);
//   	float b = (x2-x1);
//   	float c = (x1*y2-x2*y1);
	// For max iterations 
while(maxIterations--)
{
  std::unordered_set<int> inliers;
  while(inliers.size()<3){
    inliers.insert(rand()%(cloud->points.size()));
}
  float x1,y1,z1,x2,y2,z2,x3,y3,z3;
  auto itr = inliers.begin();
  x1 = cloud->points[*itr].x;
  y1 = cloud->points[*itr].y;
  z1 = cloud->points[*itr].z;
  int idx1 = *itr;
  itr++;
  x2 = cloud->points[*itr].x;
  y2 = cloud->points[*itr].y;
  z2 = cloud->points[*itr].z;
  int idx2 = *itr;
  itr++;
  x3 = cloud->points[*itr].x;
  y3 = cloud->points[*itr].y;
  z3 = cloud->points[*itr].z;
  int idx3 = *itr;
  
  float a = (((y2-y1)*(z3-z1))-((z2-z1)*(y3-y1)));
  float b = (((z2-z1)*(x3-x1))-((x2-x1)*(z3-z1)));
  float c = (((x2-x1)*(y3-y1))-((y2-y1)*(x3-x1)));
  float d = -(a*x1+b*y1+c*z1);
  float plane_length = sqrt(a*a+b*b+c*c);
  for(int index=0;index < cloud->points.size();index++)
  {
    if(inliers.count(index)>0)
    {
      continue;
    }
//     pcl::PointXYZI point = cloud->points[index];
    float x4 = cloud->points[index].x;
    float y4 = cloud->points[index].y;
    float z4 = cloud->points[index].z;
	// Randomly sample subset and fit line
    if(index!=idx1||index!=idx2||index!=idx3)
	{
	float distance = fabs(a*x4+b*y4+c*z4+d)/plane_length;
    
    if(distance<= distanceThreshold){
      inliers.insert(index);
    }
    }
  }
    if(inliers.size()>inliersResult.size())
    {
      inliersResult.clear();
      inliersResult = inliers;
    }
  }
  if (inliersResult.size () == 0)
	{
	  std::cerr << "No Inliers found." << std::endl;
	}
  typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliersResult.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}
	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudInliers, cloudOutliers);
	return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
  typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  tree->setInputCloud (cloud);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(clusterTolerance);
  ec.setMinClusterSize(minSize);
  ec.setMaxClusterSize(maxSize);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);
  
  for (pcl::PointIndices getIndices: cluster_indices)
  {
    typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
    for (int idx : getIndices.indices)
      cloud_cluster->points.push_back (cloud->points[idx]); 
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    clusters.push_back(cloud_cluster);
  }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}

// template<typename PointT>
// std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::RANSAC_SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
// {
// 	std::unordered_set<int> inliersResult;
// 	srand(time(NULL));
	
// 	// TODO: Fill in this function
// // 	float a = (y1-y2);
// //   	float b = (x2-x1);
// //   	float c = (x1*y2-x2*y1);
// 	// For max iterations 
// while(maxIterations--)
// {
//   std::unordered_set<int> inliers;
//   while(inliers.size()<3){
//     inliers.insert(rand()%(cloud->points.size()));
// }
//   float x1,y1,z1,x2,y2,z2,x3,y3,z3;
//   auto itr = inliers.begin();
//   x1 = cloud->points[*itr].x;
//   y1 = cloud->points[*itr].y;
//   z1 = cloud->points[*itr].z;
//   itr++;
//   x2 = cloud->points[*itr].x;
//   y2 = cloud->points[*itr].y;
//   z2 = cloud->points[*itr].z;
//   itr++;
//   x3 = cloud->points[*itr].x;
//   y3 = cloud->points[*itr].y;
//   z3 = cloud->points[*itr].z;
  
//   float a = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
//   float b = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
//   float c = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);
//   float d = -(a*x1+b*y1+c*z1);
//   float plane_length = std::max(0.1, sqrt(a*a+b*b+c*c));
//   for(int index=0;index < cloud->points.size();index++)
//   {
//     if(inliers.count(index)>0)
//     {
//       continue;
//     }
// //     pcl::PointXYZI point = cloud->points[index];
//     float x4 = cloud->points[index].x;
//     float y4 = cloud->points[index].y;
//     float z4 = cloud->points[index].z;
// 	// Randomly sample subset and fit line
// 	float distance = fabs(a*x4+b*y4+c*z4+d)/plane_length;
    
//     if(distance<= distanceTol){
//       inliers.insert(index);
//     }
//   }
//     if(inliers.size()>inliersResult.size())
//     {
//       inliersResult.clear();
//       inliersResult = inliers;
//     }
//   }
//   pcl::PointCloud<pcl::PointXYZI>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZI>());
// 	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZI>());

// 	for(int index = 0; index < filterCloud->points.size(); index++)
// 	{
// 		pcl::PointXYZI point = filterCloud->points[index];
// 		if(inliers.count(index))
// 			cloudInliers->points.push_back(point);
// 		else
// 			cloudOutliers->points.push_back(point);
// 	}
// 	// Measure distance between every point and fitted line
// 	// If distance is smaller than threshold count it as inlier

// 	// Return indicies of inliers from fitted line with most inliers
// 	td::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudInliers, cloudOutliers)
// 	return segResult;

// }

template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int indices, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int> &cluster, std::vector<bool> &processed,typename KdTree<PointT>::KdTree* tree, float distanceTol, int maxSize) 
{
  if ((processed[indices] == false) &&
			(cluster.size()<maxSize))
  processed[indices] = true;
  cluster.push_back(indices);
  std::vector<int> nearest = tree->search(cloud->points[indices],distanceTol);
//   int j = 0;
  for(int id : nearest)
  {
    if (!processed[id])
    {
      clusterHelper(id, cloud, cluster, processed, tree, distanceTol, maxSize);
    }
  }
}
template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud,typename KdTree<PointT>::KdTree* tree,float distanceTol,int minSize,int maxSize)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;
 	std::vector<bool> processed(cloud->points.size(),false);
  
  	int i = 0;
  	while(i < cloud->points.size()){
      if(processed[i])
      {
        i++;
        continue;
      }
      std::vector<int> cluster;
      clusterHelper(i, cloud, cluster,processed,tree, distanceTol, maxSize);
      if((cluster.size()>=minSize)&&cluster.size()<=maxSize)
        clusters.push_back(cluster);
      i++;
    }
	return clusters;

}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering2(
	typename pcl::PointCloud<PointT>::Ptr cloud,
	float clusterTolerance,
	int minSize,
	int maxSize)
{
    auto startTime = std::chrono::steady_clock::now();
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    typename KdTree<PointT>::KdTree *tree =new KdTree<PointT>;
    tree->insert(cloud);
	std::vector<std::vector<int>> cluster_indices = euclideanCluster(cloud, tree,clusterTolerance,minSize,maxSize);

	for (std::vector<std::vector<int>>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	  {
		typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
	    for (std::vector<int>::const_iterator pit = it->begin (); pit != it->end (); ++pit)
	      cloud_cluster->points.push_back (cloud->points[*pit]); 
	    cloud_cluster->width = cloud_cluster->points.size ();
	    cloud_cluster->height = 1;
	    cloud_cluster->is_dense = true;

	    clusters.push_back(cloud_cluster);
	  }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "euclidean_Clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}
