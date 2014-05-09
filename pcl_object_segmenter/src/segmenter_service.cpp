/*
 * shape_simple_classifier_node.cpp
 *
 *  Created on: Sep 7, 2013
 *      Author: aitor
 */

#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl/common/common.h>
#include <pcl/console/parse.h>
#include <pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/lexical_cast.hpp>
#include "segmentation_srv_definitions/segment.h"
#include "std_msgs/Int32MultiArray.h"

#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/planar_polygon_fusion.h>
#include <pcl/segmentation/plane_coefficient_comparator.h>
#include <pcl/segmentation/euclidean_plane_coefficient_comparator.h>
#include <pcl/segmentation/rgb_plane_coefficient_comparator.h>
#include <pcl/segmentation/edge_aware_plane_comparator.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/apps/dominant_plane_segmentation.h>
#include <pcl/features/normal_3d_omp.h>
#include <faat_pcl/3d_rec_framework/segmentation/multiplane_segmentation.h>

//#define DEBUG_WITH_VIS 1
#ifdef DEBUG_WITH_VIS
    #include<pcl/visualization/pcl_visualizer.h>
#endif

#include<pcl/common/transforms.h>


class PCLSegmenterService
{
private:
  typedef pcl::PointXYZRGB PointT;
  double chop_at_z_;
  int v1_,v2_, v3_;
  ros::ServiceServer segment_;
  ros::NodeHandle *n_;
  int seg_type_;
  std::string camera_frame_;
  std::string base_frame_;
  float max_angle_plane_to_ground_;
  float table_range_min_, table_range_max_;
  int min_cluster_size_;
  int MAX_VERTICAL_PLANE_SIZE_;

  void
  getBaseCameraTransform (Eigen::Matrix4f & trans)
  {
    tf::StampedTransform tf_transform;
    tf::TransformListener tf_listener;

    std::cout << "Getting transform from:" << camera_frame_ << " to " << base_frame_ << std::endl;

    try
    {
      tf_listener.waitForTransform (base_frame_, camera_frame_, ros::Time (0), ros::Duration (5.0));
      tf_listener.lookupTransform (base_frame_, camera_frame_, ros::Time (0), tf_transform);
    }
    catch (tf::TransformException ex)
    {
      std::cout << ex.what () << std::endl;
    }

    //cast tf_transform to eigen::Matrix4f
    tf::Vector3 p = tf_transform.getOrigin ();
    tf::Quaternion q = tf_transform.getRotation ();

    Eigen::Vector3f translation = Eigen::Vector3f (p.getX (), p.getY (), p.getZ ());

    Eigen::Quaternion<float> rot (q.getW (), q.getX (), q.getY (), q.getZ ());
    rot.normalize();
    Eigen::Matrix3f rotation = rot.toRotationMatrix();

    trans.setIdentity();
    trans.block<3,1>(0,3) = translation;
    trans.block<3,3>(0,0) = rotation;
  }

  template<typename PointT>
    void
    doSegmentation (typename pcl::PointCloud<PointT>::Ptr & xyz_points, pcl::PointCloud<pcl::Normal>::Ptr & normal_cloud,
                    std::vector<pcl::PointIndices> & indices, Eigen::Vector4f & table_plane, int seg = 0)
    {


#ifdef DEBUG_WITH_VIS
      std::cout << xyz_points->points.size() << std::endl;
      pcl::visualization::PCLVisualizer vis("cloud for segmentation");
      int v1,v2;
      vis.createViewPort(0,0,0.5,1,v1);
      vis.createViewPort(0.5,0,1,1,v2);
      vis.addPointCloud<PointT>(xyz_points, "cloud", v1);
      vis.addCoordinateSystem(0.3, v1);
      vis.addCoordinateSystem(0.3, v2);
      vis.spin();
#endif

      if (seg == 0)
      {
        int num_plane_inliers = 1000;

        pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;
        mps.setMinInliers (num_plane_inliers);
        mps.setAngularThreshold (0.017453 * 10.f); // 2 degrees
        mps.setDistanceThreshold (0.01); // 1cm
        mps.setInputNormals (normal_cloud);
        mps.setInputCloud (xyz_points);

        std::vector < pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
        std::vector < pcl::ModelCoefficients > model_coefficients;
        std::vector < pcl::PointIndices > inlier_indices;
        pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
        std::vector < pcl::PointIndices > label_indices;
        std::vector < pcl::PointIndices > boundary_indices;

        typename pcl::PlaneRefinementComparator<PointT, pcl::Normal, pcl::Label>::Ptr ref_comp (
                                                                                                new pcl::PlaneRefinementComparator<PointT, pcl::Normal,
                                                                                                    pcl::Label> ());
        ref_comp->setDistanceThreshold (0.01f, false);
        ref_comp->setAngularThreshold (0.017453 * 2);
        mps.setRefinementComparator (ref_comp);
        mps.segmentAndRefine (regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);

        std::cout << "Number of planes found:" << model_coefficients.size () << std::endl;
        if (model_coefficients.size () == 0)
          return;

        int table_plane_selected = 0;
        int max_inliers_found = -1;
        std::vector<int> plane_inliers_counts;
        plane_inliers_counts.resize (model_coefficients.size ());

        for (size_t i = 0; i < model_coefficients.size (); i++)
        {
          Eigen::Vector4f table_plane = Eigen::Vector4f (model_coefficients[i].values[0], model_coefficients[i].values[1],
                                                         model_coefficients[i].values[2], model_coefficients[i].values[3]);

          std::cout << "Number of inliers for this plane:" << inlier_indices[i].indices.size () << std::endl;
          int remaining_points = 0;
          typename pcl::PointCloud<PointT>::Ptr plane_points (new pcl::PointCloud<PointT> (*xyz_points));
          for (int j = 0; j < plane_points->points.size (); j++)
          {
            Eigen::Vector3f xyz_p = plane_points->points[j].getVector3fMap ();

            if (!pcl_isfinite (xyz_p[0]) || !pcl_isfinite (xyz_p[1]) || !pcl_isfinite (xyz_p[2]))
              continue;

            float val = xyz_p[0] * table_plane[0] + xyz_p[1] * table_plane[1] + xyz_p[2] * table_plane[2] + table_plane[3];

            if (std::abs (val) > 0.01)
            {
              plane_points->points[j].x = std::numeric_limits<float>::quiet_NaN ();
              plane_points->points[j].y = std::numeric_limits<float>::quiet_NaN ();
              plane_points->points[j].z = std::numeric_limits<float>::quiet_NaN ();
            }
            else
              remaining_points++;
          }

          plane_inliers_counts[i] = remaining_points;

          if (remaining_points > max_inliers_found)
          {
            table_plane_selected = i;
            max_inliers_found = remaining_points;
          }
        }

        size_t itt = static_cast<size_t> (table_plane_selected);
        table_plane = Eigen::Vector4f (model_coefficients[itt].values[0], model_coefficients[itt].values[1], model_coefficients[itt].values[2],
                                       model_coefficients[itt].values[3]);

        Eigen::Vector3f normal_table = Eigen::Vector3f (model_coefficients[itt].values[0], model_coefficients[itt].values[1],
                                                        model_coefficients[itt].values[2]);

        int inliers_count_best = plane_inliers_counts[itt];

        //check that the other planes with similar normal are not higher than the table_plane_selected
        for (size_t i = 0; i < model_coefficients.size (); i++)
        {
          Eigen::Vector4f model = Eigen::Vector4f (model_coefficients[i].values[0], model_coefficients[i].values[1], model_coefficients[i].values[2],
                                                   model_coefficients[i].values[3]);

          Eigen::Vector3f normal = Eigen::Vector3f (model_coefficients[i].values[0], model_coefficients[i].values[1], model_coefficients[i].values[2]);

          int inliers_count = plane_inliers_counts[i];

          std::cout << "Dot product is:" << normal.dot (normal_table) << std::endl;
          if ((normal.dot (normal_table) > 0.95) && (inliers_count_best * 0.5 <= inliers_count))
          {
            //check if this plane is higher, projecting a point on the normal direction
            std::cout << "Check if plane is higher, then change table plane" << std::endl;
            std::cout << model[3] << " " << table_plane[3] << std::endl;
            if (model[3] < table_plane[3])
            {
              PCL_WARN ("Changing table plane...");
              table_plane_selected = i;
              table_plane = model;
              normal_table = normal;
              inliers_count_best = inliers_count;
            }
          }
        }

        table_plane = Eigen::Vector4f (model_coefficients[table_plane_selected].values[0], model_coefficients[table_plane_selected].values[1],
                                       model_coefficients[table_plane_selected].values[2], model_coefficients[table_plane_selected].values[3]);

        //cluster..
        typename pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label>::Ptr
                                                                                       euclidean_cluster_comparator_ (
                                                                                                                      new pcl::EuclideanClusterComparator<
                                                                                                                          PointT, pcl::Normal,
                                                                                                                          pcl::Label> ());

        //create two labels, 1 one for points belonging to or under the plane, 1 for points above the plane
        label_indices.resize (2);

        for (int j = 0; j < xyz_points->points.size (); j++)
        {
          Eigen::Vector3f xyz_p = xyz_points->points[j].getVector3fMap ();

          if (!pcl_isfinite (xyz_p[0]) || !pcl_isfinite (xyz_p[1]) || !pcl_isfinite (xyz_p[2]))
            continue;

          float val = xyz_p[0] * table_plane[0] + xyz_p[1] * table_plane[1] + xyz_p[2] * table_plane[2] + table_plane[3];

          if (val >= 0.01f)
          {
            /*plane_points->points[j].x = std::numeric_limits<float>::quiet_NaN ();
             plane_points->points[j].y = std::numeric_limits<float>::quiet_NaN ();
             plane_points->points[j].z = std::numeric_limits<float>::quiet_NaN ();*/
            labels->points[j].label = 1;
            label_indices[0].indices.push_back (j);
          }
          else
          {
            labels->points[j].label = 0;
            label_indices[1].indices.push_back (j);
          }
        }

        std::vector<bool> plane_labels;
        plane_labels.resize (label_indices.size (), false);
        plane_labels[0] = true;

        euclidean_cluster_comparator_->setInputCloud (xyz_points);
        euclidean_cluster_comparator_->setLabels (labels);
        euclidean_cluster_comparator_->setExcludeLabels (plane_labels);
        euclidean_cluster_comparator_->setDistanceThreshold (0.035f, true);

        pcl::PointCloud < pcl::Label > euclidean_labels;
        std::vector < pcl::PointIndices > euclidean_label_indices;
        pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> euclidean_segmentation (euclidean_cluster_comparator_);
        euclidean_segmentation.setInputCloud (xyz_points);
        euclidean_segmentation.segment (euclidean_labels, euclidean_label_indices);

        for (size_t i = 0; i < euclidean_label_indices.size (); i++)
        {
          if (euclidean_label_indices[i].indices.size () >= min_cluster_size_)
          {
            indices.push_back (euclidean_label_indices[i]);
          }
        }
      }
      else if(seg == 1)
      {
        pcl::apps::DominantPlaneSegmentation<PointT> dps;
        dps.setInputCloud (xyz_points);
        dps.setMaxZBounds (3.F);
        dps.setObjectMinHeight (0.01);
        dps.setMinClusterSize (min_cluster_size_);
        dps.setWSize (9);
        dps.setDistanceBetweenClusters (0.03f);
        std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
        dps.setDownsamplingSize (0.01f);
        dps.compute_fast (clusters);
        dps.getIndicesClusters (indices);
        dps.getTableCoefficients (table_plane);
      }
      else if(seg == 4)
      {
          //use the robot pose to find horizontal planes
          std::cout << "is organized:" << xyz_points->isOrganized() << std::endl;

          pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);

          pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
          ne.setRadiusSearch(0.02f);
          ne.setInputCloud (xyz_points);
          ne.compute (*normal_cloud);

          faat_pcl::MultiPlaneSegmentation<PointT> mps;
          mps.setInputCloud(xyz_points);
          mps.setMinPlaneInliers(1000);
          mps.setResolution(0.003f);
          mps.setNormals(normal_cloud);
          mps.setMergePlanes(false);
          std::vector<faat_pcl::PlaneModel<PointT> > planes_found;
          mps.segment();
          planes_found = mps.getModels();

          /*if(planes_found.size() == 0 && xyz_points->isOrganized())
          //if(true || planes_found.size() == 0 && xyz_points->isOrganized())
          {
              PCL_WARN("No planes found, doing segmentation with standard method\n");
              mps.segment(true);
              planes_found = mps.getModels();
          }*/

          std::cout << "Number of planes:" << planes_found.size() << std::endl;

          Eigen::Matrix4f transform;
          getBaseCameraTransform(transform);

#ifdef DEBUG_WITH_VIS
          typename pcl::PointCloud<PointT>::Ptr cloud_trans(new pcl::PointCloud<PointT>);
          pcl::transformPointCloud(*xyz_points, *cloud_trans, transform);

          pcl::visualization::PointCloudColorHandlerCustom<PointT> handler(cloud_trans, 255, 0, 0);
          vis.addPointCloud(cloud_trans, handler, "cloud_transformed", v2);
          vis.spin();
#endif

          //select table plane based on the angle to the ground and the height
          faat_pcl::PlaneModel<PointT> selected_plane;
          float max_height = 0.f;
          bool good_plane_found = false;

          for(size_t i=0; i < planes_found.size(); i++)
          {
              faat_pcl::PlaneModel<PointT> plane;
              plane = planes_found[i];
              Eigen::Vector3f plane_normal = Eigen::Vector3f(plane.coefficients_.values[0],plane.coefficients_.values[1],plane.coefficients_.values[2]);
              Eigen::Matrix3f rotation = transform.block<3,3>(0,0);
              plane_normal = rotation * plane_normal;
              plane_normal.normalize();

              float angle = pcl::rad2deg(acos(plane_normal.dot(Eigen::Vector3f::UnitZ())));
              //std::cout << "angle:" << angle << std::endl;
              if(angle < max_angle_plane_to_ground_)
              {
                  //select a point on the plane and transform it to check the height relative to the ground
                  Eigen::Vector4f point = plane.plane_cloud_->points[0].getVector4fMap();
                  point[3] = 1;
                  point = transform * point;

                  float h = point[2];

                  if(h >= table_range_min_ && h <= table_range_max_)
                  {
                      std::cout << "Horizontal plane with appropiate table height " << h << std::endl;

                      if(h > max_height)
                      {
                          //select this plane as table
                          max_height = h;
                          selected_plane = plane;
                          good_plane_found = true;
                      }
                  }
              }
              else if(angle > 85 && angle < 95)
              {
                  //std::cout << "vertical plane, check if its big enough" << std::endl;
                  //std::cout << plane.plane_cloud_->points.size() << std::endl;

                  int size_plane = static_cast<int>(plane.plane_cloud_->points.size());
                  if(size_plane > MAX_VERTICAL_PLANE_SIZE_)
                  {
                      for(size_t k=0; k < plane.inliers_.indices.size(); k++)
                      {
                          xyz_points->points[plane.inliers_.indices[k]].x =
                          xyz_points->points[plane.inliers_.indices[k]].y =
                          xyz_points->points[plane.inliers_.indices[k]].z = std::numeric_limits<float>::quiet_NaN();
                      }

#ifdef DEBUG_WITH_VIS
                      std::stringstream p_name;
                      p_name << "plane" << i;
                      pcl::visualization::PointCloudColorHandlerRandom<PointT> handler(plane.plane_cloud_);
                      vis.addPointCloud<PointT>(plane.plane_cloud_, handler, p_name.str(), v1);
                      vis.spin();
#endif
                  }
              }

          }

          if(!good_plane_found)
          {
              //No table was found in the scene.
              PCL_WARN("No plane found with appropiate properties\n");
              indices.resize(0);
              table_plane = Eigen::Vector4f (std::numeric_limits<float>::quiet_NaN(),
                                             std::numeric_limits<float>::quiet_NaN(),
                                             std::numeric_limits<float>::quiet_NaN(),
                                             std::numeric_limits<float>::quiet_NaN());

              return;
          }

          table_plane = Eigen::Vector4f (selected_plane.coefficients_.values[0], selected_plane.coefficients_.values[1],
                                         selected_plane.coefficients_.values[2], selected_plane.coefficients_.values[3]);

          if(xyz_points->isOrganized())
          {
              std::vector < pcl::PointIndices > label_indices;
              pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
              labels->points.resize(xyz_points->points.size());
              labels->width = xyz_points->width;
              labels->height = xyz_points->height;

              for (size_t j = 0; j < xyz_points->points.size (); j++)
                  labels->points[j].label = 0;

              //cluster..
              typename pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label>::Ptr
                                                                                             euclidean_cluster_comparator_ (
                                                                                                                            new pcl::EuclideanClusterComparator<
                                                                                                                                PointT, pcl::Normal,
                                                                                                                                pcl::Label> ());

              //create two labels, 1 one for points belonging to or under the plane, 1 for points above the plane
              label_indices.resize (2);

              for (size_t j = 0; j < xyz_points->points.size (); j++)
              {
                Eigen::Vector3f xyz_p = xyz_points->points[j].getVector3fMap ();

                if (!pcl_isfinite (xyz_p[0]) || !pcl_isfinite (xyz_p[1]) || !pcl_isfinite (xyz_p[2]))
                  continue;

                float val = xyz_p[0] * table_plane[0] + xyz_p[1] * table_plane[1] + xyz_p[2] * table_plane[2] + table_plane[3];

                if (val >= 0.01f)
                {
                  labels->points[j].label = 1;
                  label_indices[0].indices.push_back (j);
                }
                else
                {
                  labels->points[j].label = 0;
                  label_indices[1].indices.push_back (j);
                }
              }

              std::vector<bool> plane_labels;
              plane_labels.resize (label_indices.size (), false);
              plane_labels[0] = true;

              euclidean_cluster_comparator_->setInputCloud (xyz_points);
              euclidean_cluster_comparator_->setLabels (labels);
              euclidean_cluster_comparator_->setExcludeLabels (plane_labels);
              euclidean_cluster_comparator_->setDistanceThreshold (0.035f, true);

              pcl::PointCloud < pcl::Label > euclidean_labels;
              std::vector < pcl::PointIndices > euclidean_label_indices;
              pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> euclidean_segmentation (euclidean_cluster_comparator_);
              euclidean_segmentation.setInputCloud (xyz_points);
              euclidean_segmentation.segment (euclidean_labels, euclidean_label_indices);

              for (size_t i = 0; i < euclidean_label_indices.size (); i++)
              {
                if (euclidean_label_indices[i].indices.size () >= min_cluster_size_)
                {
                  indices.push_back (euclidean_label_indices[i]);
                }
              }

          }
          else
          {
              //not organized segmentation

              pcl::ProjectInliers<PointT> proj;
              pcl::ConvexHull<PointT> hull_;
              pcl::ExtractPolygonalPrismData<PointT> prism_;
              prism_.setHeightLimits(0.01,0.5f);
              pcl::EuclideanClusterExtraction<PointT> cluster_;
              proj.setModelType (pcl::SACMODEL_NORMAL_PLANE);
              cluster_.setClusterTolerance (0.03f);
              cluster_.setMinClusterSize (min_cluster_size_);

              typename pcl::PointCloud<PointT>::Ptr table_hull (new pcl::PointCloud<PointT> (*selected_plane.convex_hull_cloud_));

              // Compute the plane coefficients
              Eigen::Vector4f model_coefficients;

              model_coefficients[0] = selected_plane.coefficients_.values[0];
              model_coefficients[1] = selected_plane.coefficients_.values[1];
              model_coefficients[2] = selected_plane.coefficients_.values[2];
              model_coefficients[3] = selected_plane.coefficients_.values[3];

              // Need to flip the plane normal towards the viewpoint
              Eigen::Vector4f vp (0, 0, 0, 0);
              // See if we need to flip any plane normals
              vp -= table_hull->points[0].getVector4fMap ();
              vp[3] = 0;
              // Dot product between the (viewpoint - point) and the plane normal
              float cos_theta = vp.dot (model_coefficients);
              // Flip the plane normal
              if (cos_theta < 0)
              {
                model_coefficients *= -1;
                model_coefficients[3] = 0;
                // Hessian form (D = nc . p_plane (centroid here) + p)
                model_coefficients[3] = -1 * (model_coefficients.dot (table_hull->points[0].getVector4fMap ()));
              }

              // ---[ Get the objects on top of the table
              pcl::PointIndices cloud_object_indices;
              prism_.setInputCloud (xyz_points);
              prism_.setInputPlanarHull (table_hull);
              prism_.segment (cloud_object_indices);

              typename pcl::PointCloud<PointT>::Ptr cloud_objects_ (new pcl::PointCloud<PointT> ());

              typename pcl::ExtractIndices<PointT> extract_object_indices;
              extract_object_indices.setInputCloud (xyz_points);
              extract_object_indices.setIndices (boost::make_shared<const pcl::PointIndices> (cloud_object_indices));
              extract_object_indices.filter (*cloud_objects_);

              if (cloud_objects_->points.size () == 0)
                return;

              // ---[ Split the objects into Euclidean clusters
              std::vector<pcl::PointIndices> clusters2;
              cluster_.setInputCloud (xyz_points);
              cluster_.setIndices (boost::make_shared<const pcl::PointIndices> (cloud_object_indices));
              cluster_.extract (clusters2);

              PCL_INFO ("Number of clusters found matching the given constraints: %zu.\n",
                  clusters2.size ());

              for (size_t i = 0; i < clusters2.size (); ++i)
              {
                indices.push_back(clusters2[i]);
              }
          }
      }
      else
      {
        pcl::apps::DominantPlaneSegmentation<PointT> dps;
        dps.setInputCloud (xyz_points);
        dps.setMaxZBounds (3.F);
        dps.setObjectMinHeight (0.01);
        dps.setMinClusterSize (min_cluster_size_);
        dps.setWSize (9);
        dps.setDistanceBetweenClusters (0.03f);
        std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
        dps.setDownsamplingSize (0.01f);
        dps.compute_full (clusters);
        dps.getIndicesClusters (indices);
        dps.getTableCoefficients (table_plane);
      }
    }

  bool
  segment (segmentation_srv_definitions::segment::Request & req, segmentation_srv_definitions::segment::Response & response)
  {
    pcl::PointCloud<PointT>::Ptr scene (new pcl::PointCloud<PointT>);
    pcl::fromROSMsg (req.cloud, *scene);

    if(chop_at_z_ > 0)
    {
        pcl::PassThrough<PointT> pass_;
        pass_.setFilterLimits (0.f, chop_at_z_);
        pass_.setFilterFieldName ("z");
        pass_.setInputCloud (scene);
        pass_.setKeepOrganized (true);
        pass_.filter (*scene);
    }
    
    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
    ne.setRadiusSearch(0.02f);
    ne.setInputCloud (scene);
    ne.compute (*normal_cloud);

    std::vector<pcl::PointIndices> indices;
    Eigen::Vector4f table_plane;
    doSegmentation<PointT>(scene, normal_cloud, indices, table_plane, seg_type_);

    for(size_t i=0; i < indices.size(); i++)
    {
        std_msgs::Int32MultiArray indx;
        for(size_t k=0; k < indices[i].indices.size(); k++)
    	{
            indx.data.push_back(indices[i].indices[k]);
        }
    	response.clusters_indices.push_back(indx);
    }
    return true;
  }
public:
  PCLSegmenterService ()
  {
    //default values
    chop_at_z_ = 2.f;
    seg_type_ = 0;
    max_angle_plane_to_ground_ = 15.f;
    table_range_min_ = 0.6f;
    table_range_max_ = 1.2f;
    min_cluster_size_ = 500;
    MAX_VERTICAL_PLANE_SIZE_ = 5000;
  }

  void
  initialize (int argc, char ** argv)
  {

    ros::init (argc, argv, "object_segmenter_service");
    n_ = new ros::NodeHandle ( "~" );
    n_->getParam ( "chop_z", chop_at_z_ );
    n_->getParam ( "seg_type", seg_type_ );
    n_->getParam ( "camera_frame", camera_frame_ );
    n_->getParam ( "base_frame", base_frame_ );
    n_->getParam ( "min_cluster_size", min_cluster_size_ );
    n_->getParam ( "max_vertical_plane_size", MAX_VERTICAL_PLANE_SIZE_ );

    segment_ = n_->advertiseService ("object_segmenter", &PCLSegmenterService::segment, this);
    std::cout << "Ready to get service calls..." << chop_at_z_ << " " << seg_type_ << std::endl;
    ros::spin ();
  }
};

int
main (int argc, char ** argv)
{
  PCLSegmenterService m;
  m.initialize (argc, argv);

  return 0;
}
