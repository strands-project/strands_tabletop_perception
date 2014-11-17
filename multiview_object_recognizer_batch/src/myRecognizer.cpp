//------------------------
//---myRecognizer.cpp
//
//
//
//------------------------

#include "myRecognizer.h"
#include <v4r/ORFramework/registered_views_source.h>
#include <v4r/ORFramework/sift_local_estimator.h>
#include <v4r/ORRecognition/graph_geometric_consistency.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <v4r/ORFramework/shot_local_estimator_omp.h>

template<typename PointInT>
class FAAT_3D_FRAMEWORK_API UniformSamplingExtractor : public faat_pcl::rec_3d_framework::KeypointExtractor<PointInT>
{
private:
    typedef typename pcl::PointCloud<PointInT>::Ptr PointInTPtr;
    bool filter_planar_;
    using faat_pcl::rec_3d_framework::KeypointExtractor<PointInT>::input_;
    using faat_pcl::rec_3d_framework::KeypointExtractor<PointInT>::radius_;
    float sampling_density_;
    boost::shared_ptr<std::vector<std::vector<int> > > neighborhood_indices_;
    boost::shared_ptr<std::vector<std::vector<float> > > neighborhood_dist_;
    float max_distance_;
    float threshold_planar_;
    bool z_adaptative_;

    void
    filterPlanar (PointInTPtr & input, pcl::PointCloud<int> & keypoints_cloud)
    {
        pcl::PointCloud<int> filtered_keypoints;
        //create a search object
        typename pcl::search::Search<PointInT>::Ptr tree;
        if (input->isOrganized ())
            tree.reset (new pcl::search::OrganizedNeighbor<PointInT> ());
        else
            tree.reset (new pcl::search::KdTree<PointInT> (false));
        tree->setInputCloud (input);

        neighborhood_indices_.reset (new std::vector<std::vector<int> >);
        neighborhood_indices_->resize (keypoints_cloud.points.size ());
        neighborhood_dist_.reset (new std::vector<std::vector<float> >);
        neighborhood_dist_->resize (keypoints_cloud.points.size ());

        filtered_keypoints.points.resize (keypoints_cloud.points.size ());
        int good = 0;

        for (size_t i = 0; i < keypoints_cloud.points.size (); i++)
        {

            if (tree->radiusSearch (keypoints_cloud[i], radius_, (*neighborhood_indices_)[good], (*neighborhood_dist_)[good]))
            {

                EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
                Eigen::Vector4f xyz_centroid;
                EIGEN_ALIGN16 Eigen::Vector3f eigenValues;
                EIGEN_ALIGN16 Eigen::Matrix3f eigenVectors;

                //compute planarity of the region
                computeMeanAndCovarianceMatrix (*input, (*neighborhood_indices_)[good], covariance_matrix, xyz_centroid);
                pcl::eigen33 (covariance_matrix, eigenVectors, eigenValues);

                float eigsum = eigenValues.sum ();
                if (!pcl_isfinite(eigsum))
                {
                    PCL_ERROR("Eigen sum is not finite\n");
                }

                float t_planar = threshold_planar_;
                if(z_adaptative_)
                {
                    t_planar *= (1 + (std::max(input->points[keypoints_cloud.points[i]].z,1.f) - 1.f));
                }

                if ((fabs (eigenValues[0] - eigenValues[1]) < 1.5e-4) || (eigsum != 0 && fabs (eigenValues[0] / eigsum) > t_planar))
                {
                    //region is not planar, add to filtered keypoint
                    keypoints_cloud.points[good] = keypoints_cloud.points[i];
                    good++;
                }
            }
        }

        neighborhood_indices_->resize (good);
        neighborhood_dist_->resize (good);
        keypoints_cloud.points.resize (good);

        neighborhood_indices_->clear ();
        neighborhood_dist_->clear ();

    }

public:

    UniformSamplingExtractor()
    {
        max_distance_ = std::numeric_limits<float>::infinity();
        threshold_planar_ = 1.e-2;
        z_adaptative_ = false;
    }

    void zAdaptative(bool b)
    {
        z_adaptative_ = b;
    }

    void setThresholdPlanar(float t)
    {
        threshold_planar_ = t;
    }

    void setMaxDistance(float d)
    {
        max_distance_ = d;
    }

    void
    setFilterPlanar (bool b)
    {
        filter_planar_ = b;
    }

    void
    setSamplingDensity (float f)
    {
        sampling_density_ = f;
    }

    void
    compute (PointInTPtr & keypoints)
    {
        keypoints.reset (new pcl::PointCloud<PointInT>);

        pcl::UniformSampling<PointInT> keypoint_extractor;
        keypoint_extractor.setRadiusSearch (sampling_density_);
        keypoint_extractor.setInputCloud (input_);

        pcl::PointCloud<int> keypoints_idxes;
        keypoint_extractor.compute (keypoints_idxes);

        if(pcl_isfinite(max_distance_))
        {
            int valid = 0;
            int original_size = (int)keypoints_idxes.size();
            for(size_t i=0; i < keypoints_idxes.size(); i++)
            {
                if(input_->points[keypoints_idxes.points[i]].z < max_distance_)
                {
                    keypoints_idxes.points[valid] = keypoints_idxes.points[i];
                    valid++;
                }
            }

            keypoints_idxes.points.resize(valid);
            keypoints_idxes.width = valid;
            PCL_WARN("filtered %d keypoints based on z-distance %f\n", (original_size - valid), max_distance_);
        }

        if (filter_planar_)
            filterPlanar (input_, keypoints_idxes);

        std::vector<int> indices;
        indices.resize (keypoints_idxes.points.size ());
        for (size_t i = 0; i < indices.size (); i++)
            indices[i] = keypoints_idxes.points[i];

        pcl::copyPointCloud (*input_, indices, *keypoints);
    }

    void
    compute (std::vector<int> & indices)
    {
        pcl::UniformSampling<PointInT> keypoint_extractor;
        keypoint_extractor.setRadiusSearch (sampling_density_);
        keypoint_extractor.setInputCloud (input_);

        pcl::PointCloud<int> keypoints_idxes;
        keypoint_extractor.compute (keypoints_idxes);

        if (filter_planar_)
            filterPlanar (input_, keypoints_idxes);

        indices.resize (keypoints_idxes.points.size ());
        for (size_t i = 0; i < indices.size (); i++)
            indices[i] = keypoints_idxes.points[i];
    }
};

void
myRecognizer::computeTablePlane (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> > & xyz_points, Eigen::Vector4f & table_plane, float z_dist)
{
    pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
    ne.setMaxDepthChangeFactor (0.02f);
    ne.setNormalSmoothingSize (20.0f);
    ne.setBorderPolicy (pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal>::BORDER_POLICY_IGNORE);
    ne.setInputCloud (xyz_points);
    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
    ne.compute (*normal_cloud);

    int num_plane_inliers = 5000;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyz_points_andy (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PassThrough<pcl::PointXYZRGB> pass_;
    pass_.setFilterLimits (0.f, z_dist);
    pass_.setFilterFieldName ("z");
    pass_.setInputCloud (xyz_points);
    pass_.setKeepOrganized (true);
    pass_.filter (*xyz_points_andy);

    pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZRGB, pcl::Normal, pcl::Label> mps;
    mps.setMinInliers (num_plane_inliers);
    mps.setAngularThreshold (0.017453 * 1.5f); // 2 degrees
    mps.setDistanceThreshold (0.01); // 1cm
    mps.setInputNormals (normal_cloud);
    mps.setInputCloud (xyz_points_andy);

    std::vector<pcl::PlanarRegion<pcl::PointXYZRGB>, Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointXYZRGB> > > regions;
    std::vector<pcl::ModelCoefficients> model_coefficients;
    std::vector<pcl::PointIndices> inlier_indices;
    pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
    std::vector<pcl::PointIndices> label_indices;
    std::vector<pcl::PointIndices> boundary_indices;

    pcl::PlaneRefinementComparator<pcl::PointXYZRGB, pcl::Normal, pcl::Label>::Ptr ref_comp (
                new pcl::PlaneRefinementComparator<pcl::PointXYZRGB,
                pcl::Normal, pcl::Label> ());
    ref_comp->setDistanceThreshold (0.01f, true);
    ref_comp->setAngularThreshold (0.017453 * 10);
    mps.setRefinementComparator (ref_comp);
    mps.segmentAndRefine (regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);

    std::cout << "Number of planes found:" << model_coefficients.size () << std::endl;

    int table_plane_selected = 0;
    int max_inliers_found = -1;
    std::vector<int> plane_inliers_counts;
    plane_inliers_counts.resize (model_coefficients.size ());

    for (size_t i = 0; i < model_coefficients.size (); i++)
    {
        Eigen::Vector4f table_plane = Eigen::Vector4f (model_coefficients[i].values[0], model_coefficients[i].values[1], model_coefficients[i].values[2],
                                                       model_coefficients[i].values[3]);

        std::cout << "Number of inliers for this plane:" << inlier_indices[i].indices.size () << std::endl;
        int remaining_points = 0;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_points (new pcl::PointCloud<pcl::PointXYZRGB> (*xyz_points_andy));
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

    /*table_plane = Eigen::Vector4f (model_coefficients[table_plane_selected].values[0], model_coefficients[table_plane_selected].values[1],
   model_coefficients[table_plane_selected].values[2], model_coefficients[table_plane_selected].values[3]);*/

    std::cout << "Table plane computed... " << std::endl;
}

void
myRecognizer::doSegmentation (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> > & xyz_points,
                              std::vector<typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr> & clusters, std::vector<pcl::PointIndices> & indices,
                              Eigen::Vector4f & table_plane)
{
    std::cout << "Start segmentation..." << std::endl;
    int seg_detail_ = 0;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyz_points_andy (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PassThrough<pcl::PointXYZRGB> pass_;
    pass_.setFilterLimits (0.f, z_dist_);
    pass_.setFilterFieldName ("z");
    pass_.setInputCloud (xyz_points);
    pass_.setKeepOrganized (true);
    pass_.filter (*xyz_points_andy);

    //   if (segmentation_ == 0)
    //   {
    //     {
    //       for (int i = 0; i < xyz_points_andy->points.size (); i++)
    //       {
    //         Eigen::Vector3f xyz_p = xyz_points_andy->points[i].getVector3fMap ();
    //
    //         if (!pcl_isfinite (xyz_p[0]) || !pcl_isfinite (xyz_p[1]) || !pcl_isfinite (xyz_p[2]))
    //           continue;
    //
    //         float val = xyz_p[0] * table_plane[0] + xyz_p[1] * table_plane[1] + xyz_p[2] * table_plane[2] + table_plane[3];
    //
    //         if (val <= 0.007)
    //         {
    //           xyz_points_andy->points[i].x = std::numeric_limits<float>::quiet_NaN ();
    //           xyz_points_andy->points[i].y = std::numeric_limits<float>::quiet_NaN ();
    //           xyz_points_andy->points[i].z = std::numeric_limits<float>::quiet_NaN ();
    //         }
    //       }
    //     }
    //
    //     seg_.setDetail (seg_detail_);
    //     indices = seg_.processPointCloudV (xyz_points_andy);
    //
    //     for (size_t i = 0; i < indices.size (); i++)
    //     {
    //       typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    //       pcl::copyPointCloud (*xyz_points, indices[i].indices, *cluster);
    //       clusters.push_back (cluster);
    //     }
    //   }
    //   else if (segmentation_ == 1)
    {
        pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
        ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
        ne.setMaxDepthChangeFactor (0.02f);
        ne.setNormalSmoothingSize (20.0f);
        ne.setBorderPolicy (pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal>::BORDER_POLICY_IGNORE);
        ne.setInputCloud (xyz_points);
        pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
        ne.compute (*normal_cloud);

        int num_plane_inliers = 5000;

        pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZRGB, pcl::Normal, pcl::Label> mps;
        mps.setMinInliers (num_plane_inliers);
        mps.setAngularThreshold (0.017453 * 1.5f); // 2 degrees
        mps.setDistanceThreshold (0.01); // 1cm
        mps.setInputNormals (normal_cloud);
        mps.setInputCloud (xyz_points_andy);

        std::vector<pcl::PlanarRegion<pcl::PointXYZRGB>, Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointXYZRGB> > > regions;
        std::vector<pcl::ModelCoefficients> model_coefficients;
        std::vector<pcl::PointIndices> inlier_indices;
        pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
        std::vector<pcl::PointIndices> label_indices;
        std::vector<pcl::PointIndices> boundary_indices;

        pcl::PlaneRefinementComparator<pcl::PointXYZRGB, pcl::Normal, pcl::Label>::Ptr ref_comp (
                    new pcl::PlaneRefinementComparator<pcl::PointXYZRGB,
                    pcl::Normal, pcl::Label> ());
        ref_comp->setDistanceThreshold (0.01f, true);
        ref_comp->setAngularThreshold (0.017453 * 10);
        mps.setRefinementComparator (ref_comp);
        mps.segmentAndRefine (regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);

        std::cout << "Number of planes found:" << model_coefficients.size () << std::endl;

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
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_points (new pcl::PointCloud<pcl::PointXYZRGB> (*xyz_points_andy));
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
        typename pcl::EuclideanClusterComparator<pcl::PointXYZRGB, pcl::Normal, pcl::Label>::Ptr
                euclidean_cluster_comparator_ (
                    new pcl::EuclideanClusterComparator<
                    pcl::PointXYZRGB,
                    pcl::Normal,
                    pcl::Label> ());

        //create two labels, 1 one for points belonging to or under the plane, 1 for points above the plane
        label_indices.resize (2);

        for (int j = 0; j < xyz_points_andy->points.size (); j++)
        {
            Eigen::Vector3f xyz_p = xyz_points_andy->points[j].getVector3fMap ();

            if (!pcl_isfinite (xyz_p[0]) || !pcl_isfinite (xyz_p[1]) || !pcl_isfinite (xyz_p[2]))
                continue;

            float val = xyz_p[0] * table_plane[0] + xyz_p[1] * table_plane[1] + xyz_p[2] * table_plane[2] + table_plane[3];

            if (val >= 0.01)
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
        /*for (size_t i = 0; i < label_indices.size (); i++)
     {
     if (label_indices[i].indices.size () > num_plane_inliers)
     {
     plane_labels[i] = true;
     }
     }*/
        plane_labels[0] = true;

        euclidean_cluster_comparator_->setInputCloud (xyz_points_andy);
        euclidean_cluster_comparator_->setLabels (labels);
        euclidean_cluster_comparator_->setExcludeLabels (plane_labels);
        euclidean_cluster_comparator_->setDistanceThreshold (0.005f, false);

        pcl::PointCloud<pcl::Label> euclidean_labels;
        std::vector<pcl::PointIndices> euclidean_label_indices;
        pcl::OrganizedConnectedComponentSegmentation<pcl::PointXYZRGB, pcl::Label> euclidean_segmentation (euclidean_cluster_comparator_);
        euclidean_segmentation.setInputCloud (xyz_points_andy);
        euclidean_segmentation.segment (euclidean_labels, euclidean_label_indices);

        for (size_t i = 0; i < euclidean_label_indices.size (); i++)
        {
            if (euclidean_label_indices[i].indices.size () > 500)
            {
                typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::copyPointCloud (*xyz_points_andy, euclidean_label_indices[i].indices, *cluster);
                clusters.push_back (cluster);
                indices.push_back (euclidean_label_indices[i]);
            }
        }

        PCL_INFO ("Got %d euclidean clusters!\n", clusters.size ());
        std::cout << "Regions:" << regions.size () << std::endl;
    }
}

struct camPosConstraints
{
    bool
    operator() (const Eigen::Vector3f & pos) const
    {
        if (pos[2] > 0)
            return true;

        return false;
    }
    ;
};

void
myRecognizer::setupRFColorOURCVFH (std::string & model_path, std::string & training_dir)
{

    bool normalize_bins_ = false;
    int nn_ = 8;
    float accept_ourcvfh_hypthreshold_ = 0.75;
    int OUR_CVFH_MAX_HYP_ = 30;
    float max_desc_distance_ = 0.6;
    std::string desc_name = "rf_our_cvfh_color";

    boost::shared_ptr < faat_pcl::rec_3d_framework::PartialPCDSource<pcl::PointXYZRGBNormal, pcl::PointXYZRGB>
            > source (new faat_pcl::rec_3d_framework::PartialPCDSource<pcl::PointXYZRGBNormal, pcl::PointXYZRGB>);
    source->setPath (model_path);
    source->setModelScale (1.f);
    source->setRadiusSphere (1.f);
    source->setTesselationLevel (1);
    source->setUseVertices (false);
    source->setDotNormal (-1.f);
    source->setLoadViews (true);

    boost::function<bool
            (const Eigen::Vector3f &)> campos_constraints;
    campos_constraints = camPosConstraints ();

    source->setCamPosConstraints (campos_constraints);
    source->generate (training_dir);

    boost::shared_ptr < faat_pcl::rec_3d_framework::Source<pcl::PointXYZRGB> > cast_source;
    cast_source = boost::static_pointer_cast<faat_pcl::rec_3d_framework::PartialPCDSource<pcl::PointXYZRGBNormal, pcl::PointXYZRGB> > (source);

    //configure normal estimator
    boost::shared_ptr < faat_pcl::rec_3d_framework::PreProcessorAndNormalEstimator<pcl::PointXYZRGB, pcl::Normal> > normal_estimator;
    normal_estimator.reset (new faat_pcl::rec_3d_framework::PreProcessorAndNormalEstimator<pcl::PointXYZRGB, pcl::Normal>);
    normal_estimator->setCMR (false);
    normal_estimator->setDoVoxelGrid (true);
    normal_estimator->setRemoveOutliers (true);
    normal_estimator->setValuesForCMRFalse (0.003f, 0.018f);

    boost::shared_ptr < faat_pcl::rec_3d_framework::ColorOURCVFHEstimator<pcl::PointXYZRGB, pcl::Histogram<1327> > > vfh_estimator;
    vfh_estimator.reset (new faat_pcl::rec_3d_framework::ColorOURCVFHEstimator<pcl::PointXYZRGB, pcl::Histogram<1327> >);
    vfh_estimator->setNormalEstimator (normal_estimator);
    vfh_estimator->setNormalizeBins (normalize_bins_);
    vfh_estimator->setUseRFForColor (true);
    vfh_estimator->setRefineClustersParam (2.5f);
    vfh_estimator->setAdaptativeMLS (false);

    vfh_estimator->setAxisRatio (1.f);
    vfh_estimator->setMinAxisValue (1.f);

    {
        //segmentation parameters for training
        std::vector<float> eps_thresholds, cur_thresholds, clus_thresholds;
        eps_thresholds.push_back (0.15);
        cur_thresholds.push_back (0.015f);
        clus_thresholds.push_back (2.5f);

        vfh_estimator->setClusterToleranceVector (clus_thresholds);
        vfh_estimator->setEpsAngleThresholdVector (eps_thresholds);
        vfh_estimator->setCurvatureThresholdVector (cur_thresholds);
    }

    if (normalize_bins_)
    {
        desc_name = "rf_our_cvfh_color_normalized";
    }

    std::cout << "Descriptor name:" << desc_name << std::endl;
    boost::shared_ptr < faat_pcl::rec_3d_framework::OURCVFHEstimator<pcl::PointXYZRGB, pcl::Histogram<1327> > > cast_estimator;
    cast_estimator
            = boost::dynamic_pointer_cast<faat_pcl::rec_3d_framework::ColorOURCVFHEstimator<pcl::PointXYZRGB, pcl::Histogram<1327> > > (vfh_estimator);

    rf_color_ourcvfh_global_.reset(new faat_pcl::rec_3d_framework::GlobalNNCVFHRecognizer<faat_pcl::Metrics::HistIntersectionUnionDistance, PointT, pcl::Histogram<1327> >);
    rf_color_ourcvfh_global_->setDataSource (cast_source);
    rf_color_ourcvfh_global_->setTrainingDir (training_dir);
    rf_color_ourcvfh_global_->setDescriptorName (desc_name);
    rf_color_ourcvfh_global_->setFeatureEstimator (cast_estimator);
    rf_color_ourcvfh_global_->setNN (nn_);
    rf_color_ourcvfh_global_->setICPIterations (0);
    rf_color_ourcvfh_global_->setNoise (0.0f);
    rf_color_ourcvfh_global_->setUseCache (false);
    rf_color_ourcvfh_global_->setAcceptHypThreshold (accept_ourcvfh_hypthreshold_);
    rf_color_ourcvfh_global_->setMaxDescDistance (max_desc_distance_);
    rf_color_ourcvfh_global_->setMaxHyp (OUR_CVFH_MAX_HYP_);
    rf_color_ourcvfh_global_->initialize (false);

    {

        std::vector<float> eps_thresholds, cur_thresholds, clus_thresholds;
        eps_thresholds.push_back (0.15);
        //eps_thresholds.push_back (0.1750f);
        cur_thresholds.push_back (0.015f);
        //cur_thresholds.push_back (0.0175f);
        cur_thresholds.push_back (0.02f);
        //cur_thresholds.push_back (0.025f);
        cur_thresholds.push_back (0.035f);
        clus_thresholds.push_back (2.5f);

        vfh_estimator->setClusterToleranceVector (clus_thresholds);
        vfh_estimator->setEpsAngleThresholdVector (eps_thresholds);
        vfh_estimator->setCurvatureThresholdVector (cur_thresholds);

        vfh_estimator->setAxisRatio (0.5f);
        vfh_estimator->setMinAxisValue (0.5f);

        vfh_estimator->setAdaptativeMLS (false);
    }
}

void
myRecognizer::init (std::string models_dir_sift, std::string model_path, std::string training_dir)
{
    multi_recog_.reset(new faat_pcl::rec_3d_framework::MultiRecognitionPipeline<PointT>);
    boost::shared_ptr < faat_pcl::CorrespondenceGrouping<PointT, PointT> > cast_cg_alg;
    boost::shared_ptr < faat_pcl::GraphGeometricConsistencyGrouping<PointT, PointT> > gcg_alg (
                new faat_pcl::GraphGeometricConsistencyGrouping<
                PointT, PointT>);

    std::cout << CG_SIZE_ << " " << CG_THRESHOLD_ << std::endl;

    gcg_alg->setGCThreshold (CG_SIZE_);
    gcg_alg->setGCSize (CG_THRESHOLD_);
    gcg_alg->setRansacThreshold (CG_THRESHOLD_);
    gcg_alg->setUseGraph (true);
    gcg_alg->setDistForClusterFactor (0);
    gcg_alg->setCheckNormalsOrientation(true);
    gcg_alg->setMaxTaken(1);
    gcg_alg->setSortCliques(true);
    gcg_alg->setDotDistance (0.25);
    gcg_alg->setPrune(false);
    gcg_alg->pruneByCC(false);

    cast_cg_alg = boost::static_pointer_cast<faat_pcl::CorrespondenceGrouping<PointT, PointT> > (gcg_alg);

    std::cout << "do sift, do_shot: " << do_new_sift_ << "," << do_shot_ << std::endl;
    if (do_new_sift_)
    {
        std::string desc_name = "sift";
        std::string idx_flann_fn = sift_idx_;

        //configure mesh source
        typedef pcl::PointXYZRGB PointT;
        boost::shared_ptr < faat_pcl::rec_3d_framework::RegisteredViewsSource<pcl::PointXYZRGBNormal, PointT, PointT>
                > mesh_source (new faat_pcl::rec_3d_framework::RegisteredViewsSource<pcl::PointXYZRGBNormal, pcl::PointXYZRGB, pcl::PointXYZRGB>);
        mesh_source->setPath (new_sift_models_);
        mesh_source->setModelStructureDir (training_input_structure_);
        mesh_source->setLoadViews(false);
        mesh_source->generate (training_dir_new_sift_);

        boost::shared_ptr < faat_pcl::rec_3d_framework::Source<PointT> > cast_source;
        cast_source
                = boost::static_pointer_cast<faat_pcl::rec_3d_framework::RegisteredViewsSource<pcl::PointXYZRGBNormal, PointT, PointT> > (mesh_source);

        boost::shared_ptr < faat_pcl::rec_3d_framework::SIFTLocalEstimation<PointT, pcl::Histogram<128> > > estimator;
        estimator.reset (new faat_pcl::rec_3d_framework::SIFTLocalEstimation<PointT, pcl::Histogram<128> >);

        boost::shared_ptr < faat_pcl::rec_3d_framework::LocalEstimator<PointT, pcl::Histogram<128> > > cast_estimator;
        cast_estimator = boost::dynamic_pointer_cast<faat_pcl::rec_3d_framework::SIFTLocalEstimation<PointT, pcl::Histogram<128> > > (estimator);

        new_sift_local_.reset (new faat_pcl::rec_3d_framework::LocalRecognitionPipeline<flann::L1, PointT, pcl::Histogram<128> > (idx_flann_fn));
        new_sift_local_->setDataSource (cast_source);
        new_sift_local_->setTrainingDir (training_dir_new_sift_);
        new_sift_local_->setDescriptorName (desc_name);
        new_sift_local_->setICPIterations(icp_iterations_);
        new_sift_local_->setFeatureEstimator (cast_estimator);
        new_sift_local_->setKdtreeSplits(512);
        new_sift_local_->setUseCache(true);
        new_sift_local_->setCGAlgorithm (cast_cg_alg);
        new_sift_local_->setKnn(sift_knn_);
        new_sift_local_->setDistanceSameKeypoint(0.005f);
        new_sift_local_->setUseCache (true);
        new_sift_local_->initialize (false);

        boost::shared_ptr<faat_pcl::rec_3d_framework::Recognizer<PointT> > cast_recog;
        cast_recog = boost::static_pointer_cast<faat_pcl::rec_3d_framework::LocalRecognitionPipeline<flann::L1, PointT, pcl::Histogram<128> > > (new_sift_local_);
        multi_recog_->addRecognizer(cast_recog);
    }

    if(do_shot_)
    {
        std::string desc_name = "shot";
        std::string idx_flann_fn = shot_idx_;
        bool use_cache = true;
        float test_sampling_density = 0.01f;

        //configure mesh source
        typedef pcl::PointXYZRGB PointT;
        boost::shared_ptr < faat_pcl::rec_3d_framework::RegisteredViewsSource<pcl::PointXYZRGBNormal, PointT, PointT>
                > mesh_source (new faat_pcl::rec_3d_framework::RegisteredViewsSource<pcl::PointXYZRGBNormal, pcl::PointXYZRGB, pcl::PointXYZRGB>);
        mesh_source->setPath (new_sift_models_);
        mesh_source->setModelStructureDir (training_input_structure_);
        mesh_source->setLoadViews(false);
        mesh_source->generate (training_dir_shot_);

        boost::shared_ptr < faat_pcl::rec_3d_framework::Source<PointT> > cast_source;
        cast_source = boost::static_pointer_cast<faat_pcl::rec_3d_framework::RegisteredViewsSource<pcl::PointXYZRGBNormal, PointT, PointT> > (mesh_source);

        boost::shared_ptr<faat_pcl::rec_3d_framework::UniformSamplingExtractor<PointT> > uniform_keypoint_extractor ( new faat_pcl::rec_3d_framework::UniformSamplingExtractor<PointT>);
        uniform_keypoint_extractor->setSamplingDensity (0.01f);
        uniform_keypoint_extractor->setFilterPlanar (true);

        boost::shared_ptr<faat_pcl::rec_3d_framework::KeypointExtractor<PointT> > keypoint_extractor;
        keypoint_extractor = boost::static_pointer_cast<faat_pcl::rec_3d_framework::KeypointExtractor<PointT> > (uniform_keypoint_extractor);

        boost::shared_ptr<faat_pcl::rec_3d_framework::PreProcessorAndNormalEstimator<PointT, pcl::Normal> > normal_estimator;
        normal_estimator.reset (new faat_pcl::rec_3d_framework::PreProcessorAndNormalEstimator<PointT, pcl::Normal>);
        normal_estimator->setCMR (false);
        normal_estimator->setDoVoxelGrid (true);
        normal_estimator->setRemoveOutliers (false);
        normal_estimator->setValuesForCMRFalse (0.003f, 0.02f);

        boost::shared_ptr<faat_pcl::rec_3d_framework::SHOTLocalEstimationOMP<PointT, pcl::Histogram<352> > > estimator;
        estimator.reset (new faat_pcl::rec_3d_framework::SHOTLocalEstimationOMP<PointT, pcl::Histogram<352> >);
        estimator->setNormalEstimator (normal_estimator);
        estimator->addKeypointExtractor (keypoint_extractor);
        estimator->setSupportRadius (0.04f);
        estimator->setAdaptativeMLS (false);

        boost::shared_ptr<faat_pcl::rec_3d_framework::LocalEstimator<PointT, pcl::Histogram<352> > > cast_estimator;
        cast_estimator = boost::dynamic_pointer_cast<faat_pcl::rec_3d_framework::LocalEstimator<PointT, pcl::Histogram<352> > > (estimator);

        boost::shared_ptr<faat_pcl::rec_3d_framework::LocalRecognitionPipeline<flann::L1, PointT, pcl::Histogram<352> > > local;
        local.reset(new faat_pcl::rec_3d_framework::LocalRecognitionPipeline<flann::L1, PointT, pcl::Histogram<352> > (shot_idx_));
        local->setDataSource (cast_source);
        local->setTrainingDir (training_dir_shot_);
        local->setDescriptorName (desc_name);
        local->setFeatureEstimator (cast_estimator);
        local->setCGAlgorithm (cast_cg_alg);
        local->setKnn(knn_shot_);
        local->setUseCache (use_cache);
        local->setThresholdAcceptHyp (1);
        uniform_keypoint_extractor->setSamplingDensity (test_sampling_density);
        local->setICPIterations (0);
        local->setKdtreeSplits (128);
        local->initialize (false);
        local->setMaxDescriptorDistance(std::numeric_limits<float>::infinity());
        uniform_keypoint_extractor->setMaxDistance(1.5f);

        /*boost::shared_ptr<faat_pcl::rec_3d_framework::ISSKeypointExtractor<PointT> > issk_extractor ( new faat_pcl::rec_3d_framework::ISSKeypointExtractor<PointT>);
        issk_extractor->setSupportRadius(0.04f);
        issk_extractor->setNonMaximaRadius(0.01f);
        keypoint_extractor = boost::static_pointer_cast<faat_pcl::rec_3d_framework::KeypointExtractor<PointT> > (issk_extractor);*/

        {
            boost::shared_ptr<UniformSamplingExtractor<PointT> > uniform_keypoint_extractor ( new UniformSamplingExtractor<PointT>);
            uniform_keypoint_extractor->setSamplingDensity (0.01f);
            uniform_keypoint_extractor->setFilterPlanar (true);
            uniform_keypoint_extractor->setMaxDistance(1.5f);
            uniform_keypoint_extractor->setThresholdPlanar(0.1);
            //uniform_keypoint_extractor->zAdaptative(true);
            //1e-2 => 0.01
            //1e-1 => 0.1
            keypoint_extractor = boost::static_pointer_cast<faat_pcl::rec_3d_framework::KeypointExtractor<PointT> > (uniform_keypoint_extractor);

            estimator.reset (new faat_pcl::rec_3d_framework::SHOTLocalEstimationOMP<PointT, pcl::Histogram<352> >);
            estimator->setNormalEstimator (normal_estimator);
            estimator->addKeypointExtractor (keypoint_extractor);
            estimator->setSupportRadius (0.04f);
            estimator->setAdaptativeMLS (false);

            cast_estimator = boost::dynamic_pointer_cast<faat_pcl::rec_3d_framework::LocalEstimator<PointT, pcl::Histogram<352> > > (estimator);
            local->setFeatureEstimator (cast_estimator);
        }


        boost::shared_ptr<faat_pcl::rec_3d_framework::Recognizer<PointT> > cast_recog;
        cast_recog = boost::static_pointer_cast<faat_pcl::rec_3d_framework::LocalRecognitionPipeline<flann::L1, PointT, pcl::Histogram<352> > > (local);
        multi_recog_->addRecognizer(cast_recog);
    }

    if (do_ourcvfh_)
    {
        setupRFColorOURCVFH (model_path, training_dir);
        boost::shared_ptr<faat_pcl::rec_3d_framework::Recognizer<PointT> > cast_recog;
        cast_recog = boost::static_pointer_cast<faat_pcl::rec_3d_framework::GlobalNNCVFHRecognizer<faat_pcl::Metrics::HistIntersectionUnionDistance, PointT, pcl::Histogram<1327> > > (rf_color_ourcvfh_global_);
        multi_recog_->addRecognizer(cast_recog);
    }

    multi_recog_->setCGAlgorithm(gcg_alg);
    multi_recog_->setVoxelSizeICP(0.005f);
    multi_recog_->setICPType(1);
    multi_recog_->setICPIterations(icp_iterations_);
    multi_recog_->initialize();
}

void
myRecognizer::recognize (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> > & xyz_points)
{
    // AITOR: CAN YOU CHANGE YOUR CODE TO ACCEPT CONST PTR TO CONST OBJECTS??

    boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> > pXYZ_points = xyz_points;
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > pXYZ_points__non_const = boost::const_pointer_cast<pcl::PointCloud<pcl::PointXYZRGB> >(pXYZ_points);

    Eigen::Vector4f table_plane;
    if(use_table_plane_)
    {
        pcl::ScopeTime t ("Computing table plane\n");
        computeTablePlane (xyz_points, table_plane);
        std::cout << table_plane << std::endl;
        table_plane_ = table_plane;

        std::vector<int> indices_above_plane;
        for (int k = 0; k < pXYZ_points__non_const->points.size (); k++)
        {
            Eigen::Vector3f xyz_p = pXYZ_points__non_const->points[k].getVector3fMap ();

            if (!pcl_isfinite (xyz_p[0]) || !pcl_isfinite (xyz_p[1]) || !pcl_isfinite (xyz_p[2]))
                continue;

            float val = xyz_p[0] * table_plane_[0] + xyz_p[1] * table_plane_[1] + xyz_p[2] * table_plane_[2] + table_plane_[3];

            if (val >= 0.01)
                indices_above_plane.push_back (static_cast<int> (k));
        }

        multi_recog_->setIndices(indices_above_plane);
    }

    if(do_ourcvfh_ && use_table_plane_)
    {
        //perform segmentation to use OUR-CVFH
        std::vector<pcl::PointIndices> indices;
        Eigen::Vector4f table_plane;
        std::vector<pcl::PointCloud<PointT>::Ptr> clusters;
        doSegmentation (xyz_points, clusters, indices, table_plane);
        multi_recog_->setSegmentation(indices);
    }

    assert(scene_normals_->points.size() == pXYZ_points__non_const->points.size());
    multi_recog_->setSceneNormals(scene_normals_);

    multi_recog_->setInputCloud (pXYZ_points__non_const);
    {
        pcl::ScopeTime ttt ("Recognition");
        multi_recog_->recognize ();
    }

    models_merged_.reset (new std::vector<ModelTPtr>);
    transforms_merged_.reset (new std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >);
    models_merged_ = multi_recog_->getModels ();
    transforms_merged_ = multi_recog_->getTransforms ();
}

/*void
myRecognizer::recognize (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> > xyz_points)
{
  // AITOR: CAN YOU CHANGE YOUR CODE TO ACCEPT CONST PTR TO CONST OBJECTS??

  boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> > pXYZ_points = xyz_points;
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > pXYZ_points__non_const = boost::const_pointer_cast<pcl::PointCloud<pcl::PointXYZRGB> >(pXYZ_points);

  Eigen::Vector4f table_plane;
  {
    pcl::ScopeTime t ("Computing table plane\n");
    computeTablePlane (xyz_points, table_plane);
  }

  std::cout << table_plane << std::endl;
  table_plane_ = table_plane;

  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters;
  std::vector<pcl::PointIndices> indices;

  models_merged_.reset (new std::vector<ModelTPtr>);
  transforms_merged_.reset (new std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >);

  if (do_new_sift_)
  {
    pcl::ScopeTime ttt ("NEW SIFT local Recognition");
    new_sift_local_->setInputCloud (pXYZ_points__non_const);
    new_sift_local_->recognize ();

    boost::shared_ptr < std::vector<ModelTPtr> > models = new_sift_local_->getModels ();
    boost::shared_ptr < std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > transforms = new_sift_local_->getTransforms ();

    for (size_t j = 0; j < models->size (); j++)
    {
      models_merged_->push_back (models->at (j));
      transforms_merged_->push_back (transforms->at (j));
    }
  }

  if (do_ourcvfh_)
  {
    doSegmentation (xyz_points, clusters, indices, table_plane);

    for (size_t i = 0; i < clusters.size (); i++)
    {
      if (indices[i].indices.size () < 500)
        continue;

      rf_color_ourcvfh_global_.setInputCloud (pXYZ_points__non_const);
      rf_color_ourcvfh_global_.setIndices (indices[i].indices);

      {
        pcl::ScopeTime t ("COLOR OUR-CVFH Recognition");
        rf_color_ourcvfh_global_.recognize ();
      }

      boost::shared_ptr < std::vector<ModelTPtr> > models = rf_color_ourcvfh_global_.getModels ();
      boost::shared_ptr < std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > transforms
          = rf_color_ourcvfh_global_.getTransforms ();

      for (size_t j = 0; j < models->size (); j++)
      {
        models_merged_->push_back (models->at (j));
        transforms_merged_->push_back (transforms->at (j));
      }
    }
  }

  //Refine the poses of the models
  if (icp_iterations_ > 0)
  {

    std::cout << "doing icp:" << models_merged_->size () << " hypotheses" << std::endl;
    PointInTPtr cloud_voxelized_icp (new pcl::PointCloud<pcl::PointXYZRGB> ());

    {
      pcl::ScopeTime t ("Voxelize stuff and remove useless points...");

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyz_points_andy (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PassThrough<pcl::PointXYZRGB> pass_;
      pass_.setFilterLimits (0.f, z_dist_);
      pass_.setFilterFieldName ("z");
      pass_.setInputCloud (xyz_points);
      pass_.setKeepOrganized (true);
      pass_.filter (*xyz_points_andy);

      for (int i = 0; i < xyz_points_andy->points.size (); i++)
      {
        Eigen::Vector3f xyz_p = xyz_points_andy->points[i].getVector3fMap ();

        if (!pcl_isfinite (xyz_p[0]) || !pcl_isfinite (xyz_p[1]) || !pcl_isfinite (xyz_p[2]))
          continue;

        float val = xyz_p[0] * table_plane[0] + xyz_p[1] * table_plane[1] + xyz_p[2] * table_plane[2] + table_plane[3];

        if (val <= 0.007)
        {
          xyz_points_andy->points[i].x = std::numeric_limits<float>::quiet_NaN ();
          xyz_points_andy->points[i].y = std::numeric_limits<float>::quiet_NaN ();
          xyz_points_andy->points[i].z = std::numeric_limits<float>::quiet_NaN ();
        }
      }

      pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid_icp;
      voxel_grid_icp.setDownsampleAllData (true);
      voxel_grid_icp.setInputCloud (xyz_points_andy);
      voxel_grid_icp.setLeafSize (icp_resolution_, icp_resolution_, icp_resolution_);
      voxel_grid_icp.filter (*cloud_voxelized_icp);
    }

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr> aligned_models;
    aligned_models.resize (models_merged_->size ());

    for (size_t i = 0; i < models_merged_->size (); i++)
    {
      ConstPointInTPtr model_cloud = models_merged_->at (i)->getAssembled (icp_resolution_);
      typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_aligned (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::transformPointCloud (*model_cloud, *model_aligned, transforms_merged_->at (i));
      aligned_models[i] = model_aligned;
    }

    float icp_max_correspondence_distance_ = 0.02f;
#pragma omp parallel for num_threads(8)
    for (int i = 0; i < static_cast<int> (aligned_models.size ()); i++)
    {
      faat_pcl::rec_3d_framework::VoxelBasedCorrespondenceEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr
                                                                                                              est (
                                                                                                                   new faat_pcl::rec_3d_framework::VoxelBasedCorrespondenceEstimation<
                                                                                                                       pcl::PointXYZRGB,
                                                                                                                       pcl::PointXYZRGB> ());

      pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZRGB>::Ptr
                                                                                      rej (
                                                                                           new pcl::registration::CorrespondenceRejectorSampleConsensus<
                                                                                               pcl::PointXYZRGB> ());

      Eigen::Matrix4f scene_to_model_trans = transforms_merged_->at (i).inverse ();

      boost::shared_ptr < distance_field::PropagationDistanceField<pcl::PointXYZRGB> > dt;
      models_merged_->at (i)->getVGDT (dt);
      //dt->getVoxelizedCloud (model_aligned_icp);

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_aligned_icp (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud;
      dt->getInputCloud (cloud);
      model_aligned_icp.reset (new pcl::PointCloud<pcl::PointXYZRGB> (*cloud));

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_voxelized_icp_transformed (new pcl::PointCloud<pcl::PointXYZRGB> ());
      pcl::transformPointCloud (*cloud_voxelized_icp, *cloud_voxelized_icp_transformed, scene_to_model_trans);

      est->setVoxelRepresentationTarget (dt);
      est->setInputSource (cloud_voxelized_icp_transformed);
      est->setInputTarget (model_aligned_icp);
      est->setMaxCorrespondenceDistance (icp_max_correspondence_distance_);
      est->setMaxColorDistance (-1, -1);

      rej->setInputTarget (model_aligned_icp);
      rej->setMaximumIterations (1000);
      rej->setInlierThreshold (0.005f);
      rej->setInputSource (cloud_voxelized_icp_transformed);

      pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> reg;
      reg.setCorrespondenceEstimation (est);
      reg.addCorrespondenceRejector (rej);
      reg.setInputTarget (model_aligned_icp); //model
      reg.setInputSource (cloud_voxelized_icp_transformed); //scene
      reg.setMaximumIterations (icp_iterations_);
      reg.setEuclideanFitnessEpsilon (1e-12);
      reg.setTransformationEpsilon (0.0001f * 0.0001f);

      pcl::registration::DefaultConvergenceCriteria<float>::Ptr convergence_criteria;
      convergence_criteria = reg.getConvergeCriteria ();
      convergence_criteria->setAbsoluteMSE (1e-12);
      convergence_criteria->setMaximumIterationsSimilarTransforms (15);
      convergence_criteria->setFailureAfterMaximumIterations (false);

      PointInTPtr output (new pcl::PointCloud<pcl::PointXYZRGB> ());
      reg.align (*output);
      Eigen::Matrix4f trans, icp_trans;
      trans = reg.getFinalTransformation () * scene_to_model_trans;
      icp_trans = trans.inverse ();
      transforms_merged_->at (i).setIdentity ();
      transforms_merged_->at (i) = icp_trans;
      pcl::transformPointCloud (*(models_merged_->at (i)->getAssembled (icp_resolution_)), *output, icp_trans);
    }
  }
}*/

boost::shared_ptr<std::vector<ModelTPtr> >
myRecognizer::getModels ()
{
    return models_merged_;
}

boost::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > >
myRecognizer::getTransforms ()
{
    return transforms_merged_;
}
