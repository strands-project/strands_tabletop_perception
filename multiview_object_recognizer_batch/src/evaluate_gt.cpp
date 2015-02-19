/*
 * evaluate_gt.cpp
 *
 *  Created on: Sep 10, 2013
 *      Author: aitor
 */

#include <pcl/common/common.h>
#include <pcl/console/parse.h>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <sstream>
#include <fstream>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

std::string MODELS_DIR_;
pcl::visualization::PCLVisualizer vis("test");
int v1,v2;
bool pose_available_ = true;

inline void
visualizeGT(std::map<int, std::vector<std::string> > ids_manual_gt,
            std::map<int, std::vector<Eigen::Matrix4f> > gt_manual_gt,
            std::map<int, std::vector<std::string> > ids_auto_gt,
            std::map<int, std::vector<Eigen::Matrix4f> > gt_auto_gt,
            std::string sequence_directory,
            std::string wname)
{
  vis.createViewPort(0,0,0.5,1,v1);
  vis.createViewPort(0.5,0,1,1,v2);
  vis.setWindowName(wname);

  for(size_t s=0; s < ids_manual_gt.size(); s++)
  {
    std::vector<std::string> gt_ids (ids_manual_gt[s]);
    std::vector<Eigen::Matrix4f> gt_transforms (gt_manual_gt[s]);
    //add scene file
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::stringstream ss;
    ss << sequence_directory << "/cloud_" << std::setw(10) << std::setfill('0') << static_cast<int>(s) << ".pcd";
    std::string scene_file = ss.str();
    pcl::io::loadPCDFile(scene_file, *scene);

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler_rgb_verified (scene);
    vis.addPointCloud(scene, handler_rgb_verified, "cloud", v1);
    //add grount truth

    if(pose_available_)
    {
      for(size_t i=0; i < gt_ids.size(); i++)
      {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr model(new pcl::PointCloud<pcl::PointXYZRGB>);
        std::stringstream model_path;
        model_path << MODELS_DIR_ << "/" << gt_ids[i] << ".pcd";
        std::string ss = model_path.str();
        pcl::io::loadPCDFile(ss, *model);

        Eigen::Matrix4f trans;
        trans = gt_manual_gt[s][i].inverse();
        pcl::transformPointCloud(*model, *model, trans);

        std::stringstream cloud_name;
        cloud_name << "model_" << i;
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler_rgb_verified (model);
        vis.addPointCloud(model, handler_rgb_verified, cloud_name.str(), v1);
      }
    }

    //add automated
    for(size_t i=0; i < ids_auto_gt[s].size(); i++)
    {
      //std::cout << ids_auto_gt[s][i] << std::endl;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr model(new pcl::PointCloud<pcl::PointXYZRGB>);
      std::stringstream model_path;
      model_path << MODELS_DIR_ << "/" << ids_auto_gt[s][i] << ".pcd";
      std::string ss = model_path.str();
      pcl::io::loadPCDFile(ss, *model);

      Eigen::Matrix4f trans;
      trans = gt_auto_gt[s][i];
      pcl::transformPointCloud(*model, *model, trans);

      std::stringstream cloud_name;
      cloud_name << "model_auto_" << i;
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler_rgb_verified (model);
      vis.addPointCloud(model, handler_rgb_verified, cloud_name.str(), v2);
    }

    vis.spin();
    vis.removeAllPointClouds();
  }
}

inline void
readCSVFile (std::string & file,
               std::map<int, std::vector<std::string> > & ids_per_scene, std::map<int, std::vector<Eigen::Matrix4f> > & transforms)
{
  int max_scene = -1;
  {
    std::ifstream in;
    in.open (file.c_str (), std::ifstream::in);

    std::string s = "";
    std::getline (in, s); //ignore first line

    while (std::getline (in, s))
    {
      std::vector<std::string> strs;
      boost::split (strs, s, boost::is_any_of (","));
      int scene = atoi (strs[3].c_str ());
      if(scene > max_scene)
        max_scene = scene;
    }
  }

  std::cout << max_scene << std::endl;
  std::cout << file << std::endl;

  std::ifstream in;
  in.open (file.c_str (), std::ifstream::in);

  std::string s = "";
  std::getline (in, s); //ignore first line
  int last_scene = -1;
  std::vector<std::string> obj_ids;
  std::vector<Eigen::Matrix4f> obj_trans;
  //ids_per_scene.resize(max_scene+1);
  //transforms.resize(max_scene+1);

  while (std::getline (in, s))
  {
    //std::cout << s << std::endl;
    std::vector<std::string> strs;
    boost::split (strs, s, boost::is_any_of (","));
    int scene = atoi (strs[3].c_str ());
    std::string obj_id = strs[4];
    Eigen::Matrix4f trans;
    trans.setIdentity ();
    trans (0, 0) = static_cast<float> (atof (strs[5].c_str ()));
    trans (0, 1) = static_cast<float> (atof (strs[6].c_str ()));
    trans (0, 2) = static_cast<float> (atof (strs[7].c_str ()));
    trans (1, 0) = static_cast<float> (atof (strs[8].c_str ()));
    trans (1, 1) = static_cast<float> (atof (strs[9].c_str ()));
    trans (1, 2) = static_cast<float> (atof (strs[10].c_str ()));
    trans (2, 0) = static_cast<float> (atof (strs[11].c_str ()));
    trans (2, 1) = static_cast<float> (atof (strs[12].c_str ()));
    trans (2, 2) = static_cast<float> (atof (strs[13].c_str ()));
    trans (0, 3) = static_cast<float> (atof (strs[14].c_str ()));
    trans (1, 3) = static_cast<float> (atof (strs[15].c_str ()));
    trans (2, 3) = static_cast<float> (atof (strs[16].c_str ()));

    if(obj_id.compare("object_x") != 0) {
      std::map<int, std::vector<std::string> >::iterator it_ids;
      it_ids = ids_per_scene.find(scene);
      if(it_ids == ids_per_scene.end())
      {
        //does not exist, insert in map
        std::vector<std::string> ids;
        ids.push_back(obj_id);
        std::vector<Eigen::Matrix4f> transformss;
        transformss.push_back(trans);

        ids_per_scene.insert(std::make_pair(scene, ids));
        transforms.insert(std::make_pair(scene, transformss));
      }
      else
      {
        it_ids->second.push_back (obj_id);
        std::map<int, std::vector<Eigen::Matrix4f> >::iterator it_trans = transforms.find(scene);
        it_trans->second.push_back (trans);
      }
    }
  }

  //std::cout << transforms.size () << " " << ids_per_scene.size () << std::endl;
}

namespace bf = boost::filesystem;

namespace evalute_utils
{
    void
    getFilesInDirectory (bf::path & dir, std::string & rel_path_so_far, std::vector<std::string> & relative_paths, std::string & ext)
    {
      bf::directory_iterator end_itr;
      for (bf::directory_iterator itr (dir); itr != end_itr; ++itr)
      {
        //check if its a directory, then get models in it
        if (bf::is_directory (*itr))
        {
#if BOOST_FILESYSTEM_VERSION == 3
          std::string so_far = rel_path_so_far + (itr->path ().filename ()).string () + "/";
#else
          std::string so_far = rel_path_so_far + (itr->path ()).filename () + "/";
#endif

          bf::path curr_path = itr->path ();
          getFilesInDirectory (curr_path, so_far, relative_paths, ext);
        }
        else
        {
          //check that it is a ply file and then add, otherwise ignore..
          std::vector<std::string> strs;
#if BOOST_FILESYSTEM_VERSION == 3
          std::string file = (itr->path ().filename ()).string ();
#else
          std::string file = (itr->path ()).filename ();
#endif

          boost::split (strs, file, boost::is_any_of ("."));
          std::string extension = strs[strs.size () - 1];

          if (extension.compare (ext) == 0)
          {
#if BOOST_FILESYSTEM_VERSION == 3
            std::string path = rel_path_so_far + (itr->path ().filename ()).string ();
#else
            std::string path = rel_path_so_far + (itr->path ()).filename ();
#endif

            relative_paths.push_back (path);
          }
        }
      }
    }

    void
    getDirectories (bf::path & dir, std::vector<std::string> & relative_paths)
    {
      bf::directory_iterator end_itr;
      for (bf::directory_iterator itr (dir); itr != end_itr; ++itr)
      {
        //check if its a directory, then get models in it
        if (bf::is_directory (*itr))
        {
#if BOOST_FILESYSTEM_VERSION == 3
            std::string path = (itr->path ().filename ()).string ();
#else
            std::string path = (itr->path ()).filename ();
#endif
          relative_paths.push_back(path);
        }
      }
    }
}

struct GTScene
{
  int n_objects_;
  std::vector<std::string> ids_;
  std::vector<Eigen::Matrix4f> transforms_;
  std::vector<Eigen::Vector4f> centroids_;
};

struct EvalResult {
  int fp_;
  int tp_;
  int fn_;
  float precission_;
  float recall_;
  double time_seconds_;
  float centroids_dist_;
  float rms_shape_;
  float rms_color_;
  std::vector<float> centroid_distances_;
  std::vector<float> rotation_distances_;
  std::vector<float> rms_shape_distances_;
  std::vector<float> rms_color_distances_;
};

void evaluate(std::map<int, std::vector<std::string> > ids_manual_gt,
               std::map<int, std::vector<Eigen::Matrix4f> > gt_manual_gt,
               std::map<int, std::vector<std::string> > ids_auto_gt,
               std::map<int, std::vector<Eigen::Matrix4f> > gt_auto_gt,
               EvalResult & ev,
               bool pose_available=true)
{
  std::cout << ids_manual_gt.size() << " " << ids_auto_gt.size() << std::endl;

  ev.tp_ = ev.fp_ = ev.fn_ = 0;

  //for each scene in the sequence compare ids and poses if available
  for(size_t s=0; s < ids_manual_gt.size(); s++)
  {
    EvalResult er; //evalResult for one scene
    std::vector<std::string> invalid_ids;
    er.fn_ = er.tp_ = er.fp_ = 0;
    std::vector<std::string> gt_ids (ids_manual_gt[s]);
    std::vector<Eigen::Matrix4f> gt_transforms (gt_manual_gt[s]);

    size_t valid = 0;
    for (size_t i = 0; i < gt_ids.size (); i++)
    {
      bool invalid = false;
      for (size_t j = 0; j < invalid_ids.size () && !invalid; j++)
      {
        if (invalid_ids[j].compare (gt_ids[i]) == 0)
        {
          invalid = true;
        }
      }

      if (!invalid)
      {
        gt_ids[valid] = gt_ids[i];
        valid++;
      }
    }

    gt_ids.resize (valid);
    std::vector<bool> its_a_tp (ids_auto_gt[s].size (), false);
    std::vector<Eigen::Matrix4f> trans_gt;
    //compare results with gt_s
    for (size_t i = 0; i < ids_auto_gt[s].size (); i++)
    {
      std::string id = ids_auto_gt[s][i];
      bool found = false;
      for (size_t j = 0; j < gt_ids.size () && !found; j++)
      {
        if (id.compare (gt_ids[j]) == 0)
        {
          if(!pose_available)
          {
            er.tp_++;
            found = true;
            gt_ids.erase (gt_ids.begin () + j);
            its_a_tp[i] = true;
          }
          else
          {
            //check if pose is correct, check translation values
            Eigen::Matrix4f gt_trans = gt_transforms[j].inverse ();
            Eigen::Vector3f gt_translation = gt_trans.block<3,1>(0,3);

            Eigen::Matrix4f trans = gt_auto_gt[s][i]; //.inverse(); //Do we need to invert here?
            Eigen::Vector3f translation = trans.block<3,1>(0,3);

            if( (gt_translation - translation).norm() < 0.1f)
            {
              er.tp_++;
              found = true;
              gt_ids.erase (gt_ids.begin () + j);
              its_a_tp[i] = true;

              trans_gt.push_back (gt_trans);
              gt_transforms.erase (gt_transforms.begin () + j);
            }
          }
        }
      }

      er.fn_ = gt_ids.size ();

      if (!found)
      {
        er.fp_++;
        trans_gt.push_back (Eigen::Matrix4f::Identity ());
      }
    }

    std::cout << "TP:" << er.tp_ << " FP:" << er.fp_ << " FN:" << er.fn_ << std::endl;

    ev.tp_ += er.tp_;
    ev.fp_ += er.fp_;
    ev.fn_ += er.fn_;
  }
}

int
main (int argc, char **argv)
{
  std::string test_directory;
  bool visualize_mv, visualize_sv;
  visualize_mv = visualize_sv = false;

  pcl::console::parse_argument (argc, argv, "-test_directory", test_directory);
  pcl::console::parse_argument (argc, argv, "-models_dir", MODELS_DIR_);
  pcl::console::parse_argument (argc, argv, "-visualize_mv", visualize_mv);
  pcl::console::parse_argument (argc, argv, "-visualize_sv", visualize_sv);
  pcl::console::parse_argument (argc, argv, "-pose_available", pose_available_);

  bf::path test_dir = test_directory;
  std::vector<std::string> directories;
  evalute_utils::getDirectories(test_dir, directories);

  EvalResult accumulated;
  accumulated.tp_ = accumulated.fp_ = accumulated.fn_ = 0;
  accumulated.centroids_dist_ = accumulated.rms_color_ = accumulated.rms_shape_ = 0.f;
  accumulated.time_seconds_ = 0;

  EvalResult accumulated_sv;
  accumulated_sv.tp_ = accumulated_sv.fp_ = accumulated_sv.fn_ = 0;
  accumulated_sv.centroids_dist_ = accumulated_sv.rms_color_ = accumulated_sv.rms_shape_ = 0.f;
  accumulated_sv.time_seconds_ = 0;

  std::sort(directories.begin(), directories.end());
  std::string wname;

  for(size_t i=0; i < directories.size(); i++)
  {
    std::stringstream dir_path;
    dir_path << test_directory << "/" << directories[i];
    std::cout << dir_path.str() << std::endl;

    std::vector<std::string> strs;
    boost::split (strs, directories[i], boost::is_any_of ("/"));
    std::string extension = strs[strs.size () - 1];

    std::stringstream csv_file;
    csv_file << dir_path.str() << "/" << extension << ".bag.csv";

    std::string csv = csv_file.str();
    std::map<int, std::vector<std::string> > ids_per_scene_manual_gt;
    std::map<int, std::vector<Eigen::Matrix4f> > gt_per_scene_manual_gt;
    readCSVFile(csv, ids_per_scene_manual_gt, gt_per_scene_manual_gt);
    std::map<int, std::vector<std::string> > ids_per_scene_auto_mv;
    std::map<int, std::vector<Eigen::Matrix4f> > gt_per_scene_auto_mv;
    //read automated CSV files (SV)
    {
      std::stringstream csv_file;
      csv_file << dir_path.str() << "/automated_MV_" << extension << ".bag.csv";

      std::string csv = csv_file.str();
      readCSVFile(csv, ids_per_scene_auto_mv, gt_per_scene_auto_mv);
    }

    //compare results MV
    EvalResult er;
    evaluate(ids_per_scene_manual_gt, gt_per_scene_manual_gt, ids_per_scene_auto_mv, gt_per_scene_auto_mv, er, pose_available_);
    accumulated.tp_ += er.tp_;
    accumulated.fp_ += er.fp_;
    accumulated.fn_ += er.fn_;

    accumulated.precission_ = accumulated.tp_ / static_cast<float> (accumulated.tp_ + accumulated.fp_);
    accumulated.recall_ = accumulated.tp_ / static_cast<float> (accumulated.tp_ + accumulated.fn_);
    //accumulated.time_seconds_ += er.time_seconds_;

    std::cout << "MV TP:" << accumulated.tp_ << std::endl;
    std::cout << "MV FP:" << accumulated.fp_ << std::endl;
    std::cout << "MV FN:" << accumulated.fn_ << std::endl;
    std::cout << "MV Precission and recall:" << accumulated.precission_ << " " << accumulated.recall_ << std::endl;

    if(visualize_mv)
    {
      wname = "multi view";
      visualizeGT(ids_per_scene_manual_gt, gt_per_scene_manual_gt, ids_per_scene_auto_mv, gt_per_scene_auto_mv,
                  dir_path.str(), wname);
    }

    std::map<int, std::vector<std::string> > ids_per_scene_auto_sv;
    std::map<int, std::vector<Eigen::Matrix4f> > gt_per_scene_auto_sv;
    //read automated CSV files (SV)
    {
      std::stringstream csv_file;
      csv_file << dir_path.str() << "/automated_SV_" << extension << ".bag.csv";
      std::cout << csv_file.str() << std::endl;

      std::string csv = csv_file.str();
      readCSVFile(csv, ids_per_scene_auto_sv, gt_per_scene_auto_sv);
    }

    EvalResult er_sv;
    evaluate(ids_per_scene_manual_gt, gt_per_scene_manual_gt, ids_per_scene_auto_sv, gt_per_scene_auto_sv, er_sv, pose_available_);
    accumulated_sv.tp_ += er_sv.tp_;
    accumulated_sv.fp_ += er_sv.fp_;
    accumulated_sv.fn_ += er_sv.fn_;

    accumulated_sv.precission_ = accumulated_sv.tp_ / static_cast<float> (accumulated_sv.tp_ + accumulated_sv.fp_);
    accumulated_sv.recall_ = accumulated_sv.tp_ / static_cast<float> (accumulated_sv.tp_ + accumulated_sv.fn_);
    //accumulated.time_seconds_ += er.time_seconds_;

    std::cout << "SV TP:" << accumulated_sv.tp_ << std::endl;
    std::cout << "SV FP:" << accumulated_sv.fp_ << std::endl;
    std::cout << "SV FN:" << accumulated_sv.fn_ << std::endl;
    std::cout << "SV Precission and recall:" << accumulated_sv.precission_ << " " << accumulated_sv.recall_ << std::endl;

    if(visualize_sv)
    {
      wname = "single view";
      visualizeGT(ids_per_scene_manual_gt, gt_per_scene_manual_gt, ids_per_scene_auto_sv, gt_per_scene_auto_sv,
                dir_path.str(), wname);
    }
  }
}
