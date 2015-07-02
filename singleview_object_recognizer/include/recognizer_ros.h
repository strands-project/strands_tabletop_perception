#include "recognition_srv_definitions/get_configuration.h"
#include "recognition_srv_definitions/recognize.h"
#include "recognition_srv_definitions/retrain_recognizer.h"

#include <v4r/ORRecognition/include/singleview_object_recognizer.h>

#include <image_transport/image_transport.h>

class RecognizerROS : public Recognizer
{
    using Recognizer::initialize;
    using Recognizer::retrain;
    using Recognizer::recognize;

private:
    bool debug_publish_;
    image_transport::Publisher image_pub_;
    boost::shared_ptr<ros::NodeHandle> n_;
    ros::Publisher vis_pc_pub_;
    ros::ServiceServer recognize_;

public:

    RecognizerROS() : Recognizer()
    {
        debug_publish_ = false;
    }

    bool
    getConfig (recognition_srv_definitions::get_configuration::Request & req,
             recognition_srv_definitions::get_configuration::Response & response)
    {
          response.models_folder.data = models_dir_;
          response.recognition_structure_folder.data = sift_structure_;
    }

    void createDebugImageAndPublish(recognition_srv_definitions::recognize::Response & response,
                                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr & scene_);


    bool retrainROS (recognition_srv_definitions::retrain_recognizer::Request & req,
             recognition_srv_definitions::retrain_recognizer::Response & response);

    bool recognizeROS (recognition_srv_definitions::recognize::Request & req,
                    recognition_srv_definitions::recognize::Response & response);

    void initialize (int argc, char ** argv);
};
