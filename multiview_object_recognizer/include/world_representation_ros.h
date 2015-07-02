#include <v4r/ORRecognition/include/world_representation.h>
#include "ros/ros.h"

#include "recognition_srv_definitions/multiview_recognize.h"

class worldRepresentationROS : public worldRepresentation
{
    ros::Publisher vis_pc_pub_;

public:
    bool recognizeROSWrapper (recognition_srv_definitions::multiview_recognize::Request & req, recognition_srv_definitions::multiview_recognize::Response & response);

    void set_vis_pc_pub(const ros::Publisher &vis_pc_pub)
    {
        vis_pc_pub_ = vis_pc_pub;
    }
};
