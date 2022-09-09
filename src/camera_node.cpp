#include <camera/CameraConfig.h>
#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camera_ov9281.h"

class CameraNode {
  ros::NodeHandle nh_;
  image_transport::CameraPublisher image_pub_;
  sensor_msgs::Image image_msg_;
  cv_bridge::CvImage bridge_image_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> info_manager_;
  std::string camera_name_;
  std::string camera_info_url_;
  std::string base_topic_;
  double framerate_;
  int camera_mode_;
  double cam_roi_crop_x_;
  double cam_roi_crop_y_;

  dynamic_reconfigure::Server<camera::CameraConfig> server_;
  dynamic_reconfigure::Server<camera::CameraConfig>::CallbackType f_;

  CameraOV9281 camera_;

 public:
  CameraNode() : nh_("~") {
    // read parameters
    nh_.param<std::string>("base_topic", base_topic_, "image");
    nh_.param<std::string>("camera_name", camera_name_, "camera");
    nh_.param<std::string>("camera_frame_id", bridge_image_.header.frame_id,
                           "camera_frame");
    nh_.param<std::string>("camera_info_url", camera_info_url_, "");
    nh_.param<double>("framerate", framerate_, 20.0);
    nh_.param<int>("camera_resolution_mode", camera_mode_, 1);
    nh_.param<double>("camera_roi_crop_x", cam_roi_crop_x_, 0.0);
    nh_.param<double>("camera_roi_crop_y", cam_roi_crop_y_, 0.0);
    
    camera_.SetMode(camera_mode_);
    //modes for OV9281
    //mode: 0, width: 1280, height: 800, pixelformat: GREY, desc:  1 lan raw8 60fps
    //mode: 1, width: 1280, height: 720, pixelformat: GREY, desc: (null)
    //mode: 2, width: 640, height: 400, pixelformat: GREY, desc: (null)
    //mode: 3, width: 320, height: 200, pixelformat: GREY, desc: (null)
    //mode: 4, width: 160, height: 100, pixelformat: GREY, desc: (null)

    image_transport::ImageTransport it(nh_);
    
    

    image_pub_ = it.advertiseCamera(base_topic_, 1);

    // reset camera info manager according to params.
    info_manager_.reset(new camera_info_manager::CameraInfoManager(
        nh_, camera_name_, camera_info_url_));

    if (!info_manager_->isCalibrated()) {
      sensor_msgs::CameraInfo camera_info;
      camera_info.header.frame_id = bridge_image_.header.frame_id;
      camera_info.width = camera_.GetWidth();
      camera_info.height = camera_.GetHeight();
      info_manager_->setCameraInfo(camera_info);
    }
    
    f_ = boost::bind(&CameraNode::ReconfigureCallback, this, _1, _2);
    server_.setCallback(f_);

    ROS_INFO("Camera node initialized, mode: %d @ %.1fHz, roi: %.0fx%.0fpx", 
              camera_mode_, framerate_, 
              camera_.GetWidth() * (1-cam_roi_crop_x_), 
              camera_.GetHeight() * (1-cam_roi_crop_y_));
  }

  void ReconfigureCallback(camera::CameraConfig &config, uint32_t level) {
    if (camera_.SetExposure(config.Exposure)) {
      ROS_ERROR("Could not set exposure to %d.", config.Exposure);
    }
    if (camera_.SetGain(config.Gain)) {
      ROS_ERROR("Could not set gain to %d.", config.Gain);
    }
    camera_.GetExposure(&config.Exposure);
    camera_.GetGain(&config.Gain);
  }

  bool CaptureAndPublish() {
    cv::Mat *cv_image;
    cv::Rect roi(camera_.GetWidth() * (cam_roi_crop_x_/2), camera_.GetHeight() * (cam_roi_crop_y_/2),
                 camera_.GetWidth() * (1-cam_roi_crop_x_), camera_.GetHeight() * (1-cam_roi_crop_y_));  
    
    camera_.Capture();
    cv_image = camera_.GetImage();

    cv::Mat cv_cropped = (*cv_image)(roi);

    bridge_image_.encoding = "mono8";
    bridge_image_.image = cv_cropped;
    bridge_image_.toImageMsg(image_msg_);
    sensor_msgs::CameraInfoPtr info(
        new sensor_msgs::CameraInfo(info_manager_->getCameraInfo()));
    info->header.frame_id = image_msg_.header.frame_id;
    image_pub_.publish(image_msg_, *info, ros::Time::now());
    return true;
  }

  void Run() {
    ros::Rate loop_rate(framerate_);
    while (nh_.ok()) {
      CaptureAndPublish();
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "camera");
  CameraNode node;
  node.Run();
  return 0;
}
