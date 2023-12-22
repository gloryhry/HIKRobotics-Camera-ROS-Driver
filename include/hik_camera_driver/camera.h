#ifndef __HIK_CAMERA_DRIVER_CAMERA_H__
#define __HIK_CAMERA_DRIVER_CAMERA_H__
#include <mvs/MvCameraControl.h>
#include <vector>
#include <pthread.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float32.h>

namespace HIKCAMERA
{
    // cv::Mat frame;            // 临时存放当前帧
    extern sensor_msgs::ImagePtr frame; // 临时存放当前帧
    extern pthread_mutex_t mutex;       // 存放帧的锁
    extern bool frame_empty;            // 用于标志是否有新帧未发布
    extern float exposure_time_set;     // 用于存放下次设置的曝光时间
    extern int exposure_auto;          // 是否自动曝光

    class Hik_camera_base
    {
    public:
        Hik_camera_base(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
        ~Hik_camera_base(){};
        bool set_camera(MV_CC_DEVICE_INFO &camera);
        bool set_params();
        bool setEnumValue(std::string name, unsigned int value);
        bool setBoolValue(std::string name, bool value);
        bool setFloatValue(std::string name, float value);
        bool setStringValue(std::string name, std::string value);
        bool setIntValue(std::string name, unsigned int value);
        bool setCommandValue(std::string name);
        bool getEnumValue(std::string name, MVCC_ENUMVALUE &value);
        bool getBoolValue(std::string name, bool &value);
        bool getFloatValue(std::string name, float &value);
        bool getStringValue(std::string name, std::string &value);
        bool getIntValue(std::string name, unsigned int &value);
        bool setFrameRate(float frame_rate);
        bool ImageStream();
        void ImagePub();
        void stopStream();
        static void *WorkThread(void *p_handle);

        bool changeExposureTime(float value);
        void exposure_callback(const std_msgs::Float32ConstPtr msg);

    public:
        int nRet = -1;
        void *m_handle = NULL;
        MV_CC_DEVICE_INFO m_stDevInfo = {0};
        pthread_t nThreadID;
        std::string camera_name;
        ros::NodeHandle private_nh;
        boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;
        image_transport::CameraPublisher camera_pub;
        ros::Subscriber exposure_sub;
        bool exposure_control = false;             // 程序控制曝光（当外部触发,无法使用自动曝光时启用）
        float exposure_time_up, exposure_time_low; // 曝光时间上下限
        float scale = 1.03;                        // 曝光时间变化率
        float light_set;                           // 控制曝光指定亮度
    };

} // namespace HIKCAMERA
#endif // __HIK_CAMERA_DRIVER_CAMERA_H__