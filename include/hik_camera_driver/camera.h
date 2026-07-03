#ifndef __HIK_CAMERA_DRIVER_CAMERA_H__
#define __HIK_CAMERA_DRIVER_CAMERA_H__
#include <MvCameraControl.h>
#include <vector>
#include <cstdint>
#include <cmath>
#include <algorithm>
#include <pthread.h>
#include <atomic>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float32.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

namespace HIKCAMERA
{
    // LiDAR 共享内存时间戳结构体 (与 livox_ros_driver2/src/lddc.h:39-42 一致)
    struct time_stamp { int64_t high; int64_t low; };
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
        bool getIntValueEx(std::string name, int64_t &value);
        bool setFrameRate(float frame_rate);
        bool ImageStream();
        void ImagePub();
        void stopStream();
        static void *WorkThread(void *p_user);

        bool restart();            // 取图超时/失败后重建句柄: stopStream + set_camera + set_params + ImageStream
        void check_and_restart();  // 主循环调用: 检测 need_restart_ 并按退避策略重启

        bool changeExposureTime(float value);
        void exposure_callback(const std_msgs::Float32ConstPtr msg);

        bool readTickFrequency();

        // LiDAR 共享内存时间戳 (可选, 与相机硬件时间戳标定方案并存)
        bool openLidarTimestampShm();   // 打开/映射 /home/{user}/timeshare
        void closeLidarTimestampShm();  // 解除映射/关闭 fd (重启与析构时调用)
        ros::Time getLidarTimestamp();  // 读 pointt->low -> ros::Time, 失败返回 ros::Time()

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

        // 时间戳标定相关
        int64_t tick_frequency_ = 0;           // 相机时间戳 tick 频率 (Hz)
        double device_to_wall_offset_ = 0.0;   // 标定偏移量 (秒)
        bool timestamp_calibrated_ = false;    // 标定是否完成
        int calib_frame_count_ = 50;           // 标定帧数
        double calib_iqr_multiplier_ = 1.5;    // IQR 离群值倍数

        // LiDAR 共享内存时间戳相关 (可选, 与相机硬件时间戳标定方案并存)
        bool  use_lidar_timestamp_ = false;  // 是否用 LiDAR 共享内存时间戳
        void *lidar_shm_ptr_ = nullptr;       // mmap 映射的 time_stamp 指针
        int   lidar_shm_fd_ = -1;            // timeshare 文件描述符
        bool  lidar_shm_ok_ = false;         // mmap 是否成功

        // 取图超时重启相关
        // WorkThread 写 / 主线程读 (原子)
        std::atomic<bool> need_restart_{false};   // WorkThread -> 主线程: 请求重启句柄
        std::atomic<bool> stop_requested_{false}; // 主线程 -> WorkThread: 请求退出
        std::atomic<bool> thread_running_{false}; // 诊断: WorkThread 是否在运行
        // 仅主线程访问 (无需原子)
        bool thread_started_ = false;             // pthread_join 守卫
        int restart_attempts_ = 0;                // 连续重启失败次数 (退避用)
        ros::Time last_restart_time_;             // 上次重启时间 (退避用)
        // 参数
        int grab_timeout_retry_threshold_ = 5;    // 连续取图超时阈值 (0=禁用)
        int restart_max_retries_ = 0;             // 重启最大尝试次数 (0=无限, 指数退避封顶 60s)
    };

} // namespace HIKCAMERA
#endif // __HIK_CAMERA_DRIVER_CAMERA_H__