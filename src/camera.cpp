#include <camera.h>

namespace HIKCAMERA
{
    namespace
    {
        constexpr char kLidarTimestampPath[] = "/tmp/livox_timeshare";

        class ImageBufferGuard
        {
        public:
            ImageBufferGuard(void *handle, MV_FRAME_OUT &frame)
                : handle_(handle), frame_(frame)
            {
            }

            ~ImageBufferGuard()
            {
                const int ret = MV_CC_FreeImageBuffer(handle_, &frame_);
                if (MV_OK != ret)
                {
                    ROS_WARN_STREAM_THROTTLE(5.0, "MV_CC_FreeImageBuffer failed! nRet [0x"
                                             << std::hex << ret << "]");
                }
            }

            ImageBufferGuard(const ImageBufferGuard &) = delete;
            ImageBufferGuard &operator=(const ImageBufferGuard &) = delete;

        private:
            void *handle_;
            MV_FRAME_OUT &frame_;
        };
    }

    sensor_msgs::ImagePtr frame; // 临时存放当前帧
    pthread_mutex_t mutex;       // 存放帧的锁
    bool frame_empty = true;     // 用于标志是否有新帧未发布
    std::atomic<float> exposure_time_set{0.0f}; // 用于存放下次设置的曝光时间
    int exposure_auto;           // 是否自动曝光

    Hik_camera_base::Hik_camera_base(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
    {
        this->private_nh = private_nh;
        std::string cam_info_url;
        private_nh.param<std::string>("camera_name", camera_name, "camera");
        private_nh.param<std::string>("Camera/cam_info_url", cam_info_url, "");
        image_transport::ImageTransport it(nh);
        cinfo_.reset(new camera_info_manager::CameraInfoManager(nh, camera_name, cam_info_url));
        camera_pub = it.advertiseCamera(camera_name + "/image", 1);
        // exposure_sub = nh.subscribe<std_msgs::Float32>(camera_name + "/set_exposure", 10, boost::bind(&Hik_camera_base::exposure_callback, this, _1));
        // 帧互斥量一次性初始化 (移出 ImageStream, 避免重启时对已初始化 mutex 重复 init 触发 POSIX UB)
        pthread_mutex_init(&mutex, NULL);
    }

    Hik_camera_base::~Hik_camera_base()
    {
        stopStream();
    }

    bool Hik_camera_base::set_params()
    {
        if (!m_handle || !device_open_)
        {
            ROS_ERROR_STREAM("Cannot configure camera parameters before opening the device.");
            return false;
        }

        float frame_rate;
        bool trigger_mode;
        int trigger_line;
        int trigger_action;
        float trigger_delay;
        bool trigger_cache;
        int Exposure;
        float Exposure_time;
        int ExposureTimeUp, ExposureTimeLow;
        int Gain_mode;
        float Gain_value;
        bool Digital_shift_mode;
        float Digital_shift;
        int brightneess;
        bool Gamma;
        float Gamma_value;
        int Gamma_selector;
        private_nh.param<float>("Camera/frame_rate", frame_rate, 10.0);
        private_nh.param<bool>("Camera/Trigger", trigger_mode, false);
        private_nh.param<int>("Camera/Tigger_line", trigger_line, 2);
        private_nh.param<int>("Camera/Trigger_action", trigger_action, 0);
        private_nh.param<float>("Camera/Trigger_delay", trigger_delay, 0.0);
        private_nh.param<bool>("Camera/Trigger_cache_enable", trigger_cache, false);

        // 时间戳来源: false=相机硬件时间戳(默认), true=LiDAR 共享内存时间戳 (软同步, 蹭 livox 的 base_time)
        private_nh.param<bool>("Camera/use_lidar_timestamp", use_lidar_timestamp_, false);

        private_nh.param<int>("Camera/Exposure", Exposure, 2);
        private_nh.param<float>("Camera/Exposure_time", Exposure_time, 10000.0);
        private_nh.param<int>("Camera/ExposureTimeUp", ExposureTimeUp, 6000);
        private_nh.param<int>("Camera/ExposureTimeLow", ExposureTimeLow, 100);
        private_nh.param<int>("Camera/Gain_mode", Gain_mode, 2);
        private_nh.param<float>("Camera/Gain_value", Gain_value, 0.0);
        private_nh.param<bool>("Camera/Digital_shift_mode", Digital_shift_mode, true);
        private_nh.param<float>("Camera/Digital_shift", Digital_shift, 3.0);
        private_nh.param<int>("Camera/brightneess", brightneess, 100);
        private_nh.param<bool>("Camera/GammaEnable", Gamma, false);
        private_nh.param<float>("Camera/Gamma_value", Gamma_value, 1.0);
        private_nh.param<int>("Camera/Gamma_selector", Gamma_selector, 1);
        private_nh.param<bool>("Camera/Exposure_control", exposure_control, false);

        if (ExposureTimeLow > ExposureTimeUp)
        {
            ROS_ERROR_STREAM("Invalid configured exposure range: ["
                             << ExposureTimeLow << ", " << ExposureTimeUp << "]");
            return false;
        }

        exposure_time_set.store(Exposure_time, std::memory_order_relaxed);
        exposure_auto = Exposure;
        exposure_time_low = ExposureTimeLow;
        exposure_time_up = ExposureTimeUp;
        light_set = brightneess;

        if (!setEnumValue("AcquisitionMode", MV_ACQ_MODE_CONTINUOUS))
            return false;
        ROS_INFO_STREAM("AcquisitionMode set to Continuous.");
        // 设置触发
        if (trigger_mode)
        {
            if (!setEnumValue("TriggerMode", MV_TRIGGER_MODE_ON))
                return false;
            ROS_INFO_STREAM("TriggerMode set to ON");
            if (trigger_line >= 0 && trigger_line <= 8 && trigger_line != 5 && trigger_line != 6)
            {
                if (!setEnumValue("TriggerSource", trigger_line))
                    return false;
                ROS_INFO_STREAM("TriggerSource set to " << trigger_line);
                // TriggerSource
                // 0:Line0
                // 1:Line1
                // 2:Line2
                // 3.Line3
                // 4:Counter0
                // 7:Software
                // 8:FrequencyConverter
                if (trigger_action >= 0 && trigger_action <= 3)
                {
                    if (!setEnumValue("TriggerActivation", trigger_action))
                        return false;
                    ROS_INFO_STREAM("TriggerActivation set to " << trigger_action);
                }
                else
                {
                    ROS_ERROR_STREAM("Not Exist Trigger Action: " << trigger_action);
                    return false;
                }
                if (!setFloatValue("TriggerDelay", trigger_delay))
                    return false;
                ROS_INFO_STREAM("TriggerDelay set to " << trigger_delay << "us.");
                nRet = MV_CC_SetBoolValue(m_handle, "TriggerCacheEnable", trigger_cache);
                if (MV_OK == nRet)
                    ROS_INFO_STREAM("TriggerCacheEnable set to " << trigger_cache);
                else
                    ROS_WARN_STREAM("TriggerCacheEnable is unavailable; continuing without it. nRet [0x"
                                    << std::hex << nRet << "]");
            }
            else
            {
                ROS_ERROR_STREAM("Not Exist Trigger Source: " << trigger_line);
                return false;
            }
        }
        else
        {
            if (!setEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF))
                return false;
            ROS_INFO_STREAM("TriggerMode set to OFF");
        }
        // 设置帧率
        if (!setFrameRate(frame_rate))
            return false;
        // 设置曝光
        if (Exposure == 0)
        {
            if (!setEnumValue("ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF))
                return false;
            ROS_INFO_STREAM("ExposureAuto set to OFF");

            MVCC_FLOATVALUE exposure_range = {0};
            nRet = MV_CC_GetFloatValue(m_handle, "ExposureTime", &exposure_range);
            if (MV_OK != nRet)
            {
                ROS_ERROR("Get ExposureTime range failed! nRet [%x]\n", nRet);
                return false;
            }
            exposure_time_sdk_min_ = exposure_range.fMin;
            exposure_time_sdk_max_ = exposure_range.fMax;
            if (!std::isfinite(exposure_time_sdk_min_) ||
                !std::isfinite(exposure_time_sdk_max_) ||
                exposure_time_sdk_min_ > exposure_time_sdk_max_)
            {
                ROS_ERROR_STREAM("Invalid SDK ExposureTime range: ["
                                 << exposure_time_sdk_min_ << ", "
                                 << exposure_time_sdk_max_ << "]");
                return false;
            }

            if (exposure_control)
            {
                exposure_time_low = std::max(exposure_time_sdk_min_,
                                             static_cast<float>(ExposureTimeLow));
                exposure_time_up = std::min(exposure_time_sdk_max_,
                                            static_cast<float>(ExposureTimeUp));
            }
            else
            {
                exposure_time_low = exposure_time_sdk_min_;
                exposure_time_up = exposure_time_sdk_max_;
            }

            if (exposure_time_low > exposure_time_up)
            {
                ROS_ERROR_STREAM("Configured exposure range does not overlap SDK range: configured=["
                                 << ExposureTimeLow << ", " << ExposureTimeUp
                                 << "], sdk=[" << exposure_time_sdk_min_ << ", "
                                 << exposure_time_sdk_max_ << "]");
                return false;
            }
            exposure_time_range_ready_ = true;
            ROS_INFO_STREAM("ExposureTime SDK range: [" << exposure_time_sdk_min_
                            << ", " << exposure_time_sdk_max_ << "]us, effective range: ["
                            << exposure_time_low << ", " << exposure_time_up << "]us");
            if (!changeExposureTime(Exposure_time))
                return false;
            ROS_INFO_STREAM("ExposureTime set to "
                            << exposure_time_set.load(std::memory_order_relaxed) << "us");
        }
        else if (Exposure > 0 && Exposure <= 2)
        {
            exposure_time_range_ready_ = false;
            if (exposure_control)
            {
                ROS_WARN_STREAM("Exposure_control requires Exposure=0; disabling program-controlled exposure.");
                exposure_control = false;
            }
            if (!setEnumValue("ExposureAuto", Exposure) ||
                !setIntValue("AutoExposureTimeLowerLimit", ExposureTimeLow) ||
                !setIntValue("AutoExposureTimeUpperLimit", ExposureTimeUp))
            {
                return false;
            }
            ROS_INFO_STREAM("ExposureAuto set to " << Exposure);
            ROS_INFO_STREAM("AutoExposureTimeLowerLimit set to " << ExposureTimeLow << "us");
            ROS_INFO_STREAM("AutoExposureTimeUpperLimit set to " << ExposureTimeUp << "us");
        }
        else
        {
            ROS_ERROR_STREAM("Not Exist Exposure Mode: " << Exposure);
            return false;
        }
        // 设置Gain
        if (Gain_mode == 0)
        {
            if (!setEnumValue("GainAuto", MV_GAIN_MODE_OFF) ||
                !setFloatValue("Gain", Gain_value))
            {
                return false;
            }
            ROS_INFO_STREAM("GainAuto set to OFF");
            ROS_INFO_STREAM("Gain set to " << Gain_value << "dB");
        }
        else if (Gain_mode > 0 && Gain_mode <= 2)
        {
            if (!setEnumValue("GainAuto", Gain_mode))
                return false;
            ROS_INFO_STREAM("GainAuto set to " << Gain_mode);
        }
        else
        {
            ROS_ERROR_STREAM("Not Exist Gain Mode: " << Gain_mode);
            return false;
        }
        // 设置白平衡
        nRet = MV_CC_SetEnumValue(m_handle, "BalanceWhiteAuto",
                                 MV_BALANCEWHITE_AUTO_CONTINUOUS);
        if (MV_OK == nRet)
            ROS_INFO_STREAM("BalanceWhiteAuto set to Continuous");
        else
            ROS_WARN_STREAM("BalanceWhiteAuto is unavailable; continuing without it. nRet [0x"
                            << std::hex << nRet << "]");
        // 设置图像像素格式，不同型号的相机，支持的像素格式有差异，以实际的为准
        // 0x01080001:Mono8
        // 0x01100003:Mono10
        // 0x010C0004:Mono10Packed
        // 0x01100005:Mono12
        // 0x010C0006:Mono12Packed
        // 0x01100007:Mono16
        // 0x02180014:RGB8Packed
        // 0x02100032:YUV422_8
        // 0x0210001F:YUV422_8_UYVY
        // 0x01080008:BayerGR8
        // 0x01080009:BayerRG8
        // 0x0108000A:BayerGB8
        // 0x0108000B:BayerBG8
        // 0x0110000e:BayerGB10
        // 0x01100012:BayerGB12
        // 0x010C002C:BayerGB12Packed
        if (!setEnumValue("PixelFormat", PixelType_Gvsp_BayerRG10_Packed))
            return false;
        // 设置亮度
        nRet = MV_CC_SetIntValueEx(m_handle, "Brightness", brightneess);
        if (MV_OK == nRet)
            ROS_INFO_STREAM("Brightness set to " << brightneess);
        else
            ROS_WARN_STREAM("Brightness is unavailable; continuing without it. nRet [0x"
                            << std::hex << nRet << "]");
        // 设置数字偏移
        if (Digital_shift_mode)
        {
            nRet = MV_CC_SetBoolValue(m_handle, "DigitalShiftEnable", true);
            if (MV_OK == nRet)
            {
                ROS_INFO_STREAM("DigitalShiftEnable set to ON");
                nRet = MV_CC_SetFloatValue(m_handle, "DigitalShift", Digital_shift);
                if (MV_OK == nRet)
                    ROS_INFO_STREAM("DigitalShift set to " << Digital_shift);
                else
                    ROS_WARN_STREAM("DigitalShift is unavailable; continuing without it. nRet [0x"
                                    << std::hex << nRet << "]");
            }
            else
            {
                ROS_WARN_STREAM("DigitalShiftEnable is unavailable; continuing without it. nRet [0x"
                                << std::hex << nRet << "]");
            }
        }
        else
        {
            nRet = MV_CC_SetBoolValue(m_handle, "DigitalShiftEnable", false);
            if (MV_OK == nRet)
                ROS_INFO_STREAM("DigitalShiftEnable set to OFF");
            else
                ROS_WARN_STREAM("DigitalShiftEnable is unavailable; continuing without it. nRet [0x"
                                << std::hex << nRet << "]");
        }
        // // 本相机不支持调节Gamma
        // // 设置Gamma
        // setBoolValue("GammaEnable", Gamma);
        // if (Gamma)
        // {
        //     if (Gamma_selector == 1) // 	1:User
        //     {
        //         setEnumValue("GammaSelector", Gamma_selector);
        //     }
        //     else if (Gamma_selector == 2) //  2:sRGB
        //     {
        //         setEnumValue("GammaSelector", Gamma_selector);
        //         setFloatValue("Gamma", Gamma_value);
        //     }
        //     else
        //     {
        //         ROS_ERROR_STREAM("Not Exist Gamma Selector Mode: " << Gamma_selector);
        //     }
        // }
        return true;
    }

    bool Hik_camera_base::set_camera(const MV_CC_DEVICE_INFO &camera)
    {
        if (m_handle != NULL)
        {
            ROS_ERROR_STREAM("Camera handle is already initialized.");
            return false;
        }

        m_stDevInfo = camera;
        void *new_handle = NULL;
        nRet = MV_CC_CreateHandle(&new_handle, &m_stDevInfo);
        if (MV_OK != nRet)
        {
            ROS_ERROR("Error: CreateHandle fail! nRet [%x]\n", nRet);
            return false;
        }

        unsigned int nAccessMode = MV_ACCESS_Exclusive;
        unsigned short nSwitchoverKey = 0;
        nRet = MV_CC_OpenDevice(new_handle, nAccessMode, nSwitchoverKey);
        if (MV_OK != nRet)
        {
            ROS_ERROR("error: OpenDevice fail! nRet [%x]\n", nRet);
            const int destroy_ret = MV_CC_DestroyHandle(new_handle);
            if (MV_OK != destroy_ret)
                ROS_WARN("DestroyHandle after OpenDevice failure failed! nRet [0x%x]", destroy_ret);
            return false;
        }

        m_handle = new_handle;
        device_open_ = true;

        // ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
        if (m_stDevInfo.nTLayerType == MV_GIGE_DEVICE)
        {
            int nPacketSize = MV_CC_GetOptimalPacketSize(m_handle);
            if (nPacketSize > 0)
            {
                nRet = MV_CC_SetIntValueEx(m_handle, "GevSCPSPacketSize", nPacketSize);
                if (nRet != MV_OK)
                {
                    ROS_WARN("Warning: Set Packet Size fail! nRet [0x%x]!\n", nRet);
                }
            }
            else
            {
                ROS_WARN("Warning: Get Packet Size fail! nRet [0x%x]!\n", nPacketSize);
            }
        }
        return true;
    }

    bool Hik_camera_base::setFrameRate(float frame_rate)
    {
        if (!std::isfinite(frame_rate))
        {
            ROS_ERROR_STREAM("AcquisitionFrameRate must be finite, got " << frame_rate);
            return false;
        }

        if (!setBoolValue("AcquisitionFrameRateEnable", true))
            return false;

        MVCC_FLOATVALUE frame_rate_range = {0};
        nRet = MV_CC_GetFloatValue(m_handle, "AcquisitionFrameRate", &frame_rate_range);
        if (MV_OK != nRet)
        {
            ROS_ERROR("Get AcquisitionFrameRate range failed! nRet [%x]\n", nRet);
            return false;
        }
        if (!std::isfinite(frame_rate_range.fMin) ||
            !std::isfinite(frame_rate_range.fMax) ||
            frame_rate_range.fMin > frame_rate_range.fMax)
        {
            ROS_ERROR_STREAM("Invalid SDK AcquisitionFrameRate range: ["
                             << frame_rate_range.fMin << ", " << frame_rate_range.fMax << "]");
            return false;
        }

        const float applied_rate = std::max(frame_rate_range.fMin,
                                            std::min(frame_rate, frame_rate_range.fMax));
        if (std::fabs(applied_rate - frame_rate) > 1e-3f)
        {
            ROS_WARN_STREAM("AcquisitionFrameRate " << frame_rate
                            << " fps is outside SDK range [" << frame_rate_range.fMin
                            << ", " << frame_rate_range.fMax << "]; clamped to "
                            << applied_rate << " fps.");
        }
        if (!setFloatValue("AcquisitionFrameRate", applied_rate))
            return false;

        ROS_INFO_STREAM("AcquisitionFrameRate set to " << applied_rate << " fps");
        return true;
    }

    bool Hik_camera_base::setEnumValue(std::string name, unsigned int value)
    {
        nRet = MV_CC_SetEnumValue(m_handle, name.c_str(), value);
        if (MV_OK != nRet)
        {
            ROS_ERROR("%s set fail! nRet [%x]\n", name.c_str(), nRet);
            return false;
        }
        return true;
    }

    bool Hik_camera_base::setBoolValue(std::string name, bool value)
    {
        nRet = MV_CC_SetBoolValue(m_handle, name.c_str(), value);
        if (MV_OK != nRet)
        {
            ROS_ERROR("%s set fail! nRet [%x]\n", name.c_str(), nRet);
            return false;
        }
        return true;
    }

    bool Hik_camera_base::setFloatValue(std::string name, float value)
    {
        nRet = MV_CC_SetFloatValue(m_handle, name.c_str(), value);
        if (MV_OK != nRet)
        {
            ROS_ERROR("%s set fail! nRet [%x]\n", name.c_str(), nRet);
            return false;
        }
        return true;
    }

    bool Hik_camera_base::setStringValue(std::string name, std::string value)
    {
        nRet = MV_CC_SetStringValue(m_handle, name.c_str(), value.c_str());
        if (MV_OK != nRet)
        {
            ROS_ERROR("%s set fail! nRet [%x]\n", name.c_str(), nRet);
            return false;
        }
        return true;
    }

    bool Hik_camera_base::setIntValue(std::string name, int64_t value)
    {
        nRet = MV_CC_SetIntValueEx(m_handle, name.c_str(), value);
        if (MV_OK != nRet)
        {
            ROS_ERROR("%s set fail! nRet [%x]\n", name.c_str(), nRet);
            return false;
        }
        return true;
    }

    bool Hik_camera_base::setCommandValue(std::string name)
    {
        nRet = MV_CC_SetCommandValue(m_handle, name.c_str());
        if (MV_OK != nRet)
        {
            ROS_ERROR("%s set fail! nRet [%x]\n", name.c_str(), nRet);
            return false;
        }
        return true;
    }

    bool Hik_camera_base::getEnumValue(std::string name, MVCC_ENUMVALUE &value)
    {
        nRet = MV_CC_GetEnumValue(m_handle, name.c_str(), &value);
        if (MV_OK != nRet)
        {
            ROS_ERROR("get %s failed! nRet [%x]\n", name.c_str(), nRet);
            return false;
        }
        return true;
    }

    bool Hik_camera_base::getBoolValue(std::string name, bool &value)
    {
        nRet = MV_CC_GetBoolValue(m_handle, name.c_str(), &value);
        if (MV_OK != nRet)
        {
            ROS_ERROR("get %s failed! nRet [%x]\n", name.c_str(), nRet);
            return false;
        }
        return true;
    }

    bool Hik_camera_base::getFloatValue(std::string name, float &value)
    {
        MVCC_FLOATVALUE temp_value = {0};
        nRet = MV_CC_GetFloatValue(m_handle, name.c_str(), &temp_value);
        if (MV_OK != nRet)
        {
            ROS_ERROR("get %s failed! nRet [%x]\n", name.c_str(), nRet);
            return false;
        }
        value = temp_value.fCurValue;
        return true;
    }

    bool Hik_camera_base::getStringValue(std::string name, std::string &value)
    {
        MVCC_STRINGVALUE stStringValue = {0};
        nRet = MV_CC_GetStringValue(m_handle, name.c_str(), &stStringValue);
        if (MV_OK != nRet)
        {
            ROS_ERROR("get %s failed! nRet [%x]\n", name.c_str(), nRet);
            return false;
        }
        value = stStringValue.chCurValue;
        return true;
    }

    bool Hik_camera_base::getIntValue(std::string name, int64_t &value)
    {
        MVCC_INTVALUE_EX stValue = {0};
        nRet = MV_CC_GetIntValueEx(m_handle, name.c_str(), &stValue);
        if (MV_OK != nRet)
        {
            ROS_ERROR("get %s failed! nRet [%x]\n", name.c_str(), nRet);
            return false;
        }
        value = stValue.nCurValue;
        return true;
    }

    bool Hik_camera_base::readTickFrequency()
    {
        // 1. GigE 标准节点
        if (getIntValue("GevTimestampTickFrequency", tick_frequency_) && tick_frequency_ > 0)
        {
            ROS_INFO("Tick frequency (GevTimestampTickFrequency): %ld Hz", tick_frequency_);
            return true;
        }
        ROS_WARN("GevTimestampTickFrequency read failed, trying USB3 nodes...");
        // 2. USB3 节点
        if (getIntValue("DeviceTickFrequency", tick_frequency_) && tick_frequency_ > 0)
        {
            ROS_INFO("Tick frequency (DeviceTickFrequency): %ld Hz", tick_frequency_);
            return true;
        }
        // 3. 备选 USB3 节点名
        if (getIntValue("DeviceTimestampTickFrequency", tick_frequency_) && tick_frequency_ > 0)
        {
            ROS_INFO("Tick frequency (DeviceTimestampTickFrequency): %ld Hz", tick_frequency_);
            return true;
        }
        // 4. Fallback: 1 GHz
        ROS_WARN("Cannot read tick frequency from camera, using default 1e9 Hz (1 GHz)");
        tick_frequency_ = 1000000000;
        return true;
    }

    // 打开/映射 LiDAR 共享内存时间戳 /tmp/livox_timeshare
    bool Hik_camera_base::openLidarTimestampShm()
    {
        // 先清理可能残留的旧映射 (重启路径)
        closeLidarTimestampShm();

        lidar_shm_fd_ = open(kLidarTimestampPath, O_RDONLY | O_CLOEXEC);
        if (lidar_shm_fd_ < 0)
        {
            ROS_WARN_STREAM("Lidar timestamp: open(" << kLidarTimestampPath
                            << ") failed (" << errno << "), "
                            << "ensure livox_ros_driver2 is running; "
                            << "fallback to camera HW timestamp.");
            lidar_shm_ok_ = false;
            return false;
        }
        lidar_shm_ptr_ = mmap(nullptr, sizeof(time_stamp),
                              PROT_READ, MAP_SHARED, lidar_shm_fd_, 0);
        if (lidar_shm_ptr_ == MAP_FAILED)
        {
            ROS_WARN_STREAM("Lidar timestamp: mmap(" << kLidarTimestampPath
                            << ") failed (" << errno << "), "
                            << "fallback to camera HW timestamp.");
            close(lidar_shm_fd_);
            lidar_shm_fd_ = -1;
            lidar_shm_ptr_ = nullptr;
            lidar_shm_ok_ = false;
            return false;
        }
        lidar_shm_ok_ = true;
        ROS_INFO_STREAM("Lidar timestamp: mmap(" << kLidarTimestampPath
                        << ") ok, using LiDAR base_time.");
        return true;
    }

    void Hik_camera_base::closeLidarTimestampShm()
    {
        if (lidar_shm_ptr_ != nullptr && lidar_shm_ptr_ != MAP_FAILED)
            munmap(lidar_shm_ptr_, sizeof(time_stamp));
        if (lidar_shm_fd_ >= 0)
            close(lidar_shm_fd_);
        lidar_shm_ptr_ = nullptr;
        lidar_shm_fd_ = -1;
        lidar_shm_ok_ = false;
    }

    // 读 pointt->low (纳秒) -> ros::Time (秒); 不可用或未写入返回 ros::Time()
    ros::Time Hik_camera_base::getLidarTimestamp()
    {
        if (!lidar_shm_ok_ || lidar_shm_ptr_ == nullptr || lidar_shm_ptr_ == MAP_FAILED)
            return ros::Time();
        int64_t b = reinterpret_cast<time_stamp*>(lidar_shm_ptr_)->low;
        if (b == 0)  // LiDAR 驱动尚未写入或被清零
            return ros::Time();
        return ros::Time(static_cast<double>(b) / 1000000000.0);
    }

    bool Hik_camera_base::ImageStream()
    {
        if (!m_handle || !device_open_)
        {
            ROS_ERROR_STREAM("Cannot start camera stream before opening the device.");
            return false;
        }

        // 读取标定参数
        private_nh.param<int>("Camera/calib_frame_count", calib_frame_count_, 50);
        private_nh.param<double>("Camera/calib_iqr_multiplier", calib_iqr_multiplier_, 1.5);
        ROS_INFO("Timestamp calibration: %d frames, IQR multiplier=%.1f (tick freq auto-derived)",
                 calib_frame_count_, calib_iqr_multiplier_);

        // 可选: 启用 LiDAR 共享内存时间戳 (soft-sync, 蹭 livox 的 base_time)
        if (use_lidar_timestamp_)
            openLidarTimestampShm();  // 失败不致命, 取流时自动回退到相机硬件时间戳
        else
            closeLidarTimestampShm();  // 关闭开关时确保无残留映射

        // 新 WorkThread 可运行
        stop_requested_ = false;

        // 开始取流
        nRet = MV_CC_StartGrabbing(m_handle);
        if (MV_OK != nRet)
        {
            ROS_ERROR("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
            return false;
        }
        grabbing_ = true;
        // 互斥量已在构造函数一次性初始化, 此处不再重复 init
        nRet = pthread_create(&nThreadID, NULL, WorkThread, this);
        if (0 != nRet)
        {
            ROS_ERROR("thread create failed.ret = %d\n", nRet);
            const int stop_ret = MV_CC_StopGrabbing(m_handle);
            if (MV_OK != stop_ret)
                ROS_WARN_STREAM("MV_CC_StopGrabbing after thread creation failure returned [0x"
                                << std::hex << stop_ret << "]");
            grabbing_ = false;
            return false;
        }
        thread_started_ = true;
        // start to loop
        return true;
    }

    void Hik_camera_base::stopStream()
    {
        // 1. 请求 WorkThread 退出
        stop_requested_ = true;
        // 2. 无论线程是否启动成功, 都与 StartGrabbing 成对停止
        if (grabbing_ && m_handle)
        {
            nRet = MV_CC_StopGrabbing(m_handle);
            if (MV_OK != nRet)
                ROS_WARN_STREAM("MV_CC_StopGrabbing fail/warn nRet [0x" << std::hex << nRet << "]");
            grabbing_ = false;
        }
        if (thread_started_)
        {
            // join 是权威同步点: 此后 WorkThread 已退出
            pthread_join(nThreadID, nullptr);
            thread_started_ = false;
        }
        // 3. 关闭设备 + 销毁句柄 (无早返回, 修复原先 StopGrabbing 失败就跳过后续清理的泄漏)
        if (m_handle)
        {
            if (device_open_)
            {
                nRet = MV_CC_CloseDevice(m_handle);
                if (MV_OK != nRet)
                    ROS_WARN_STREAM("MV_CC_CloseDevice fail nRet [0x" << std::hex << nRet << "]");
                device_open_ = false;
            }
            nRet = MV_CC_DestroyHandle(m_handle);
            if (MV_OK != nRet)
                ROS_WARN_STREAM("MV_CC_DestroyHandle fail nRet [0x" << std::hex << nRet << "]");
            m_handle = NULL;
        }
        // 解除 LiDAR 共享内存映射
        closeLidarTimestampShm();
    }

    bool Hik_camera_base::changeExposureTime(float value)
    {
        if (!exposure_time_range_ready_)
        {
            ROS_ERROR_STREAM("ExposureTime range is not initialized.");
            return false;
        }
        if (!std::isfinite(value))
        {
            ROS_ERROR_STREAM("ExposureTime must be finite, got " << value);
            return false;
        }

        const float applied_value = std::max(exposure_time_low,
                                             std::min(value, exposure_time_up));
        if (std::fabs(applied_value - value) > 1e-3f)
        {
            ROS_WARN_STREAM_THROTTLE(5.0, "ExposureTime " << value
                                     << "us is outside effective range ["
                                     << exposure_time_low << ", " << exposure_time_up
                                     << "]us; clamped to " << applied_value << "us.");
        }

        const int ret = MV_CC_SetFloatValue(m_handle, "ExposureTime", applied_value);
        if (MV_OK != ret)
        {
            ROS_WARN_STREAM_THROTTLE(5.0, "ExposureTime set failed! nRet [0x"
                                     << std::hex << ret << "]");
            return false;
        }
        exposure_time_set.store(applied_value, std::memory_order_relaxed);
        return true;
    }

    void Hik_camera_base::exposure_callback(const std_msgs::Float32ConstPtr msg)
    {
        float exposure = msg->data;
        float get_exposure = 0.0f;
        if (getFloatValue("ExposureTime", get_exposure) &&
            std::fabs(get_exposure - exposure) > 1e-3f)
        {
            exposure_time_set.store(exposure, std::memory_order_relaxed);
        }
    }

    void *Hik_camera_base::WorkThread(void *p_user)
    {
        Hik_camera_base *self = (Hik_camera_base *)p_user;
        void *p_handle = self->m_handle;

        int nRet = MV_OK;
        std::vector<unsigned char> bgr_buffer;

        // ========================================
        // 时间戳标定阶段 (推导 tick 频率 + 标定 offset)
        // ========================================
        self->timestamp_calibrated_ = false;
        while (ros::ok() && !self->stop_requested_ && !self->timestamp_calibrated_)
        {
            struct CalibSample
            {
                uint64_t dev_ticks;
                double wall_sec;
            };
            std::vector<CalibSample> samples;
            int calib_count = self->calib_frame_count_;
            double iqr_mult = self->calib_iqr_multiplier_;
            samples.reserve(calib_count);

            ROS_INFO("Timestamp calibration: collecting %d frames...", calib_count);
            while ((int)samples.size() < calib_count && ros::ok() && !self->stop_requested_)
            {
                MV_FRAME_OUT frame_out = {0};
                nRet = MV_CC_GetImageBuffer(p_handle, &frame_out, 200);
                if (nRet == MV_OK)
                {
                    ImageBufferGuard frame_guard(p_handle, frame_out);
                    const MV_FRAME_OUT_INFO_EX &frame_info = frame_out.stFrameInfo;
                    CalibSample s;
                    s.dev_ticks = ((uint64_t)frame_info.nDevTimeStampHigh << 32)
                                | frame_info.nDevTimeStampLow;
                    s.wall_sec  = ros::Time::now().toSec();
                    samples.push_back(s);
                }
                else if (nRet != MV_E_NODATA)
                {
                    ROS_WARN_THROTTLE(5.0, "Calibration frame failed, nRet [0x%x]", nRet);
                }
            }

            if ((int)samples.size() < 2)
            {
                ROS_ERROR("Timestamp calibration failed: need >=2 frames, got %zu!", samples.size());
                return NULL;
            }

            // ---- 第1步: 从帧间 delta 推导 tick 频率 ----
            // 使用 ros::Time::now() 帧间差 (单位秒)，不依赖 nHostTimeStamp
            std::vector<double> freq_samples;
            for (size_t i = 1; i < samples.size(); i++)
            {
                int64_t delta_ticks = (int64_t)(samples[i].dev_ticks - samples[i - 1].dev_ticks);
                double delta_wall = samples[i].wall_sec - samples[i - 1].wall_sec;
                if (delta_wall > 0.0 && delta_ticks > 0)
                {
                    freq_samples.push_back((double)delta_ticks / delta_wall);
                }
            }

            if (freq_samples.empty())
            {
                ROS_ERROR("Timestamp calibration failed: cannot derive tick frequency; retrying.");
                continue;
            }

            // IQR 滤波 tick 频率
            std::sort(freq_samples.begin(), freq_samples.end());
            {
                size_t fn = freq_samples.size();
                double fq1 = freq_samples[fn / 4];
                double fq3 = freq_samples[3 * fn / 4];
                double fiqr = fq3 - fq1;
                double flo = fq1 - iqr_mult * fiqr;
                double fhi = fq3 + iqr_mult * fiqr;
                std::vector<double> ffiltered;
                for (size_t i = 0; i < fn; i++)
                {
                    if (freq_samples[i] >= flo && freq_samples[i] <= fhi)
                        ffiltered.push_back(freq_samples[i]);
                }
                if (ffiltered.empty())
                {
                    ROS_ERROR("Timestamp calibration failed: all frequency samples rejected; retrying.");
                    continue;
                }
                self->tick_frequency_ = (int64_t)ffiltered[ffiltered.size() / 2];
                ROS_INFO("Tick frequency: %ld Hz (from %zu/%zu delta samples, %zu rejected)",
                         self->tick_frequency_, ffiltered.size(), fn, fn - ffiltered.size());
            }

            // ---- 第2步: 用推导出的 tick 频率标定 wall clock offset ----
            std::vector<double> offset_samples;
            int64_t tick_freq = self->tick_frequency_;
            for (size_t i = 0; i < samples.size(); i++)
            {
                double dev_sec = (double)samples[i].dev_ticks / (double)tick_freq;
                offset_samples.push_back(samples[i].wall_sec - dev_sec);
            }

            // IQR 滤波 offset
            std::sort(offset_samples.begin(), offset_samples.end());
            size_t on = offset_samples.size();
            double oq1 = offset_samples[on / 4];
            double oq3 = offset_samples[3 * on / 4];
            double oiqr = oq3 - oq1;
            double olo = oq1 - iqr_mult * oiqr;
            double ohi = oq3 + iqr_mult * oiqr;

            std::vector<double> ofiltered;
            for (size_t i = 0; i < on; i++)
            {
                if (offset_samples[i] >= olo && offset_samples[i] <= ohi)
                    ofiltered.push_back(offset_samples[i]);
            }

            if (ofiltered.empty())
            {
                ROS_ERROR("Timestamp calibration failed: all offset samples rejected; retrying.");
                continue;
            }

            self->device_to_wall_offset_ = ofiltered[ofiltered.size() / 2];
            self->timestamp_calibrated_ = true;

            // 计算 offset 标准差
            double sum = 0.0, stddev = 0.0;
            for (size_t i = 0; i < ofiltered.size(); i++)
                sum += ofiltered[i];
            double mean = sum / ofiltered.size();
            for (size_t i = 0; i < ofiltered.size(); i++)
                stddev += (ofiltered[i] - mean) * (ofiltered[i] - mean);
            stddev = sqrt(stddev / ofiltered.size());

            ROS_INFO("Timestamp calibration done: offset=%.6fs, stddev=%.3fms, "
                     "offset_samples=%zu/%zu (rejected %zu)",
                     self->device_to_wall_offset_, stddev * 1000.0,
                     ofiltered.size(), on, on - ofiltered.size());
        }
        // ========================================
        // 标定完毕，开始正常取流
        // ========================================

        // 取流失败分类与计数：保持线程运行，并对持续错误做节流告警
        static const uint64_t kGrabWarnEveryN = 100;  // 连续超时每 N 次打一条 WARN
        uint64_t timeout_cnt = 0;                     // 连续超时计数（取到一帧后清零）
        uint64_t err_cnt     = 0;                     // 连续真错误计数
        float last_exposure_set = -1.0f;              // 上次写入相机的曝光值，用于去抖

        while (ros::ok() && !self->stop_requested_)
        {
            // 仅当曝光时间实际变化时才写入 GenICam ExposureTime 节点
            const float target_exposure = exposure_time_set.load(std::memory_order_relaxed);
            if (exposure_auto == 0 && std::fabs(target_exposure - last_exposure_set) > 1e-3f)
            {
                if (self->changeExposureTime(target_exposure))
                {
                    last_exposure_set = exposure_time_set.load(std::memory_order_relaxed);
                }
            }
            MV_FRAME_OUT frame_out = {0};
            nRet = MV_CC_GetImageBuffer(p_handle, &frame_out, 1000);
            if (nRet == MV_OK)
            {
                ImageBufferGuard frame_guard(p_handle, frame_out);
                const MV_FRAME_OUT_INFO_EX &frame_info = frame_out.stFrameInfo;
                err_cnt = 0;
                timeout_cnt = 0;
                // 时间戳来源: 优先 LiDAR 共享内存时间戳 (软同步, 蹭 livox 的 base_time);
                // 未启用/共享内存不可用/LiDAR 尚未写入(capture_time 为零) -> 回退到相机硬件时间戳标定方案
                ros::Time capture_time;
                if (self->use_lidar_timestamp_ && self->lidar_shm_ok_)
                    capture_time = self->getLidarTimestamp();
                if (capture_time.isZero())
                {
                    uint64_t dev_ticks = ((uint64_t)frame_info.nDevTimeStampHigh << 32)
                                       | frame_info.nDevTimeStampLow;
                    double dev_sec = (double)dev_ticks / (double)self->tick_frequency_;
                    capture_time = ros::Time(dev_sec + self->device_to_wall_offset_);
                }

                if (frame_info.nWidth == 0 || frame_info.nHeight == 0 ||
                    frame_info.nFrameLen == 0 || frame_out.pBufAddr == NULL)
                {
                    ROS_ERROR_STREAM_THROTTLE(5.0, "Invalid frame metadata: width="
                                              << frame_info.nWidth << ", height="
                                              << frame_info.nHeight << ", length="
                                              << frame_info.nFrameLen);
                    continue;
                }

                const size_t pixel_count = static_cast<size_t>(frame_info.nWidth) *
                                           static_cast<size_t>(frame_info.nHeight);
                if (pixel_count > std::numeric_limits<unsigned int>::max() / 3U)
                {
                    ROS_ERROR_STREAM_THROTTLE(5.0, "BGR frame size exceeds SDK conversion limit: "
                                              << frame_info.nWidth << "x" << frame_info.nHeight);
                    continue;
                }
                const unsigned int expected_bgr_size =
                    static_cast<unsigned int>(pixel_count * 3U);
                try
                {
                    bgr_buffer.resize(expected_bgr_size);
                }
                catch (const std::bad_alloc &)
                {
                    ROS_ERROR_STREAM("Failed to allocate " << expected_bgr_size
                                     << " bytes for BGR frame conversion.");
                    return NULL;
                }

                MV_CC_PIXEL_CONVERT_PARAM_EX convert_param = {0};
                convert_param.nWidth = frame_info.nWidth;
                convert_param.nHeight = frame_info.nHeight;
                convert_param.pSrcData = frame_out.pBufAddr;
                convert_param.nSrcDataLen = frame_info.nFrameLen;
                convert_param.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
                convert_param.pDstBuffer = bgr_buffer.data();
                convert_param.nDstBufferSize = expected_bgr_size;
                convert_param.enSrcPixelType = frame_info.enPixelType;
                nRet = MV_CC_ConvertPixelTypeEx(p_handle, &convert_param);
                if (MV_OK != nRet)
                {
                    ROS_ERROR_STREAM_THROTTLE(5.0, "MV_CC_ConvertPixelTypeEx failed! nRet [0x"
                                              << std::hex << nRet << "]");
                    continue;
                }
                if (convert_param.nDstLen != expected_bgr_size)
                {
                    ROS_ERROR_STREAM_THROTTLE(5.0, "Unexpected converted frame length: got "
                                              << convert_param.nDstLen << ", expected "
                                              << expected_bgr_size);
                    continue;
                }

                cv::Mat srcImage(frame_info.nHeight, frame_info.nWidth, CV_8UC3,
                                 bgr_buffer.data());
                sensor_msgs::ImagePtr msg =
                    cv_bridge::CvImage(std_msgs::Header(), "bgr8", srcImage).toImageMsg();
                msg->header.stamp = capture_time;
                pthread_mutex_lock(&mutex);
                frame_empty = false;
                frame = msg;
                pthread_mutex_unlock(&mutex);
            }
            else if (nRet == MV_E_NODATA || nRet == MV_E_TIMEOUT)
            {
                // 取流超时/无数据：保持等待，不重建相机句柄
                timeout_cnt++;
                if (timeout_cnt % kGrabWarnEveryN == 0)
                {
                    ROS_WARN_STREAM("Grab timeout/no-data streak: " << timeout_cnt
                                     << ", last nRet [0x" << std::hex << nRet << "]");
                }
            }
            else
            {
                // 真错误码：保持线程运行，等待 SDK 后续取图调用恢复
                err_cnt++;
                ROS_ERROR_STREAM_THROTTLE(5.0, "Grab failed! nRet [0x" << std::hex << nRet
                                          << "], streak=" << err_cnt);
                ros::WallDuration(0.1).sleep();
            }
        }
        return NULL;
    }

    void Hik_camera_base::ImagePub()
    {
        // 临界区内只做 shared_ptr 拷贝与 frame_empty 置位，
        // publish / cvtColor / mean / 曝光调整等耗时操作移到锁外，避免阻塞 WorkThread 取流。
        sensor_msgs::ImagePtr local_frame;
        bool have_frame = false;
        pthread_mutex_lock(&mutex);
        if (frame_empty == false)
        {
            local_frame = frame; // shared_ptr 拷贝，引用计数保证锁外仍有效
            frame_empty = true;
            have_frame = true;
        }
        pthread_mutex_unlock(&mutex);

        if (!have_frame)
        {
            return;
        }

        local_frame->header.frame_id = camera_name;
        sensor_msgs::CameraInfoPtr ci_(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
        ci_->header.frame_id = local_frame->header.frame_id;
        ci_->header.stamp = local_frame->header.stamp;
        camera_pub.publish(*local_frame, *ci_);

        if (exposure_control)
        {
            cv_bridge::CvImagePtr cv_ptr;
            cv_ptr = cv_bridge::toCvCopy(local_frame, sensor_msgs::image_encodings::BGR8);
            cv::Mat temp_img = cv_ptr->image;
            cv::Mat imgGray;
            cv::cvtColor(temp_img, imgGray, CV_BGR2GRAY);
            cv::Scalar grayScalar = cv::mean(imgGray);
            float imgGrayLight = grayScalar.val[0];
            const float current_exposure = exposure_time_set.load(std::memory_order_relaxed);
            if (imgGrayLight < light_set - 10 && current_exposure * scale < exposure_time_up)
            {
                exposure_time_set.store(current_exposure * scale, std::memory_order_relaxed);
            }
            else if (imgGrayLight > light_set + 10 && current_exposure / scale > exposure_time_low)
            {
                exposure_time_set.store(current_exposure / scale, std::memory_order_relaxed);
            }
        }
    }

} // namespace HIKCAMERA
