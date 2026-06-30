#include <camera.h>

namespace HIKCAMERA
{
    sensor_msgs::ImagePtr frame; // 临时存放当前帧
    pthread_mutex_t mutex;       // 存放帧的锁
    bool frame_empty = true;     // 用于标志是否有新帧未发布
    float exposure_time_set;     // 用于存放下次设置的曝光时间
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

    bool Hik_camera_base::set_params()
    {
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

        // 取图超时重启参数
        private_nh.param<int>("Camera/grab_timeout_retry", grab_timeout_retry_threshold_, 5);
        private_nh.param<int>("Camera/restart_max_retries", restart_max_retries_, 0);
        // 触发模式下无触发信号时取图超时是正常的, 重启无意义 -> 自动禁用
        if (trigger_mode && grab_timeout_retry_threshold_ > 0)
        {
            ROS_WARN_STREAM("Trigger mode ON: disabling grab-timeout restart "
                            "(set grab_timeout_retry=0 explicitly).");
            grab_timeout_retry_threshold_ = 0;
        }
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

        exposure_time_set = Exposure_time;
        exposure_auto = Exposure;
        exposure_time_low = ExposureTimeLow;
        exposure_time_up = ExposureTimeUp;
        light_set = brightneess;

        setEnumValue("AcquisitionMode", MV_ACQ_MODE_CONTINUOUS);
        ROS_INFO_STREAM("AcquisitionMode set to Continuous.");
        // 设置触发
        if (trigger_mode)
        {
            setEnumValue("TriggerMode", MV_TRIGGER_MODE_ON);
            ROS_INFO_STREAM("TriggerMode set to ON");
            if (trigger_line >= 0 && trigger_line <= 8 && trigger_line != 5 && trigger_line != 6)
            {
                ROS_INFO_STREAM("TriggerSource set to " << trigger_line);
                setEnumValue("TriggerSource", trigger_line);
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
                    ROS_INFO_STREAM("TriggerActivation set to " << trigger_action);
                    setEnumValue("TriggerActivation", trigger_action);
                }
                else
                {
                    ROS_WARN_STREAM("Not Exist Trigger Action: " << trigger_action);
                }
                setFloatValue("TriggerDelay", trigger_delay);
                ROS_INFO_STREAM("TriggerDelay set to " << trigger_delay << "us.");
                setBoolValue("TriggerCacheEnable", trigger_cache);
                ROS_INFO_STREAM("TriggerCacheEnable set to " << trigger_cache);
            }
            else
            {
                ROS_WARN_STREAM("Not Exist Trigger Source: " << trigger_line);
            }
        }
        else
        {
            ROS_INFO_STREAM("TriggerMode set to OFF");
            setEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF);
        }
        // 设置帧率
        setFrameRate(frame_rate);
        ROS_INFO_STREAM("Frame Rate set to " << frame_rate << " fps");
        // 设置曝光
        if (Exposure == 0)
        {
            setEnumValue("ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
            setFloatValue("ExposureTime", Exposure_time);
            ROS_INFO_STREAM("ExposureAuto set to OFF");
            ROS_INFO_STREAM("ExposureTime set to " << Exposure_time << "us");
        }
        else if (Exposure > 0 && Exposure <= 2)
        {
            setEnumValue("ExposureAuto", Exposure);
            setIntValue("AutoExposureTimeLowerLimit", ExposureTimeLow);
            setIntValue("AutoExposureTimeUpperLimit", ExposureTimeUp);
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
            setEnumValue("GainAuto", MV_GAIN_MODE_OFF);
            setFloatValue("Gain", Gain_value);
            ROS_INFO_STREAM("GainAuto set to OFF");
            ROS_INFO_STREAM("Gain set to " << Gain_value << "dB");
        }
        else if (Gain_mode > 0 && Gain_mode <= 2)
        {
            setEnumValue("GainAuto", Gain_mode);
            ROS_INFO_STREAM("GainAuto set to " << Gain_mode);
        }
        else
        {
            ROS_ERROR_STREAM("Not Exist Gain Mode: " << Gain_mode);
            return false;
        }
        // 设置白平衡
        setEnumValue("BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
        ROS_INFO_STREAM("BalanceWhiteAuto set to Continuous");
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
        setEnumValue("PixelFormat", PixelType_Gvsp_BayerRG10_Packed);
        // 设置亮度
        setIntValue("Brightness", brightneess);
        ROS_INFO_STREAM("Brightness set to " << brightneess);
        // 设置数字偏移
        if (Digital_shift_mode)
        {
            setBoolValue("DigitalShiftEnable", true);
            setFloatValue("DigitalShift", Digital_shift);
            ROS_INFO_STREAM("DigitalShiftEnable set to ON");
            ROS_INFO_STREAM("DigitalShift set to " << Digital_shift);
        }
        else
        {
            setBoolValue("DigitalShiftEnable", false);
            ROS_INFO_STREAM("DigitalShiftEnable set to OFF");
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

    bool Hik_camera_base::set_camera(MV_CC_DEVICE_INFO &camera)
    {
        m_stDevInfo = camera;
        // 创建句柄
        nRet = MV_CC_CreateHandle(&m_handle, &m_stDevInfo);
        if (MV_OK != nRet)
        {
            ROS_ERROR("Error: CreateHandle fail! nRet [%x]\n", nRet);
            return false;
        }
        // 连接设备
        unsigned int nAccessMode = MV_ACCESS_Exclusive;
        unsigned short nSwitchoverKey = 0;
        nRet = MV_CC_OpenDevice(m_handle, nAccessMode, nSwitchoverKey);
        if (MV_OK != nRet)
        {
            ROS_ERROR("error: OpenDevice fail! nRet [%x]\n", nRet);
            return false;
        }
        // ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
        if (m_stDevInfo.nTLayerType == MV_GIGE_DEVICE)
        {
            int nPacketSize = MV_CC_GetOptimalPacketSize(m_handle);
            if (nPacketSize > 0)
            {
                nRet = MV_CC_SetIntValue(m_handle, "GevSCPSPacketSize", nPacketSize);
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
        nRet = MV_CC_SetFrameRate(m_handle, frame_rate);
        if (MV_OK != nRet)
        {
            ROS_ERROR("frame rate set fail! nRet [%x]\n", nRet);
            return false;
        }
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

    bool Hik_camera_base::setIntValue(std::string name, unsigned int value)
    {
        nRet = MV_CC_SetIntValue(m_handle, name.c_str(), value);
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
        MVCC_FLOATVALUE temp_value;
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

    bool Hik_camera_base::getIntValue(std::string name, unsigned int &value)
    {
        MVCC_INTVALUE stHeight = {0};
        nRet = MV_CC_GetIntValue(m_handle, name.c_str(), &stHeight);
        if (MV_OK != nRet)
        {
            ROS_ERROR("get %s failed! nRet [%x]\n", name.c_str(), nRet);
            return false;
        }
        value = stHeight.nCurValue;
        return true;
    }

    bool Hik_camera_base::getIntValueEx(std::string name, int64_t &value)
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
        if (getIntValueEx("GevTimestampTickFrequency", tick_frequency_) && tick_frequency_ > 0)
        {
            ROS_INFO("Tick frequency (GevTimestampTickFrequency): %ld Hz", tick_frequency_);
            return true;
        }
        ROS_WARN("GevTimestampTickFrequency read failed, trying USB3 nodes...");
        // 2. USB3 节点
        if (getIntValueEx("DeviceTickFrequency", tick_frequency_) && tick_frequency_ > 0)
        {
            ROS_INFO("Tick frequency (DeviceTickFrequency): %ld Hz", tick_frequency_);
            return true;
        }
        // 3. 备选 USB3 节点名
        if (getIntValueEx("DeviceTimestampTickFrequency", tick_frequency_) && tick_frequency_ > 0)
        {
            ROS_INFO("Tick frequency (DeviceTimestampTickFrequency): %ld Hz", tick_frequency_);
            return true;
        }
        // 4. Fallback: 1 GHz
        ROS_WARN("Cannot read tick frequency from camera, using default 1e9 Hz (1 GHz)");
        tick_frequency_ = 1000000000;
        return true;
    }

    bool Hik_camera_base::ImageStream()
    {
        // 读取标定参数
        private_nh.param<int>("Camera/calib_frame_count", calib_frame_count_, 50);
        private_nh.param<double>("Camera/calib_iqr_multiplier", calib_iqr_multiplier_, 1.5);
        ROS_INFO("Timestamp calibration: %d frames, IQR multiplier=%.1f (tick freq auto-derived)",
                 calib_frame_count_, calib_iqr_multiplier_);

        // 重置线程控制标志 (新 WorkThread 可运行, 清除残留重启请求)
        stop_requested_ = false;
        need_restart_   = false;
        thread_running_ = false;

        // 开始取流
        nRet = MV_CC_StartGrabbing(m_handle);
        if (MV_OK != nRet)
        {
            ROS_ERROR("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
            return false;
        }
        // 互斥量已在构造函数一次性初始化, 此处不再重复 init
        nRet = pthread_create(&nThreadID, NULL, WorkThread, this);
        if (0 != nRet)
        {
            ROS_ERROR("thread create failed.ret = %d\n", nRet);
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
        // 2. join WorkThread 前先 StopGrabbing, 解除在途 GetOneFrameTimeout 阻塞
        if (thread_started_)
        {
            if (m_handle)
            {
                nRet = MV_CC_StopGrabbing(m_handle);
                if (MV_OK != nRet)
                    ROS_WARN_STREAM("MV_CC_StopGrabbing fail/warn nRet [0x" << std::hex << nRet << "]");
            }
            // join 是权威同步点: 此后 WorkThread 已退出, pData/m_pBufForSaveImage 已由其内部 free
            pthread_join(nThreadID, nullptr);
            thread_started_ = false;
        }
        // 3. 关闭设备 + 销毁句柄 (无早返回, 修复原先 StopGrabbing 失败就跳过后续清理的泄漏)
        if (m_handle)
        {
            nRet = MV_CC_CloseDevice(m_handle);
            if (MV_OK != nRet)
                ROS_WARN_STREAM("MV_CC_CloseDevice fail nRet [0x" << std::hex << nRet << "]");
            nRet = MV_CC_DestroyHandle(m_handle);
            if (MV_OK != nRet)
                ROS_WARN_STREAM("MV_CC_DestroyHandle fail nRet [0x" << std::hex << nRet << "]");
            m_handle = NULL;
        }
        thread_running_ = false;
    }

    bool Hik_camera_base::restart()
    {
        ROS_WARN_STREAM("Camera restart: stopping stream and joining work thread...");
        stopStream(); // 置 stop_requested_ -> StopGrabbing 解除阻塞 -> join WorkThread -> Close+Destroy

        // 丢弃旧帧, 避免重启后发布陈旧图像
        pthread_mutex_lock(&mutex);
        frame.reset();
        frame_empty = true;
        pthread_mutex_unlock(&mutex);

        // 复用已保存的设备信息, 无需重新枚举
        if (!set_camera(m_stDevInfo))
        {
            ROS_ERROR_STREAM("Restart: set_camera failed!");
            return false;
        }
        if (!set_params())
        {
            ROS_ERROR_STREAM("Restart: set_params failed!");
            return false;
        }
        if (!ImageStream())
        {
            ROS_ERROR_STREAM("Restart: ImageStream failed!");
            return false;
        }
        ROS_WARN_STREAM("Camera restart: stream resumed, new work thread started.");
        return true;
    }

    void Hik_camera_base::check_and_restart()
    {
        if (!need_restart_)
            return;

        ros::Time now = ros::Time::now();
        // 指数退避: 1,2,4,8,16,32, 封顶 60s
        double backoff = std::min(60.0, std::pow(2.0, std::min(restart_attempts_, 6)));
        if (restart_attempts_ > 0 && (now - last_restart_time_).toSec() < backoff)
            return; // 仍在退避窗口内, 等待
        last_restart_time_ = now;

        if (restart_max_retries_ > 0 && restart_attempts_ >= restart_max_retries_)
        {
            ROS_ERROR_STREAM("Restart attempts " << restart_attempts_
                             << " >= max " << restart_max_retries_ << ", shutting down node.");
            ros::shutdown();
            return;
        }

        if (restart())
        {
            restart_attempts_ = 0; // ImageStream 已清 need_restart_
        }
        else
        {
            restart_attempts_++;
            need_restart_ = true; // 重新武装 (ImageStream 成功路径会清, 失败则保持)
            ROS_ERROR_STREAM("Restart attempt " << restart_attempts_
                             << " failed; will retry in ~" << backoff << "s");
        }
    }

    bool Hik_camera_base::changeExposureTime(float value)
    {
        // 停止取流
        nRet = MV_CC_StopGrabbing(m_handle);
        if (MV_OK != nRet)
        {
            printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
            return false;
        }
        // 更改曝光速度
        setFloatValue("ExposureTime", value);
        // 开始取流
        nRet = MV_CC_StartGrabbing(m_handle);
        if (MV_OK != nRet)
        {
            ROS_ERROR("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
            return false;
        }
        return true;
    }

    void Hik_camera_base::exposure_callback(const std_msgs::Float32ConstPtr msg)
    {
        float exposure = msg->data;
        float get_exposure;
        getFloatValue("ExposureTime", get_exposure);
        if (get_exposure != exposure)
        {
            // changeExposureTime(exposure);
            exposure_time_set = exposure;
        }
    }

    void *Hik_camera_base::WorkThread(void *p_user)
    {
        Hik_camera_base *self = (Hik_camera_base *)p_user;
        void *p_handle = self->m_handle;
        self->thread_running_ = true;
        // RAII: 保证所有 return 路径都把 thread_running_ 置 false
        struct RunningGuard
        {
            Hik_camera_base *s;
            ~RunningGuard() { s->thread_running_ = false; }
        } guard{self};

        int nRet = MV_OK;
        // ch:获取数据包大小 | en:Get payload size
        MVCC_INTVALUE stParam;
        memset(&stParam, 0, sizeof(MVCC_INTVALUE));
        nRet = MV_CC_GetIntValue(p_handle, "PayloadSize", &stParam);
        if (MV_OK != nRet)
        {
            printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
            return NULL;
        }
        MV_FRAME_OUT_INFO_EX stImageInfo = {0};
        MV_CC_PIXEL_CONVERT_PARAM_EX stConvertParam = {0};
        memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
        unsigned char *pData =
            (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);
        unsigned char *m_pBufForSaveImage = (unsigned char *)malloc(stParam.nCurValue * 3);
        if (NULL == pData)
            return NULL;
        unsigned int nDataSize = stParam.nCurValue;

        // ========================================
        // 时间戳标定阶段 (推导 tick 频率 + 标定 offset)
        // ========================================
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
            while ((int)samples.size() < calib_count && ros::ok())
            {
                nRet = MV_CC_GetOneFrameTimeout(p_handle, pData, nDataSize, &stImageInfo, 200);
                if (nRet == MV_OK)
                {
                    CalibSample s;
                    s.dev_ticks = ((uint64_t)stImageInfo.nDevTimeStampHigh << 32)
                                | stImageInfo.nDevTimeStampLow;
                    s.wall_sec  = ros::Time::now().toSec();
                    samples.push_back(s);
                }
                else if (nRet != MV_E_NODATA)
                {
                    ROS_WARN("Calibration frame failed, nRet [0x%x]", nRet);
                }
            }

            if ((int)samples.size() < 2)
            {
                ROS_ERROR("Timestamp calibration failed: need >=2 frames, got %zu!", samples.size());
                if (!self->stop_requested_)
                    self->need_restart_ = true; // 标定失败也应重启, 避免节点僵尸
                free(pData);
                free(m_pBufForSaveImage);
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
                ROS_ERROR("Timestamp calibration failed: cannot derive tick frequency!");
                if (!self->stop_requested_)
                    self->need_restart_ = true;
                free(pData);
                free(m_pBufForSaveImage);
                return NULL;
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
                ROS_ERROR("Timestamp calibration failed: all offset samples rejected!");
                if (!self->stop_requested_)
                    self->need_restart_ = true;
                free(pData);
                free(m_pBufForSaveImage);
                return NULL;
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

        // 取流失败分类与计数：超时容忍并周期告警，真错误累计后退出避免静默死循环
        static const uint64_t kGrabWarnEveryN = 100;  // 连续超时每 N 次打一条 WARN
        static const uint64_t kGrabErrBreakN  = 50;   // 连续真错误超过 N 次退出取流
        uint64_t timeout_cnt = 0;                     // 连续超时计数（取到一帧后清零）
        uint64_t err_cnt     = 0;                     // 连续真错误计数
        float last_exposure_set = -1.0f;              // 上次写入相机的曝光值，用于去抖

        while (ros::ok() && !self->stop_requested_)
        {
            // 仅当曝光时间实际变化时才写入，避免取流过程中高频调用废弃接口 MV_CC_SetExposureTime
            if (exposure_auto == 0 && std::fabs(exposure_time_set - last_exposure_set) > 1e-3f)
            {
                nRet = MV_CC_SetExposureTime(p_handle, exposure_time_set);
                if (MV_OK == nRet)
                {
                    last_exposure_set = exposure_time_set;
                }
                else
                {
                    ROS_WARN_STREAM_THROTTLE(5.0, "Exposure time set failed! nRet [0x" << std::hex << nRet << "]");
                }
            }
            nRet = MV_CC_GetOneFrameTimeout(p_handle, pData, nDataSize, &stImageInfo, 1000);
            if (nRet == MV_OK)
            {
                err_cnt = 0;
                timeout_cnt = 0;
                // 使用相机设备时间戳（已标定对齐到 PC 时钟）
                uint64_t dev_ticks = ((uint64_t)stImageInfo.nDevTimeStampHigh << 32)
                                   | stImageInfo.nDevTimeStampLow;
                double dev_sec = (double)dev_ticks / (double)self->tick_frequency_;
                ros::Time capture_time(dev_sec + self->device_to_wall_offset_);

                stConvertParam.nWidth = stImageInfo.nWidth;
                stConvertParam.nHeight = stImageInfo.nHeight;
                stConvertParam.pSrcData = pData;
                stConvertParam.nSrcDataLen = nDataSize;
                stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
                stConvertParam.pDstBuffer = m_pBufForSaveImage;
                stConvertParam.nDstBufferSize = nDataSize * 3;
                stConvertParam.enSrcPixelType = stImageInfo.enPixelType;
                MV_CC_ConvertPixelTypeEx(p_handle, &stConvertParam);
                cv::Mat srcImage;
                srcImage = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, m_pBufForSaveImage);
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
                // 取流超时/无数据：容忍偶发超时，连续累计达阈值则请求重启句柄
                timeout_cnt++;
                if (timeout_cnt % kGrabWarnEveryN == 0)
                {
                    ROS_WARN_STREAM("Grab timeout/no-data streak: " << timeout_cnt
                                     << ", last nRet [0x" << std::hex << nRet << "]");
                }
                if (self->grab_timeout_retry_threshold_ > 0 &&
                    timeout_cnt >= (uint64_t)self->grab_timeout_retry_threshold_)
                {
                    ROS_ERROR_STREAM("Grab timeout streak " << timeout_cnt
                                     << " >= threshold " << self->grab_timeout_retry_threshold_
                                     << ", requesting camera restart.");
                    self->need_restart_ = true; // 通知主线程重启, 避免节点活但无图
                    break;
                }
                // 偶发超时: continue (下次 MV_OK 会复位 timeout_cnt)
            }
            else
            {
                // 真错误码：累计后请求重启并退出取流循环，避免线程静默空转死循环
                err_cnt++;
                ROS_ERROR_STREAM("Grab failed! nRet [0x" << std::hex << nRet
                                 << "], streak=" << err_cnt);
                if (err_cnt >= kGrabErrBreakN)
                {
                    ROS_ERROR_STREAM("Grab error streak reached " << kGrabErrBreakN
                                     << ", requesting restart.");
                    if (!self->stop_requested_)
                        self->need_restart_ = true;
                    break;
                }
            }
        }
        if (pData)
        {
            free(pData);
            pData = NULL;
        }
        if (m_pBufForSaveImage)
        {
            free(m_pBufForSaveImage);
            m_pBufForSaveImage = NULL;
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
            if (imgGrayLight < light_set - 10 && exposure_time_set * scale < exposure_time_up)
            {
                exposure_time_set *= scale;
            }
            else if (imgGrayLight > light_set + 10 && exposure_time_set / scale > exposure_time_low)
            {
                exposure_time_set /= scale;
            }
        }
    }

} // namespace HIKCAMERA