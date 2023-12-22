#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <mvs/MvCameraControl.h>
#include <hik_camera_driver/camera.h>

#define Debug

std::vector<MV_CC_DEVICE_INFO> get_all_camera()
{
    std::vector<MV_CC_DEVICE_INFO> cameras;
    cameras.clear();
    int nRet = -1;
    void *m_handle = NULL;
    unsigned int nTLayerType = MV_GIGE_DEVICE | MV_USB_DEVICE;
    MV_CC_DEVICE_INFO_LIST m_stDevList = {0};
    nRet = MV_CC_EnumDevices(nTLayerType, &m_stDevList);
    if (MV_OK != nRet)
    {
        ROS_ERROR_STREAM("error: EnumDevices fail: " << nRet);
        return cameras;
    }
    int i = 0;
    if (m_stDevList.nDeviceNum == 0)
    {
        ROS_WARN_STREAM("No camera found!");
        return cameras;
    }
    for (int index = 0; index < m_stDevList.nDeviceNum; index++)
    {
        int nDeviceIndex = index;
        MV_CC_DEVICE_INFO m_stDevInfo = {0};
        memcpy(&m_stDevInfo, m_stDevList.pDeviceInfo[nDeviceIndex], sizeof(MV_CC_DEVICE_INFO));
        std::cout << "Device " << index << " : " << std::endl;
        if (m_stDevInfo.nTLayerType == MV_GIGE_DEVICE)
        {
            std::cout.width(15);
            std::cout << "Type"
                      << ": Gige" << std::endl;
            std::cout.width(15);
            std::cout << "Serial Number"
                      << ": " << m_stDevInfo.SpecialInfo.stGigEInfo.chSerialNumber << std::endl;
            std::cout.width(15);
            std::cout << "Model Name"
                      << ": " << m_stDevInfo.SpecialInfo.stGigEInfo.chModelName << std::endl;
            int nIp1 = ((m_stDevInfo.SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
            int nIp2 = ((m_stDevInfo.SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
            int nIp3 = ((m_stDevInfo.SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
            int nIp4 = (m_stDevInfo.SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);
            std::cout.width(15);
            std::cout << "IP"
                      << ": " << nIp1 << "." << nIp2 << "." << nIp3 << "." << nIp4 << std::endl;
            std::cout.width(15);
            std::cout << "Net Export"
                      << ": " << m_stDevInfo.SpecialInfo.stGigEInfo.nNetExport << std::endl;
            cameras.push_back(m_stDevInfo);
        }
        else if (m_stDevInfo.nTLayerType == MV_USB_DEVICE)
        {
            std::cout.width(15);
            std::cout << "Type"
                      << ": USB" << std::endl;
            std::cout.width(15);
            std::cout << "Serial Number"
                      << ": " << m_stDevInfo.SpecialInfo.stUsb3VInfo.chSerialNumber << std::endl;
            std::cout.width(15);
            std::cout << "Device Number"
                      << ": " << m_stDevInfo.SpecialInfo.stUsb3VInfo.nDeviceNumber << std::endl;
            std::cout.width(15);
            std::cout << "Model Name"
                      << ": " << m_stDevInfo.SpecialInfo.stUsb3VInfo.chModelName << std::endl;
            std::cout.width(15);
            std::cout << "Device GUID"
                      << ": " << m_stDevInfo.SpecialInfo.stUsb3VInfo.chDeviceGUID << std::endl;
            cameras.push_back(m_stDevInfo);
        }
    }
    return cameras;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hik_driver");
    ros::NodeHandle nh, private_nh("~");
    std::vector<MV_CC_DEVICE_INFO> cameras;
    cameras = get_all_camera();
    if (cameras.size() == 0)
    {
        return EXIT_SUCCESS;
    }

    std::string serial_number;
    private_nh.param<std::string>("Camera/serial_number", serial_number, "00DA1999130");

    HIKCAMERA::Hik_camera_base camera(nh, private_nh);
    if (serial_number == "")
    {
        camera.set_camera(cameras[0]);
    }
    else
    {
        bool flag = false;
        for (auto &temp : cameras)
        {
            char *temp_serial_number;
            if (temp.nTLayerType == MV_GIGE_DEVICE)
            {
                temp_serial_number = (char *)temp.SpecialInfo.stGigEInfo.chSerialNumber;
            }
            else if (temp.nTLayerType == MV_USB_DEVICE)
            {
                temp_serial_number = (char *)temp.SpecialInfo.stUsb3VInfo.chSerialNumber;
            }
            std::string string_serial_number = temp_serial_number;
            if (string_serial_number == serial_number)
            {
                camera.set_camera(temp);
                flag = true;
                break;
            }
        }
        if (!flag)
        {
            ROS_ERROR_STREAM("Can Not Find Camera: " << serial_number);
            return EXIT_SUCCESS;
        }
    }
    camera.set_params();
    camera.ImageStream();

    ros::Rate rate(100);
#ifdef Debug
    int i = 1;
#endif
    while (ros::ok())
    {
        camera.ImagePub();
#ifdef Debug
        i++;
        if (i % 100 == 0)
        {
            float gain, exposure_time, FrameRate, gamma, DigitalShift;
            camera.getFloatValue("ExposureTime", exposure_time);
            camera.getFloatValue("Gain", gain);
            camera.getFloatValue("ResultingFrameRate", FrameRate);
            camera.getFloatValue("DigitalShift", DigitalShift);
            ROS_INFO_STREAM("-----------------------");
            ROS_INFO_STREAM("ExposureTime: " << exposure_time);
            ROS_INFO_STREAM("Gain        : " << gain);
            ROS_INFO_STREAM("DigitalShift: " << DigitalShift);
            i = 0;
        }
#endif
        ros::spinOnce();
        rate.sleep();
    }
    camera.stopStream();

    return EXIT_SUCCESS;
}