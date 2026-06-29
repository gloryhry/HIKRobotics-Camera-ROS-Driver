
#ifndef _MV_CAMERA_CTRL_H_
#define _MV_CAMERA_CTRL_H_

#include "MvErrorDefine.h"
#include "CameraParams.h"
#include "MvObsoleteInterfaces.h"



#ifndef MV_CAMCTRL_API

#if (defined (_WIN32) || defined(WIN64))
#if defined(MV_CAMCTRL_EXPORTS)
#define MV_CAMCTRL_API __declspec(dllexport)
#else
#define MV_CAMCTRL_API __declspec(dllimport)
#endif
#else
#ifndef __stdcall
#define __stdcall
#endif

#ifndef MV_CAMCTRL_API
#define  MV_CAMCTRL_API
#endif
#endif

#endif

#ifdef MV_CAMCTRL_API

#if (defined (_WIN32) || defined(WIN64))
	#if defined(MV_CAMCTRL_EXPORTS)
		#define MV_CAMCTRL_API __declspec(dllexport)
	#else
		#define MV_CAMCTRL_API __declspec(dllimport)
	#endif
	#else
		#ifndef __stdcall
			#define __stdcall
		#endif

		#if defined(MV_CAMCTRL_EXPORTS)
			#define  MV_CAMCTRL_API __attribute__((visibility("default")))
		#else
			#define  MV_CAMCTRL_API
		#endif
	#endif

#endif

#ifndef IN
    #define IN
#endif

#ifndef OUT
    #define OUT
#endif

#ifdef __cplusplus
extern "C" {
#endif 


/****************************** ch: 摘要 | en: Instructions**********************************************/

/** 
 *   @~chinese
 *     该头文件主要包含14部分：
 *     0.回调函数定义
 *     1.SDK初始化
 *     2.相机的配置(枚举/打开/关闭）和取流接口
 *     3.采集卡的配置(枚举/打开/关闭）
 *     4.相机/采集卡属性万能配置接口&读写寄存器接口
 *     5.相机/采集卡 升级
 *     6.相机和采集卡 注册异常回调和事件接口
 *     7.仅GigE Vision设备支持的接口
 *     8.仅CameraLink 设备支持的接口
 *     9.仅USB3 Vision设备支持的接口
 *     10.GenTL相关接口
 *     11.图像保存、格式转换等相关接口
 *     12.适用于支持串口通信的设备接口
 *     13.适用于支持液态镜头的设备接口

 *   @~english
 *     This header file mainly includes 14 sections:
 *     0.Callback function definition
 *     1.SDK initialization
 *     2.Camera configuration (enumeration/open/close) and streaming API
 *     3.Frame grabber configuration (enumeration/open/close)
 *     4.Universal property configuration API & register read/write API for cameras/frame grabbers
 *     5.Firmware upgrade for cameras/frame grabbers
 *     6.Exception callback registration and event API for cameras and frame grabbers
 *     7.API exclusively for GigE Vision devices
 *     8.API exclusively for CameraLink devices
 *     9.API exclusively for USB3 Vision (U3V) devices
 *     10.GenTL-related API
 *     11.Image saving and format conversion API
 *     12.API for devices supporting serial communication
 *     13.API for devices supporting liquid lens
**/


/*******************Part0 ch: 回调函数定义 | en: Callback function definition*******************/

/// \addtogroup 回调函数定义
/// @{

/********************************************************************//**
 *  @~chinese
 *  @brief  取图回调函数
 *  @param  pData                       [OUT]           图像数据指针
 *  @param  pFrameInfo                  [OUT]           图像信息结构体
 *  @param  pUser                       [OUT]           用户自定义变量

 *  @~english
 *  @brief  Image Callback function
 *  @param  pData                       [OUT]           It refers to the pointer to image data. 
 *  @param  pFrameInfo                  [OUT]           It refers to the image information structure. 
 *  @param  pUser                       [OUT]           It refers to the user-defined variable.
*****************************************************************************/
typedef void(__stdcall *MvImageCallbackEx)(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser);

/********************************************************************//**
 *  @~chinese
 *  @brief  取图回调函数
 *  @param  pstFrame                    [OUT]           图像数据和图像信息
 *  @param  pUser                       [OUT]           用户自定义变量
 *  @param  bAutoFree                   [OUT]           true-回调函数退出后自动释放图像缓存，false-回调结束不释放图像缓存，需调用 MV_CC_FreeImageBuffer() 


 *  @~english
 *  @brief  Image callback function
 *  @param  pstFrame                    [OUT]           It refers to the image data and information.
 *  @param  pUser                       [OUT]           It refers to the user-defined variable. 
 *  @param  bAutoFree                   [OUT]           Whether to release image buffer automatically after callback is completed. 
                                                       true: the image buffer is released automatically after callback is completed, 
                                                       false: the image buffer cannot be released automatically, and it is required to call MV_CC_FreeImageBuffer(). 

*****************************************************************************/
typedef void(__stdcall *MvImageCallbackEx2)(MV_FRAME_OUT* pstFrame, void *pUser, bool bAutoFree);

/********************************************************************//**
 *  @~chinese
 *  @brief  事件回调函数
 *  @param  pEventInfo                  [OUT]           事件信息
 *  @param  pUser                       [OUT]           用户自定义变量

 *  @~english
 *  @brief  Event callback function
 *  @param  pEventInfo                  [OUT]           It refers to the event information.
 *  @param  pUser                       [OUT]           It refers to the user-defined variable.
*****************************************************************************/
typedef void(__stdcall *MvEventCallback)(MV_EVENT_OUT_INFO * pEventInfo, void* pUser);

/********************************************************************//**
 *  @~chinese
 *  @brief  流异常回调函数
 *  @param  pstStreamExceptionInfo      [OUT]           流异常信息
 *  @param  pUser                       [OUT]           用户自定义变量

 *  @~english
 *  @brief  Stream exception callback function.
 *  @param  pstStreamExceptionInfo      [OUT]           It refers to the stream exception information. 
 *  @param  pUser                       [OUT]           It refers to the user-defined variable. 
*****************************************************************************/
typedef void(__stdcall *MvStreamExceptionCallback)(MV_CC_STREAM_EXCEPTION_INFO* pstStreamExceptionInfo, void* pUser);

/********************************************************************//**
 *  @~chinese
 *  @brief  异常消息回调
 *  @param  nMsgType                    [OUT]           异常类型
 *  @param  pUser                       [OUT]           用户自定义变量

 *  @~english
 *  @brief  Exception callback function. 
 *  @param  nMsgType                    [OUT]           It refers to the exception type.
 *  @param  pUser                       [OUT]           It refers to the user-defined variable. 
*****************************************************************************/
typedef void(__stdcall *MvExceptionCallback)(unsigned int nMsgType, void *pUser);

/********************************************************************//**
 *  @~chinese
 *  @brief  液态镜头正常消息回调
 *  @param  pLiquidLensMsg              [OUT]           液态镜头上传的消息
 *  @param  pUser                       [OUT]           用户自定义变量

 *  @~english
 *  @brief  Liquid lens normal message callback function.
 *  @param  pLiquidLensMsg              [OUT]           It refers to the uploaded message of liquid lens uploaded.
 *  @param  pUser                       [OUT]           It refers to the user-defined variable. 
*****************************************************************************/
typedef void(__stdcall *MvLiquidLensMsgCallback)(MV_CC_LIQUIDLENS_MSG* pLiquidLensMsg, void* pUser);

/********************************************************************//**
 *  @~chinese
 *  @brief  液态镜头异常消息回调
 *  @param  pLiquidLensMsg              [OUT]           液态镜头上传的消息
 *  @param  pUser                       [OUT]           用户自定义变量

 *  @~english
 *  @brief  Liquid lens exception message callback function. 
 *  @param  pLiquidLensMsg              [OUT]           It refers to the uploaded message of liquid lens uploaded.
 *  @param  pUser                       [OUT]           It refers to the user-defined variable. 
*****************************************************************************/
typedef void(__stdcall *MvLiquidLensExceptionCallback)(MV_CC_LIQUIDLENS_EXCEPTION_MSG* pLiquidLensMsg, void* pUser);
/// @}

/**************************Part1 ch: SDK 初始化 | en: SDK Initialization ******************************************/

/// \addtogroup SDK初始化
/// @{

/********************************************************************//**
 *  @~chinese
 *  @brief    初始化SDK
 *  @return   成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。

 *  @~english
 *  @brief   Initializes SDK resources.  
 *  @return  Returns MV_OK for success, and returns corresponding error code for failure.  
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_Initialize();

/********************************************************************//**
 *  @~chinese
 *  @brief    反初始化SDK，释放资源
 *  @return   成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks  main函数退出前调用

 *  @~english
 *  @brief   Releases SDK resources. 
 *  @return  Returns MV_OK for success, and returns corresponding error code for failure.  
 *  @remarks Call this API before exiting the main function. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_Finalize();

/// @}

/// \addtogroup SDK版本信息
/// @{

/********************************************************************//**
 *  @~chinese
 *  @brief  获取SDK版本号
 *  @return 返回4字节版本号
                |  主   |   次  |  修正  |  测试 |
                | :---: | :---: | :---: | :---: | 
                | 8bits | 8bits | 8bits | 8bits |
 *  @remarks \li 比如返回值为0x01000001，即SDK版本号为V1.0.0.1。
             \li  MV_CC_GetSDKVersion() 引入头文件、Lib文件后，SDK环境搭建完毕，可直接调用。
 
 *  @~english
 *  @brief  Gets SDK Version
 *  @return Always return 4 Bytes of version number 
            |Main    |Sub    |Rev  |  Test|
             8bits  8bits  8bits  8bits
 *  @remarks For example, if the return value is 0x01000001, the SDK version is V1.0.0.1.
 ************************************************************************/
MV_CAMCTRL_API unsigned int __stdcall MV_CC_GetSDKVersion();

/// @}



/**************************Part2 ch: 相机的控制和取流  | en: Camera control and streaming******************************************/

/// \addtogroup 相机初始化
/// @{

/********************************************************************//**
 *  @~chinese
 *  @brief  枚举设备
 *  @param  nTLayerType                 [IN]            枚举传输层, 参数定义参见CameraParams.h定义
                | 传输层协议类型定义 | 值 | 说明 |
                | :--- | :---: | :--- | 
                | \ref MV_UNKNOW_DEVICE | 0x00000000 | 未知设备类型 |
                | \ref MV_GIGE_DEVICE | 0x00000001 | GigE Vision设备 |
                | \ref MV_1394_DEVICE | 0x00000002 | 1394-a/b设备 |
                | \ref MV_USB_DEVICE | 0x00000004 | USB3 Vision设备 |
                | \ref MV_CAMERALINK_DEVICE | 0x00000008 | 串口设备，包含Camera Link设备和串口光源控制器 |
                | \ref MV_VIR_GIGE_DEVICE | 0x00000010 | 虚拟GigE Vision设备 |
                | \ref MV_VIR_USB_DEVICE | 0x00000020 | 虚拟USB3 Vision设备 |
                | \ref MV_GENTL_GIGE_DEVICE | 0x00000040 | 自研网卡下GigE设备 |
                | \ref MV_GENTL_CAMERALINK_DEVICE | 0x00000080 | CameraLink设备 |
                | \ref MV_GENTL_CXP_DEVICE | 0x00000100 | CoaXPress设备 |
                | \ref MV_GENTL_XOF_DEVICE | 0x00000200 | XoF设备 |
                | \ref MV_GENTL_VIR_DEVICE | 0x00000800 | 虚拟设备 |
 *  @param  pstDevList                  [IN][OUT]       设备列表
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 
 *  @remarks \li 设备列表的内存是在SDK内部分配的，多线程调用该接口时会进行设备列表内存的释放和申请，建议尽量避免多线程枚举操作。\n
 *           \li 参数枚举传输层，适配传入MV_GIGE_DEVICE、MV_1394_DEVICE、MV_USB_DEVICE、MV_CAMERALINK_DEVICE；MV_GIGE_DEVICE该参数传出所有GiGE相关的设备信息（包含虚拟GiGE和GenTL下的GiGE设备），MV_USB_DEVICE该参数传出所有USB3 Vision设备，包含虚拟USB3 Vision设备。\n
 *           \li GE1104 采集卡下的相机仅支持通过MV_GIGE_DEVICE类型枚举，不支持使用MV_GENTL_GIGE_DEVICE。
 
 *  @~english
 *  @brief  Enumerates devices, including cameras connected to frame grabbers.
 *  @param  nTLayerType                 [IN]            It refers to the transport layer protocol type. For more details, refer to CameraParams.h. for example, #define MV_GIGE_DEVICE 0x00000001
 *  @param  pstDevList                  [IN][OUT]       It refers to the device list. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks The memory of device list is internally allocated. When this API is called in multiple threads, the SDK will release and apply for the device list memory. 
             It is recommended to avoid multithreaded enumeration operations.
 *  @remarks For the parameter nTLayerType, the following parameters are supported: MV_GIGE_DEVICE, MV_1394_DEVICE, MV_USB_DEVICE, and MV_CAMERALINK_DEVICE. 
             MV_GIGE_DEVICE sends out information of all GigE devices (including virtual GigE devices and GigE devices of GenTL), and MV_USB_DEVICE sends out information of USB devices (including virtual USB devices).
 *  @remarks Cameras under the GE1104 frame grabber can only be enumerated via MV_GIGE_DEVICE; MV_GENTL_GIGE_DEVICE is not supported.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_EnumDevices(IN unsigned int nTLayerType, IN OUT MV_CC_DEVICE_INFO_LIST* pstDevList);

/********************************************************************//**
 *  @~chinese
 *  @brief  根据厂商名字枚举设备
 *  @param  nTLayerType                 [IN]            枚举传输层，参数定义参见CameraParams.h
            | 传输层协议类型定义 | 值 | 说明 |
            | :--- | :---: | :--- | 
            | \ref MV_UNKNOW_DEVICE | 0x00000000 | 未知设备类型 |
            | \ref MV_GIGE_DEVICE | 0x00000001 | GigE Vision设备 |
            | \ref MV_1394_DEVICE | 0x00000002 | 1394-a/b设备 |
            | \ref MV_USB_DEVICE | 0x00000004 | USB3 Vision设备 |
            | \ref MV_CAMERALINK_DEVICE | 0x00000008 | 串口设备，包含Camera Link设备和串口光源控制器 |
            | \ref MV_VIR_GIGE_DEVICE | 0x00000010 | 虚拟GigE Vision设备 |
            | \ref MV_VIR_USB_DEVICE | 0x00000020 | 虚拟USB3 Vision设备 |
            | \ref MV_GENTL_GIGE_DEVICE | 0x00000040 | 自研网卡下GigE设备 |
            | \ref MV_GENTL_CAMERALINK_DEVICE | 0x00000080 | CameraLink设备 |
            | \ref MV_GENTL_CXP_DEVICE | 0x00000100 | CoaXPress设备 |
            | \ref MV_GENTL_XOF_DEVICE | 0x00000200 | XoF设备 |
            | \ref MV_GENTL_VIR_DEVICE | 0x00000800 | 虚拟设备 |
 *  @param  pstDevList                  [IN][OUT]       设备列表
 *  @param  strManufacturerName         [IN]            厂商名字
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 
 *  @remarks \li 参数枚举传输层，适配传入MV_GIGE_DEVICE、MV_1394_DEVICE、MV_USB_DEVICE、MV_CAMERALINK_DEVICE；MV_GIGE_DEVICE该参数传出所有GigE Vision相关的设备信息（包含虚拟GigE Vision和GenTL下的GigE Vision设备），MV_USB_DEVICE该参数传出所有USB3 Vision设备，包含虚拟USB3 Vision设备。\n
 *           \li 设备列表的内存是在SDK内部分配的，多线程调用该接口时会进行设备列表内存的释放和申请,建议尽量避免多线程枚举操作。\n
 *           \li MV_GENTL_GIGE_DEVICE、MV_GENTL_CAMERALINK_DEVICE、MV_GENTL_CXP_DEVICE、MV_GENTL_XOF_DEVICE传输层可以返回对采集卡下的相机信息。
 *           \li GE1104 采集卡下的相机仅支持通过MV_GIGE_DEVICE类型枚举，不支持使用MV_GENTL_GIGE_DEVICE。

 *  @~english
 *  @brief  Enumerates devices according to manufacturers.
 *  @param  nTLayerType                 [IN]            It refers to the transport layer protocol type. For more details, refer to CameraParams.h. for example, #define MV_GIGE_DEVICE 0x00000001
 *  @param  pstDevList                  [IN][OUT]       It refers to the device list.
 *  @param  strManufacturerName         [IN]            It refers to the manufacturers. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks For the parameter nTLayerType, the following parameters are supported: MV_GIGE_DEVICE, MV_1394_DEVICE, MV_USB_DEVICE, and MV_CAMERALINK_DEVICE. 
             MV_GIGE_DEVICE sends out information of all GigE devices (including virtual GigE devices and GigE devices of GenTL), and MV_USB_DEVICE sends out information of USB devices (including virtual USB devices).
 *  @remarks The memory of device list is internally allocated. When this API is called in multiple threads, the SDK will release and apply for the device list memory. 
             It is recommended to avoid multithreaded enumeration operations.
 *  @remarks Cameras under the GE1104 frame grabber can only be enumerated via MV_GIGE_DEVICE; MV_GENTL_GIGE_DEVICE is not supported.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_EnumDevicesEx(IN unsigned int nTLayerType, IN OUT MV_CC_DEVICE_INFO_LIST* pstDevList, IN const char* strManufacturerName);


/********************************************************************//**
 *  @~chinese
 *  @brief  枚举设备扩展（可指定排序方式枚举、根据厂商名字过滤）
 *  @param  nTLayerType                 [IN]            枚举传输层（区分每一种传输层类型，不耦合），参数定义参见CameraParams.h
            | 传输层协议类型定义 | 值 | 说明 |
            | :--- | :---: | :--- | 
            | \ref MV_UNKNOW_DEVICE | 0x00000000 | 未知设备类型 |
            | \ref MV_GIGE_DEVICE | 0x00000001 | GigE设备 |
            | \ref MV_1394_DEVICE | 0x00000002 | 1394-a/b设备 |
            | \ref MV_USB_DEVICE | 0x00000004 | USB3 Vision设备 |
            | \ref MV_CAMERALINK_DEVICE | 0x00000008 | 串口设备，包含Camera Link设备和串口光源控制器 |
            | \ref MV_VIR_GIGE_DEVICE | 0x00000010 | 虚拟GigE设备 |
            | \ref MV_VIR_USB_DEVICE | 0x00000020 | 虚拟USB3 Vision设备 |
            | \ref MV_GENTL_GIGE_DEVICE | 0x00000040 | 自研网卡下GigE设备 |
            | \ref MV_GENTL_CAMERALINK_DEVICE | 0x00000080 | CameraLink设备 |
            | \ref MV_GENTL_CXP_DEVICE | 0x00000100 | CoaXPress设备 |
            | \ref MV_GENTL_XOF_DEVICE | 0x00000200 | XoF设备 |
            | \ref MV_GENTL_VIR_DEVICE | 0x00000800 | 虚拟设备 |
 *  @param  pstDevList                  [IN][OUT]       设备列表
 *  @param  strManufacturerName         [IN]            厂商名字（可传NULL，即不过滤）
 *  @param  enSortMethod                [IN]            排序方式
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 
 *  @remarks \li 设备列表的内存是在SDK内部分配的，多线程调用该接口时会进行设备列表内存的释放和申请，建议尽量避免多线程枚举操作。
             \li strManufacturerName可传入NULL，若传入NULL则返回排好序的所有设备列表，若不为NULL则只返回排好序的指定厂商设备列表。\n
             \li MV_GENTL_GIGE_DEVICE、MV_GENTL_CAMERALINK_DEVICE、MV_GENTL_CXP_DEVICE、MV_GENTL_XOF_DEVICE传输层可以返回对采集卡下的相机信息。\n
 *  @note    MV_CC_EnumDevicesEx2() 与 MV_CC_EnumDevicesEx() 接口的差异如下：
            \li  MV_CC_EnumDevicesEx() 传入MV_GIGE_DEVICE，除了枚举网段内的网口相机以外，还会枚举虚拟网口相机和自研采集卡下的网口相机；若传入MV_USB_DEVICE，则会枚举USB口相机和虚拟USB口相机。\n
            \li MV_CC_EnumDevicesEx2() 传入MV_GIGE_DEVICE，仅枚举网段内的网口相机；若传入MV_USB_DEVICE，则仅枚举USB口相机。\n
            \li  MV_CC_EnumDevicesEx2() 多出排序的功能，由参数MV_SORT_METHOD enSortMethod决定排序方式。
            \li GE1104 采集卡下的相机仅支持通过MV_GIGE_DEVICE类型枚举，不支持使用MV_GENTL_GIGE_DEVICE。

 *  @~english
 *  @brief  Enumerates devices, supporting enumerating devices by specified sorting method and filtering by manufacturer name. 
 *  @param  nTLayerType                 [IN]            It refers to the transport layer protocol type. For more details, refer to CameraParams.h. for example, #define MV_GIGE_DEVICE 0x00000001
 *  @param  pstDevList                  [IN][OUT]       It refers to the device list. 
 *  @param  strManufacturerName         [IN]            It refers to the name of the manufacturer (NULL means not filtering). 
 *  @param  enSortMethod                [IN]            It refers to the sorting type. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 *  @remarks The memory of device list is internally allocated. When this API is called in multiple threads, the SDK will release and apply for the device list memory. 
             It is recommended to avoid multithreaded enumeration operations.
             strManufacturerName can be set to NULL, which indicates enumerating all devices according to the specified sorting type; 
             if not set to NULL, the sorted device list of specified manufacturers will be returned.
             Cameras under the GE1104 frame grabber can only be enumerated via MV_GIGE_DEVICE; MV_GENTL_GIGE_DEVICE is not supported.

 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_EnumDevicesEx2(IN unsigned int nTLayerType, IN OUT MV_CC_DEVICE_INFO_LIST* pstDevList, IN const char* strManufacturerName, IN MV_SORT_METHOD enSortMethod);

/********************************************************************//**
 *  @~chinese
 *  @brief  通过采集卡句柄枚举设备
 *  @param  handle                    [IN]            采集卡句柄
 *  @param  pstDevList                [OUT]           设备列表
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks \li 设备列表的内存是在SDK内部分配的，多线程调用该接口时会进行设备列表内存的释放和申请，\n
            \li 建议尽量避免多线程枚举操作。

 *  @~english
 *  @brief  Enumerates devices by frame grabber handle. 
 *  @param  handle                   [IN]            It refers to the frame grabber handle. 
 *  @param  pstDevList               [OUT]           It refers to the device list. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks The memory of device list is internally allocated. When this API is called in multiple threads, the SDK will release and apply for the device list memory. It is recommended to avoid multithreaded enumeration operations.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_EnumDevicesByInterface(IN void* handle, OUT MV_CC_DEVICE_INFO_LIST* pstDevList);

/********************************************************************//**
 *  @~chinese
 *  @brief  设备是否可连接
 *  @param  pstDevInfo                  [IN]            设备信息结构体
 *  @param  nAccessMode                 [IN]            访问权限，参数定义参见CameraParams.h定义 
                |   宏定义  |  宏定义值   |  含义  | 
                | :--- | :---: | :--- | 
                | \ref MV_ACCESS_Exclusive | 1 | 独占权限，其他APP只允许读CCP寄存器 |
                | \ref MV_ACCESS_ExclusiveWithSwitch | 2 | 可以从5模式下抢占权限，然后以独占权限打开 |
                | \ref MV_ACCESS_Control | 3 | 控制权限，其他APP允许读所有寄存器 |
                | \ref MV_ACCESS_ControlWithSwitch | 4 | 可以从5模式下抢占权限，然后以控制权限打开 |
                | \ref MV_ACCESS_ControlSwitchEnable | 5 | 以可被抢占的控制权限打开 |
                | \ref MV_ACCESS_ControlSwitchEnableWithKey | 6 | 可以从5模式下抢占权限，然后以可被抢占的控制权限打开 |
                | \ref MV_ACCESS_Monitor | 7 | 读模式打开设备，适用于控制权限下 |
 *  @return 若设备可达，返回true；若设备不可达，返回false。
 *  @remarks \li GigE Vision相机: 读取设备CCP寄存器的值，判断当前状态是否具有某种访问权限。\n
             如果设备(MV_GENTL_GIGE_DEVICE/MV_GENTL_GIGE_DEVICE)不支持MV_ACCESS_ExclusiveWithSwitch、MV_ACCESS_ControlWithSwitch、MV_ACCESS_ControlSwitchEnable、MV_ACCESS_ControlSwitchEnableWithKey这四种模式，接口返回false。（目前设备不支持这4种抢占模式，国际上主流的厂商的设备也都暂不支持这4种模式。）
             \li GigE Vision/GenTL 类型设备：按照nAccessMode，返回当前是否可以被连接;
             @note
             \li 该接口支持虚拟相机，USB3 Vision相机，CXP, XoF, CameraLink采集卡相机, nAccessMode无效，如果相机没有被连接返回true， 如果设备被第三方连接，则返回false
             \li 该接口不支持CameraLink设备(返回false)
 *  @~english
 *  @brief  Checks if the specified device can be accessed. 
 *  @param  pstDevInfo                  [IN]            It refers to device information. 
 *  @param  nAccessMode                 [IN]             It refers to access mode. Refer to the 'CameraParams.h' for parameter definitions, for example, #define MV_ACCESS_Exclusive 1  (This parameter is only valid for devices of type MV_GIGE-DEVICE/MV_GENTL_GIGE-DEVICE)
 *  @return Returns true for accessible status, and false for inaccessible status.
 *  @remarks You can read the device CCP register value to check the current access permission. 
             Return false if the device(MV_GENTL_GIGE_DEVICE/MV_GENTL_GIGE_DEVICE)  does not support the modes MV_ACCESS_ExclusiveWithSwitch, MV_ACCESS_ControlWithSwitch, MV_ACCESS_ControlSwitchEnable and MV_ACCESS_ControlSwitchEnableWithKey. Currently, the device does not support the 4 modes, neither do the devices from other mainstream manufacturers. 
             This API supports virtual cameras, USB3 Vision cameras, CoaXPress (CXP), XoF, and CameraLink frame grabber cameras. The nAccessMode parameter has no actual effect. It returns true if the camera is not connected, and false if the device is occupied by a third party.
             This API does not support CameraLink devices (returns false).
 **************************************************************************/
MV_CAMCTRL_API bool __stdcall MV_CC_IsDeviceAccessible(IN MV_CC_DEVICE_INFO* pstDevInfo, IN unsigned int nAccessMode);

/********************************************************************//**
 *  @~chinese
 *  @brief  创建设备句柄
 *  @param  handle                      [IN][OUT]       设备句柄
 *  @param  pstDevInfo                  [IN]            设备信息结构体
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 
 *  @remarks \li 根据输入的设备信息，创建库内部必须的资源和初始化内部模块。\n
             \li 通过该接口创建句柄，调用SDK接口，会默认生成SDK日志文件，如果不需要生成日志文件，可以将日志配置文件中的日志等级改成off。
 
 *  @~english
 *  @brief  Creates a device handle. 
 *  @param  handle                      [IN][OUT]       It refers to the device handle. 
 *  @param  pstDevInfo                  [IN]            It refers to device information. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks Create required resources within library and initialize internal module according to input device information.
             SDK log file will be created by default when you call the API to create the device handle. If log file generation is not required, you can set the log level to off in the log configuration file.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_CreateHandle(IN OUT void ** handle, IN const MV_CC_DEVICE_INFO* pstDevInfo);

/********************************************************************//**
 *  @~chinese
 *  @brief  打开设备
 *  @param  handle                      [IN]            设备句柄
 *  @param  nAccessMode                 [IN]            访问权限, 参数定义参见CameraParams.h定义 
                |   宏定义  |  宏定义值   |  含义  | 
                | :--- | :---: | :--- | 
                | \ref MV_ACCESS_Exclusive | 1 | 独占权限，其他APP只允许读CCP寄存器 |
                | \ref MV_ACCESS_ExclusiveWithSwitch | 2 | 可以从5模式下抢占权限，然后以独占权限打开 |
                | \ref MV_ACCESS_Control | 3 | 控制权限，其他APP允许读所有寄存器 |
                | \ref MV_ACCESS_ControlWithSwitch | 4 | 可以从5模式下抢占权限，然后以控制权限打开 |
                | \ref MV_ACCESS_ControlSwitchEnable | 5 | 以可被抢占的控制权限打开 |
                | \ref MV_ACCESS_ControlSwitchEnableWithKey | 6 | 可以从5模式下抢占权限，然后以可被抢占的控制权限打开 |
                | \ref MV_ACCESS_Monitor | 7 | 读模式打开设备，适用于控制权限下 |
 *  @param  nSwitchoverKey              [IN]            切换访问权限时的密钥                                                        （仅对 GigE Vision 类型的设备有效）
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 根据设置的设备参数，找到对应的设备，连接设备, 调用接口时可不传入nAccessMode和nSwitchoverKey，此时默认设备访问模式为独占权限。\n
            \li GigE Vision 类型设备，目前相机固件暂不支持MV_ACCESS_ExclusiveWithSwitch、MV_ACCESS_ControlWithSwitch、MV_ACCESS_ControlSwitchEnable、MV_ACCESS_ControlSwitchEnableWithKey这四种抢占模式, 可通过SDK接口设置。\n
            \li MV_GENTL_GIGE_DEVICE 设备只支持 nAccessMode 是 MV_ACCESS_Exclusive 、MV_ACCESS_Control 、MV_ACCESS_Monitor权限。\n
            \li 对于USB3 Vision设备，CXP，Cameralink(MV_CAMERALINK_DEVICE、MV_GENTL_CAMERALINK_DEVICE)，XoF设备，虚拟GigE Vision，虚拟USB3 Vision设备：nAccessMode、nSwitchoverKey这两个参数无效； 默认以控制权限打开设备。\n
            \li 该接口支持网口设备不枚举直接打开，不支持U口和GenTL设备不枚举打开设备。\n

  *  @~english
  *  @brief  Turns on the device. 
  *  @param  handle                      [IN]            It refers to the device handle. 
  *  @param  nAccessMode                 [IN]            It refers to access mode. Refer to the 'CameraParams.h'.
  *  @param  nSwitchoverKey              [IN]            It refers to the secret key for switching access permission.  (only valid for devices of type MV_GIGE_DEVICE.)
  *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
  *  @remarks You can find the device and connect it according to the configured device parameters. 
              The parameters nAccessMode and nSwitchoverKey are optional, and the device access mode is exclusive permission by default. 
              For GigE devices, the camera firmware does not support the following preemption modes: MV_ACCESS_ExclusiveWithSwitch, MV_ACCESS_ControlWithSwitch, MV_ACCESS_ControlSwitchEnable, and MV_ACCESS_ControlSwitchEnableWithKey.
              For GenTL devices, the camera firmware only supports the following modes: MV_ACCESS_Exclusive, MV_ACCESS_Control, and MV_ACCESS_Monitor. 
              For USB3 Vision, CXP, camera link, XoF, virtual GEV, and virtual USB3 Vision devices, the parameters nAccessMode and nSwitchoverKey are invalid, and the device is opened with control permission via MV_ACCESS_Control by default. 
              This API allows turning on GigE devices without enumeration, but it does not suport turning on USB or GenTL devices without enumeration. 
 ************************************************************************/
#ifndef __cplusplus
MV_CAMCTRL_API int __stdcall MV_CC_OpenDevice(IN void* handle, IN unsigned int nAccessMode, IN unsigned short nSwitchoverKey);
#else
MV_CAMCTRL_API int __stdcall MV_CC_OpenDevice(IN void* handle, IN unsigned int nAccessMode = MV_ACCESS_Exclusive, IN unsigned short nSwitchoverKey = 0);
#endif

/********************************************************************//**
 *  @~chinese
 *  @brief  判断设备是否处于连接状态
 *  @param  handle                      [IN]            设备句柄
 *  @return 若设备处于连接状态，返回true；没连接或失去连接，返回false
 
 *  @~english
 *  @brief  Checks if the camera is connected. 
 *  @param  handle                      [IN]             It refers to the device handle. 
 *  @return Returns true if the device is connected, and returns false if the device is not connected or disconnected. 
 ***********************************************************************/
MV_CAMCTRL_API bool __stdcall MV_CC_IsDeviceConnected(IN void* handle);

/********************************************************************//**
 *  @~chinese
 *  @brief  获取设备信息，取流之前调用
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstDevInfo                  [IN][OUT]       返回给调用者有关设备信息结构体指针
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks \li 支持用户在打开设备后获取设备信息，不支持通过GenTL设备信息创建的设备。 \n
             \li 若该设备是GigE设备，则调用该接口存在阻塞风险，因此不建议在取流过程中调用该接口。
 
 *  @~english
 *  @brief  Gets device information, used before image grabbing. 
 *  @param  handle                      [IN]            It refers to the device handle. 
 *  @param  pstDevInfo                  [IN][OUT]       It refers to the pointer to device information structure. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks Call this API after MV_CC_OpenDevice() to get the device information.The API does not support devices created using GenTL device information.
             For a GigE device, there is a blocking risk when calling the API, so it is not recommended to call the API during image grabbing. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetDeviceInfo(IN void * handle, IN OUT MV_CC_DEVICE_INFO* pstDevInfo);

/********************************************************************//**
 *  @~chinese
 *  @brief  获取各种类型的信息
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstInfo                     [IN][OUT]       返回给调用者有关设备各种类型的信息结构体指针
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks \li 接口里面输入需要获取的信息类型（指定 MV_ALL_MATCH_INFO 结构体中的nType类型），获取对应的信息（在 MV_ALL_MATCH_INFO 结构体中pInfo里返回）。\n
            \li 该接口的调用前置条件取决于所获取的信息类型，获取GigE设备的 \ref MV_MATCH_TYPE_NET_DETECT 信息需在开启抓图之后调用，获取USB3 Vision设备的 \ref MV_MATCH_TYPE_USB_DETECT 信息需在打开设备之后调用。\n
            \li 信息类型 MV_MATCH_TYPE_NET_DETECT 对应结构体\ref MV_MATCH_INFO_NET_DETECT ， 只支持MV_GIGE_DEVICE相机/MV_GENTL_GIGE_DEVICE相机。 \n
            \li 信息类型 MV_MATCH_TYPE_USB_DETECT 对应结构体\ref MV_MATCH_INFO_USB_DETECT ， 只支持MV_USB_DEVICE 类型相机。 \n
            @note 该接口不支持MV_CAMERALINK_DEVICE设备。
 
 *  @~english
 *  @brief  Gets information of all types. 
 *  @param  handle                      [IN]            It refers to the device handle. 
 *  @param  pstInfo                     [IN][OUT]       It refers to the pointer to information structure. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks Input required information type (specify nType in structure MV_ALL_MATCH_INFO ), and get corresponding information (returned via pInfo in structure MV_ALL_MATCH_INFO ).
             The calling precondition of this API is determined by the required information type. To obtain MV_MATCH_TYPE_NET_DETECT information of GigE devices, this API should be called after image grabbing starts. To obtain MV_MATCH_TYPE_USB_DETECT information of USB3 Vision devices, this API should be called after the device is turned on.
             The information type MV_MATCH_TYPE_NET_DETECT corresponds to the structure MV_MATCH_INFO_NET_DETECT, which only supports  cameras of  MV_GIGE_DEVICE and MV_GENTL_GIGE_DEVICE types
             The information type MV_MATCH_TYPE_USB_DETECT corresponds to the structure MV_MATCH_INFO_USB_DETECT, which only supports cameras of MV_USB_DEVICE type
             This API is not supported by MV_CAMERALINK_DEVICE device. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetAllMatchInfo(IN void* handle, IN OUT MV_ALL_MATCH_INFO* pstInfo);

/********************************************************************//**
 *  @~chinese
 *  @brief  关闭设备
 *  @param  handle                      [IN]            设备句柄
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 通过 MV_CC_OpenDevice() 连接设备后，可以通过该接口断开设备连接，释放资源。
 
 *  @~english
 *  @brief  Turns off the device. 
 *  @param  handle                      [IN]            It refers to the device handle. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks This API is used to disconnect the device and release resources. 
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_CloseDevice(IN void* handle);

/********************************************************************//**
 *  @~chinese
 *  @brief  销毁设备句柄
 *  @param  handle                      [IN]            设备句柄
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 
 *  @remarks 若向本接口传入采集卡句柄，其效果和 MV_CC_DestroyInterface() 相同;
 
 *  @~english
 *  @brief  Destroys the device handle. 
 *  @param  handle                      [IN]            It refers to the device handle. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks If a frame grabber handle is passed to MV_CC_DestroyHandle, its effect is identical to MV_CC_DestroyInterface.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_DestroyHandle(IN void * handle);

/// @}

/// \addtogroup 图像采集
/// @{

/********************************************************************//**
 *  @~chinese
 *  @brief  注册图像数据回调
 *  @param  handle                      [IN]            设备句柄
 *  @param  cbOutput                    [IN]            回调函数指针
 *  @param  pUser                       [IN]            用户自定义变量
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 通过该接口可以设置图像数据回调函数，在 MV_CC_CreateHandle() 之后即可调用。 \n
 *  @remarks 图像数据采集有两种方式，两种方式不能复用： \n
 *          \li 调用 MV_CC_RegisterImageCallBackEx() 设置图像数据回调函数，然后调用 MV_CC_StartGrabbing() 开始采集，采集的图像数据在设置的回调函数中返回。 \n 
 *          \li 调用 MV_CC_StartGrabbing() 开始采集，然后在应用层循环调用 MV_CC_GetImageBuffer() 和 MV_CC_FreeImageBuffer() 获取指定像素格式的帧数据，获取帧数据时上层应用程序需要根据帧率控制好调用该接口的频率。 \n
            @note 该接口不支持 MV_CAMERALINK_DEVICE() 的设备。 \n
 
 *  @~english
 *  @brief  Registers an image data callback (extended API 1).
 *  @param  handle                      [IN]            It refers to the device handle. 
 *  @param  cbOutput                    [IN]            It refers to the pointer to the callback function. 
 *  @param  pUser                       [IN]            It refers to user-defined variable. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks Before calling this API to register an image data callback, you should call the API MV_CC_CreateHandle().
             There are two image acquisition methods, and the two methods cannot be used together:
             Method  1:  Call MV_CC_RegisterImageCallBackEx() to register an image data callback, and then call MV_CC_StartGrabbing() to start image acquisition. The collected image data will be returned in the configured callback function. 
             Method  2: Call MV_CC_StartGrabbing() to start image acquisition, and the call MV_CC_GetOneFrameTimeout() repeatedly in application layer to get frame data in specified pixel format. When getting frame data, the upper application program should control the frequency of calling this API according to frame rate. 
             This API is not supported by devices with MV_CAMERALINK_DEVICE
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_RegisterImageCallBackEx(IN void* handle, IN MvImageCallbackEx cbOutput, IN void* pUser);

/********************************************************************//**
*  @~chinese
*  @brief  注册图像数据回调
*  @param  handle                      [IN]            设备句柄
*  @param  cbOutput                    [IN]            回调函数指针
*  @param  bAutoFree                   [IN]            图像缓存自动回收标记
  \li true：回调结束后，图像缓存会被SDK回收
  \li false：回调结束后，需要调用 MV_CC_FreeImageBuffer() 接口才能回收图像缓存
*  @param  pUser                       [IN]            用户自定义变量
*  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码
*  @remarks \li 通过该接口可以设置图像数据回调函数，在 MV_CC_CreateHandle() 之后即可调用。
            \li 该接口不支持 \ref MV_CAMERALINK_DEVICE 类型的设备。
            \li 该方式与 MV_CC_RegisterImageCallBackEx() 等注册图像回调函数的接口互斥； 
            \li 该接口与 MV_CC_GetImageBuffer() 、 MV_CC_GetOneFrameTimeout() 等主动取流接口互斥； 
            \li 该接口中 \ref MvImageCallbackEx2 的“MV_FRAME_OUT* pstFrame”为SDK内部变量。如需在回调函数外使用，您需浅拷贝pstFrame结构体(不需要拷贝图像数据)。

 *  @~english
 *  @brief  Registers an image data callback (extended API 2). 
 *  @param  handle                      [IN]            It refers to the device handle. 
 *  @param  cbOutput                    [IN]            It refers to the pointer to the callback function. 
 *  @param  bAutoFree                   [IN]            It refers to the mark for automatic releasing of image buffer. (true:The image buffer will be released and reused by SDK after callback. false:After callback, it is required to call MV_CC_FreeImageBuffer() to release and reuse the image buffer.)
 *  @param  pUser                       [IN]            It refers to the user-defined variable. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks Before calling this API to register an image data callback, you should call the API MV_CC_CreateHandle().
            When getting frame data, the upper application program should control the frequency of calling this API according to frame rate. 
            This API is not supported by devices with MV_CAMERALINK_DEVICE.
            The pstFrame parameter in the callback function is an internal temporary variable of the SDK, and its content must be copied before it can be used outside the callback.
***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_RegisterImageCallBackEx2(IN void* handle, IN MvImageCallbackEx2 cbOutput, IN void* pUser, IN bool bAutoFree);

/********************************************************************//**
 *  @~chinese
 *  @brief  开始取流
 *  @param  handle                      [IN]            设备句柄
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 该接口不支持 \ref MV_CAMERALINK_DEVICE 类型的设备。
 
 *  @~english
 *  @brief  Starts image grabbing. 
 *  @param  handle                      [IN]            It refers to the device handle. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 *  @remarks This API is not supported by MV_CAMERALINK_DEVICE.
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_StartGrabbing(IN void* handle);

/********************************************************************//**
 *  @~chinese
 *  @brief  停止取流
 *  @param  handle                      [IN]            设备句柄
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 该接口不支持 \ref MV_CAMERALINK_DEVICE 类型的设备。
 
 *  @~english
 *  @brief  Stops image grabbing. 
 *  @param  handle                      [IN]            It refers to the device handle. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 *  @remarks This API is not supported by MV_CAMERALINK_DEVICE.
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_StopGrabbing(IN void* handle);

/********************************************************************//**
 *  @~chinese
 *  @brief  获取一帧图片（与 MV_CC_Display() 不能同时使用）
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstFrame                    [IN][OUT]       图像数据和图像信息
 *  @param  nMsec                       [IN]            等待超时时间，输入INFINITE时表示无限等待，直到收到一帧数据或者停止取流
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks \li 调用该接口获取图像数据帧之前需要先调用 MV_CC_StartGrabbing() 启动图像采集。该接口为主动式获取帧数据，上层应用程序需要根据帧率，控制好调用该接口的频率。该接口支持设置超时时间，SDK内部等待直到有数据时返回，可以增加取流平稳性，适合用于对平稳性要求较高的场合。 \n
             \li 该接口与 MV_CC_FreeImageBuffer() 配套使用，当处理完取到的数据后，需要用 MV_CC_FreeImageBuffer() 接口将pstFrame内的数据指针权限进行释放。 \n
             \li 该接口与 MV_CC_GetOneFrameTimeout() 相比，有着更高的效率。且其取流缓存的分配由sdk内部自动分配或者外部注册，而 MV_CC_GetOneFrameTimeout() 接口是需要客户自行分配。\n 
             \li 该接口在调用 MV_CC_Display() 后无法取流。 \n
 
 *  @~english
 *  @brief  Gets one frame of image. (This API cannot be used with MV_CC_Display() at the same time.) 
 *  @param  handle                      [IN]            It refers to the device handle. 
 *  @param  pstFrame                    [IN][OUT]       It refers to image data and information. 
 *  @param  nMsec                       [IN]            It refers to the timeout duration, unit: millisecond. You can input INFINITE to set unlimited timeout duration, and image grabbing will not stop until a frame of data is received or the image grabbing is manually stopped. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks Before calling this API to get image data frame, call MV_CC_StartGrabbing to start image acquisition.
             This API can get frame data actively, the upper layer program should control the frequency of calling this API according to the frame rate. This API supports setting timeout duration for receiving image data frames, which helps ensure stable image acquisition. It is applicable to scenes with high-stability requirement for getting images. 
             This API and MV_CC_FreeImageBuffer should be called in pairs, after processing the acquired data, you should call MV_CC_FreeImageBuffer to release the data pointer permission of pstFrame. 
             This API's image buffer is allocated by the SDK internally or registered externally, it has higher image acquisition efficiency than MV_CC_GetOneFrameTimeout() , whose image buffer needs to be manually allocated by the user. 
             This API cannot be called to grab images after calling MV_CC_Display(). 
 *****************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetImageBuffer(IN void* handle, IN OUT MV_FRAME_OUT* pstFrame, IN unsigned int nMsec);

/********************************************************************//**
 *  @~chinese
 *  @brief  释放图像缓存（此接口用于释放不再使用的图像缓存，与 MV_CC_GetImageBuffer() 配套使用）
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstFrame                    [IN]            图像数据和图像数据
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks \li 该接口与 MV_CC_GetImageBuffer() 配套使用，使用 MV_CC_GetImageBuffer() 接口取到的图像数据pstFrame，需要用 MV_CC_FreeImageBuffer() 接口进行权限释放。 \n
             \li 该接口对于取流效率高于 MV_CC_GetOneFrameTimeout() 接口，且 MV_CC_GetImageBuffer() 在不进行Free的情况下，最大支持输出的节点数与 MV_CC_SetImageNodeNum() 接口所设置的节点数相等，默认节点数是1。\n 
 
 *  @~english
 *  @brief  Releases image buffer. (This API is used to release the image buffer that is no longer used, and it is used with MV_CC_GetImageBuffer() in pairs.) 
 *  @param  handle                      [IN]            It refers to the device handle. 
 *  @param  pstFrame                    [IN]            It refers to image data and information. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks This API and MV_CC_GetImageBuffer() should be called in pairs. After calling MV_CC_GetImageBuffer() to get image data pstFrame, call MV_CC_FreeImageBuffer() to release the permission. 
             This API has higher efficiency of image acquisition than the API MV_CC_GetOneFrameTimeout(). The max. number of nodes that can be outputted by MV_CC_GetImageBuffer()(without freeing the buffer) is the same as the "nNum" set by the API MV_CC_SetImageNodeNum(). 
 **********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_FreeImageBuffer(IN void* handle, IN MV_FRAME_OUT* pstFrame);

/********************************************************************//**
 *  @~chinese
 *  @brief  采用超时机制获取一帧图片，SDK内部等待直到有数据时返回
 *  @param  handle                      [IN]            设备句柄
 *  @param  pData                       [IN][OUT]       图像数据接收指针
 *  @param  nDataSize                   [IN]            接收缓存大小
 *  @param  pstFrameInfo                [IN][OUT]       图像信息结构体
 *  @param  nMsec                       [IN]            等待超时时间
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 调用该接口获取图像数据帧之前需要先调用 MV_CC_StartGrabbing() 启动图像采集。该接口为主动式获取帧数据，上层应用程序需要根据帧率，控制好调用该接口的频率。该接口支持设置超时时间，SDK内部等待直到有数据时返回，可以增加取流平稳性，适合用于对平稳性要求较高的场合。\n 
 
 *  @~english
 *  @brief  Gets one frame of image with timeout, and the SDK waits internally to return until data is available. 
 *  @param  handle                      [IN]            It refers to the device handle. 
 *  @param  pData                       [IN][OUT]       It refers to the pointer to receive image data. 
 *  @param  nDataSize                   [IN]            It refers to received buffer size. 
 *  @param  pstFrameInfo                [IN][OUT]       It refers to the structure of image information. 
 *  @param  nMsec                       [IN]            It refers to timeout duration. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks Before calling this API to get image data frame, call MV_CC_StartGrabbing() to start image acquisition. 
             This API can get frame data actively, the upper layer program should control the frequency of calling this API according to the frame rate.
             This API supports setting timeout duration for receiving image data frames, which helps ensure stable image acquisition. It is applicable to scenes with high-stability requirement for getting images. 
             This API is supported by both USB3 vision devices and GigE devices.
             This API is not supported by MV_CAMERALINK_DEVICE.
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetOneFrameTimeout(IN void* handle, IN OUT unsigned char* pData , IN unsigned int nDataSize, IN OUT MV_FRAME_OUT_INFO_EX* pstFrameInfo, IN unsigned int nMsec);

/********************************************************************//**
 *  @~chinese
 *  @brief  清除取流数据缓存
 *  @param  handle                      [IN]            设备句柄
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks \li 该接口允许用户在不停止取流的时候，就能清除缓存中不需要的图像。 \n
             \li 该接口在连续模式切触发模式后，可以清除历史数据。 \n
             \li 该接口当前仅支持清除SDK内部的图像缓存，暂不支持清除采集卡内的缓存。
 
 *  @~english
 *  @brief  Clears the image buffer. 
 *  @param  handle                      [IN]            It refers to the device handle. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks Call this API to clear the images you do not need in the buffer when the streaming is in progress. 
             After switching to trigger mode from continuous mode, you can call this API to clear historical data buffer. 
             Call this API to clear internal image buffer of SDK. The clearing of the buffer inside the frame grabbers is not supported. 
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_ClearImageBuffer(IN void* handle);

/********************************************************************//**
 *  @~chinese
 *  @brief  获取当前图像缓存区的有效图像个数
 *  @param  handle                      [IN]            设备句柄
 *  @param  pnValidImageNum             [IN][OUT]       当前图像缓存区中有效图像个数的指针
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 该接口只统计SDK内部的有效图像个数，不包括采集卡缓存内的有效图像个数
 
 *  @~english
 *  @brief  Gets the number of valid images in the current image buffer. 
 *  @param  handle                      [IN]            It refers to the device handle. 
 *  @param  pnValidImageNum             [IN][OUT]       It refers to the pointer to the number of valid images in the current image buffer. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks This API can only be called to get the number of valid images in the SDK, not the number of those in frame grabbers. 
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetValidImageNum(IN void* handle, IN OUT unsigned int *pnValidImageNum);

/********************************************************************//**
 *  @~chinese
 *  @brief  设置SDK内部图像缓存节点个数，大于等于1，在抓图前调用
 *  @param  handle                      [IN]            设备句柄
 *  @param  nNum                        [IN]            缓存节点个数
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks \li 调用该接口可以设置SDK内部图像缓存节点个数，在调用 MV_CC_StartGrabbing() 开始抓图前调用。 \n
             \li 该接口不支持MV_CAMERALINK_DEVICE设备。 \n
             \li 网口、U口相机会在MV_CC_StartGrabbing时进行内存校验，若系统内存不足，则MV_CC_StartGrabbing返回异常。 \n
             \li 其他采集卡下相机，MV_CC_StartGrabbing时检测系统内存资源不够，SDK内部将重新计算预分配的缓存节点个数，在该情况下，SDK实际分配的节点个数以重新计算的节点个数为准。

 *  @~english
 *  @brief  Sets the number of nodes for SDK internal image buffer. The value is no less than 1, and this API should be called before image grabbing. 
 *  @param  handle                      [IN]            It refers to the device handle. 
 *  @param  nNum                        [IN]            It refers to the number of buffer nodes, which cannot be less than 1. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks Call this API before MV_CC_StartGrabbing() to set the number of nodes for SDK internal image buffer.
             This API does not support devices of type MV_CAMERALINK_DEVICE
             For cameras with network interface or U port, the SDK will perform a memory check when calling MV_CC_StartGrabbing. If the system memory is insufficient, MV_CC_StartGrabbing will return an exception
             For cameras using other capture cards, when calling MV_CC_StartGrabbing, the SDK detects that the system memory is insufficient and will internally recalculate the number of pre-allocated buffer nodes. In this case, the actual number of nodes allocated by the Software Development Kit (SDK) will be based on the recalculated number.
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetImageNodeNum(IN void* handle, IN unsigned int nNum);

/********************************************************************//**
 *  @~chinese
 *  @brief  设置取流策略
 *  @param  handle                      [IN]            设备句柄
 *  @param  enGrabStrategy              [IN]            策略枚举值
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 该接口定义了四种取流策略，用户可以根据实际需求进行选择。具体描述如下：
            \li OneByOne：从旧到新一帧一帧的从输出缓存列表中获取图像，打开设备后默认为该策略
            \li LatestImagesOnly：仅从输出缓存列表中获取最新的一帧图像，同时清空输出缓存列表
            \li LatestImages：从输出缓存列表中获取最新的OutputQueueSize帧图像，其中OutputQueueSize范围为1-ImageNodeNum，可用 MV_CC_SetOutputQueueSize() 接口设置，ImageNodeNum默认为1，可用 MV_CC_SetImageNodeNum() 接口设置 OutputQueueSize设置成1等同于LatestImagesOnly策略，OutputQueueSize设置成ImageNodeNum等同于OneByOne策略
            \li UpcomingImage：在调用取流接口时忽略输出缓存列表中所有图像，并等待设备即将生成的一帧图像。
            @note
            \li WINDOWS
            该接口支持 \ref MV_GIGE_DEVICE 、 \ref MV_USB_DEVICE 、 \ref MV_GENTL_GIGE_DEVICE 、 \ref MV_GENTL_CAMERALINK_DEVICE 、 \ref MV_GENTL_CXP_DEVICE 、 \ref MV_GENTL_XOF_DEVICE 设备。
            \endif
            \li LINUX
            该接口支持 \ref MV_USB_DEVICE 、 \ref MV_GENTL_GIGE_DEVICE 、 \ref MV_GENTL_CAMERALINK_DEVICE 、 \ref MV_GENTL_CXP_DEVICE 、 \ref MV_GENTL_XOF_DEVICE 设备。
            \endif

 *  @~english
 *  @brief  Sets image grabbing strategy. 
 *  @param  handle                      [IN]            It refers to the device handle. 
 *  @param  enGrabStrategy              [IN]            It refers to strategy enumeration.
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks There are four defined image grabbing strategies, from which you can choose the suitable one according to the actual requirement. Details are following. 
                OneByOne:Gets image frames one by one in chronological order. It is the default strategy. 
                LatestImagesOnly:Only gets the latest one frame from the output buffer list, and clears the rest images in the list. 
                LatestImages:Gets the latest image of OutputQueueSize frame from the output buffer list. The range of OutputQueueSize is between 1 and ImageNodeNum.
                             If the OutputQueueSize value is set to 1, the strategy is same to LatestImagesOnly, and if the OutputQueueSize value is set to ImageNodeNum, the strategy is same to OneByOne.
                             You can set the OutputQueueSize via API MV_CC_SetOutputQueueSize(), and set the ImageNodeNum via API MV_CC_SetImageNodeNum(). 
                UpcomingImage:Ignores all images in the output buffer list during calling this API, and waits for the upcoming image generated by the device. 
                Device Types Supported by Windows Platform: MV_GIGE_DEVICE、MV_USB_DEVICE、MV_GENTL_GIGE_DEVICE、MV_GENTL_CAMERALINK_DEVICE、MV_GENTL_CXP_DEVICE、MV_GENTL_XOF_DEVICE.
                Device Types Supported by Linux Platform: MV_USB_DEVICE、MV_GENTL_GIGE_DEVICE、MV_GENTL_CAMERALINK_DEVICE、MV_GENTL_CXP_DEVICE、MV_GENTL_XOF_DEVICE.

 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetGrabStrategy(IN void* handle, IN MV_GRAB_STRATEGY enGrabStrategy);

/********************************************************************//**
 *  @~chinese
 *  @brief  设置输出缓存个数（只有在 \ref MV_GrabStrategy_LatestImages 策略下才有效，范围：1-ImageNodeNum）
 *  @param  handle                      [IN]            设备句柄
 *  @param  nOutputQueueSize            [IN]            输出缓存个数
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 该接口需与LatestImages取流策略配套调用，用于设置LatestImages策略下最多允许缓存图像的个数。可以在取流过程中动态调节输出缓存个数。
            @note
            \li WINDOWS
            该接口支持 \ref MV_GIGE_DEVICE 、 \ref MV_USB_DEVICE 、 \ref MV_GENTL_GIGE_DEVICE 、 \ref MV_GENTL_CAMERALINK_DEVICE 、 \ref MV_GENTL_CXP_DEVICE 、 \ref MV_GENTL_XOF_DEVICE 设备。
            \endif
            \li LINUX
            该接口支持 \ref MV_USB_DEVICE 、 \ref MV_GENTL_GIGE_DEVICE 、 \ref MV_GENTL_CAMERALINK_DEVICE 、 \ref MV_GENTL_CXP_DEVICE 、 \ref MV_GENTL_XOF_DEVICE 设备。
            \endif

 *  @~english
 *  @brief  Sets the number of output buffers, range: [1, ImageNodeNum]. 
 *  @param  handle                      [IN]            It refers to the device handle. 
 *  @param  nOutputQueueSize            [IN]            It refers to the number of output buffers. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks This API is valid only when MV_CC_SetGrabStrategy() is set to LatestImages. It is called to set the max. number of buffer images under the LatestImages strategy. You can adjust the number of output buffers during image grabbing. 
             The user may change the output queue size while grabbing images.
             This API only support MV_GIGE_DEVICE, MV_USB_DEVICE device on Windows, and only support MV_USB_DEVICE device on Linux.
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetOutputQueueSize(IN void* handle, IN unsigned int nOutputQueueSize);

/********************************************************************//**
 *  @~chinese
 *  @brief  分配对齐内存
 *  @param  nBufSize                     [IN]            分配内存的长度
 *  @param  nAlignment                   [IN]            内存对齐字节数 (必须是大于0，并且是2的整数次幂)
 *  @return 成功，返回申请内存地址；失败，返回 NULL

 *  @~english
 *  @brief  Allocates aligned memory
 *  @param  nBufSize                     [IN]        It refers to allocation length of memory     
 *  @param  nAlignment                   [IN]        It refers to memory alignment size (must be greater than 0 and a power of 2)
 *  @return Returns memory address for success, and returns NULL for failure.
 *  @remarks 
************************************************************************/
MV_CAMCTRL_API void *  __stdcall MV_CC_AllocAlignedBuffer(IN uint64_t  nBufSize, IN unsigned int nAlignment);

/********************************************************************//**
 *  @~chinese
 *  @brief  对齐内存释放
 *  @param  pBuffer                      [IN]            内存地址
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 对齐内存的释放，搭配 MV_CC_AllocAlignedBuffer() 使用

 *  @~english 
 *  @brief   Releases aligned memory
 *  @param   pBuffer                      [IN]        It refers to memory address
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 *  @remarks This API and MV_CC_AllocAlignedBuffer should be called in pairs.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_FreeAlignedBuffer(IN void* pBuffer);

/********************************************************************//**
 *  @~chinese
 *  @brief  获取设备payload大小（payload包含图像数据和Chunk数据等）和内存对齐方式，用于SDK外部注册缓存时，应用层分配足够的缓存及正确的内存对齐方式
 *  @param  handle                      [IN]            设备句柄
 *  @param  pnPayloadSize               [IN OUT]        负载长度
 *  @param  pnAlignment                 [IN OUT]        负载内存对齐的字节数
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 获取到的payload大小如果想要改大，需要按照对齐字节数的整数倍增加，否则MV_CC_AllocAlignedBuffer申请的内存可能出现不按要求对齐的情况

 *  @~english
 *  @brief  Gets the device payload size (including image data and Chunk data, and more.) and memory alignment method. 
           It is used by the application layer to allocate sufficient buffer and correct memory alignment when registering external memory for SDK. 
 *  @param  handle                      [IN]            It refers to the device handle. 
 *  @param  pnPayloadSize               [IN OUT]        It refers to the payload size. 
 *  @param  pnAlignment                 [IN OUT]        It refers to alignment bytes.
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks If you want to increase the size of the obtained payload, it must be increased by an integer multiple of the aligned byte count. Otherwise, the memory allocated by MV_CC_AllocAlignedBuffer may not be aligned as required.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetPayloadSize(IN void* handle, IN OUT uint64_t* pnPayloadSize, IN OUT unsigned int* pnAlignment);

/********************************************************************//**
 *  @~chinese
 *  @brief  应用程序分配缓存，并注册到SDK内部，供SDK使用
 *  @param  handle                      [IN]            设备句柄
 *  @param  pBuffer                     [IN]            内存地址
 *  @param  nBufSize                    [IN]            内存长度
 *  @param  pUser                       [IN]            用户指针 
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 可以使用 MV_CC_GetPayloadSize() 获取缓存大小，并使用 MV_CC_AllocAlignedBuffer() 分配空间，之后使用 MV_CC_RegisterBuffer() 注册
            \li 注册的缓存需要由应用层通知SDK取消注册（ MV_CC_UnRegisterBuffer() ）后，进行释放（ MV_CC_FreeAlignedBuffer() ）。
            \li 使用该接口后，仅支持 MV_CC_GetImageBuffer() / MV_CC_FreeImageBuffer() / MV_CC_RegisterImageCallBackEx() 获取图像，不支持其他接口获取图像。\n
            @note \li 使用该接口后，如果之前配置了SDK内部节点（ MV_CC_SetImageNodeNum() ）无效。\n
            \li 双USB接口相机要求至少注册3块空间到SDK内部。其他相机暂无限制，但是为了避免缓存不足，请配置足够的缓存到底层。\n

 *  @~english
 *  @brief  Application allocates cache and registers it internally to the SDK for SDK usage
 *  @param  handle                      [IN]            It refers to the device handle. 
 *  @param  pBuffer                     [IN]            It refers to external memory address
 *  @param  nBufSize                    [IN]            It refers to external length of memory 
 *  @param  pUser                       [IN]            It refers to user-defined variable.
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks Registering memory can be done by using MV_CC_GetPayloadSize to obtain the memory size, and allocating the memory size using MV_CC_AllocAlignedBuffer
            The registered memory needs to be notified by the application layer to the SDK to cancel the registration (MV_CC_UnregisterBuffer) and then released (MV_CC_FreeAlignedBuffer)
            After using this API, only MV_CC_GetImageBuffer、MV_CC_FreeImageBuffer/MV_CC_RegisterImageCallBackEx is supported for image retrieval, and other API are not supported for image retrieval
            After using this API, if the SDK internal node (MV_CC_SetImageNodeNum) was previously configured, it is invalid
            The dual USB API camera requires at least 3 spaces to be registered inside the SDK;There is no limit for other cameras for the time being, but to avoid insufficient cache, please configure sufficient cache into the SDK
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_RegisterBuffer(IN void* handle, IN void *pBuffer, IN uint64_t nBufSize, IN void* pUser);

/********************************************************************//**
 *  @~chinese
 *  @brief  外部内存取消SDK内部注册
 *  @param  handle                      [IN]            设备句柄
 *  @param  pBuffer                     [IN]            外部内存地址
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。

 *  @~english
 *  @brief   Revokes external memory
 *  @param  handle                      [IN]            It refers to the device handle
 *  @param  pBuffer                     [IN]            It refers to external memory address
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_UnRegisterBuffer(IN void* handle, IN void* pBuffer);

/// @}

/// \addtogroup 图像渲染
/// @{

/********************************************************************//**
 *  @~chinese
 *  @brief  显示一帧图像（扩展接口）
 *  @param  handle                      [IN]            设备句柄
 *  @param  hWnd                        [IN]            窗口句柄
 *  @param  pstDisplayInfo              [IN]            图像信息
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 该接口支持渲染的图像宽、高可达到Int类型。
 *  \if WINDOWS
 *  可选择GDI或D3D渲染模式，默认选择GDI模式。关于渲染模式的说明如下：
    \li GDI模式：对电脑的显卡性能没有要求，适用于所有电脑。
    \li D3D模式：适用于已安装显卡驱动且显卡内存大于1GB的电脑。在该模式下，客户端预览的图像效果会优于GDI模式下的图像效果。该模式支持的最大图像分辨率为16384 * 163840。
    \li  MV_CC_DisplayOneFrameEx() 支持PixelType_Gvsp_RGB8_Packed，PixelType_Gvsp_BGR8_Packed，PixelType_Gvsp_Mono8三种像素格式的渲染宽高大小至int类型，其余像素格式渲染仅支持宽高至short。
    \endif

 *  @~english
 *  @brief  Displays one frame of image (extended API 1). 
 *  @param  handle                      [IN]            It refers to the device handle. 
 *  @param  hWnd                        [IN]            It refers to window handle. 
 *  @param  pstDisplayInfo              [IN]            It refers to image information. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks TThis API supports converting image width and image height to integer type data. The rendering mode can be set to GDI (default) or D3D. See details about rendering modes below. 
             The GDI mode is applicable to all computers with no requirements on graphic card performance. 
             The D3D mode is suitable for computers with a graphics card driver and the memory of the graphics card is greater than 1GB. In this mode, the image effect of the client preview is better than that of the GDI mode. The max. supported resolution is 16384 × 163840. 
  ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_DisplayOneFrameEx(IN void* handle, IN void* hWnd, IN MV_DISPLAY_FRAME_INFO_EX* pstDisplayInfo);

/********************************************************************//**
 *  @~chinese
 *  @brief  显示一帧图像（扩展接口2）
 *  @param  handle                      [IN]            设备句柄
 *  @param  hWnd                        [IN]            窗口句柄
 *  @param  pstImage                    [IN]            图像信息
 *  \if WINDOWS
 *  @param  enRenderMode                [IN]            渲染方式：0-GDI 1-D3D 2-OpenGL
 *  \endif
 *  \if LINUX
 *  @param  enRenderMode                [IN]            渲染方式：0-OpenGL
 *  \endif
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks
 *  \if WINDOWS
 *  此接口支持4GB以上超大图渲染，调用时需要输入 MV_CC_IMAGE 中 nImageLen 的值。 \n
             D3D和OpenGL模式适用于安装显卡驱动且显卡内存大于1GB的电脑，该模式下客户端预览的图像效果会优于GDI模式下的图像效果。渲染模式为D3D时，支持的最大分辨率为16384 * 163840。 \n
             根据图像大小是否超过4GB，该接口可选的渲染模式不同，详情如下：
             \li 若图像大小大于4GB，仅支持使用OpenGL模式渲染图像，并且支持渲染RGB8_Packed，BGR8_Packed和Mono8格式的图像。\n
             \li 若图像大小小于4GB，可根据实际情况选择GDI、D3D或OpenGL模式。若选择渲染的图像格式为RGB8_Packed，BGR8_Packed或Mono8格式，该渲染图像的宽高可达到int类型。 \n
 *  \endif
 *  \if LINUX
 *  \li 支持PixelType_Gvsp_RGB8_Packed，PixelType_Gvsp_BGR8_Packed，PixelType_Gvsp_Mono8三种像素格式图像大小超过4GB的渲染。 \n
    \li 若图像大小未超过4GB，支持宽高大小至int类型。 \n
    \li 调用时需要输入 MV_CC_IMAGE 结构体中nImageLen的值; \n
 *  \endif
 
 *  @~english
 *  @brief  Displays one frame of image (extended API 2). 
 *  @param  handle                      [IN]            It refers to the device handle. 
 *  @param  hWnd                        [IN]            It refers to window handle. 
 *  @param  pstImage                    [IN]            It refers to the image information. 
 *  @param  enRenderMode                [IN]            It refers to image rendering mode. 0: GDI; 1: D3D; 2: OpenGL.
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks This API supports rendering of images over 4 GB by entering nImageBufLen value of structure MV_CC_IMAGE. 
             D3D mode and OpenGL mode are suitable for computers with a graphics card driver and the memory of the graphics card is greater than 1GB. In these modes, the image effect of the client preview is better than that of the GDI mode. For D3D mode, the max. supported resolution is 16384 × 163840. 
             Rendering mode and supported rendering width and rendering height vary from images over 4 GB and images no more than 4 GB. 
             For an image over 4 GB, only OpenGL rendering mode is supported for image in pixel formats including RGB8_Packed, BGR8_Packed, and Mono8. 
             For image size under 4 GB, you can choose GDI, D3D, or OpenGL according to actual demand. For an image with pixel formats including RGB8_Packed, BGR8_Packed, and Mono8, it supports converting image width and image height to integer type data. 
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_DisplayOneFrameEx2(IN void* handle, IN void* hWnd, IN MV_CC_IMAGE* pstImage, unsigned int enRenderMode);

/// \cond WINDOWS

/********************************************************************//**
 *  @~chinese
 *  @brief  在图像上绘制矩形框辅助线
 *  @param  handle                      [IN]            设备句柄
 *  @param  pRectInfo                   [IN]            矩形辅助线的信息
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks  该接口仅支持windows平台
 
 *  @~english
 *  @brief  Draws auxiliary rectangle frames on the image. 
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  pRectInfo                   [IN]            It refers to the information of auxiliary rectangle frame. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks This API only supports windows platform.
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_DrawRect(IN void* handle, IN MVCC_RECT_INFO* pRectInfo);

/********************************************************************//**
 *  @~chinese
 *  @brief  在图像上绘制圆形辅助线
 *  @param  handle                      [IN]            设备句柄
 *  @param  pCircleInfo                 [IN]            圆形辅助线的信息
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks  该接口仅支持windows平台
 
 *  @~english
 *  @brief  Draws auxiliary circle frames on the image. 
 *  @param  handle                      [IN]           It refers to the device handle.
 *  @param  pCircleInfo                 [IN]           It refers to the information of auxiliary circle frame. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks This API only supports windows platform.
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_DrawCircle(IN void* handle, IN MVCC_CIRCLE_INFO* pCircleInfo);

/********************************************************************//**
 *  @~chinese
 *  @brief  在图像上绘制线条
 *  @param  handle                      [IN]            设备句柄
 *  @param  pLinesInfo                  [IN]            线条辅助线信息
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks  该接口仅支持windows平台
 
 *  @~english
 *  @brief  Draws lines on the image. 
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  pLinesInfo                  [IN]            It refers to the information of line
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks  This API only supports windows platform.
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_DrawLines(IN void* handle, IN MVCC_LINES_INFO* pLinesInfo);
/// \endcond

/// @}

/**************************Part3 ch: 采集卡的配置  | en: Frame grabber control ******************************************/

/// \addtogroup 采集卡初始化
/// @{
/********************************************************************//**
 *  @~chinese
 *  @brief   枚举采集卡
 *  @param   nTLayerType        [IN]             采集卡接口类型
                                    | 采集卡接口类型定义 | 值 | 说明 |
                                    | :--- | :---: | :--- | 
                                    | \ref MV_GIGE_INTERFACE | 0x00000001 | GigE Vision采集卡 |
                                    | \ref MV_CAMERALINK_INTERFACE | 0x00000004 | Camera Link采集卡 |
                                    | \ref MV_CXP_INTERFACE | 0x00000008 | CoaXPress采集卡 |
                                    | \ref MV_XOF_INTERFACE | 0x00000010 | XoFLink采集卡 |
                                    | \ref MV_VIR_INTERFACE | 0x00000020 | 虚拟采集卡 |
                                    | \ref MV_LC_INTERFACE | 0x00000040 | 光源控制卡 |
 *  @param   pInterfaceInfoList [IN][OUT]       采集卡列表
 *  @return  成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @if LINUX
 *  @remarks 该接口不支持arm和Linux32平台
 *  @endif

 *  @~english
 *  @brief   Enumerates frame grabbers. 
 *  @param   nTLayerType         [IN]             It refers to the frame grabber interface type.  eg: (MV_GIGE_INTERFACE | MV_CAMERALINK_INTERFACE | MV_CXP_INTERFACE| MV_XOF_INTERFACE | MV_VIR_INTERFACE | MV_LC_INTERFACE）
 *  @param   pInterfaceInfoList   [IN][OUT]       It refers to the frame grabber list. 
 *  @return  Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks This API do not support arm and Linux32 platform.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_EnumInterfaces(IN unsigned int nTLayerType, IN OUT MV_INTERFACE_INFO_LIST* pInterfaceInfoList);

/********************************************************************//**
 *  @~chinese
 *  @brief   创建采集卡句柄
 *  @param   handle             [OUT]       采集卡句柄
 *  @param   pInterfaceInfo     [IN]        采集卡信息
 *  @return  成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @if LINUX
 *  @remarks 该接口不支持arm和Linux32平台
 *  @endif

 *  @~english
 *  @brief   Creates frame grabber handle. 
 *  @param   handle              [OUT]      It refers to the frame grabber handle. 
 *  @param   pInterfaceInfo      [IN]       It refers to the frame grabber information. 
 *  @return  Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks This API do not support arm and Linux32 platform.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_CreateInterface(IN OUT void ** handle, IN MV_INTERFACE_INFO* pInterfaceInfo);

/********************************************************************//**
 *  @~chinese
 *  @brief   通过采集卡ID创建采集卡句柄
 *  @param   handle         [IN][OUT]       采集卡句柄
 *  @param   pInterfaceID   [IN]            采集卡ID
 *  @return  成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @if LINUX
 *  @remarks 该接口不支持arm和Linux32平台
 *  @endif

 *  @~english
 *  @brief   Creates frame grabber handle by frame grabber ID. 
 *  @param   handle             [IN][OUT]         It refers to the frame grabber handle. 
 *  @param   pInterfaceID       [IN]              It refers to the frame grabber ID such as frame grabber serial No. 
 *  @return  Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks This API do not support arm and Linux32 platform.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_CreateInterfaceByID(IN OUT void ** handle, IN const char* pInterfaceID);

/********************************************************************//**
 *  @~chinese
 *  @brief   打开采集卡
 *  @param   handle         [IN]        采集卡句柄
 *  @param   pReserved      [IN]        预留，直接填NULL
 *  @return  成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @if LINUX
 *  @remarks 该接口不支持arm和Linux32平台
 *  @endif

 *  @~english
 *  @brief   Turns on the frame grabber. 
 *  @param   handle         [IN]       It refers to the frame grabber handle. 
 *  @param   pReserved      [IN]       Reserved (NULL). 
 *  @return   Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks This API do not support arm and Linux32 platform.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_OpenInterface(IN void* handle, IN char* pReserved);

/********************************************************************//**
 *  @~chinese
 *  @brief   关闭采集卡
 *  @param   handle  [IN]       采集卡句柄
 *  @return  成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @if LINUX
 *  @remarks 该接口不支持arm和Linux32平台
 *  @endif

 *  @~english
 *  @brief   Turns off the frame grabber. 
 *  @param   handle  [IN]          It refers to the frame grabber handle. 
 *  @return  Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks This API do not support arm and Linux32 platform.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_CloseInterface(IN void* handle);

/********************************************************************//**
 *  @~chinese
 *  @brief   销毁采集卡句柄
 *  @param   handle  [IN]采集卡句柄
 *  @return  成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 若传入相机句柄，其效果和 MV_CC_DestroyHandle() 相同; 
 *  @if LINUX
 *  @remarks 该接口不支持arm和Linux32平台
 *  @endif

 *  @~english
 *  @brief   Destroys the frame grabber handle. 
 *  @param   handle  [IN] It refers to the frame grabber handle. 
 *  @return  Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks If camera handle is passed, the effect is the same as the MV_CC_DestroyHandle. This API do not support arm and Linux32 platform.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_DestroyInterface(IN void* handle);

/// @}


/*******************Part4 ch: 相机/采集卡属性万能配置接口 | en: Universal configuration API for camera/frame grabber properties*******************/

/// \addtogroup 属性配置
/// @{

/********************************************************************//**
 *  @~chinese
 *  @brief  获取Integer属性值
 *  @param  handle                      [IN]            设备句柄/采集卡句柄
 *  @param  strKey                      [IN]            属性键值，如获取宽度信息则为"Width"
 *  @param  pstIntValue                 [IN][OUT]       返回给调用者有关设备属性结构体指针
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 连接设备之后调用该接口可以获取int类型的指定节点的值。具体可查看客户端的属性描述。
 
 *  @~english
 *  @brief  Gets the value of integer type node. 
 *  @param  handle                      [IN]            It refers to the device handle. 
 *  @param  strKey                      [IN]            It refers to key value (node name), for example, using "Width" to get the image width. 
 *  @param  pstIntValue                 [IN][OUT]       It refers to the pointer to device feature structure. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks Call this API after connecting to the device to get the value of specified node of integer type. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetIntValueEx(IN void* handle,IN const char* strKey,IN OUT MVCC_INTVALUE_EX *pstIntValue);

/********************************************************************//**
 *  @~chinese
 *  @brief  设置Integer型属性值
 *  @param  handle                      [IN]            设备句柄/采集卡句柄
 *  @param  strKey                      [IN]            属性键值，如获取宽度信息则为"Width"
 *  @param  nValue                      [IN]            想要设置的设备的属性值
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 连接设备之后调用该接口可以设置int类型的指定节点的值，具体可以查看客户端属性描述。
 
 *  @~english
 *  @brief  Sets the value of integer type node
 *  @param  handle                      [IN]            It refers to the device handle or frame grabber handle. 
 *  @param  strKey                      [IN]            It refers to key value (node name), for example, using "Width" to set width. 
 *  @param  nValue                      [IN]            It refers to the device node value to be set. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks Call this API after connecting to the device to get the value of specified node of integer type. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetIntValueEx(IN void* handle,IN const char* strKey,IN int64_t nValue);

/********************************************************************//**
 *  @~chinese
 *  @brief  获取Enum属性值
 *  @param  handle                      [IN]            设备句柄/采集卡句柄
 *  @param  strKey                      [IN]            属性键值，如获取像素格式信息则为"PixelFormat"
 *  @param  pstEnumValue                [IN][OUT]       返回给调用者有关设备属性结构体指针
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 连接设备之后调用该接口可以获取Enum类型的指定节点的值。
 
 *  @~english
 *  @brief  Gets the value of enumeration type node. 
 *  @param  handle                      [IN]            It refers to the device handle or frame grabber handle. 
 *  @param  strKey                      [IN]            It refers to key value (node name), for example, using "PixelFormat" to get pixel format. 
 *  @param  pstEnumValue                [IN][OUT]       It refers to the pointer to device feature structure.
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks Call this API after connecting to the device to get the value of specified node of Enum type.  
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetEnumValue(IN void* handle,IN const char* strKey,IN OUT MVCC_ENUMVALUE *pstEnumValue);

/********************************************************************//**
 *  @~chinese
 *  @brief  获取Enum属性值
 *  @param  handle                      [IN]            设备句柄/采集卡句柄
 *  @param  strKey                      [IN]            属性键值，如获取像素格式信息则为"PixelFormat"
 *  @param  pstEnumValue                [IN][OUT]       返回给调用者有关设备属性结构体指针
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks \li 连接设备之后调用该接口可以获取Enum类型的指定节点的值。\n
 *           \li 区别与 MV_CC_GetEnumValue() ，此接口返回的枚举有效个数扩展到256个。

 *  @~english
 *  @brief  Gets the value of enumeration type node (extended API). 
 *  @param  handle                      [IN]            It refers to the device handle or frame grabber handle. 
 *  @param  strKey                      [IN]            It refers to key value (node name), for example, using "PixelFormat" to get pixel format. 
 *  @param  pstEnumValue                [IN][OUT]       It refers to the pointer to device feature structure.
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks Call this API after connecting to the device to get the value of specified node of Enum type
            The valid enumerations this API returns extend to 256.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetEnumValueEx(IN void* handle, IN const char* strKey, IN OUT MVCC_ENUMVALUE_EX *pstEnumValue);

/********************************************************************//**
 *  @~chinese
 *  @brief  设置Enum型属性值
 *  @param  handle                      [IN]            设备句柄/采集卡句柄
 *  @param  strKey                      [IN]            属性键值，如获取像素格式信息则为"PixelFormat"
 *  @param  nValue                      [IN]            想要设置的设备的属性值
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 连接设备之后调用该接口可以设置Enum类型的指定节点的值。
 
 *  @~english
 *  @brief  Sets the value of enumeration type node. 
 *  @param  handle                      [IN]            It refers to the device handle or frame grabber handle. 
 *  @param  strKey                      [IN]            It refers to key value (node name), for example, using "PixelFormat" to set pixel format. 
 *  @param  nValue                      [IN]            It refers to the device node value to be set. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks  Call this API after connecting to the device to get the value of specified node of Enum type 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetEnumValue(IN void* handle,IN const char* strKey,IN unsigned int nValue);

/********************************************************************//**
 *  @~chinese
 *  @brief  获取Enum型节点指定值的符号
 *  @param  handle                      [IN]            设备句柄/采集卡句柄
 *  @param  strKey                      [IN]            属性键值，如获取像素格式信息则为"PixelFormat"
 *  @param  pstEnumEntry                [IN][OUT]       想要获取的设备的属性符号
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 连接设备之后调用该接口可以获取Enum类型的指定节点的值所对应的符号。
 
 *  @~english
 *  @brief  Gets the enumerator name according to the node name and assigned value. 
 *  @param  handle                      [IN]            It refers to the device handle or frame grabber handle.
 *  @param  strKey                      [IN]            It refers to key value (node name), for example, using "PixelFormat" to get pixel format. 
 *  @param  pstEnumEntry                [IN][OUT]       It refers to the enumerator name. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks Call this API after connecting to the device to get the value of specified node of Enum type. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetEnumEntrySymbolic(IN void* handle,IN const char* strKey,IN OUT MVCC_ENUMENTRY* pstEnumEntry);

/********************************************************************//**
 *  @~chinese
 *  @brief  设置Enum型属性值
 *  @param  handle                      [IN]            设备句柄/采集卡句柄
 *  @param  strKey                      [IN]            属性键值，如获取像素格式信息则为"PixelFormat"
 *  @param  strValue                    [IN]            想要设置的设备的属性字符串
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 连接设备之后调用该接口可以设置Enum类型的指定节点的值。
           
 *  @~english
 *  @brief  Sets the value of enumeration type node
 *  @param  handle                      [IN]            It refers to the device handle or frame grabber handle.
 *  @param  strKey                      [IN]            It refers to key value (node name), for example, using "PixelFormat" to set pixel format. 
 *  @param  strValue                    [IN]            It refers to device property string to be set. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks Call this API after connecting to the device to get the value of specified node of Enum type
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetEnumValueByString(IN void* handle,IN const char* strKey,IN const char* strValue);

/********************************************************************//**
 *  @~chinese
 *  @brief  获取Float属性值
 *  @param  handle                      [IN]            设备句柄/采集卡句柄
 *  @param  strKey                      [IN]            属性键值
 *  @param  pstFloatValue               [IN][OUT]       返回给调用者有关设备属性结构体指针
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 连接设备之后调用该接口可以获取float类型的指定节点的值。
 
 *  @~english
 *  @brief  Gets the value of float type node. 
 *  @param  handle                      [IN]            It refers to the device handle or frame grabber handle.
 *  @param  strKey                      [IN]            It refers to the key value. 
 *  @param  pstFloatValue               [IN][OUT]       It refers to the structure pointer to the returned device features. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks Call this API after connecting to the device to get the value of specified node of float type. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetFloatValue(IN void* handle,IN const char* strKey,IN OUT MVCC_FLOATVALUE *pstFloatValue);

/********************************************************************//**
 *  @~chinese
 *  @brief  设置Float型属性值
 *  @param  handle                      [IN]            设备句柄/采集卡句柄
 *  @param  strKey                      [IN]            属性键值
 *  @param  fValue                      [IN]            想要设置的设备的属性值
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 连接设备之后调用该接口可以设置float类型的指定节点的值。
 
 *  @~english
 *  @brief  Sets the value of float type node. 
 *  @param  handle                      [IN]            It refers to the device handle or frame grabber handle.
 *  @param  strKey                      [IN]            It refers to the key value. 
 *  @param  fValue                      [IN]            It refers to device node value to be set.
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks Call this API after connecting to the device to get the value of specified node of float type. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetFloatValue(IN void* handle,IN const char* strKey,IN float fValue);
    
/********************************************************************//**
 *  @~chinese
 *  @brief  获取Boolean属性值
 *  @param  handle                      [IN]            设备句柄/采集卡句柄
 *  @param  strKey                      [IN]            属性键值
 *  @param  pbValue                     [IN][OUT]       返回给调用者有关设备属性值
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 连接设备之后调用该接口可以获取bool类型的指定节点的值。
 
 *  @~english
 *  @brief  Gets the value of boolean type node. 
 *  @param  handle                      [IN]            It refers to the device handle or frame grabber handle.
 *  @param  strKey                      [IN]            It refers to the key value. 
 *  @param  pbValue                     [IN][OUT]       It refers to the structure pointer for returning device features. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks Call this API after connecting to the device to get the value of specified node of bool type. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetBoolValue(IN void* handle,IN const char* strKey,IN OUT bool *pbValue);

/********************************************************************//**
 *  @~chinese
 *  @brief  设置Boolean型属性值
 *  @param  handle                      [IN]            设备句柄/采集卡句柄
 *  @param  strKey                      [IN]            属性键值
 *  @param  bValue                      [IN]            想要设置的设备的属性值
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 连接设备之后调用该接口可以设置bool类型的指定节点的值。
 
 *  @~english
 *  @brief  Sets the value of boolean type node. 
 *  @param  handle                      [IN]            It refers to the device handle or frame grabber handle.
 *  @param  strKey                      [IN]            It refers to the key value. 
 *  @param  bValue                      [IN]            It refers to device node value to be set. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks Call this API after connecting to the device to get the value of specified node of bool type. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetBoolValue(IN void* handle,IN const char* strKey,IN bool bValue);

/********************************************************************//**
 *  @~chinese
 *  @brief  获取String属性值
 *  @param  handle                      [IN]            设备句柄/采集卡句柄
 *  @param  strKey                      [IN]            属性键值
 *  @param  pstStringValue              [IN][OUT]       返回给调用者有关设备属性结构体指针
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 连接设备之后调用该接口可以获取string类型的指定节点的值。
 
 *  @~english
 *  @brief  Gets the value of string type node. 
 *  @param  handle                      [IN]            It refers to the device handle or frame grabber handle.
 *  @param  strKey                      [IN]            It refers to the key value. 
 *  @param  pstStringValue              [IN][OUT]       It refers to the structure pointer to the returned device features. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarksCall this API after connecting to the device to get the value of specified node of string type. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetStringValue(IN void* handle,IN const char* strKey,IN OUT MVCC_STRINGVALUE *pstStringValue);

/********************************************************************//**
 *  @~chinese
 *  @brief  设置String型属性值
 *  @param  handle                      [IN]            设备句柄/采集卡句柄
 *  @param  strKey                      [IN]            属性键值
 *  @param  strValue                    [IN]            想要设置的设备的属性值
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 连接设备之后调用该接口可以设置string类型的指定节点的值。
 
 *  @~english
 *  @brief  Sets the value of string type node. 
 *  @param  handle                      [IN]            It refers to the device handle or frame grabber handle.
 *  @param  strKey                      [IN]            It refers to the key value. 
 *  @param  strValue                    [IN]            It refers to device node value to be set. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks Call this API after connecting to the device to get the value of specified node of string type. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetStringValue(IN void* handle,IN const char* strKey,IN const char* strValue);

/********************************************************************//**
 *  @~chinese
 *  @brief  设置Command型属性值
 *  @param  handle                      [IN]            设备句柄/采集卡句柄
 *  @param  strKey                      [IN]            属性键值
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 连接设备之后调用该接口可以设置指定的Command类型节点。
 
 *  @~english
 *  @brief  Sets the value of command type node. 
 *  @param  handle                      [IN]            It refers to the device handle or frame grabber handle.
 *  @param  strKey                      [IN]            It refers to the key value. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks Call this API after connecting to the device to get the value of specified node of Command type. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetCommandValue(IN void* handle,IN const char* strKey);

/********************************************************************//**
 *  @~chinese
 *  @brief  获得当前节点的访问模式
 *  @param  handle                      [IN]            设备句柄/采集卡句柄
 *  @param  strName                     [IN]            节点名称
 *  @param  penAccessMode               [IN][OUT]       节点的访问模式
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。

 *  @~english
 *  @brief  Gets access mode of the current node. 
 *  @param  handle                      [IN]            It refers to the device handle or frame grabber handle.
 *  @param  strName                     [IN]            It refers to the node name. 
 *  @param  penAccessMode               [IN][OUT]       It refers to access mode for node. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_XML_GetNodeAccessMode(IN void* handle, IN const char * strName, IN OUT enum MV_XML_AccessMode *penAccessMode);

/********************************************************************//**
 *  @~chinese
 *  @brief  获得当前节点的类型
 *  @param  handle                      [IN]            设备句柄/采集卡句柄
 *  @param  strName                     [IN]            节点名称
 *  @param  penInterfaceType            [IN][OUT]       节点的类型
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 在调用 MV_CC_GetIntValueEx() 、 MV_CC_SetIntValueEx() 等万能接口之前，您可调用该接口获取节点类型，方便选择合适的接口进行节点值的设置和获取。

 *  @~english
 *  @brief  Gets the type of the current node. 
 *  @param  handle                      [IN]            It refers to the device handle or frame grabber handle.
 *  @param  strName                     [IN]            It refers to the node name. 
 *  @param  penInterfaceType            [IN][OUT]       It refers to node type. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks You can call this API to get the node type in advance before calling the universal API, so that you can choose the proper universal API to get and set node value. 
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_XML_GetNodeInterfaceType(IN void* handle, IN const char * strName, IN OUT enum MV_XML_InterfaceType *penInterfaceType);

/********************************************************************//**
 *  @~chinese
 *  @brief  导入设备属性
 *  @param  handle                      [IN]            设备句柄/采集卡句柄
 *  @param  strFileName                 [IN]            属性文件名
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 
 
 *  @~english
 *  @brief  Loads device features. 
 *  @param  handle                      [IN]            It refers to the device handle or frame grabber handle.
 *  @param  strFileName                 [IN]            It refers to the feature file name. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_FeatureLoad(IN void* handle, IN const char* strFileName);

/********************************************************************//**
 *  @~chinese
 *  @brief  导入设备属性并保存错误信息列表
 *  @param  handle                      [IN]            设备句柄/采集卡句柄
 *  @param  strFileName                 [IN]            属性文件名
 *  @param  pstNodeErrorList            [IN OUT]        错误信息列表
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 
 *  @remarks \li 部分节点导入失败时，接口返回MV_OK，通过错误信息列表中stNodeError获取出错节点及失败原因
 *           \li pstNodeErrorList该参数在外部申请并由内部填充数据。该参数可接受填入null，代表用户不关心导入时的错误信息。

 *  @~english
 *  @brief  Loads device features, and saves error information list. 
 *  @param  handle                      [IN]            It refers to the device handle or frame grabber handle.
 *  @param  strFileName                 [IN]            It refers to the feature file name. 
 *  @param  pstNodeErrorList            [IN OUT]        It refers to the error message list.
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 *  @remarks If loading part of the nodes fails, the API will return MV_OK. You can get the error node and the reason for failure through stNodeError in the error message list.
 *           The parameter pstNodeErrorList is requested by the user externally and filled with data internally, and the value null indicates that the user is not concerned about error occurred when loading.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_FeatureLoadEx(IN void* handle, IN const char* strFileName, IN OUT MVCC_NODE_ERROR_LIST* pstNodeErrorList);

/********************************************************************//**
 *  @~chinese
 *  @brief  保存设备属性
 *  @param  handle                      [IN]            设备句柄/采集卡句柄
 *  @param  strFileName                 [IN]            属性文件名
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 
 
 *  @~english
 *  @brief  Saves device features.  
 *  @param  handle                      [IN]            It refers to device handle / frame grabber handle. 
 *  @param  strFileName                 [IN]            It refers to the feature file name. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.  
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_FeatureSave(IN void* handle, IN const char* strFileName);

/// @}

/// \addtogroup 读写寄存器
/// @{

/********************************************************************//**
 *  @~chinese
 *  @brief  读内存
 *  @param  handle                      [IN]            设备句柄/采集卡句柄
 *  @param  pBuffer                     [IN][OUT]       作为返回值使用，保存读到的内存值（GigE Vision设备内存值是按照大端模式存储的，采集卡设备和采集卡下相机按照大端存储，其它协议设备按照小端存储）
 *  @param  nAddress                    [IN]            待读取的内存地址，该地址可以从设备的Camera.xml文件中获取，形如xxx_RegAddr的xml节点值
 *  @param  nLength                     [IN]            待读取的内存长度
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 读取设备某段寄存器的数据。
 
 *  @~english
 *  @brief  Read data from device register. 
 *  @param  handle                      [IN]            It refers to the device handle or frame grabber handle.
 *  @param  pBuffer                     [IN][OUT]       It refers to data buffer for saving memory value that is read  (GEV memory value is stored based on big-endian mode, frame grabber device is stored based on big-endian mode, and memory value of other devices is stored based on little-endian mode)
 *  @param  nAddress                    [IN]            It refers to memory address to be read. It can be acquired from Camera.xml file, in a form similar to XML node value of xxx_RegAddr. 
 *  @param  nLength                     [IN]            It refers to length of memory to be read. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks Access device and read the data from certain register. 
*************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_ReadMemory(IN void* handle , IN OUT void *pBuffer, IN int64_t nAddress, IN int64_t nLength);

/********************************************************************//**
 *  @~chinese
 *  @brief  写内存
 *  @param  handle                      [IN]            设备句柄/采集卡句柄
 *  @param  pBuffer                     [IN]            待写入的内存值（注意GigE Vision设备内存值要按照大端模式存储，采集卡设备和采集卡下相机按照大端存储，其它协议设备按照小端存储）
 *  @param  nAddress                    [IN]            待写入的内存地址，该地址可以从设备的Camera.xml文件中获取，形如xxx_RegAddr的xml节点值
 *  @param  nLength                     [IN]            待写入的内存长度
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 访问设备，把一段数据写入某段寄存器。
 
 *  @~english
 *  @brief  Writes data into device register. 
 *  @param  handle                      [IN]            It refers to the device handle or frame grabber handle.
 *  @param  pBuffer                     [IN]            It refers to memory value to be written(GEV memory value is stored based on big-endian mode, frame grabber device is stored based on big-endian mode, and memory value of other devices is stored based on little-endian mode)
 *  @param  nAddress                    [IN]            It refers to memory address to be written to. It can be acquired from Camera.xml file, in a form similar to XML node value of xxx_RegAddr. 
 *  @param  nLength                     [IN]            It refers to length of memory to be written. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks Access device for writing data to certain segment of register. 
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_WriteMemory(IN void* handle, IN const void *pBuffer, IN int64_t nAddress, IN int64_t nLength);

/// @}

/// \addtogroup 设备XML文件
/// @{

/********************************************************************//**
 *  @~chinese
 *  @brief  清除GenICam节点缓存
 *  @param  handle                      [IN]            设备句柄/采集卡句柄
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remark 在加载工业相机节点时需要读取GenICam配置文件，该接口可以起到清除GenICam缓存的功能。
 
 *  @~english
 *  @brief  Clears the buffer of the GenICam node. 
 *  @param  handle                      [IN]            It refers to the device handle or frame grabber handle.
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_InvalidateNodes(IN void* handle);

/********************************************************************//**
 *  @~chinese
 *  @brief  获取设备属性树XML
 *  @param  handle                      [IN]            设备句柄/采集卡句柄
 *  @param  pData                       [IN][OUT]       XML数据接收缓存
 *  @param  nDataSize                   [IN]            接收缓存大小
 *  @param  pnDataLen                   [IN][OUT]       实际数据大小
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks \li 当pData为NULL或nDataSize比实际的xml文件小时，不拷贝数据，由pnDataLen返回xml文件大小。 \n
             \li 当pData为有效缓存地址，且缓存足够大时，拷贝完整数据保存在该缓存里面，并由pnDataLen返回xml文件实际大小。 \n
 
 *  @~english
 *  @brief  Gets the XML file of device feature tree. 
 *  @param  handle                      [IN]            It refers to the device handle or frame grabber handle.
 *  @param  pData                       [IN][OUT]       It refers to received XML data buffer. 
 *  @param  nDataSize                   [IN]            It refers to received buffer size. 
 *  @param  pnDataLen                   [IN][OUT]       It refers to actual data size. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks When pData is NULL or when the value of nDataSize is less than the XML file size, no data will be copied, and the XML file size will be returned by pnDataLen. 
             When pData is valid and the buffer size is enough, the complete data will be copied and stored in the buffer, and the XML file size will be returned by pnDataLen. 
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_XML_GetGenICamXML(IN void* handle, IN OUT unsigned char* pData, IN unsigned int nDataSize, IN OUT unsigned int* pnDataLen);

/// @}

/// \addtogroup 读写相机文件
/// @{

/********************************************************************//**
 *  @~chinese
 *  @brief  从设备读取文件
 *  @param  handle                      [IN]            设备句柄/采集卡句柄
 *  @param  pstFileAccess               [IN]            文件存取结构体
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 
 
 *  @~english
 *  @brief  Reads the file from the device. 
 *  @param  handle                      [IN]            It refers to the device handle or frame grabber handle.
 *  @param  pstFileAccess               [IN]            It refers to the file access structure. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_FileAccessRead(IN void* handle, IN MV_CC_FILE_ACCESS * pstFileAccess);

/********************************************************************//**
 *  @~chinese
 *  @brief  从设备读取文件（扩展接口，文件是Data数据）
 *  @param  handle                      [IN]            设备句柄/采集卡句柄
 *  @param  pstFileAccessEx             [IN]            文件存取结构体
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 该接口直接使用缓存数据，进行读写操作，避免直接操作文件出现无权限的问题。该接口是 MV_CC_FileAccessRead() 的扩展接口。
 
 *  @~english
 *  @brief  Reads the Data file from the device (extended). 
 *  @param  handle                      [IN]            It refers to the device handle or frame grabber handle.
 *  @param  pstFileAccessEx             [IN]            It refers to the file access structure. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_FileAccessReadEx(IN void* handle, IN OUT MV_CC_FILE_ACCESS_EX * pstFileAccessEx);

/********************************************************************//**
 *  @~chinese
 *  @brief  将文件写入设备
 *  @param  handle                      [IN]            设备句柄/采集卡句柄
 *  @param  pstFileAccess               [IN]            文件存取结构体
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 
 *  @~english
 *  @brief  Writes file to device.
 *  @param  handle                      [IN]            It refers to the device handle or frame grabber handle.
 *  @param  pstFileAccess               [IN]            It refers to the file access structure. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_FileAccessWrite(IN void* handle, IN MV_CC_FILE_ACCESS * pstFileAccess);

/********************************************************************//**
 *  @~chinese
 *  @brief  将缓存(buffer)写入设备
 *  @param  handle                        [IN]            设备句柄/采集卡句柄
 *  @param  pstFileAccessEx               [IN][OUT]       文件存取结构体
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 该接口直接使用缓存数据，进行写操作，避免直接将文件写入C盘，系统保护出现写失败。该接口是 MV_CC_FileAccessWrite() 的扩展接口。
 
 *  @~english
 *  @brief  Writes buffer to device.
 *  @param  handle                        [IN]            It refers to the device handle or frame grabber handle.
 *  @param  pstFileAccessEx               [IN][OUT]       It refers to the file access structure. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 *  @remarks Call this API to write the file by using the buffer data in case of the error of system protection when operating files in C disk. This API is the extended API of MV_CC_FileAccessWrite(). 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_FileAccessWriteEx(IN void* handle, IN OUT MV_CC_FILE_ACCESS_EX * pstFileAccessEx);

/********************************************************************//**
 *  @~chinese
 *  @brief  获取文件存取的进度
 *  @param  handle                      [IN]            设备句柄/采集卡句柄
 *  @param  pstFileAccessProgress       [IN][OUT]       进度内容
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 （当前文件存取的状态）
 
 *  @~english
 *  @brief  Gets file access progress. 
 *  @param  handle                      [IN]            It refers to the device handle or frame grabber handle.
 *  @param  pstFileAccessProgress       [IN][OUT]       It refers to the file access progress. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetFileAccessProgress(IN void* handle, IN OUT MV_CC_FILE_ACCESS_PROGRESS * pstFileAccessProgress);

/// @}

/*******************Part5 ch: 相机和采集卡 升级 | en:  Camera /Frame grabber  upgrade *******************/

/// \addtogroup 设备升级
/// @{

/********************************************************************//**
 *  @~chinese
 *  @brief  设备本地升级
 *  @param  handle                      [IN]            设备句柄
 *  @param  strFilePathName             [IN]            文件名
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks  通过该接口可以将升级固件文件发送给设备进行升级。该接口需要等待升级固件文件成功传给设备端之后再返回，响应时间可能较长。
 
 *  @~english
 *  @brief  Upgrades device via local file. 
 *  @param  handle                      [IN]            It refers to the device handle. 
 *  @param  strFilePathName             [IN]            It refers to the file name. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 *  @remarks Provided this API to send upgrade firmware to device. 
             It may take a long response time since the API will only be recalled after the upgrade firmware is sent to device. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_LocalUpgrade(IN void* handle, IN const void* strFilePathName);

/********************************************************************//**
 *  @~chinese
 *  @brief  获取升级进度
 *  @param  handle                      [IN]            设备句柄
 *  @param  pnProcess                   [IN][OUT]       进度接收地址
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 
 *  @~english
 *  @brief  Gets the upgrade progress. 
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  pnProcess                   [IN][OUT]       It refers to address for receiving upgrade progress. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetUpgradeProcess(IN void* handle, IN OUT unsigned int* pnProcess);

/// @}

/*******************Part6  ch: 相机和采集卡 注册异常回调和事件接口 | en:  Exception callback registration and event API for cameras and frame grabbers*******************/

/// \addtogroup 事件及异常
/// @{

/********************************************************************//**
 *  @~chinese
 *  @brief  注册异常消息回调，在打开设备之后调用
 *  @param  handle                      [IN]            设备句柄
 *  @param  cbException                 [IN]            异常回调函数指针
 *  @param  pUser                       [IN]            用户自定义变量
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks \li 该接口需要在 MV_CC_OpenDevice() 打开设备之后调用。\n
 			 \li 设备异常断开连接后可以在回调里面获取到异常消息，GigE设备掉线之后需要先调用 MV_CC_CloseDevice() 接口关闭设备，再调用 MV_CC_OpenDevice() 接口重新打开设备。
    | 宏定义  |  宏定义值   | 含义   | 
    | :---: | :---: | :---: |
    | \ref MV_EXCEPTION_DEV_DISCONNECT  | 0x00008001  | 设备断开连接 | 
 
 *  @~english
 *  @brief  Register callback function for getting exception information of cameras and dongles. 
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  cbException                 [IN]            It refers to the pointer to the exception callback function. 
 *  @param  pUser                       [IN]            It refers to the user-defined variable. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 *  @remarks Call this API after turning on the device by calling MV_CC_OpenDevice(). 
             When the device is exceptionally disconnected, you can get the exception message from callback function. 
             For disconnected GigE device, call MV_CC_CloseDevice() to turn off the device, and then call MV_CC_OpenDevice() to turn on the device again. 
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_RegisterExceptionCallBack(IN void* handle, IN MvExceptionCallback cbException, IN void* pUser);

/********************************************************************//**
 *  @~chinese
 *  @brief  注册全部事件回调，在打开设备之后调用
 *  @param  handle                      [IN]            设备句柄
 *  @param  cbEvent                     [IN]            事件回调函数指针
 *  @param  pUser                       [IN]            用户自定义变量
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks    \li 通过该接口设置事件回调，可以在回调函数里面获取采集、曝光等事件信息。
                \li 该接口不支持CameraLink设备。
 
 *  @~english
 *  @brief  Registers a callback for all events. 
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  cbEvent                     [IN]            It refers to the pointer to the event callback function. 
 *  @param  pUser                       [IN]            It refers to user-defined variable. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 *  @remarks Call this API after connecting to the device to set the event callback function to get event information including acquisition and exposure.
             This API is not supported by CameraLink device.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_RegisterAllEventCallBack(IN void* handle, IN MvEventCallback cbEvent, IN void* pUser);

/********************************************************************//**
 *  @~chinese
 *  @brief  注册单个事件回调，在打开设备之后调用
 *  @param  handle                      [IN]            设备句柄
 *  @param  strEventName                [IN]            事件名称
 *  @param  cbEvent                     [IN]            事件回调函数指针
 *  @param  pUser                       [IN]            用户自定义变量
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码
 *  @note \li 通过该接口设置事件回调，可以在回调函数里面获取采集、曝光等事件信息。
          \li 该接口不支持CameraLink设备。
 
 *  @~english
 *  @brief  Registers a callback for single event. 
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  strEventName                [IN]            It refers to the event name. 
 *  @param  cbEvent                     [IN]            It refers to the pointer to the event callback function. 
 *  @param  pUser                       [IN]            It refers to user-defined variable. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 *  @remarks Call this API after connecting to the device to set the event callback function to get event information including acquisition and exposure.
             This API is not supported by CameraLink device .
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_RegisterEventCallBackEx(IN void* handle, IN const char* strEventName, IN MvEventCallback cbEvent, IN void* pUser);

/********************************************************************//**
 *  @~chinese
 *  @brief  注册流异常消息回调
 *  @param  handle                      [IN]            设备句柄
 *  @param  cbStreamException           [IN]            异常回调函数指针
 *  @param  pUser                       [IN]            用户自定义变量
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
*  @remarks \li 该接口注册的回调函数中不能调用 MV_CC_StopGrabbing() 、 MV_CC_CloseDevice() 和 MV_CC_DestroyHandle() ，只用于消息通知。
            \li 该接口不支持虚拟相机和导入三方cti的场景。

 *  @~english
 *  @brief  Registers a stream exception callback. 
 *  @param  handle                      [IN]            It refers to the device handle. 
 *  @param  cbStreamException           [IN]            It refers to the pointer to the exception callback function. 
 *  @param  pUser                       [IN]            It refers to the user-defined variable. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks In the callback function registered via this API, MV_CC_StopGrabbing(), MV_CC_CloseDevice(), and MV_CC_DestroyHandle() cannot be called, and the callback function can only be used for message notification. 
            This API does not support virtual cameras and scenarios involving the import of third-party CTI files.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_RegisterStreamExceptionCallBack(IN void* handle, IN MvStreamExceptionCallback cbStreamException, IN void* pUser);

/********************************************************************//**
 *  @~chinese
 *  @brief  开启设备指定事件
 *  @param  handle                      [IN]            设备句柄
 *  @param  strEventName                [IN]            事件名称
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 
 *  @~english
 *  @brief  Enables specified event of device. 
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  strEventName                [IN]            It refers to the event name.
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_EventNotificationOn(IN void* handle, IN const char* strEventName);

/********************************************************************//**
 *  @~chinese
 *  @brief  关闭设备指定事件
 *  @param  handle                      [IN]            设备句柄
 *  @param  strEventName                [IN]            事件名称
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 
 *  @~english
 *  @brief  Disable specified event of device
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  strEventName                [IN]            It refers to the event name.
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_EventNotificationOff(IN void* handle, IN const char* strEventName);

/// @}

/*******************Part7 ch: 仅GigE设备支持的接口 | en: API exclusively for GigE devices*******************/

/// \addtogroup GigE相机
/// @{

/********************************************************************//**
 *  @~chinese
 *  @brief  设置枚举超时时间，仅支持GigE协议，范围:[1, UINT_MAX)
 *  @param  nMilTimeout                 [IN]            超时时间，应为无符号整数,默认100ms
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 
 *  @remarks \li 在调用 MV_CC_EnumDevices() 等枚举接口前使用该接口，可设置枚举GIGE设备的网卡最大超时时间（默认100ms），可以减少最大超时时间，以加快枚举GIGE设备的速度。\n
             \li 该接口仅支持输入无符号整数。

 *  @~english
 *  @brief  Sets enumeration timeout duration, range: [1, UINT_MAX). Only GigE protocol is supported. 
 *  @param  nMilTimeout                 [IN]             It refers to the timeout duration, unit: millisecond. The value should be an integer (100 by default). 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks Call this API before calling enumeration APIs including MV_CC_EnumDevices() to set the timeout duration for enumerating GigE devices (100 ms by default). You can accelerate the enumeration by reducing the timeout duration. 
 *  @remarks Only supports GigE vision devices. 
             
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_SetEnumDevTimeout(IN unsigned int nMilTimeout);

/********************************************************************//**
 *  @~chinese
 *  @brief  强制IP
 *  @param  handle                      [IN]            设备句柄
 *  @param  nIP                         [IN]            设置的IP
 *  @param  nSubNetMask                 [IN]            子网掩码
 *  @param  nDefaultGateWay             [IN]            默认网关
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks \li 强制设置设备网络参数（包括IP、子网掩码、默认网关），强制设置之后将需要重新创建设备句柄，支持 [GigE Vision]( \ref MV_GIGE_DEVICE )设备和 [GenTL]( \ref MV_GENTL_GIGE_DEVICE )设备。 \n
             \li 如果设备为DHCP的状态，调用该接口强制设置设备网络参数之后设备将会重启。 \n
 
 *  @~english
 *  @brief  Sets device network parameters forcefully, including IP address, subnet mask, and default gateway. 
 *  @param  handle                      [IN]            It refers to the device handle. 
 *  @param  nIP                         [IN]            It refers to the IP address. 
 *  @param  nSubNetMask                 [IN]            It refers to the subnet mask. 
 *  @param  nDefaultGateWay             [IN]            It refers to the default gateway. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks After forcing the configuration of device network parameters (including IP address, subnet mask,and default gateway), create device handle again.
             This API is supported GigE Vision(MV_GIGE_DEVICE) and GenTL(MV_GENTL_GIGE_DEVICE) device.
             The device will restart after calling this API to set network parameters forcefully when the device is in DHCP status. 
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_ForceIpEx(IN void* handle, IN unsigned int nIP, IN unsigned int nSubNetMask, IN unsigned int nDefaultGateWay);

/********************************************************************//**
 *  @~chinese
 *  @brief  配置IP方式
 *  @param  handle                      [IN]            设备句柄
 *  @param  nType                       [IN]            IP类型，见MV_IP_CFG_x
    | 宏定义  |  宏定义值  |  含义   | 
    | :---: | :---: | :---: | 
    | \ref MV_IP_CFG_STATIC  |  0x05000000  | 固定IP地址模式   | 
    | \ref MV_IP_CFG_DHCP   | 0x06000000   | DHCP自动获取IP模式   | 
    | \ref MV_IP_CFG_LLA   | 0x04000000   | LLA(Link-local address)，链路本地地址   | 
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 发送命令设置设备的IP方式，如DHCP、LLA等，仅支持GigE Vision(MV_GIGE_DEVICE)和GenTL(MV_GENTL_GIGE_DEVICE)的设备。
 
 *  @~english
 *  @brief  Configures IP mode. 
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  nType                       [IN]            It refers to IP type. Refer to MV_IP_CFG_x for more details. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 *  @remarks This API is only supported by GigE vision devices and GenTL(MV_GENTL_GIGE_DEVICE) device. You can send command to set the MVC IP configuration mode, including DHCP and LLA. 
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_SetIpConfig(IN void* handle, IN unsigned int nType);

/********************************************************************//**
 *  @~chinese
 *  @brief  设置仅使用某种模式,type: MV_NET_TRANS_x，不设置时，默认优先使用driver
 *  @param  handle                      [IN]            设备句柄
 *  @param  nType                       [IN]            网络传输模式，见MV_NET_TRANS_x
    |  宏定义  |  宏定义值 |   含义  |  
    | :---: | :---: | :---: | 
    | \ref MV_NET_TRANS_DRIVER |   0x00000001 |   驱动模式  |  
    | \ref MV_NET_TRANS_SOCKET |   0x00000002 |   Socket模式  |
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 通过该接口可以设置SDK内部优先使用的网络模式，默认优先使用驱动模式，仅GigE Vision设备支持。
 
 *  @~english
 *  @brief  Sets SDK internal priority network mode. If it is not set, driver mode is used by default. 
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  nType                       [IN]            It refers to the network transmission mode. See MV_NET_TRANS_x for details. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 *  @remarksSet You can call this API to set the internal priority network mode for the SDK (driver mode by default). This API is supported by GigE vision devices only. 
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_SetNetTransMode(IN void* handle, IN unsigned int nType);

/********************************************************************//**
 *  @~chinese
 *  @brief  获取网络传输信息
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstInfo                     [IN][OUT]       信息结构体
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 通过该接口可以获取网络传输相关信息，包括已接收数据大小、丢帧数量等，在 MV_CC_StartGrabbing() 开启采集之后调用。仅GigE Vision相机支持。
 
 *  @~english
 *  @brief  Gets network transmission information. 
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  pstInfo                     [IN][OUT]       It refers to network transmission information structure. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 *  @remarks Call this API to get information about network transmission after grabbing images via calling MV_CC_StartGrabbing(), including received data size and the number of lost frames. 
             This API is supported only by GigE Vision devices.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_GetNetTransInfo(IN void* handle, IN OUT MV_NETTRANS_INFO* pstInfo);

/********************************************************************//**
 *  @~chinese
 *  @brief  设置枚举命令的回复包类型
 *  @param  nMode                       [IN]            回复包类型（默认广播），0-单播，1-广播
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 
 *  @remarks 该接口只对GigE Vision相机有效。

 *  @~english
 *  @brief  Sets the ACK mode of enumeration command. 
 *  @param  nMode                       [IN]            It refers to the ACK mode (default broadcast). 0: unicast; 1: broadcast. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 *  @remarks This API is only supported by GigE devices. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_SetDiscoveryMode(IN unsigned int nMode);

/********************************************************************//**
 *  @~chinese
 *  @brief  设置GVSP取流超时时间
 *  @param  handle                      [IN]            设备句柄
 *  @param  nMillisec                   [IN]            超时时间，默认300ms，范围：>=10ms
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 
 *  @remarks 连接设备之后，取流动作发生前，调用该接口可以设置GVSP取流超时时间。GVSP取流超时设置过短可能造成图像异常，设置过长可能造成取流时间变长。
 
 *  @~english
 *  @brief  Sets timeout duration for image grabbing via GVSP. 
 *  @param  handle                      [IN]            It refers to the device handle. 
 *  @param  nMillisec                   [IN]            It refers to timeout duration (unit:millisecond), range:>10ms. The default value is 300 ms. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 *  @remarks After connecting to the device and before starting image grabbing, call this API to set timeout duration for image grabbing via GVSP. 
             Image exception might occur if timeout duration is too short, and the streaming duration will become longer if timeout duration is too long. 
 *           
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_SetGvspTimeout(IN void* handle, IN unsigned int nMillisec);

/********************************************************************//**
 *  @~chinese
 *  @brief  获取GVSP取流超时时间
 *  @param  handle                      [IN]            设备句柄
 *  @param  pnMillisec                  [IN][OUT]       超时时间指针，以毫秒为单位
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 
 *  @remarks 该接口用于获取当前的GVSP取流超时时间
 
 *  @~english
 *  @brief  Gets timeout for image grabbing via GVSP. 
 *  @param  handle                      [IN]            It refers to the device handle. 
 *  @param  pnMillisec                  [IN][OUT]       It refers to pointer to the timeout duration, unit: millisecond. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 *  @remarks Call this API to get the current timeout duration of image grabbing via GVSP. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_GetGvspTimeout(IN void* handle, IN OUT unsigned int* pnMillisec);

/********************************************************************//**
 *  @~chinese
 *  @brief  设置GVCP命令超时时间
 *  @param  handle                      [IN]            设备句柄
 *  @param  nMillisec                   [IN]            超时时间(ms)，默认500ms，范围：[0,10000]
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 
 *  @remarks 连接设备之后调用该接口可以设置GVCP命令超时时间。
 
 *  @~english
 *  @brief  Sets timeout for GVCP command. 
 *  @param  handle                      [IN]            It refers to the device handle. 
 *  @param  nMillisec                   [IN]            It refers to the timeout duration, range: [0, 10000], unit: millisecond. It is 500 ms by default. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 *  @remarks Call this API after connecting to the device to set the timeout of GVCP command. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_SetGvcpTimeout(IN void* handle, IN unsigned int nMillisec);

/********************************************************************//**
 *  @~chinese
 *  @brief  获取GVCP命令超时时间
 *  @param  handle                      [IN]            设备句柄
 *  @param  pnMillisec                  [IN][OUT]       超时时间指针，以毫秒为单位
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 
 *  @remarks 该接口用于获取当前的GVCP超时时间。
 
 *  @~english
 *  @brief  Gets timeout duration for GVCP command. 
 *  @param  handle                      [IN]            It refers to the device handle. 
 *  @param  pnMillisec                  [IN][OUT]       It refers to pointer to the timeout duration, unit: millisecond. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 *  @remarks Call this API to get the current GVCP timeout duration. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_GetGvcpTimeout(IN void* handle, IN OUT unsigned int* pnMillisec);

/********************************************************************//**
 *  @~chinese
 *  @brief  设置重传GVCP命令次数
 *  @param  handle                      [IN]            设备句柄
 *  @param  nRetryGvcpTimes             [IN]            重传次数，范围：0-100
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 
 *  @remarks 该接口用于在GVCP包传输异常时，增加重传的次数，在一定程度上可以避免设备掉线，范围为0-100。
 
 *  @~english
 *  @brief  Sets the number of times for resending GVCP command.
 *  @param  handle                      [IN]            It refers to the device handle. 
 *  @param  nRetryGvcpTimes             [IN]            It refers to the number of times for resending. It should be between 0 and 100. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 *  @remarks Call this API to increase the resending times when exception occurred during GVCP packet transmission, range: [0, 100]. To some extent, it can prevent the device from getting offline. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_SetRetryGvcpTimes(IN void* handle, IN unsigned int nRetryGvcpTimes);

/********************************************************************//**
 *  @~chinese
 *  @brief  获取重传GVCP命令次数
 *  @param  handle                      [IN]            设备句柄
 *  @param  pnRetryGvcpTimes            [IN][OUT]       重传次数指针
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 
 *  @remarks 该接口用于获取当前的GVCP重传次数，默认3次。
 
 *  @~english
 *  @brief  Gets the number of times for resending GVCP command. 
 *  @param  handle                      [IN]            It refers to the device handle. 
 *  @param  pnRetryGvcpTimes            [IN][OUT]       It refers to the pointer to resending times. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 *  @remarks Call this API to get the current resending times of GVCP command (3 by default). 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_GetRetryGvcpTimes(IN void* handle, IN OUT unsigned int* pnRetryGvcpTimes);

/********************************************************************//**
 *  @~chinese
 *  @brief  获取最佳的packet size，该接口目前只支持GigE Vision设备
 *  @param  handle                      [IN]            设备句柄
 *  @return 最佳packetsize
 *  @remarks \li 获取最佳的packet size，对应GigE Vision设备是SCPS，对应USB3 Vision设备是每次从驱动读取的包大小，该大小即网络上传输一个包的大小。\n
             \li 该接口需要在 MV_CC_OpenDevice() 之后、 MV_CC_StartGrabbing() 之前调用。\n
             \li 该接口不支持CameraLink设备、USB3 Vision设备。\n
             \li 该接口不支持GenTL设备（协议不支持），如果是GenTL方式添加的网口相机，建议根据网络实际情况配置GevSCPSPacketSize，或者配置1500。
 
 *  @~english
 *  @brief  Gets the optimal packet size. 
 *  @param  handle                      [IN]            It refers to the device handle. 
 *  @return Returns optimal packet size. 
 *  @remarks The optimal packet size for GigE Vision device is SCPS.
             This API should be called after calling MV_CC_OpenDevice(), and before calling MV_CC_StartGrabbing(). 
             This API is not supported by CameraLink device and USB3 Vision device. 
			 This API is not supported by GenTL device (unsupported protocols). For GigE Vision cameras added via GenTL, configure GevSCPSPacketSize or configure 1500 as needed. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetOptimalPacketSize(IN void* handle);

/********************************************************************//**
 *  @~chinese
 *  @brief  设置是否打开重发包支持，及重发包设置
 *  @param  handle                      [IN]            设备句柄
 *  @param  bEnable                     [IN]            是否支持重发包
 *  @param  nMaxResendPercent           [IN]            最大重发比
 *  @param  nResendTimeout              [IN]            重发超时时间，范围：0-10000ms
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 
 *  @remarks 连接设备之后调用该接口可以设置重发包属性，仅GigE Vision设备支持。
 
 *  @~english
 *  @brief  Sets whether to enable packet resending, and sets corresponding parameters. 
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  bEnable                     [IN]            Whether to enable packet resending. 
 *  @param  nMaxResendPercent           [IN]            It refers to the max. resending percent. 
 *  @param  nResendTimeout              [IN]            It refers to resending timeout duration. rang：0-10000ms
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks Call this API after connecting to the device to set packet resending parameters. This API is only supported by GigE vision devices. 
 ************************************************************************/
#ifndef __cplusplus
MV_CAMCTRL_API int __stdcall MV_GIGE_SetResend(IN void* handle, IN unsigned int bEnable, IN unsigned int nMaxResendPercent, IN unsigned int nResendTimeout);
#else
MV_CAMCTRL_API int __stdcall MV_GIGE_SetResend(IN void* handle, IN unsigned int bEnable, IN unsigned int nMaxResendPercent = 100, IN unsigned int nResendTimeout = 50);
#endif

/********************************************************************//**
 *  @~chinese
 *  @brief  设置重传命令最大尝试次数
 *  @param  handle                      [IN]            设备句柄
 *  @param  nRetryTimes                 [IN]            重传命令最大尝试次数，默认20
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 
 *  @remarks 该接口必须在调用 MV_GIGE_SetResend() 开启重传包功能之后调用，否则失败且返回MV_E_CALLORDER
 
 *  @~english
 *  @brief  Sets the max. command resending times. 
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  nRetryTimes                 [IN]            It refers to the max. command resending times. It is 20 by default. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks This API should be called after resending packet is enabled via calling MV_GIGE_SetResend(). If APIs are not called in order, it will be failed and MV_E_CALLORDER will be returned. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall  MV_GIGE_SetResendMaxRetryTimes(IN void* handle, IN unsigned int nRetryTimes);

/********************************************************************//**
 *  @~chinese
 *  @brief  获取重传命令最大尝试次数
 *  @param  handle                      [IN]            设备句柄
 *  @param  pnRetryTimes                [IN][OUT]       重传命令最大尝试次数
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 
 *  @remarks 该接口必须在调用 MV_GIGE_SetResend() 开启重传包功能之后调用，否则失败且返回MV_E_CALLORDER。
 
 *  @~english
 *  @brief  Gets the max. command resending times. 
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  pnRetryTimes                [IN][OUT]       It refers to the max. times to retry resending lost packets
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks This API should be called after resending packet is enabled via calling MV_GIGE_SetResend(). If APIs are not called in order, it will be failed and MV_E_CALLORDER will be returned. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall  MV_GIGE_GetResendMaxRetryTimes(IN void* handle, IN OUT unsigned int* pnRetryTimes);

/********************************************************************//**
 *  @~chinese
 *  @brief  设置同一重传包多次请求之间的时间间隔
 *  @param  handle                      [IN]            设备句柄
 *  @param  nMillisec                   [IN]            同一重传包多次请求之间的时间间隔，默认10ms
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 
 *  @remarks 该接口必须在调用 MV_GIGE_SetResend() 开启重传包功能之后调用，否则失败且返回MV_E_CALLORDER。
 
 *  @~english
 *  @brief  Sets the time interval of two resending requests for one packet. 
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  nMillisec                   [IN]            It refers to the time interval of two resending requests for one packet. It is 10 ms by default. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks This API should be called after resending packet is enabled via calling MV_GIGE_SetResend(). If APIs are not called in order, MV_GIGE_SetResendTimeInterval() calling will be failed and MV_E_CALLORDER will be returned. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall  MV_GIGE_SetResendTimeInterval(IN void* handle, IN unsigned int nMillisec);

/********************************************************************//**
 *  @~chinese
 *  @brief  获取同一重传包多次请求之间的时间间隔
 *  @param  handle                      [IN]            设备句柄
 *  @param  pnMillisec                  [IN][OUT]       同一重传包多次请求之间的时间间隔
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 
 *  @remarks 该接口必须在调用 MV_GIGE_SetResend() 开启重传包功能之后调用，否则失败且返回MV_E_CALLORDER。
 
 *  @~english
 *  @brief  Gets the time interval of two resending requests for one packet. 
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  pnMillisec                  [IN][OUT]       It refers to the time interval of two resending requests for one packet. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks This API should be called after resending packet is enabled via calling MV_GIGE_SetResend(). If APIs are not called in order, MV_GIGE_GetResendTimeInterval() calling will be failed and MV_E_CALLORDER will be returned. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall  MV_GIGE_GetResendTimeInterval(IN void* handle, IN OUT unsigned int* pnMillisec);

/********************************************************************//**
 *  @~chinese
 *  @brief  设置传输模式，可以为单播模式、组播模式等
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstTransmissionType         [IN]            传输模式结构体
    | 宏定义 | 宏定义值  | 含义 | 
    | :---: | :---: | :---: |
    | \ref MV_GIGE_TRANSTYPE_UNICAST |  0 | 单播 |
    | \ref MV_GIGE_TRANSTYPE_MULTICAST | 1 |组播 |
    | \ref MV_GIGE_TRANSTYPE_LIMITEDBROADCAST  | 2 | 表示局域网内广播 |
    | \ref MV_GIGE_TRANSTYPE_SUBNETBROADCAST  | 3 | 表示子网内广播 |
    | \ref MV_GIGE_TRANSTYPE_CAMERADEFINED | 4 | 表示从相机获取 |
    | \ref MV_GIGE_TRANSTYPE_UNICAST_DEFINED_PORT | 5 | 表示用户自定义应用端接收图像数据Port号 |
    | \ref MV_GIGE_TRANSTYPE_UNICAST_WITHOUT_RECV | 00010000 | 表示设置了单播，但本实例不接收图像数据 |
    | \ref MV_GIGE_TRANSTYPE_MULTICAST_WITHOUT_RECV | 00010001 | 表示组播模式，但本实例不接收图像数据 |
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 
 *  @remarks 通过该接口可以设置传输模式为单播、组播等模式，仅GigE Vision设备支持。

 *  @~english
 *  @brief  Sets the transmission mode, including unicast and multicast. 
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  pstTransmissionType         [IN]            It refers to the transmission mode structure. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks Call this API to set the transmission mode as unicast mode and multicast mode.This API is only supported by GigE vision devices. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_SetTransmissionType(IN void* handle, IN MV_TRANSMISSION_TYPE * pstTransmissionType);

/********************************************************************//**
 *  @~chinese
 *  @brief   发出动作命令
 *  @param   pstActionCmdInfo           [IN]            动作命令信息
 *  @param   pstActionCmdResults        [IN][OUT]       动作命令返回信息列表
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 
 *  @remarks 仅GigE Vision设备支持, 支持最多16个组播地址同时发送命令。

 *  @~english
 *  @brief  Sends action commands. 
 *  @param   pstActionCmdInfo           [IN]            It refers to information of action commands. 
 *  @param   pstActionCmdResults        [IN][OUT]       It refers to list of returned information about action commands. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks This API is supported only by GigE Vision devices, Supports sending commands to up to 16 multicast addresses simultaneously.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_IssueActionCommand(IN MV_ACTION_CMD_INFO* pstActionCmdInfo, IN OUT MV_ACTION_CMD_RESULT_LIST* pstActionCmdResults);

/********************************************************************//**
 *  @~chinese
 *  @brief  获取组播状态
 *  @param  pstDevInfo                  [IN]            设备信息结构体
 *  @param  pbStatus                    [IN][OUT]       组播状态,true:组播状态，false:非组播
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 
 *  @remarks 该接口用于判断设备当前是否处于组播状态，解决客户端枚举时需要打开设备判断组播的问题。 \n
             仅支持标准GigE Vision设备。

 *  @~english
 *  @brief  Gets multicast status. 
 *  @param  pstDevInfo                  [IN]            It refers to device information.
 *  @param  pbStatus                    [IN][OUT]       It refers to status (true: multicast status; false: not multicast status). 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 *  @remarks When enumerating the device, call this API to check if the device is in multicast status without turning on the device. 
			 This API only support GigE Vision Device.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_GetMulticastStatus(IN MV_CC_DEVICE_INFO* pstDevInfo, IN OUT bool* pbStatus);

/// @}

/*******************Part8 ch: 仅CameraLink 设备支持的接口 | en: API exclusively for CameraLink devices*******************/

/// \addtogroup 串口设备
/// @{

/********************************************************************//**
 *  @~chinese
 *  @brief  获取串口信息列表
 *  @param  pstSerialPortList           [IN][OUT]       串口信息列表
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 
 *  @remarks 该接口用于获取本地的串口信息。

 *  @~english
 *  @brief  Gets serial port information list. 
 *  @param  pstSerialPortList           [IN][OUT]       It refers to serial port information list. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 *  @remarks This API is used to get local serial port information. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CAML_GetSerialPortList(IN OUT MV_CAML_SERIAL_PORT_LIST* pstSerialPortList);

/********************************************************************//**
 *  @~chinese
 *  @brief  设置取指定枚举串口
 *  @param  pstSerialPortList           [IN][OUT]       串口信息列表
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 
 *  @remarks 该接口用于设置枚举CameraLink 设备的指定串口。

 *  @~english
 *  @brief  Specifies the serial ports for enumerations. 
 *  @param  pstSerialPortList           [IN]       It refers to serial port information list. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks Call this API to specify serial ports for camera link device enumeration. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CAML_SetEnumSerialPorts(IN MV_CAML_SERIAL_PORT_LIST* pstSerialPortList);

/***********************************************************************************************************//**
 *  @~chinese
 *  @brief  设置设备波特率
 *  @param  handle                      [IN]            设备句柄
 *  @param  nBaudrate                   [IN]            设置的波特率值，数值参考CameraParams.h中宏定义
            | CamercaLink波特率定义 | 值 | 对应的波特率值 |
            | :--- | :---: | :--- | 
            | \ref MV_CAML_BAUDRATE_9600    | 0x00000001 | 9600 |
            | \ref MV_CAML_BAUDRATE_19200   | 0x00000002 | 19200 |
            | \ref MV_CAML_BAUDRATE_38400   | 0x00000004 | 38400 |
            | \ref MV_CAML_BAUDRATE_57600   | 0x00000008 | 57600 |
            | \ref MV_CAML_BAUDRATE_115200  | 0x00000010 | 115200 |
            | \ref MV_CAML_BAUDRATE_230400  | 0x00000020 | 230400 |
            | \ref MV_CAML_BAUDRATE_460800  | 0x00000040 | 460800 |
            | \ref MV_CAML_BAUDRATE_921600  | 0x00000080 | 921600 |
            | \ref MV_CAML_BAUDRATE_AUTOMAX | 0x40000000 | 最大值 |
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 该接口支持在设备未连接时调用。通过GenTL协议访问设备时，需要先连接设备，才能调用该接口。\n
 			 因硬件/系统/外部干扰等因素，配置高波特率可能导致通信异常，建议配置波特率最大小于115200。
 
 *  @~english
 *  @brief  Sets baud rate for the device. 
 *  @param  handle                      [IN]            It refers to the device handle. 
 *  @param  nBaudrate                   [IN]            It refers to baud rate to set. Refer to the 'CameraParams.h' for parameter definitions. for example, #define MV_CAML_BAUDRATE_9600  0x00000001
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 *  @remarks You can call this API when the device is not connected. If the device is accessed via GenTL protocol, call this API after the device is connected.            
             High baud rate may cause communication exception due to factors such as hardware specification, system configuration, and external interference. 
             It is recommended to configure a baud rate of less than 115200
************************************************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CAML_SetDeviceBaudrate(IN void* handle, IN unsigned int nBaudrate);

/********************************************************************//**
 *  @~chinese
 *  @brief  获取设备波特率
 *  @param  handle                      [IN]            设备句柄
 *  @param  pnCurrentBaudrate           [IN][OUT]       波特率信息指针，数值参考CameraParams.h中宏定义
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 该接口支持在设备未连接时调用。
 
 *  @~english
 *  @brief  Gets baud rate for devices. 
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  pnCurrentBaudrate           [IN][OUT]       It refers to the pointer to baud rate information. See the 'CameraParams.h' for parameter definitions, for example, #define MV_CAML_BAUDRATE_9600  0x00000001
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 *  @remarks You can call this API when the device is not connected. 
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CAML_GetDeviceBaudrate(IN void* handle,IN OUT unsigned int* pnCurrentBaudrate);

/********************************************************************//**
 *  @~chinese
 *  @brief  获取设备与主机间连接支持的波特率
 *  @param  handle                      [IN]            设备句柄
 *  @param  pnBaudrateAblity            [IN][OUT]       支持的波特率信息的指针。 所有支持波特率的"或运算"结果，单个数值参考CameraParams.h中宏定义
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 该接口支持在设备未连接时调用。
 
 *  @~english
 *  @brief  Gets the supported baud rate of the connection between the device and host. 
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  pnBaudrateAblity            [IN][OUT]       It refers to the pointer to the supported baud rate information. See 'CameraParams.h' for the definitions of single value of the OR operation results of all supported baud rate.
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 *  @remarks You can call this API when the device is not connected. 
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CAML_GetSupportBaudrates(IN void* handle,IN OUT unsigned int* pnBaudrateAblity);

/********************************************************************//**
 *  @~chinese
 *  @brief  设置串口操作等待时长
 *  @param  handle                      [IN]            设备句柄
 *  @param  nMillisec                   [IN]            串口操作的等待时长, 单位为ms
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 
 *  @~english
 *  @brief  Sets the waiting duration for serial port operation. 
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  nMillisec                   [IN]            It refers to waiting time of serial port operation, unit: millisecond. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CAML_SetGenCPTimeOut(IN void* handle, IN unsigned int nMillisec);

/// @}

/*******************Part9 ch: 仅USB3 Vision设备支持的接口 | en: API exclusively for USB3 Vision (USB3 Vision) devices*******************/

/// \addtogroup U3V相机
/// @{

/********************************************************************//**
 *  @~chinese
 *  @brief  设置USB3 Vision的传输包大小
 *  @param  handle                      [IN]            设备句柄
 *  @param  nTransferSize               [IN]            传输的包大小, Byte，默认为1M，需4的倍数，rang：>=0x8000，建议最大值：[windows] rang <= 0x400000；[Linux] rang <= 0x200000
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 增加传输包大小可以适当降低取流时的CPU占用率。但不同的PC和不同USB扩展卡存在不同的兼容性，如果该参数设置过大可能会出现取不到图像的风险。
             MacOS系统中，传输包大小设置过小，可能导致开始取流接口报前置条件错误，该值和图像大小与传输通道个数有关，无法给出准确的最小值，MacOS系统中尽量不做修改。
 
 *  @~english
 *  @brief  Sets transmission packet size of USB3 vision cameras. 
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  nTransferSize               [IN]            It refers to the size of the transmission packet (unit: byte), and the default value is 1 MB (1,048,576 bytes), Multiples of 4. rang: >=0x8000. 
                                                        Recommended maximum values: [Windows] range ≤ 0x400000; [Linux] range ≤ 0x200000.
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 *  @remarks Increasing the packet size can reduce the CPU usage, but for different computers and USB expansion cards, the compatibility is different. If the packet size is too large, image acquisition might fail. 
             On macOS systems, setting the TransferSize too small may cause MV_CC_StartGrabbing to return MV_E_PRECONDITION. This value is related to the image size and TransferWays, so it is not possible to provide an exact minimum value. It is recommended to avoid modifying this setting whenever possible.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_USB_SetTransferSize(IN void* handle, IN unsigned int nTransferSize);

/********************************************************************//**
 *  @~chinese
 *  @brief  获取USB3 Vision的传输包大小
 *  @param  handle                      [IN]            设备句柄
 *  @param  pnTransferSize              [IN][OUT]       传输的包大小指针, Byte
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 该接口用于获取当前的USB3 Vision传输包大小，默认1M。
 
 *  @~english
 *  @brief  Gets transmission packet size of USB3 vision cameras. 
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  pnTransferSize              [IN][OUT]       It refers to the pointer to the size of the transmission packet (unit: byte). 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 *  @remarks Call this API to get the packet size of the current USB3 vision device (1 MB by default). 
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_USB_GetTransferSize(IN void* handle, IN OUT unsigned int* pnTransferSize);

/********************************************************************//**
 *  @~chinese
 *  @brief  设置USB3 Vision的传输通道个数
 *  @param  handle                      [IN]            设备句柄
 *  @param  nTransferWays               [IN]            传输通道个数，范围：1-10
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 用户可以根据PC的性能、设备出图帧率、图像大小和内存使用率等因素对该参数进行调节。但不同的PC和不同的USB扩展卡存在不同的兼容性。
 
 *  @~english
 *  @brief  Sets the number of transmission channels of USB3 vision cameras. 
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  nTransferWays               [IN]            It refers to the number of transmission channels. It should be between 1 to 10. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks This parameter can be adjusted based on computer performance, device image frame rate, device image size, and device memory usage. But compatibility differs due to different PC and USB expansion cards. 
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_USB_SetTransferWays(IN void* handle, IN unsigned int nTransferWays);

/********************************************************************//**
 *  @~chinese
 *  @brief  获取USB3 Vision的传输通道个数
 *  @param  handle                      [IN]            设备句柄
 *  @param  pnTransferWays              [IN][OUT]       传输通道个数指针
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 该接口用于获取当前的USB3 Vision异步取流节点个数，U口相机传输通道个数和像素格式对应的负载包大小相关，可通过（最大异步注册长度/像素格式对应的负载包大小）计算得出。 \n
 *          2000W设备的MONO8默认为3个，YUV为默认2个，RGB为默认1个，其它情况默认8个节点。
 
 *  @~english
 *  @brief  Gets the number of transmission channels of USB3 vision cameras. 
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  pnTransferWays              [IN][OUT]       It refers to the pointer to the number of transmission channels. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks This API is used to get the current number of USB3 Vision asynchronous image acquisition nodes. 
             For USB3 vision cameras, the number of transmission channels is closely related to the packet size corresponding to the pixel format, and it can be calculated based on the max. asynchronous registration length/packet size of pixel format. 
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_USB_GetTransferWays(IN void* handle, IN OUT unsigned int* pnTransferWays);

/********************************************************************//**
 *  @~chinese
 *  @brief  设置USB3 Vision的事件缓存节点个数
 *  @param  handle                      [IN]            设备句柄
 *  @param  nEventNodeNum               [IN]            事件缓存节点个数，范围：1-64
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 该接口用于设置当前的USB3 Vision事件缓存节点个数，默认情况下为5个。
 
 *  @~english
 *  @brief  Sets the number of event buffer nodes of USB3 vision cameras. 
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  nEventNodeNum               [IN]            It refers to the number of event buffer nodes, range: [1, 64]. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks Call this API to set the number of the buffer nodes for the current USB3 vision event. The default value is 5. 
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_USB_SetEventNodeNum(IN void* handle, IN unsigned int nEventNodeNum);

/********************************************************************//**
 *  @~chinese
 *  @brief  设置USB3 Vision的同步读写超时时间，范围为:[1000, INT_MAX)，默认1000 ms
 *  @param  handle                      [IN]            设备句柄
 *  @param  nMills                      [IN]            设置同步读写超时时间,默认时间为1000ms
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 增加设置同步读取时间接口，兼容部分相机配置参数很慢，超过1000ms的情况

 *  @~english
 *  @brief  Sets the timeout duration for sync reading and writing of USB3 vision devices (1000 ms by default), range: [1000, INT_MAX]. 
 *  @param  handle               [IN]            It refers to the device handle.
 *  @param  nMills               [IN]            It refers to the timeout duration for sync reading and writing (1000 by default), unit: millisecond. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.  
 *  @remarks Increasing the timeout duration for sync reading and writing can help deal with the problem that some cameras' parameter configuration process is very slow (more than 1000 ms). 
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_USB_SetSyncTimeOut(IN void* handle, IN unsigned int nMills);

/********************************************************************//**
 *  @~chinese
 *  @brief  获取USB3 Vision相机同步读写超时时间
 *  @param  handle                      [IN]            设备句柄
 *  @param  pnMills                     [IN][OUT]       获取的超时时间(ms)
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 该接口用于获取当前的USB3 Vision同步读写超时时间大小，默认1000ms。
 
 *  @~english
 *  @brief  Gets the timeout duration for sync reading and writing of USB3 vision devices. 
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  pnMills                     [IN][OUT]       It refers to the timeout duration, unit: millisecond.
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 *  @remarks Call this API to get the timeout duration for sync reading and writing of USB3 vision cameras (1000 ms by default).
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_USB_GetSyncTimeOut(IN void* handle, IN OUT unsigned int* pnMills);

/// @}


/*******************Part10 ch: GenTL相关接口 | en: GenTL-related API*******************/

/// \addtogroup GenTL
/// @{

/******************************************************************************//**
 *  @~chinese
 *  @brief  通过GenTL枚举Interfaces
 *  @param  pstIFList                   [IN][OUT]       Interfaces列表
 *  @param  strGenTLPath                [IN]            GenTL的cti文件路径
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 
 *  @remarks \li Interfaces列表的内存是在SDK内部分配的，多线程调用该接口时会进行设备列表内存的释放和申请，建议尽量避免多线程枚举操作。\n
             \li 暂不支持直接调用MvProducerU3V.cti和MvProducerGEV.cti， 支持调用其他.cti
             
 *  @~english
 *  @brief  Enumerates interfaces via GenTL. 
 *  @param  pstIFList                   [IN][OUT]       It refers to interface list. 
 *  @param  strGenTLPath                [IN]            It refers to CTI file path of GenTL. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 *  @remarks The memory of device list is internally allocated. When this API is called in multiple threads, the SDK will release and apply for the device list memory. 
             It is recommended to avoid multithreaded enumeration operations. 
             MvProducerU3V.cti and MvProducerGEV.cti calling are unsupported.
 *******************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_EnumInterfacesByGenTL(IN OUT MV_GENTL_IF_INFO_LIST* pstIFList, IN const char * strGenTLPath);

/********************************************************************//**
 *  @~chinese
 *  @brief  卸载cti库
 *  @param  pGenTLPath                [IN]            枚举卡时加载的cti文件路径
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 
 *  @remarks 卸载前需要保证通过该cti枚举出的相机已全部关闭，否则报错前置条件错误。
 
 *  @~english
 *  @brief  Unload the CTI library. 
 *  @param  pGenTLPath                [IN]            It refers to the CTI file path during the enumeration. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 *  @remarks Make sure that all cameras enumerated by the CTI file are closed before calling this API. Otherwise, MV_E_PRECONDITION error will be returned. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_UnloadGenTLLibrary(IN const char * pGenTLPath);

/*****************************************************************************************************//**
 *  @~chinese
 *  @brief  通过GenTL Interface枚举设备
 *  @param  pstIFInfo                   [IN]            Interface信息
 *  @param  pstDevList                  [IN][OUT]           设备列表
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 
 *  @remarks 设备列表的内存是在SDK内部分配的，多线程调用该接口时会进行设备列表内存的释放和申请。
 *  @note   尽量避免多线程枚举操作。
 
 *  @~english
 *  @brief  Enumerates devices via GenTL interface. 
 *  @param  pstIFInfo                   [IN]            It refers to interface information. 
 *  @param  pstDevList                  [IN][OUT]       It refers to the device list. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 *  @remarks The memory of device list is internally allocated. When this API is called in multiple threads, the SDK will release and apply for the device list memory. 
             It is recommended to avoid multithreaded enumeration operations. 
 *****************************************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_EnumDevicesByGenTL(IN MV_GENTL_IF_INFO* pstIFInfo, IN OUT MV_GENTL_DEV_INFO_LIST* pstDevList);

/********************************************************************//**
 *  @~chinese
 *  @brief  通过GenTL设备信息创建设备句柄
 *  @param  handle                      [IN][OUT]       设备句柄
 *  @param  pstDevInfo                  [IN]            设备信息结构体指针
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 
 *  @remarks 根据输入的设备信息，创建库内部必须的资源和初始化内部模块。
 
 *  @~english
 *  @brief  Creates the device handle by GenTL related device information. 
 *  @param  handle                      [IN][OUT]       It refers to interface information. 
 *  @param  pstDevInfo                  [IN]            It refers to the struct pointer to device Information. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks Create required resources within library and initialize internal module according to input device information. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_CreateHandleByGenTL(IN OUT void ** handle, IN const MV_GENTL_DEV_INFO* pstDevInfo);

/// @}

/*******************Part11 ch: 图像保存、格式转换等相关接口 | en: Image saving and format conversion API*******************/

/// \addtogroup 图像处理
/// @{

/********************************************************************//**
 *  @~chinese
 *  @brief  保存图片，支持Bmp和Jpeg.
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstSaveParam                [IN][OUT]       保存图片参数结构体
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 
 *  @remarks \li 通过该接口可以将从设备采集到的原始图像数据转换成JPEG或者BMP等格式并存放在指定内存中，然后用户可以将转换之后的数据直接保存成图片文件。\n
             \li 该接口调用无接口顺序要求，有图像源数据就可以进行转换，可以先调用 MV_CC_GetOneFrameTimeout() 或者 MV_CC_RegisterImageCallBackEx() 设置回调函数，获取一帧图像数据，然后再通过该接口转换格式。\n
             \li 该接口支持长乘宽至UINT_MAX，其中 MV_CC_SaveImageEx2() 支持长乘宽最大至 USHRT_MAX，JPEG格式最大支持宽高为65500。
 
 *  @~english
 *  @brief  Saves images, supporting BMP and JPEG. 
 *  @param  handle                      [IN]            It refers to the device handle. 
 *  @param  pstSaveParam                [IN][OUT]       It refers to the structure of image saving parameters. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks Call this API to convert the collected original images to JPEG or BMP format and save them to specified memory. You can then save the converted data as image files. 
             This API requires no specific calling sequence. The conversion will be executed when there is any image data. You can call MV_CC_GetOneFrameTimeout() or MV_CC_RegisterImageCallBackEx() to set the callback function and get one image frame, then call this API to convert the format. 
             This API supports setting the nWidth/nHeight/Length parameter to UINT_MAX: MV_CC_SaveImageEx2() supports setting the max. parameter to USHRT_MAX, and JPEG format supports the max. width and height value 65500.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SaveImageEx3(IN void* handle, IN OUT MV_SAVE_IMAGE_PARAM_EX3* pstSaveParam);

/********************************************************************//**
 *  @~chinese
 *  @brief  保存图像到文件
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstSaveFileParam            [IN][OUT]       保存图片文件参数结构体
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 
 *  @remarks \li 该接口支持BMP/JPEG/PNG/TIFF。\n
             \li  该接口支持保存的图像长乘宽至UINT_MAX， MV_CC_SaveImageToFile() 支持长乘宽最大至USHRT_MAX。JPEG格式最大支持宽高为65500 px。 \n
             \li  该接口是 MV_CC_SaveImageToFile() 接口的扩展接口。
             
 *  @~english
 *  @brief  Saves image to file (extended API 1) 
 *  @param  handle                      [IN]            It refers to the device handle. 
 *  @param  pstSaveFileParam            [IN][OUT]       It refers to the structure of image file saving parameters. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks It supports saving images in BMP, JPEG, PNG, and TIFF formats. 
             this API support the parameter nWidth/nHeight/Length to UINT_MAX. 
			 For images in JPEG format, the supported max. width and height values are 65500. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SaveImageToFileEx(IN void* handle, IN OUT MV_SAVE_IMAGE_TO_FILE_PARAM_EX* pstSaveFileParam);

/********************************************************************//**
 *  @~chinese
 *  @brief  保存图像到文件
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstImage                    [IN]            图像信息
 *  @param  pSaveImageParam             [IN]            存图参数
 *  @param  pcImagePath                 [IN]            存图路径,Windows平台长度不超过260字节，Linux平台不超过255字节
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 
 *  @remarks \li 该接口支持4G以上超大图的PNG/TIFF存图，非超大图像支持BMP/JPEG/TIFF/PNG。 \n
			 \li JPEG格式最大支持宽高为65500 px。
             
 *  @~english
 *  @brief  Saves image to file (extended API 2) 
 *  @param  handle                      [IN]            It refers to the device handle. 
 *  @param  pstImage                    [IN]            It refers to the image information. 
 *  @param  pSaveImageParam             [IN]            It refers to the image saving parameter. 
 *  @param  pcImagePath                 [IN]            It refers to the image saving path. On  Windows length does not exceed 260 bytes, and on Linux, it does not exceed 255 bytes.
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 *  @remarks It supports saving images over 4 GB in PNG and TIFF formats, and images under 4 GB in BMP, JPEG, TIFF, and PNG formats. 
			 For images in JPEG format, the supported max. width and height values are 65500. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SaveImageToFileEx2(IN void* handle, IN MV_CC_IMAGE* pstImage, IN MV_CC_SAVE_IMAGE_PARAM* pSaveImageParam, IN const char* pcImagePath);

/********************************************************************//**
 *  @~chinese
 *  @brief  图像旋转
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstRotateParam              [IN][OUT]       图像旋转参数结构体
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 
 *  @remarks 该接口只支持MONO8/RGB24/BGR24格式数据的90/180/270度旋转。
 
 *  @~english
 *  @brief  Rotates images. 
 *  @param  handle                      [IN]            It refers to the device handle. 
 *  @param  pstRotateParam              [IN][OUT]       It refers to image rotation parameters structure. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 *  @remarks This API only supports 90°, 180°, and 270° rotation of images in Mono 8, RGB 24, and BGR 24 formats. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_RotateImage(IN void* handle, IN OUT MV_CC_ROTATE_IMAGE_PARAM* pstRotateParam);

/********************************************************************//**
 *  @~chinese
 *  @brief  图像翻转
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstFlipParam                [IN][OUT]       图像翻转参数结构体
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 
 *  @remarks 该接口只支持MONO8/RGB24/BGR24格式数据的垂直和水平翻转。
 
 *  @~english
 *  @brief  Flips images
 *  @param  handle                      [IN]            It refers to the device handle. 
 *  @param  pstFlipParam                [IN][OUT]       It refers to the structure of image flipping parameters. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 *  @remarks This API only support vertical and horizontal flipping of images in Mono 8, RGB 24, and BGR 24 formats. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_FlipImage(IN void* handle, IN OUT MV_CC_FLIP_IMAGE_PARAM* pstFlipParam);

/********************************************************************//**
 *  @~chinese
 *  @brief  像素格式转换
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstCvtParam                 [IN][OUT]       像素格式转换参数结构体
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 
 *  @remarks \li 通过该接口可以将从设备采集到的原始图像数据转换成用户所需的像素格式并存放在指定内存中。 \n
             \li 该接口调用无接口顺序要求，有图像源数据就可以进行转换，可以先调用 MV_CC_GetOneFrameTimeout() 或者 MV_CC_RegisterImageCallBackEx() 设置回调函数，获取一帧图像数据，然后再通过该接口转换格式。如果设备当前采集图像是JPEG压缩的格式，则不支持调用该接口进行转换。 \n
             \li 该接口支持转换的图像长乘宽至UINT_MAX。 \n
             \li 像素格式转换能力的详情，如下表所示。第一列为输入像素格式，第一行为转换后的像素格式。“  √  ”代表可以转换为目标像素格式。
                | 输入格式\输出格式 | Mono8 | RGB24 | BGR24 | RGB Planar | YUV422 | YUV422 YUYV |
                | ----------------- | ----- | ----- | ----- | ---------- | ------ | ----------- |
                | Mono8             |   √   |   √   |   √   |            |   √    |      √      |
                | Mono10            |   √   |   √   |   √   |            |   √    |      √      |
                | Mono10P           |   √   |   √   |   √   |            |   √    |      √      |
                | Mono12            |   √   |   √   |   √   |            |   √    |      √      |
                | Mono12P           |   √   |   √   |   √   |            |   √    |      √      |
                | BayerGR8          |   √   |   √   |   √   |      √     |   √    |      √      |
                | BayerRG8          |   √   |   √   |   √   |      √     |   √    |      √      |
                | BayerGB8          |   √   |   √   |   √   |      √     |   √    |      √      |
                | BayerBG8          |   √   |   √   |   √   |      √     |   √    |      √      |
                | BayerRBGG8        |   √   |   √   |   √   |      √     |   √    |      √      |
                | BayerBRGG8        |   √   |   √   |   √   |      √     |   √    |      √      |
                | BayerGR10         |   √   |   √   |   √   |            |   √    |      √      |
                | BayerRG10         |   √   |   √   |   √   |            |   √    |      √      |
                | BayerGB10         |   √   |   √   |   √   |            |   √    |      √      |
                | BayerBG10         |   √   |   √   |   √   |            |   √    |      √      |
                | BayerGR12         |   √   |   √   |   √   |            |   √    |      √      |
                | BayerRG12         |   √   |   √   |   √   |            |   √    |      √      |
                | BayerGB12         |   √   |   √   |   √   |            |   √    |      √      |
                | BayerBG12         |   √   |   √   |   √   |            |   √    |      √      |
                | BayerGR10P        |   √   |   √   |   √   |            |   √    |      √      |
                | BayerRG10P        |   √   |   √   |   √   |            |   √    |      √      |
                | BayerGB10P        |   √   |   √   |   √   |            |   √    |      √      |
                | BayerBG10P        |   √   |   √   |   √   |            |   √    |      √      |
                | BayerGR12P        |   √   |   √   |   √   |            |   √    |      √      |
                | BayerRG12P        |   √   |   √   |   √   |            |   √    |      √      |
                | BayerGB12P        |   √   |   √   |   √   |            |   √    |      √      |
                | BayerBG12P        |   √   |   √   |   √   |            |   √    |      √      |
                | RGB8P             |   √   |   √   |   √   |      √     |   √    |      √      |
                | BGR8P             |   √   |   √   |   √   |      √     |   √    |      √      |
                | YUV422P           |   √   |   √   |   √   |            |   √    |      √      |
                | YUV422 YUYV       |   √   |   √   |   √   |            |   √    |      √      |
 
 *  @~english
 *  @brief  Converts pixel format.
 *  @param  handle                      [IN]            It refers to the device handle. 
 *  @param  pstCvtParam                 [IN][OUT]       It refers to the structure of pixel format conversion parameters.
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 *  @remarks Call this API to convert the collected original images to images in required pixel format and save them to specified memory.  
             This API requires no specific calling sequence. The conversion will be executed when there is any image data. 
             You can call MV_CC_GetOneFrameTimeout() or MV_CC_RegisterImageCallBackEx() to set the callback function and get one image frame, then call this API to convert the format. 
             If the collected image is in compressed JPEG format, it cannot be converted via this API. 
             this API support the parameter nWidth/nHeight/Length to UINT_MAX.
             Comparing with the API MV_CC_ConvertPixelType, this API support the parameter nWidth/nHeight/Length to UINT_MAX. 

 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_ConvertPixelTypeEx(IN void* handle, IN OUT MV_CC_PIXEL_CONVERT_PARAM_EX* pstCvtParam);

/********************************************************************//**
 *  @~chinese
 *  @brief  设置插值算法类型
 *  @param  handle                      [IN]            设备句柄
 *  @param  nBayerCvtQuality            [IN]            Bayer的插值方法  0-快速 1-均衡（默认为均衡） 2-最优 3-最优+
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 
 *  @remarks 设置内部图像转换接口的Bayer插值算法类型参数， MV_CC_ConvertPixelTypeEx() 、 MV_CC_GetImageForRGB() 和 MV_CC_GetImageForBGR() 接口内部使用的插值算法是该接口所设定的。

 *  @~english
 *  @brief  Sets the interpolation method of Bayer format. 
 *  @param  handle                      [IN]            It refers to the device handle. 
 *  @param  nBayerCvtQuality            [IN]            It refers to interpolation method. 0: fast; 1: equilibrated; 2: optimal (default); 3: optimal plus. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 *  @remarks Call this API to set the Bayer interpolation algorithm type parameter for the APIs: MV_CC_ConvertPixelTypeEx() , MV_CC_GetImageForRGB() , and MV_CC_GetImageForBGR(). 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetBayerCvtQuality(IN void* handle, IN unsigned int nBayerCvtQuality);

/********************************************************************//**
 *  @~chinese
 *  @brief  插值算法平滑使能设置
 *  @param  handle                      [IN]            设备句柄
 *  @param  bFilterEnable               [IN]            平滑使能(默认关闭)
 *  @return 成功，返回#MV_OK；错误，返回错误码 
 *  @remarks 设置内部图像转换接口的贝尔插值平滑使能参数， MV_CC_ConvertPixelTypeEx() 、 MV_CC_SaveImageEx3() 和 MV_CC_SaveImageToFileEx() 内部使用的插值算法是该接口所设定的。

 *  @~english
 *  @brief  Enables or disables the smoothing function of interpolation algorithm. 
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  bFilterEnable               [IN]            Whether to enable the smoothing function of interpolation algorithm (disabled by default). 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 *  @remarks This API is used to enable or disable the smoothing function of Bayer interpolation, and it determines the interpolation algorithm of the APIs: MV_CC_ConvertPixelTypeEx()、MV_CC_SaveImageToFileEx and MV_CC_SaveImageEx3(). 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetBayerFilterEnable(IN void* handle, IN bool bFilterEnable);

/********************************************************************//**
 *  @~chinese
 *  @brief  设置Bayer格式的Gamma值
 *  @param  handle                      [IN]            设备句柄
 *  @param  fBayerGammaValue            [IN]            Gamma值:0.1 ~ 4.0
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 
 *  @remarks 设置该值后，在Bayer图像（Bayer8/10/12/16）转RGB/BGR图像(RGB24/48、RGBA32/64、BGR24/48、BGRA32/64)时起效。 \n 相关接口： MV_CC_ConvertPixelTypeEx() 、 MV_CC_SaveImageEx3() 、 MV_CC_SaveImageToFile() 。

 *  @~english
 *  @brief  Sets the Gamma value in Bayer pattern. 
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  fBayerGammaValue            [IN]            It refers to the Gamma value, range: [0.1, 4.0].
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 *  @remarks After setting this value, it takes effect when converting Bayer images (Bayer8/10/12/16) to RGB/BGR images (RGB24/48, RGBA32/64, BGR24/48, BGRA32/64). Related API: MV_CC_ConvertPixelTypeEx, MV_CC_SaveImageEx3, MV_CC_SaveImageToFileEx.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetBayerGammaValue(IN void* handle, IN float fBayerGammaValue);

/********************************************************************//**
 *  @~chinese
 *  @brief   设置Mono8/Bayer8/10/12/16格式的Gamma值
 *  @param   handle                           [IN] 设备句柄
 *  @param   MvGvspPixelType enSrcPixelType   [IN] 像素格式,支持Mono8, Bayer（Bayer8/10/12/16）
 *  @param   fGammaValue                      [IN] Gamma值:0.1 ~ 4.0
 *  @return  成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 
 *  @remarks \li 设置Mono8的gamma值后，在调用 MV_CC_ConvertPixelTypeEx() 将Mono8转成Mono8时gamma值起效。 \n
 *           \li 设置Bayer的gamma值后，在Bayer图像（Bayer8/10/12/16）转RGB/BGR图像(RGB24/48、RGBA32/64、BGR24/48、BGRA32/64)时起效。相关接口： MV_CC_ConvertPixelType() 、 MV_CC_SaveImageToFile() 、 MV_CC_SaveImageEx3() 。
 *           \li 该接口兼容 MV_CC_SetBayerGammaValue() ，新增支持Mono8像素格式。
 
 *  @~english
 *  @brief  Sets Gamma value of Mono 8 or Bayer 8/10/12/16 pattern. 
 *  @param  handle                           [IN]            It refers to the device handle.
 *  @param  MvGvspPixelType enSrcPixelType   [IN]            It refers to the pixel format. Supports PixelType_Gvsp_Mono8 and Bayer 8/10/12/16. 
 *  @param  fGammaValue                      [IN]            It refers to the Gamma value, range: [0.1, 4.0]. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 *  @remarks The Gamma value in Mono 8 pattern set via this API will be used when MV_CC_ConvertPixelType() is called to convert Mono 8 to Mono 8.
 *  @remarks The Gamma value in Bayer8/10/12/16 pattern set via this API will be used when calling MV_CC_ConvertPixelTypeEx() , MV_CC_SaveImageEx3() , or MV_CC_SaveImageToFileEx() to convert Bayer 8/10/12/16 format to RGB 24/48, RGBA 32/64, BGR 24/48 or BGRA 32/64. 
 *  @remarks This API is compatible with MV_CC_SetBayerGammaValue() , and it supports Mono 8 pixel format. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetGammaValue(IN void* handle, IN enum MvGvspPixelType enSrcPixelType, IN float fGammaValue);

/********************************************************************//**
 *  @~chinese
 *  @brief  设置Bayer格式的Gamma信息
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstGammaParam               [IN]            Gamma信息   
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 
 *  @remarks 设置该值后，在Bayer图像（Bayer8/10/12/16）转RGB/BGR图像(RGB24/48、RGBA32/64、BGR24/48、BGRA32/64)时起效。 相关接口： MV_CC_ConvertPixelTypeEx() 、 MV_CC_SaveImageEx3() 、 MV_CC_SaveImageToFileEx() 。
 *  @note 以相机实际支持情况为准。

 *  @~english
 *  @brief  Sets the Gamma value of Bayer pattern. 
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  pstGammaParam               [IN]            It refers to the Gamma information.
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 *  @remarks The Gamma value set by this API will be used when calling MV_CC_ConvertPixelTypeEx() , MV_CC_SaveImageEx3(), MV_CC_SaveImageToFileEx(), to convert Bayer 8/10/12/16 format to RGB24/48, BGR24/48, RGBA32/64, or BGRA32/64.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetBayerGammaParam(IN void* handle, IN MV_CC_GAMMA_PARAM* pstGammaParam);

/********************************************************************//**
 *  @~chinese
 *  @brief  设置Bayer格式的CCM使能和矩阵，量化系数默认1024
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstCCMParam                 [IN]            CCM参数
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 
 *  @remarks 开启CCM并设置CCM矩阵后，在Bayer图像（Bayer8/10/12/16）转RGB/BGR图像(RGB24/48、RGBA32/64、BGR24/48、BGRA32/64)时起效。 相关接口： MV_CC_ConvertPixelTypeEx() 、 MV_CC_SaveImageEx3() 、 MV_CC_SaveImageToFileEx() 。 

 *  @~english
 *  @brief  Enables/disables CCM and sets CCM parameters in Bayer pattern. The default quantitative scale is 1024. 
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  pstCCMParam                 [IN]            It refers to the CCM parameters. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 *  @remarks After the API is called to enable CCM and set the CCM, the CCM parameters will take effect when MV_CC_ConvertPixelTypeEx() or MV_CC_SaveImageEx3() or MV_CC_SaveImageToFileEx() is called to convert Bayer 8/10/12/16 format to RGB 24/48, RGBA 32/64, BGR 24/48, or BGRA 32/64. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetBayerCCMParam(IN void* handle, IN MV_CC_CCM_PARAM* pstCCMParam);

/********************************************************************//**
 *  @~chinese
 *  @brief  设置Bayer格式的CCM使能和矩阵
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstCCMParam                 [IN]            CCM参数
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 
 *  @remarks 开启CCM并设置CCM矩阵后，在Bayer图像（Bayer8/10/12/16）转RGB/BGR图像(RGB24/48、RGBA32/64、BGR24/48、BGRA32/64)时起效。 相关接口： MV_CC_ConvertPixelTypeEx() 、 MV_CC_SaveImageEx3() 、 MV_CC_SaveImageToFileEx() 。

 *  @~english
 *  @brief  Enables and disables CCM, and sets CCM parameters of Bayer pattern. 
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  pstCCMParam                 [IN]            It refers to the color correction parameter structure. 
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks After the API is called to enable CCM and set the CCM, the CCM parameters will take effect when MV_CC_ConvertPixelTypeEx() or MV_CC_SaveImageEx3() or MV_CC_SaveImageToFileEx() is called to convert Bayer 8/10/12/16 format to RGB 24/48, RGBA 32/64, BGR 24/48, or BGRA 32/64. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetBayerCCMParamEx(IN void* handle, IN MV_CC_CCM_PARAM_EX* pstCCMParam);

/********************************************************************//**
 *  @~chinese
 *  @brief  图像对比度调节
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstContrastParam            [IN][OUT]       对比度调节参数
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 
 *  @remarks 

 *  @~english
 *  @brief  Adjusts image contrast. 
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  pstContrastParam            [IN][OUT]       It refers to the contrast parameter structure. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_ImageContrast(IN void* handle, IN OUT MV_CC_CONTRAST_PARAM* pstContrastParam);

/********************************************************************//**
 *  @~chinese
 *  @brief  图像去紫边
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstPurpleFringingParam      [IN][OUT]       去紫边参数
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 
 *  @remarks 像素格式仅支持PixelType_Gvsp_RGB8_Packed和PixelType_Gvsp_BGR8_Packed

 *  @~english
 *  @brief  Corrects purple fringing of the image. 
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  pstPurpleFringingParam      [IN][OUT]       It refers to purple fringing correction parameter. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks This API only supports processing images in PixelType_Gvsp_RGB8_Packed and PixelType_Gvsp_BGR8_Packed formats. 
 *  ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_PurpleFringing(IN void* handle, IN MV_CC_PURPLE_FRINGING_PARAM* pstPurpleFringingParam);

/********************************************************************//**
 *  @~chinese
 *  @brief  设置ISP参数
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstParam                    [IN]            ISP配置参数
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks  该接口仅支持windows平台

 *  @~english
 *  @brief  Sets ISP parameters. 
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  pstParam                    [IN][OUT]       It refers to the ISP parameter structure. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks This API only supports windows platform.
 *  ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetISPConfig(void* handle, IN MV_CC_ISP_CONFIG_PARAM* pstParam);

/********************************************************************//**
 *  @~chinese
 *  @brief  对图像进行ISP算法处理
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstInputImage               [IN]            输入图像结构体
 *  @param  pstOutputImage              [IN][OUT]       输出图像结构体
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 需要先调用 MV_CC_SetISPConfig() 传入配置文件, 配置文件由ISP工具生成
 *  @remarks  该接口仅支持windows平台

 *  @~english
 *  @brief  Processes the images with ISP algorithm. 
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  pstInputImage               [IN]            It refers to the input image structure. 
 *  @param  pstOutputImage              [IN][OUT]       It refers to the output image structure. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks Before calling this API, call MV_CC_SetISPConfig() to import configuration file generated by the ISP tool. 
 *  @remarks This API only supports windows platform.
 *  ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_ISPProcess(void* handle, IN MV_CC_IMAGE* pstInputImage, MV_CC_IMAGE* pstOutputImage);

/********************************************************************//**
 *  @~chinese
 *  @brief  无损解码
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstDecodeParam              [IN][OUT]       无损解码参数结构体
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 
 *  @remarks 将从相机中取到的无损压缩码流解码成裸数据，同时支持解析当前相机实时图像的水印信息（如果输入的无损码流不是当前相机或者不是实时取流的，则水印解析可能异常）;
            若解码失败，请检查以下情况：
            \li CPU是否支持 SSE AVX指令集。
            \li 当前帧是否异常（丢包等），若存在，可能导致解码异常。
            \li 相机出图是否异常，即使不丢包也会异常。

 *  @~english
 *  @brief  Decodes lossless compression stream into raw data. 
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  pstDecodeParam              [IN][OUT]       It refers to the structure of lossless decoding parameters. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks Supports decoding the lossless compression stream of camera to raw data, and parsing the watermark of real-time images of the current camera. If the inputted lossless stream is not real-time or does not belong to the current camera, an exception may occur during watermark parsing. 
			 If the decoding fails, check if it is one of the following circumstances: 
             (1) CPU does not support SSE AVX instruction set.
             (2) IException occurs on the current frame (e.g., packet loss).
             (3) Exception occurs on image generating from camera, even without packet loss. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_HB_Decode(IN void* handle, IN OUT MV_CC_HB_DECODE_PARAM* pstDecodeParam);

/********************************************************************//**
 *  @~chinese
 *  @brief  开始录像
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstRecordParam              [IN]            录像参数结构体
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 
 *  @note   该接口最大支持Width×Height为8192×8192大小，最小支持96×96。若超出，会导致调用 MV_CC_InputOneFrame() 错误。
 
 *  @~english
 *  @brief  Starts recording. 
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  pstRecordParam              [IN]            It refers to the recording parameter structure. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 *  @remarks  The max. supported width × height is 8192x8192. The min. supported width × height is 96x96. If the value exceeds, an error will occur when calling MV_CC_InputOneFrame().
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_StartRecord(IN void* handle, IN MV_CC_RECORD_PARAM* pstRecordParam);

/********************************************************************//**
 *  @~chinese
 *  @brief  输入录像数据
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstInputFrameInfo           [IN]            录像数据结构体
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 
 *  @remarks 不支持HB和JPEG直接输入，需要先解码。
 
 *  @~english
 *  @brief  Inputs raw data for recording. 
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  pstInputFrameInfo           [IN]            It refers to the record data structure. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 *  @remarks Direct input of HB and JPEG is not supported; decoding is required first.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_InputOneFrame(IN void* handle, IN MV_CC_INPUT_FRAME_INFO * pstInputFrameInfo);


/********************************************************************//**
 *  @~chinese
 *  @brief  输入录像数据
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstInputFrameInfo           [IN]            录像数据结构体
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 
 *  @remarks 支持HB和JPEG直接输入，SDK内部解码。
 
 *  @~english
 *  @brief  Inputs raw data for recording. 
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  pstInputFrameInfo           [IN]            It refers to the record data structure. 
 *  @return Direct input of HB and JPEG is supported, with decoding handled internally by the SDK.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_InputOneFrameEx(IN void* handle, IN MV_CC_INPUT_FRAME_INFO_EX * pstInputFrameInfo);

/********************************************************************//**
 *  @~chinese
 *  @brief  停止录像
 *  @param  handle                      [IN]            设备句柄
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。 
 
 *  @~english
 *  @brief  Stops recording. 
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_StopRecord(IN void* handle);

/********************************************************************//**
 *  @~chinese
 *  @brief  重构图像(用于分时曝光功能)
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstReconstructParam         [IN][OUT]       重构图像参数
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 使用时，需与线阵相机的分时频闪或分时曝光等场景搭配。假设设置分时曝光节点为2，则相机会将2个不同曝光值所对应的两张图像交叠合并为1张图像（实际高度为2张图像的高度）发送给上层应用程序。 \n
             此时，调用该接口并传入分时曝光值nExposureNum为2，可将相机发送的1张图像分割为2张图像，并且这2张图像分别对应1个曝光值。 \n
             若使用面阵相机或使用线阵相机，但未启用分时曝光/分时频闪功能，则调用该接口进行图像分割是毫无意义的，尤其是Bayer格式，会打乱图像内容。 \n

 *  @~english
 *  @brief  Reconstructs the image for multi-light control. 
 *  @param  handle                      [IN]            It refers to the device handle. 
 *  @param  pstReconstructParam         [IN][OUT]       It refers to the image reconstruction parameters. 
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 *  @remarks When used, this API must be paired with multi-light control of line scan cameras. If the multi-light exposure node is 2, the camera will reconstruct 2 images with different exposure values into one image (with its height the sum of two images) and send it to the upper layer application.
             If this API is called then and the value of nExposureNum is set to 2, the reconstructed image will be later divided to 2 images with two corresponding exposure values.
             If using an area scan camera or a line scan camera without enabling the multi-light control function, calling this interface for image splitting is meaningless, especially for Bayer format images, as it may disrupt the image content.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_ReconstructImage(IN void* handle, IN OUT MV_RECONSTRUCT_IMAGE_PARAM* pstReconstructParam);

/// @}



/**************************Part12 ch: 支持串口通信的设备接口 | en: API for devices supporting serial communication ******************************************/

/// \addtogroup 串口控制相关
/// @{

/********************************************************************//**
 *  @~chinese
 *  @brief    打开串口
 *  @param    handle                      [IN]            设备句柄
 *  @return   成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码" 。
 *  @remarks  此接口适用于支持串口通信的相机

 *  @~english
 *  @brief    Opens the serial port. 
 *  @param    handle                      [IN]            It refers to the device handle.
 *  @return   Returns MV_OK for success, and returns corresponding Error Code for failure.   
 *  @remarks  This API is compatible with cameras supporting serial communication
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SerialPort_Open(IN void* handle);

/********************************************************************//**
 *  @~chinese
 *  @brief    向串口写数据，一次最大写512字节的数据
 *  @param    handle                      [IN]            设备句柄
 *  @param    pBuffer                     [IN]            数据
 *  @param    nLength                     [IN]            数据长度
 *  @param    pnWriteLen                  [OUT]           实际写成功的数据长度
 *  @return   成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码" 。
 *  @remarks  接口为阻塞模式，数据全部发送完成或者发送失败时返回

 *  @~english
 *  @brief   Writes data to serial port, allowing a maximum of 512 bytes written at a time.  
 *  @param    handle                      [IN]            It refers to the device handle.
 *  @param    pBuffer                     [IN]            It refers to the data buffer. 
 *  @param    nLength                     [IN]            It refers to the data length.
 *  @param    pnWriteLen                  [OUT]           It refers to the actual written data length. 
 *  @return   Returns MV_OK for success, and returns corresponding Error Code for failure. 
 *  @remarks  The API is in blocking mode, returning the result only when all data has been successfully transmitted or transmission fails. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SerialPort_Write(IN void* handle, IN const void *pBuffer, IN unsigned int nLength, OUT unsigned int* pnWriteLen);

/********************************************************************//**
 *  @~chinese
 *  @brief    读串口数据
 *  @param    handle                      [IN]            设备句柄
 *  @param    pBuffer                     [IN]            数据
 *  @param    nLength                     [IN]            数据长度
 *  @param    pnReadLen                   [OUT]           实际读到的数据长度
 *  @param    nMsec                       [IN]            超时时间，单位：ms
 *  @return   成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码" 。
 *  @remarks  接口为阻塞模式，当有收到数据、到达超时时间、出现异常时，立即返回

 *  @~english
 *  @brief    Reads the serial port data. 
 *  @param    handle                      [IN]            It refers to the device handle. 
 *  @param    pBuffer                     [IN]            It refers to the data buffer. 
 *  @param    nLength                     [IN]            It refers to the data buffer length.
 *  @param    pnReadLen                   [OUT]           It refers to the reader data length. 
 *  @param    nMsec                       [IN]            It refers to the timeout duration, unit: millisecond. 
 *  @return   Returns MV_OK for success, and returns corresponding Error Code for failure.   
 *  @remarks  The API is in blocking mode, returning the result only when data is received, timed out, or exception occurs. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SerialPort_Read(IN void* handle, IN void *pBuffer, IN unsigned int nLength, OUT unsigned int* pnReadLen, IN unsigned int nMsec);

/********************************************************************//**
 *  @~chinese
 *  @brief    清空已接收的串口数据
 *  @param    handle                      [IN]            设备句柄
 *  @return   成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码" 。
 *  @remarks  

 *  @~english
 *  @brief    Clears the received serial port data.  
 *  @param    handle                      [IN]            It refers to the device handle. 
 *  @return   Returns MV_OK for success, and returns corresponding Error Code for failure.  
 *  @remarks  
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SerialPort_ClearBuffer(IN void* handle);

/********************************************************************//**
 *  @~chinese
 *  @brief    关闭串口
 *  @param    handle                      [IN]            设备句柄
 *  @return   成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码" 。
 *  @remarks  

 *  @~english
 *  @brief    Closes the serial port.  
 *  @param    handle                      [IN]            It refers to the device handle. 
 *  @return   Returns MV_OK for success, and returns corresponding Error Code for failure.    
 *  @remarks  
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SerialPort_Close(IN void* handle);

/// @}

/**************************Part13 ch: 支持液态镜头的设备接口 | en: API for devices supporting liquid lens ******************************************/

/// \addtogroup 液态镜头控制相关
/// @{
/********************************************************************//**
 *  @~chinese
 *  @brief    打开液态镜头
 *  @param    handle                      [IN]            设备句柄
 *  @param    pstLiquidLensInfo           [OUT]           液态镜头基本信息
 *  @return   成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码" 。
 *  @remarks  

 *  @~english
 *  @brief    Open the liquid lens.  
 *  @param    handle                      [IN]            It refers to the device handle. 
 *  @param    pstLiquidLensInfo           [OUT]           It refers to the information of liquid lens.
 *  @return   Returns MV_OK for success, and returns corresponding Error Code for failure.    
 *  @remarks  
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_LiquidLens_Open(IN void* handle, OUT MV_CC_LIQUIDLENS_INFO* pstLiquidLensInfo);

/********************************************************************//**
 *  @~chinese
 *  @brief    关闭液态镜头
 *  @param    handle                      [IN]            设备句柄
 *  @return   成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码" 。
 *  @remarks  

 *  @~english
 *  @brief    Close the liquid lens.  
 *  @param    handle                      [IN]            It refers to the device handle. 
 *  @return   Returns MV_OK for success, and returns corresponding Error Code for failure.    
 *  @remarks  
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_LiquidLens_Close(IN void* handle);


/********************************************************************//**
 *  @~chinese
 *  @brief  设置光焦度
 *  @param  handle                      [IN]            设备句柄
 *  @param  nFocalPower                 [IN]            光焦度值. 范围[INT16_MIN, INT16_MAX]
 *  @param  nTriggerEnable              [IN]            是否触发相机，0-不触发， 非0-触发
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 当nTriggerEnable不为0时，用户需要先把相机设置为软触发模式，接口调用后，相机将触发一帧图像，否则无法出图。

 *  @~english
 *  @brief  Sets the focal power of liquid lens.
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  nFocalPower                 [IN]            It refers to the focal power value. range[INT16_MIN, INT16_MAX]
 *  @param  nTriggerEnable              [IN]            It refers to whether to trigger the camera. 0: not trigger; non-zero: trigger.
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 *  @remarks When nTriggerEnable is not 0, the user needs to set the camera to software trigger mode first. After calling this API, the camera will trigger one frame of image. Otherwise, the camera will not output images.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_LiquidLens_SetFocalPower(IN void* handle, IN int nFocalPower, IN int nTriggerEnable);

/********************************************************************//**
 *  @~chinese
 *  @brief  获取光焦度信息
 *  @param  handle                      [IN]            设备句柄
 *  @param  pnFocalPower               [OUT]           当前光焦度
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。

 *  @~english
 *  @brief  Gets the focal power information of liquid lens.
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  pnFocalPower               [OUT]           It refers to the current focal power.
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_LiquidLens_GetFocalPower(IN void* handle, OUT int* pnFocalPower);

/********************************************************************//**
 *  @~chinese
 *  @brief  执行自动对焦
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstAutoFocusParam           [IN]            自动对焦参数
 *  @param  nOutputImage                [IN]            对焦过程中是否输出图像，0: 丢弃对焦过程中的图像，非0：保留对焦过程中的图像
 *  @param  pnResultFocalPower          [OUT]           最佳清晰位置的光焦度值，如果不需要此值，可以置空
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks \li 当nOutputImage非0时，用户可以通过注册回调或者MV_CC_GetImageBuffer主动获取图像，但在对焦前建议增大图像缓存节点个数， 且如果是回调取图，回调中不要进行耗时操作，否则SDK内部会主动丢弃图像。\n
             \li 对焦过程中会将相机切换成软触发模式，对焦结束后，会将触发模式恢复到对焦前的设定。\n
             \li 为了提高对焦的有效性，对焦前请确保对焦画面亮度均匀。\n

 *  @~english
 *  @brief  Executes auto focus for liquid lens.
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  pstAutoFocusParam           [IN]            It refers to the auto focus parameters.
 *  @param  nOutputImage                [IN]            It refers to whether to output images during auto focus. 
 *                                                      0: discard images during auto focus; non-zero: retain images during auto focus. 
 *  @param  pnResultFocalPower          [OUT]           It refers to the focal power value at the best focus position, set to 0 if this value is not required.
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 *  @remarks When nOutputImage is non-zero, users can obtain images by registering a callback or using MV_CC_GetImageBuffer. However, before focusing, it is recommended to increase the number of image buffer nodes. 
 *           Additionally, if images are acquired through the callback, avoid performing time-consuming operations within the callback; otherwise, the SDK will actively discard images.
 *           During auto focus, the camera will be switched to software trigger mode. After auto focus is completed, the trigger mode will be restored to the setting before auto focus.
 *           To improve the effectiveness of auto focus, please ensure the focus image brightness is uniform before focusing.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_LiquidLens_AutoFocus(IN void* handle, IN MV_CC_LIQUIDLENS_AUTOFOCUS_PARAM* pstAutoFocusParam, IN int nOutputImage, OUT int* pnResultFocalPower);

/********************************************************************//**
 *  @~chinese
 *  @brief  执行光焦度区间扫描
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstRangeScanParam           [IN]            区间扫描参数
 *  @param  cbLiquidLensMsg             [IN]            液态镜头上传消息回调函数
 *  @param  pUser                       [IN]            用户自定义变量
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks \li 调用接口前需要用户先把相机设置为软触发模式，否则扫描过程中相机无法触发出图。相机触发出图的时机受触发延迟时间的影响，可以通过MV_CC_LiquidLens_SetTriggerDelayTime接口设置。\n
             \li 在回调中可以获取到光焦度变化事件和扫描结束事件，如果是光焦度变化事件，消息数据返回的是当前光焦度值。\n
             \li 注意：请勿在回调函数中执行耗时操作，否则会阻塞串口数据的读写，影响设备通信性能。\n

 *  @~english
 *  @brief  Executes focal power range scan for liquid lens.
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  pstRangeScanParam           [IN]            It refers to the range scan parameters.
 *  @param  cbLiquidLensMsg             [IN]            It refers to the callback function for liquid lens messages.
 *  @param  pUser                       [IN]            It refers to the user-defined variable.
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 *  @remarks Before calling this API, the user needs to set the camera to software trigger mode first. Otherwise, the camera cannot trigger and output images during the scan process. 
 *           The timing of camera trigger image output is affected by the trigger delay time, which can be set through the MV_CC_LiquidLens_SetTriggerDelayTime API.\n
 *           In the callback, you can obtain focal power change events and scan end events. If it is a focal power change event, the message data returns the current focal power value.
 *           Note: Do not perform time-consuming operations in the callback function, otherwise it will block serial port data read/write and affect device communication performance.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_LiquidLens_FocalPowerRangeScan(IN void* handle, IN MV_CC_LIQUIDLENS_RANGE_SCAN_PARAM* pstRangeScanParam, IN MvLiquidLensMsgCallback cbLiquidLensMsg, IN void* pUser);

/********************************************************************//**
 *  @~chinese
 *  @brief  执行多焦度轮询
 *  @param  handle                      [IN]            设备句柄
 *  @param  pstMultiFPScanParam         [IN]            多焦度扫描参数
 *  @param  cbLiquidLensMsg             [IN]            液态镜头上传消息回调函数
 *  @param  pUser                       [IN]            用户自定义变量
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks \li 调用接口前需要用户先把相机设置为软触发模式，否则扫描过程中相机无法触发出图。相机触发出图的时机受触发延迟时间的影响，可以通过MV_CC_LiquidLens_SetTriggerDelayTime接口设置。\n
             \li 在回调中可以获取到光焦度变化事件和扫描结束事件，如果是光焦度变化事件，消息数据返回的是当前光焦度值。\n
             \li 注意：请勿在回调函数中执行耗时操作，否则会阻塞串口数据的读写，影响设备通信性能。\n

 *  @~english
 *  @brief  Executes multi focal power scan for liquid lens.
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  pstMultiFPScanParam         [IN]            It refers to the multi focal power scan parameters.
 *  @param  cbLiquidLensMsg             [IN]            It refers to the callback function for liquid lens messages.
 *  @param  pUser                       [IN]            It refers to the user-defined variable.
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 *  @remarks Before calling this API, the user needs to set the camera to software trigger mode first. Otherwise, the camera cannot trigger and output images during the scan process.
 *           The timing of camera trigger image output is affected by the trigger delay time, which can be set through the MV_CC_LiquidLens_SetTriggerDelayTime API.
 *           In the callback, you can obtain focal power change events and scan end events. If it is a focal power change event, the message data returns the current focal power value.
 *           Note: Do not perform time-consuming operations in the callback function, otherwise it will block serial port data read/write and affect device communication performance.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_LiquidLens_MultiFocalPowerScan(IN void* handle, IN MV_CC_LIQUIDLENS_MULTI_FP_SCAN_PARAM* pstMultiFPScanParam, IN MvLiquidLensMsgCallback cbLiquidLensMsg, IN void* pUser);

/********************************************************************//**
 *  @~chinese
 *  @brief  设置触发延时
 *  @param  handle                      [IN]            设备句柄
 *  @param  nDelayTimeMs                [IN]            触发延迟时间，单位：毫秒,不能大于65535
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 触发延迟时间决定相机触发出图的时机，比如发送设置光焦度命令后，等待一个触发延迟时间后，再触发出图（如果需要）。

 *  @~english
 *  @brief  Sets the trigger delay time for liquid lens.
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  nDelayTimeMs                [IN]            It refers to the trigger delay time, unit: millisecond,cannot exceed 65535.
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 *  @remarks The trigger delay time determines when the camera triggers to output images. For example, after sending a focal power setting command, wait for the trigger delay time before triggering image output (if needed).
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_LiquidLens_SetTriggerDelayTime(IN void* handle, IN unsigned int nDelayTimeMs);

/********************************************************************//**
 *  @~chinese
 *  @brief  获取触发延时
 *  @param  handle                      [IN]            设备句柄
 *  @param  pnDelayTimeMs               [OUT]           触发延迟时间
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。

 *  @~english
 *  @brief  Gets the trigger delay time for liquid lens.
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  pnDelayTimeMs               [OUT]           It refers to the trigger delay time.
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_LiquidLens_GetTriggerDelayTime(IN void* handle, OUT unsigned int* pnDelayTimeMs);
/********************************************************************//**
 *  @~chinese
 *  @brief  保存参数到指定用户集
 *  @param  handle                      [IN]            设备句柄
 *  @param  enUserSet                   [IN]            用户集
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。

 *  @~english
 *  @brief  Saves parameters to the specified user set of liquid lens.
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  enUserSet                   [IN]            It refers to the user set.
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_LiquidLens_SaveUserSet(IN void* handle, IN MV_CC_LIQUIDLENS_USERSET enUserSet);

/********************************************************************//**
 *  @~chinese
 *  @brief  获取当前用户集
 *  @param  handle                      [IN]            设备句柄
 *  @param  penUserSet                  [OUT]           当前用户集
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。

 *  @~english
 *  @brief  Gets the current user set of liquid lens.
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  penUserSet                  [OUT]           It refers to the current user set.
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_LiquidLens_GetCurrentUserSet(IN void* handle, OUT MV_CC_LIQUIDLENS_USERSET* penUserSet);
/********************************************************************//**
 *  @~chinese
 *  @brief  加载指定用户集
 *  @param  handle                      [IN]            设备句柄
 *  @param  enUserSet                   [IN]            用户集
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。

 *  @~english
 *  @brief  Loads the specified user set for liquid lens.
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  enUserSet                   [IN]            It refers to the user set.
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_LiquidLens_LoadUserSet(IN void* handle, IN MV_CC_LIQUIDLENS_USERSET enUserSet);

/********************************************************************//**
 *  @~chinese
 *  @brief  设置默认用户集
 *  @param  handle                      [IN]            设备句柄
 *  @param  enUserSet                   [IN]            用户集
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 调用此接口后，镜头重新上电后，将默认加载设定的用户集。

 *  @~english
 *  @brief  Sets the default user set for liquid lens.
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  enUserSet                   [IN]            It refers to the user set.
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
 *  @remarks After calling this API, the lens will load the set user set by default after power-on.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_LiquidLens_SetDefaultUserSet(IN void* handle, IN MV_CC_LIQUIDLENS_USERSET enUserSet);

/********************************************************************//**
 *  @~chinese
 *  @brief  获取默认用户集
 *  @param  handle                      [IN]            设备句柄
 *  @param  penUserSet                  [OUT]           默认用户集
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。

 *  @~english
 *  @brief  Gets the default user set of liquid lens.
 *  @param  handle                      [IN]            It refers to the device handle.
 *  @param  penUserSet                  [OUT]           It refers to the default user set.
 *  @return Returns MV_OK for success, and returns corresponding Error Code for failure.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_LiquidLens_GetDefaultUserSet(IN void* handle, OUT MV_CC_LIQUIDLENS_USERSET* penUserSet);
/********************************************************************//**
*  @~chinese
 *  @brief  设置液态镜头温度变化检测时间间隔
 *  @param  handle              [IN]    设备句柄
 *  @param  nIntervalSeconds    [IN]    检测时间间隔（单位：秒），不能大于65535
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  
 *  @~english
 *  @brief  Set temperature detection interval for liquid lens
 *  @param  handle              [IN]    It refers to the device handle.
 *  @param  nIntervalSeconds    [IN]    It refers to the detection interval in seconds,cannot exceed 65535.
 *  @return Success, return \ref status code "MV_OK"; Failure, return \ref status code "status code".
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_LiquidLens_SetTempDetectInterval(IN void* handle, IN unsigned int nIntervalSeconds);
/********************************************************************//**
 *  @~chinese
 *  @brief  获取液态镜头温度变化检测时间间隔
 *  @param  handle              [IN]    设备句柄
 *  @param  pnIntervalSeconds   [OUT]   检测时间间隔（单位：秒）
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  
 *  @~english
 *  @brief  Get temperature detection interval for liquid lens
 *  @param  handle              [IN]    It refers to the device handle.
 *  @param  pnIntervalSeconds   [OUT]   It refers to the detection interval in seconds
 *  @return Success, return \ref status code "MV_OK"; Failure, return \ref status code "status code".
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_LiquidLens_GetTempDetectInterval(IN void* handle, OUT unsigned int* pnIntervalSeconds);
/********************************************************************//**
 *  @~chinese
 *  @brief  注册液态镜头异常事件回调接口
 *  @param  handle          [IN]    设备句柄
 *  @param  cbLiquidLensException     [IN]    异常事件回调函数指针
 *  @param  pUser           [IN]    用户自定义数据
 *  @return 成功，返回\ref 状态码 "MV_OK"；失败，返回\ref 状态码 "状态码"。
 *  @remarks 镜头发生异常时，可以在回调里面获取到异常消息，比如镜头异常断开。

 *  @~english
 *  @brief  Register liquid lens exception event callback interface
 *  @param  handle                    [IN]    It refers to the device handle.
 *  @param  cbLiquidLensException     [IN]    It refers to the exception event callback function pointer
 *  @param  pUser                     [IN]    It refers to the user defined data
 *  @return Success, return \ref status code "MV_OK"; Failure, return \ref status code "status code".
 *  @remarks When the lens encounters exceptions, you can obtain the exception information through the callback, such as obtaining the exception message after the lens is abnormally disconnected.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_LiquidLens_RegisterExceptionCallBack(IN void* handle, IN MvLiquidLensExceptionCallback cbLiquidLensException, IN void* pUser);
/// @}
#ifdef __cplusplus
}
#endif 

#endif //_MV_CAMERA_CTRL_H_
