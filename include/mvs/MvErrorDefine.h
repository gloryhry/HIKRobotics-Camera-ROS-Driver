
#ifndef _MV_ERROR_DEFINE_H_
#define _MV_ERROR_DEFINE_H_

#include "MvISPErrorDefine.h"

/********************************************************************/
///  \~chinese
///  \name 正确码定义
///  @{
///  \~english
///  \name Definition of correct code
///  @{
#define MV_OK                                       0x00000000  ///< \~chinese 成功，无错误             \~english Successed, no error
/// @}

/********************************************************************/
///  \~chinese
///  \name 通用错误码定义:范围0x80000000-0x800000FF
///  @{
///  \~english
///  \name Definition of General error code
///  @{
#define MV_E_HANDLE                                 0x80000000  ///< \~chinese 错误或无效的句柄         \~english Error or invalid handle
#define MV_E_SUPPORT                                0x80000001  ///< \~chinese 不支持的功能             \~english Not supported function
#define MV_E_BUFOVER                                0x80000002  ///< \~chinese 缓存已满                 \~english Buffer overflow
#define MV_E_CALLORDER                              0x80000003  ///< \~chinese 函数调用顺序错误         \~english Function calling order error
#define MV_E_PARAMETER                              0x80000004  ///< \~chinese 错误的参数               \~english Incorrect parameter
#define MV_E_RESOURCE                               0x80000006  ///< \~chinese 资源申请失败(内存)       \~english Applying resource failed (Memory)
#define MV_E_NODATA                                 0x80000007  ///< \~chinese 超时，未收到数据         \~english timeout, no data received
#define MV_E_PRECONDITION                           0x80000008  ///< \~chinese 前置条件有误，或运行环境已发生变化   \~english Precondition error, or running environment changed
#define MV_E_VERSION                                0x80000009  ///< \~chinese 版本不匹配               \~english Version mismatches
#define MV_E_NOENOUGH_BUF                           0x8000000A  ///< \~chinese 内存空间不足（传入内存偏小）       \~english Insufficient memory (Input is Too Small)
#define MV_E_ABNORMAL_IMAGE                         0x8000000B  ///< \~chinese 异常图像，可能是丢包导致图像不完整   \~english Abnormal image, maybe incomplete image because of lost packet
#define MV_E_LOAD_LIBRARY                           0x8000000C  ///< \~chinese 动态导入DLL失败          \~english Load library failed
#define MV_E_NOOUTBUF                               0x8000000D  ///< \~chinese 没有可输出的缓存         \~english No Avaliable Buffer
#define MV_E_ENCRYPT                                0x8000000E  ///< \~chinese 加密错误                 \~english Encryption error
#define MV_E_OPENFILE                               0x8000000F  ///< \~chinese 打开文件出现错误         \~english open file error
#define MV_E_BUF_IN_USE                             0x80000010  ///< \~chinese 缓存地址已使用           \~english Buffer already in use
#define MV_E_BUF_INVALID                            0x80000011  ///< \~chinese 无效的缓存地址           \~english Buffer address invalid
#define MV_E_NOALIGN_BUF                            0x80000012  ///< \~chinese 缓存对齐异常             \~english Buffer alignmenterror error
#define MV_E_NOENOUGH_BUF_NUM                       0x80000013  ///< \~chinese 缓存个数不足             \~english Insufficient cache count
#define MV_E_PORT_IN_USE                            0x80000014  ///< \~chinese 串口被占用               \~english Port is in use
#define MV_E_IMAGE_DECODEC                          0x80000015  ///< \~chinese 解码错误(SDK校验图像异常)\~english Decoding error (SDK verification image exception)
#define MV_E_UINT32_LIMIT                           0x80000016  ///< \~chinese 图像大小超过unsigned int返回，接口不支持   \~english  The image size exceeds the maximum value of an unsigned integer. The API does not support this.
#define MV_E_IMAGE_HEIGHT                           0x80000017  ///< \~chinese 图像高度异常（残帧丢弃） \~english image height anomaly (discard incomplete images)
#define MV_E_NOENOUGH_DDR                           0x80000018  ///< \~chinese 采集卡DDR缓存不足        \~english The frame grabber's DDR memory buffer is insufficient.
#define MV_E_NOENOUGH_STREAM                        0x80000019  ///< \~chinese 采集卡流通道不足         \~english The capture card has insufficient streaming channels.
#define MV_E_NORESPONSE                             0x8000001A  ///< \~chinese 设备无响应               \~english No response from device

#define MV_E_WRITEFILE                              0x8000001B  ///< \~chinese 写入文件异常             \~english Write file failed
#define MV_E_READFILE                               0x8000001C  ///< \~chinese 读取文件异常             \~english Read file failed
#define MV_E_FILELENGTH                             0x8000001D  ///< \~chinese 文件长度异常             \~english File length error
#define MV_E_RESOURCE_EVENT                         0x8000001E  ///< \~chinese 事件创建失败             \~english Event creation failed
#define MV_E_RESOURCE_THREAD                        0x8000001F  ///< \~chinese 线程创建失败             \~english Thread creation failed

#define MV_E_DEV_OFFLINE                            0x80000020  ///< \~chinese 设备已掉线               \~english Device is offline
#define MV_E_DEV_SUPPORT                            0x80000021  ///< \~chinese 设备不支持               \~english Device not supported
#define MV_E_PLATFORM_SUPPORT                       0x80000022  ///< \~chinese 当前平台未实现           \~english Current platform not implemented
#define MV_E_SERIAL_BUFFER_FULL                     0x80000023  ///< \~chinese 设备串口缓存空间不足     \~english Device serial port buffer space insufficient
#define MV_E_CHANNEL_INDEX                          0x80000024  ///< \~chinese 流通道索引无效           \~english Stream channel index invalid
#define MV_E_PARAMETER_RANGE                        0x80000025  ///< \~chinese 参数超出范围限制         \~english Parameter out of range
#define MV_E_RESOURCE_IO                            0x80000026  ///< \~chinese IO资源异常               \~english IO resource exception
#define MV_E_IMAGE_INFO_INVALID                     0x80000027  ///< \~chinese 图像信息(宽、高和长度等)异常  \~english Abnormal image information (width, height, length, etc.)
#define MV_E_RESOURCE_IN_USE                        0x80000028  ///< \~chinese 请求的资源（采集卡、设备和流等）已被占用 \~english The requested resource (capture card, device, stream, etc.) is in use.

#define MV_E_DEV_NOT_IMPLEMENTED                    0x80000041  ///< \~chinese 命令在设备中未实现       \~english Command not implemented in device
#define MV_E_DEV_INVALID_PARAMETER                  0x80000042  ///< \~chinese 命令参数无效或超出范围   \~english Command parameter invalid or out of range
#define MV_E_DEV_INVALID_ADDRESS                    0x80000043  ///< \~chinese 尝试访问不存在的寄存器地址   \~english Attempted to access non-existent register address
#define MV_E_DEV_WRITE_PROTECT                      0x80000044  ///< \~chinese 尝试写入只读寄存器       \~english Attempted to write to read-only register
#define MV_E_DEV_BAD_ALIGNMENT                      0x80000045  ///< \~chinese 尝试访问地址未根据底层技术对齐的寄存器   \~english Attempted to access register address not aligned according to underlying technology
#define MV_E_DEV_ACCESS_DENIED                      0x80000046  ///< \~chinese 尝试读取不可读或写入不可写的寄存器地址   \~english Attempted to read from unreadable or write to unwritable register address
#define MV_E_DEV_BUSY                               0x80000047  ///< \~chinese 设备当前正忙             \~english Device is currently busy
#define MV_E_DEV_MSG_TIMEOUT                        0x80000048  ///< \~chinese 设备应答超时             \~english Device response timeout
#define MV_E_DEV_INVALID_HEADER                     0x80000049  ///< \~chinese 接收到的命令头部无效     \~english Received command header invalid
#define MV_E_DEV_UNKNOWN                            0x8000004A  ///< \~chinese 设备返回的未知错误码（并未转换成对应错误码）   \~english Unknown error code returned by device (not converted to corresponding error code)
#define MV_E_DEV_INVALID_PARAMS                     0x8000004B  ///< \~chinese 设备返回无效的参数       \~english Device returned invalid parameters
#define MV_E_DEV_WRONG_CONFIG                       0x8000004C  ///< \~chinese 当前设备的配置不允许执行发送的命令   \~english Current device configuration does not allow execution of sent command
#define MV_E_DEV_CRC                                0x8000004D  ///< \~chinese CRC错误                  \~english CRC error

#define MV_E_INTERNAL                               0x800000FE  ///< \~chinese SDK内部错误              \~english SDK internal error
#define MV_E_UNKNOW                                 0x800000FF  ///< \~chinese 未知的错误               \~english Unknown error

/// @}

/********************************************************************/
///  \~chinese
///  \name GenICam系列错误:范围0x80000100-0x800001FF
///  @{
///  \~english
///  \name GenICam Series Error Codes: Range from 0x80000100 to 0x800001FF
///  @{
#define MV_E_GC_GENERIC                             0x80000100  ///< \~chinese 通用错误                 \~english General error
#define MV_E_GC_ARGUMENT                            0x80000101  ///< \~chinese 参数非法                 \~english Illegal parameters
#define MV_E_GC_RANGE                               0x80000102  ///< \~chinese 值超出范围               \~english The value is out of range
#define MV_E_GC_PROPERTY                            0x80000103  ///< \~chinese 属性                     \~english Property
#define MV_E_GC_RUNTIME                             0x80000104  ///< \~chinese 运行环境有问题           \~english Running environment error
#define MV_E_GC_LOGICAL                             0x80000105  ///< \~chinese 逻辑错误                 \~english Logical error
#define MV_E_GC_ACCESS                              0x80000106  ///< \~chinese 节点访问条件有误         \~english Node accessing condition error
#define MV_E_GC_TIMEOUT                             0x80000107  ///< \~chinese 超时                     \~english Timeout
#define MV_E_GC_DYNAMICCAST                         0x80000108  ///< \~chinese 转换异常                 \~english Transformation exception

#define MV_E_GC_NODE_NOT_FOUND                      0x80000109  ///< \~chinese 节点不存在               \~english Node does not exist
#define MV_E_GC_NODE_VERIFY                         0x8000010A  ///< \~chinese 节点校验失败             \~english Node validation failed
#define MV_E_GC_FILE                                0x8000010B  ///< \~chinese GenICam 文件异常         \~english GenICam file  error
#define MV_E_GC_URL_DESC                            0x8000010C  ///< \~chinese 设备URL描述符异常        \~english Device XML url error

#define MV_E_GC_UNKNOW                              0x800001FF  ///< \~chinese GenICam未知错误          \~english GenICam unknown error
/// @}

/********************************************************************/
///  \~chinese
///  \name GigE_STATUS对应的错误码:范围0x80000200-0x800002FF
///  @{
///  \~english
///  \name GigE_STATUS Error Codes: Range from 0x80000200 to 0x800002FF
///  @{
#define MV_E_NOT_IMPLEMENTED                        0x80000200  ///< \~chinese 命令不被设备支持         \~english The command is not supported by device
#define MV_E_INVALID_ADDRESS                        0x80000201  ///< \~chinese 访问的目标地址不存在     \~english The target address being accessed does not exist
#define MV_E_WRITE_PROTECT                          0x80000202  ///< \~chinese 目标地址不可写           \~english The target address is not writable
#define MV_E_ACCESS_DENIED                          0x80000203  ///< \~chinese 设备无访问权限           \~english No permission
#define MV_E_BUSY                                   0x80000204  ///< \~chinese 设备忙，或网络断开       \~english Device is busy, or network disconnected
#define MV_E_PACKET                                 0x80000205  ///< \~chinese 网络包数据错误           \~english Network data packet error
#define MV_E_NETER                                  0x80000206  ///< \~chinese 网络相关错误             \~english Network error
#define MV_E_DRIVERATTACH                           0x80000207  ///< \~chinese GIGE驱动未绑定           \~english GigE Driver Not Assigned
#define MV_E_PACKET_ID_MISMATCH                     0x80000208  ///< \~chinese 收到的包ID不匹配         \~english Packet ID mismatch
#define MV_E_IMAGE_BUFFER_OVERFLOW                  0x80000209  ///< \~chinese 图像缓冲区溢出           \~english Image buffer overflow
#define MV_E_NO_BUFFER_FOR_USE                      0x8000020A  ///< \~chinese 没有可用的缓冲区供使用   \~english No available buffer for use
#define MV_E_XML_INFO_PACKET_ERR                    0x8000020B  ///< \~chinese XML信息包解析错误        \~english XML information packet parsing error
#define MV_E_TIMEOUT                                0x8000020C  ///< \~chinese 超时                     \~english Network error
#define MV_E_NET_TRANSMISSION_TYPE_ERR              0x8000020D  ///< \~chinese 网络传输类型参数错误（独播、组播等)   \~english Network transmission type parameter error (unicast, multicast, etc.)
#define MV_E_SUPPORT_MODIFY_DEVICE_IP               0x8000020E  ///< \~chinese 在固定IP模式下不支持修改设备IP模式   \~english Device IP mode modification not supported in fixed IP mode
#define MV_E_KEY_VERIFICATION                       0x8000020F  ///< \~chinese 秘钥校验错误             \~english Key verification error
#define MV_E_VALUE_NOT_EXPECTED                     0x80000210  ///< \~chinese 值不符合预期             \~english Value does not match expectation
#define MV_E_DEV_DISCONNECT                         0x80000211  ///< \~chinese 设备已断开连接           \~english Device disconnected
#define MV_E_UDP_INIT                               0x80000212  ///< \~chinese UDP初始化失败            \~english UDP initialization failed
#define MV_E_UDP_SEND_DATA                          0x80000213  ///< \~chinese UDP发送数据失败（网络异常）   \~english UDP send data failed (network exception)
#define MV_E_UDP_RECV_DATA                          0x80000214  ///< \~chinese UDP接收数据失败（网络异常）   \~english UDP receive data failed (network exception)
#define MV_E_UDP_CONNECT                            0x80000215  ///< \~chinese UDP连接失败              \~english UDP connection failed
#define MV_E_UDP_RESET_CONNECT                      0x80000216  ///< \~chinese UDP重置连接失败          \~english UDP reset connection failed
#define MV_E_MULTICAST_ADD_DEVICE                   0x80000217  ///< \~chinese 添加组播设备失败         \~english Failed to add multicast device
#define MV_E_MULTICAST_IP_INVALID                   0x80000218  ///< \~chinese IP地址异常，不在组播范围内   \~english IP address abnormal, not within multicast range

#define MV_E_IP_CONFLICT                            0x80000221  ///< \~chinese 设备IP冲突               \~english Device IP conflict

/********************************************************************/
///  \~chinese
///  \name USB_STATUS对应的错误码:范围0x80000300-0x800003FF
///  @{
///  \~english
///  \name USB_STATUS Error Codes: Range from 0x80000300 to 0x800003FF
///  @{
#define MV_E_USB_READ                               0x80000300  ///< \~chinese 读usb出错                \~english Reading USB error
#define MV_E_USB_WRITE                              0x80000301  ///< \~chinese 写usb出错                \~english Writing USB error
#define MV_E_USB_DEVICE                             0x80000302  ///< \~chinese 设备异常                 \~english Device exception
#define MV_E_USB_GENICAM                            0x80000303  ///< \~chinese GenICam相关错误          \~english GenICam error
#define MV_E_USB_BANDWIDTH                          0x80000304  ///< \~chinese 带宽不足                 \~english Insufficient bandwidth
#define MV_E_USB_DRIVER                             0x80000305  ///< \~chinese 驱动不匹配或者未装驱动   \~english Driver mismatch or unmounted drive
#define MV_E_USB_UNKNOW                             0x800003FF  ///< \~chinese USB未知的错误            \~english USB unknown error
/// @}

/********************************************************************/
///  \~chinese
///  \name 升级时对应的错误码:范围0x80000400-0x800004FF
///  @{
///  \~english
///  \name Upgrade Error Codes: Range from 0x80000400 to 0x800004FF
///  @{
#define MV_E_UPG_FILE_MISMATCH                      0x80000400  ///< \~chinese 升级固件不匹配           \~english Firmware mismatches
#define MV_E_UPG_LANGUSGE_MISMATCH                  0x80000401  ///< \~chinese 升级固件语言不匹配       \~english Firmware language mismatches
#define MV_E_UPG_CONFLICT                           0x80000402  ///< \~chinese 升级冲突（设备已经在升级了再次请求升级即返回此错误） \~english Upgrading conflicted (repeated upgrading requests during device upgrade)
#define MV_E_UPG_INNER_ERR                          0x80000403  ///< \~chinese 升级时设备内部出现错误   \~english Camera internal error during upgrade

#define MV_E_UPG_UNKNOW                             0x800004FF  ///< \~chinese 升级时未知错误           \~english Unknown error during upgrade
/// @}

/********************************************************************/
///  \~chinese
///  \name 图像处理相关:范围0x80000500-0x800005FF
///  @{
///  \~english
///  \name Image Processing Error Codes: Range from 0x80000500 to 0x800005FF
///  @{

//Mediaprocess范围： 0x80000500 - 0x8000052F
#define MV_E_SUPPORT_PIXEL_FORMAT                   0x80000500  ///< \~chinese 不支持的像素格式         \~english Unsupported pixel format
#define MV_E_SUPPORT_IMAGE_TYPE                     0x80000501  ///< \~chinese 不支持的图像类型 (比如BMP,TIFF,JPG等）   \~english The image types are not supported: such as BMP, TIFF, JPEG, etc.
#define MV_E_NOENOUGH_INPUT_DATA                    0x80000502  ///< \~chinese 输入图像数据不足         \~english Insufficient input image data

//渲染范围：0x80000530 - 0x8000055F
#define MV_E_SR_NOT_INITIAL                         0x80000530  ///< \~chinese 未初始化                 \~english Not initialized
#define MV_E_SR_SUPPORT_FUNCTION                    0x80000531  ///< \~chinese 不支持接口               \~english Interface not supported
#define MV_E_SR_SUPPORT_ENGINE                      0x80000532  ///< \~chinese 不支持渲染引擎           \~english Render engine not supported
#define MV_E_SR_SUPPORT_PIXELTYPE                   0x80000533  ///< \~chinese 不支持像素格式           \~english Pixel format not supported
#define MV_E_SR_SUPPORT_TEXTURESIZE                 0x80000534  ///< \~chinese 不支持纹理大小           \~english Texture size not supported
#define MV_E_SR_SUPPORT_WND                         0x80000535  ///< \~chinese 不支持显示窗口           \~english Display window not supported
#define MV_E_SR_SUPPORT_EFFECT                      0x80000536  ///< \~chinese 不支持显示效果           \~english Display effect not supported
#define MV_E_SR_SUPPORT_VIEWTYPE                    0x80000537  ///< \~chinese 不支持视角变换           \~english View transformation not supported
#define MV_E_SR_SUPPORT_STATE                       0x80000538  ///< \~chinese 不支持渲染状态           \~english Render state not supported

#define MV_E_SR_SUBPORT                             0x80000539  ///< \~chinese 端口号无效               \~english Invalid port number
#define MV_E_SR_PORT_USING                          0x8000053A  ///< \~chinese 端口号被占用             \~english Port number in use
#define MV_E_SR_D3D_RESOURCE                        0x8000053B  ///< \~chinese 创建D3D相关资源失败      \~english Failed to create D3D related resources
#define MV_E_SR_SWAPCHAIN                           0x8000053C  ///< \~chinese 交换链相关错误           \~english Swap chain related error
#define MV_E_SR_SHADER                              0x8000053D  ///< \~chinese shader操作时相关错误     \~english Shader operation related error
#define MV_E_SR_FONT                                0x8000053E  ///< \~chinese 写字时相关错误           \~english Writing related error
#define MV_E_SR_LOAD_LIBRARY                        0x8000053F  ///< \~chinese 动态加载库失败           \~english Failed to dynamically load library
#define MV_E_SR_OPENGL_RESOURCE                     0x80000540  ///< \~chinese 创建OpenGL相关资源失败   \~english Failed to create OpenGL related resources
#define MV_E_SR_CONTEXT                             0x80000541  ///< \~chinese context操作失败          \~english Context operation failed
#define MV_E_SR_PRESENT                             0x80000542  ///< \~chinese 因为显卡原因Present接口返回失败           \~english Present interface returned failure due to graphics card reason
#define MV_E_SR_INVALID_RECT                        0x80000543  ///< \~chinese 无效的矩形区域           \~english Invalid rectangular area
#define MV_E_SR_INVALID_FLOAT                       0x80000544  ///< \~chinese 无效的归一化浮点值       \~english Invalid normalized floating point value
#define MV_E_SR_INVALID_COLOR                       0x80000545  ///< \~chinese 无效的颜色               \~english Invalid color
#define MV_E_SR_INVALID_POINT                       0x80000546  ///< \~chinese 无效的点                 \~english Invalid point
#define MV_E_SR_RUNTIME                             0x80000547  ///< \~chinese 运行环境错误             \~english Runtime environment error

/// @}

/********************************************************************/
///  \~chinese
///  \name 液态镜头错误码:范围0x80000600-0x800006FF
///  @{
///  \~english
///  \name LiuidLens Error Codes: Range from 0x80000600 to 0x800006FF
///  @{

#define MV_E_LIQUIDLENS_CMD_NOT_SUPPORT             0x80000601  ///< \~chinese 不支持的命令             \~english Unsupported command
#define MV_E_LIQUIDLENS_REGISTER_NOT_EXIST          0x80000602  ///< \~chinese 寄存器不存在             \~english Register does not exist
#define MV_E_LIQUIDLENS_PERMISSION_DENIED           0x80000603  ///< \~chinese 权限错误，访问被拒       \~english Permission error, access denied
#define MV_E_LIQUIDLENS_CHECKSUM_ERROR              0x80000604  ///< \~chinese checksum校验失败         \~english Checksum verification failed
#define MV_E_LIQUIDLENS_PACKET_FORMAT_ERROR         0x80000605  ///< \~chinese 数据包格式错误           \~english Data packet format error
#define MV_E_LIQUIDLENS_DATA_FOAMAT_ERROR           0x80000606  ///< \~chinese Data字段格式错误         \~english Data field format error
#define MV_E_LIQUIDLENS_DATA_OUT_RANGE              0x80000607  ///< \~chinese 参数超出范围             \~english Parameter out of range
#define MV_E_LIQUIDLENS_WRITE_DATA_LENGTH_ERROR     0x80000608  ///< \~chinese 写入数据长度与寄存器长度不匹配   \~english Write data length does not match register length
#define MV_E_LIQUIDLENS_DEVICE_BUSY                 0x80000609  ///< \~chinese 设备忙无法响应           \~english Device busy, unable to respond
#define MV_E_LIQUIDLENS_DATA_INCORRECT_ORDER        0x8000060A  ///< \~chinese 命令顺序错误             \~english Command order error
#define MV_E_LIQUIDLENS_RUN_COND_NOT_MET            0x8000060B  ///< \~chinese 运行条件不满足,比如光焦度区间扫描参数配置有问题   \~english Run condition not met, for example, light focal length range scan parameter configuration is problematic

#define MV_E_LIQUIDLENS_COMMANDTIMEOUT              0x80000631  ///< \~chinese 镜头指令超时             \~english Lens command timeout
#define MV_E_LIQUIDLENS_OFFLINE                     0x80000632  ///< \~chinese 镜头离线                 \~english Lens offline
#define MV_E_LIQUIDLENS_AF_IMAGE_ABNORMAL           0x80000633  ///< \~chinese 对焦时图像异常,比如画面全黑全白等   \~english Image abnormal during focusing, such as completely black or white画面
#define MV_E_LIQUIDLENS_ACK_DATA_LENGTH_ERROR       0x80000634  ///< \~chinese 串口回包的数据长度错误   \~english Serial port response data length error
#define MV_E_LIQUIDLENS_TRIGGER_MODE_NOT_OPEN       0x80000635  ///< \~chinese 触发模式未打开           \~english Trigger mode not opened
#define MV_E_LIQUIDLENS_NOT_SOFT_TRIGGER_MODE       0x80000636  ///< \~chinese 不是软触发模式           \~english Not soft trigger mode
#define MV_E_LIQUIDLENS_DEVICE_NOT_GRABBING         0x80000637  ///< \~chinese 设备未处于取流状态       \~english Device not in grabbing state
#define MV_E_LIQUIDLENS_STRATEGY_NOT_ONEBYONE       0x80000638  ///< \~chinese 流策略不支持             \~english Stream strategy not supported
#define MV_E_LIQUIDLENS_AF_IMAGE_LOST               0x80000639  ///< \~chinese 对焦时图像缺失或数量异常   \~english Image missing or abnormal count during focusing
#define MV_E_LIQUIDLENS_AF_NOT_CONVERGED            0x8000063A  ///< \~chinese 对焦时达到最大迭代次数但还没有满足收敛条件   \~english Reached maximum iteration count during focusing but convergence condition not met
#define MV_E_LIQUIDLENS_SERIAL_PORT_PARAMS_FAIL     0x8000063B  ///< \~chinese 串口参数配置失败         \~english Serial port parameter configuration failed
#define MV_E_LIQUIDLENS_INIT_FAILED                 0x8000063C  ///< \~chinese 镜头初始化失败           \~english Lens initialization failed
#define MV_E_LIQUIDLENS_TASK_EXECUTING              0x8000063D  ///< \~chinese 设备忙碌，当前已有任务在执行       \~english Device is busy, a task is currently executing
#define MV_E_LIQUIDLENS_AF_SHARPNESS_CALC_FAILED    0x8000063E  ///< \~chinese 自动对焦时，清晰度计算失败        \~english Sharpness calculation failed during auto focus
#define MV_E_LIQUIDLENS_AF_FRAME_RATE_LOW           0x8000063F  ///< \~chinese 自动对焦时，帧率过低        \~english Frame rate is too low during auto focus

#define MV_E_LIQUIDLENS_UNDEFINED_ERROR             0x800006FF  ///< \~chinese 未定义错误               \~english Undefined error

#endif //_MV_ERROR_DEFINE_H_
