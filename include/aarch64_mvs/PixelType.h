
#ifndef _MV_PIXEL_TYPE_H_
#define _MV_PIXEL_TYPE_H_

/************************************************************************/
/*     GigE Vision (2.0.03) PIXEL FORMATS                               */
/************************************************************************/

// Indicate if pixel is monochrome or RGB
#define MV_GVSP_PIX_MONO                                0x01000000 
#define MV_GVSP_PIX_RGB                                 0x02000000 // deprecated in version 1.1
#define MV_GVSP_PIX_COLOR                               0x02000000
#define MV_GVSP_PIX_CUSTOM                              0x80000000
#define MV_GVSP_PIX_COLOR_MASK                          0xFF000000

// Indicate effective number of bits occupied by the pixel (including padding).
// This can be used to compute amount of memory required to store an image.
#define MV_PIXEL_BIT_COUNT(n)                           ((n) << 16)

#define MV_GVSP_PIX_EFFECTIVE_PIXEL_SIZE_MASK           0x00FF0000
#define MV_GVSP_PIX_EFFECTIVE_PIXEL_SIZE_SHIFT          16

// Pixel ID: lower 16-bit of the pixel formats
#define MV_GVSP_PIX_ID_MASK                             0x0000FFFF
#define MV_GVSP_PIX_COUNT                               0x46 // next Pixel ID available

/// \addtogroup 像素格式定义
///@{

///< \~chinese 图片格式定义
enum MvGvspPixelType
{
    // Undefined pixel type
#ifdef WIN32
	PixelType_Gvsp_Undefined                =   0xFFFFFFFF,     ///< 未定义的像素类型

#else
	PixelType_Gvsp_Undefined                =   -1,             ///< 未定义的像素类型

#endif
    // Mono buffer format defines
    PixelType_Gvsp_Mono1p                   = (MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(1) | 0x0037),                  ///< Mono1p
    PixelType_Gvsp_Mono2p                   = (MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(2) | 0x0038),                  ///< Mono2p
    PixelType_Gvsp_Mono4p                   = (MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(4) | 0x0039),                  ///< Mono4p
    PixelType_Gvsp_Mono8                    = (MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(8) | 0x0001),                  ///< Mono8
    PixelType_Gvsp_Mono8_Signed             = (MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(8) | 0x0002),                  ///< Mono8_Signed
    PixelType_Gvsp_Mono10                   = (MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(16) | 0x0003),                 ///< Mono10
    PixelType_Gvsp_Mono10_Packed            = (MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(12) | 0x0004),                 ///< Mono10_Packed
    PixelType_Gvsp_Mono12                   = (MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(16) | 0x0005),                 ///< Mono12
    PixelType_Gvsp_Mono12_Packed            = (MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(12) | 0x0006),                 ///< Mono12_Packed
    PixelType_Gvsp_Mono14                   = (MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(16) | 0x0025),                 ///< Mono14
    PixelType_Gvsp_Mono16                   = (MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(16) | 0x0007),                 ///< Mono16

    // Bayer buffer format defines 
    PixelType_Gvsp_BayerGR8                 = (MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(8) | 0x0008),                  ///< BayerGR8
    PixelType_Gvsp_BayerRG8                 = (MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(8) | 0x0009),                  ///< BayerRG8
    PixelType_Gvsp_BayerGB8                 = (MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(8) | 0x000A),                  ///< BayerGB8
    PixelType_Gvsp_BayerBG8                 = (MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(8) | 0x000B),                  ///< BayerBG8
    PixelType_Gvsp_BayerRBGG8               = (MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(8) | 0x0046),                  ///< BayerRBGG8
    PixelType_Gvsp_BayerBRGG8               = (MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(8) | 0x0047),                  ///< BayerBRGG8
    PixelType_Gvsp_BayerGR10                = (MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(16) | 0x000C),                 ///< BayerGR10
    PixelType_Gvsp_BayerRG10                = (MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(16) | 0x000D),                 ///< BayerRG10
    PixelType_Gvsp_BayerGB10                = (MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(16) | 0x000E),                 ///< BayerGB10
    PixelType_Gvsp_BayerBG10                = (MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(16) | 0x000F),                 ///< BayerBG10
    PixelType_Gvsp_BayerGR12                = (MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(16) | 0x0010),                 ///< BayerGR12
    PixelType_Gvsp_BayerRG12                = (MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(16) | 0x0011),                 ///< BayerRG12
    PixelType_Gvsp_BayerGB12                = (MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(16) | 0x0012),                 ///< BayerGB12
    PixelType_Gvsp_BayerBG12                = (MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(16) | 0x0013),                 ///< BayerBG12
    PixelType_Gvsp_BayerGR10_Packed         = (MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(12) | 0x0026),                 ///< BayerGR10_Packed
    PixelType_Gvsp_BayerRG10_Packed         = (MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(12) | 0x0027),                 ///< BayerRG10_Packed
    PixelType_Gvsp_BayerGB10_Packed         = (MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(12) | 0x0028),                 ///< BayerGB10_Packed
    PixelType_Gvsp_BayerBG10_Packed         = (MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(12) | 0x0029),                 ///< BayerBG10_Packed
    PixelType_Gvsp_BayerGR12_Packed         = (MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(12) | 0x002A),                 ///< BayerGR12_Packed
    PixelType_Gvsp_BayerRG12_Packed         = (MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(12) | 0x002B),                 ///< BayerRG12_Packed
    PixelType_Gvsp_BayerGB12_Packed         = (MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(12) | 0x002C),                 ///< BayerGB12_Packed
    PixelType_Gvsp_BayerBG12_Packed         = (MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(12) | 0x002D),                 ///< BayerBG12_Packed
    PixelType_Gvsp_BayerGR16                = (MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(16) | 0x002E),                 ///< BayerGR16
    PixelType_Gvsp_BayerRG16                = (MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(16) | 0x002F),                 ///< BayerRG16
    PixelType_Gvsp_BayerGB16                = (MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(16) | 0x0030),                 ///< BayerGB16
    PixelType_Gvsp_BayerBG16                = (MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(16) | 0x0031),                 ///< BayerBG16

    // RGB Packed buffer format defines 
    PixelType_Gvsp_RGB8_Packed              =   (MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(24) | 0x0014),                  ///< RGB8_Packed
    PixelType_Gvsp_BGR8_Packed              =   (MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(24) | 0x0015),                  ///< BGR8_Packed
    PixelType_Gvsp_RGBA8_Packed             =   (MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(32) | 0x0016),                  ///< RGBA8_Packed
    PixelType_Gvsp_BGRA8_Packed             =   (MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(32) | 0x0017),                  ///< BGRA8_Packed
    PixelType_Gvsp_RGB10_Packed             =   (MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(48) | 0x0018),                  ///< RGB10_Packed
    PixelType_Gvsp_BGR10_Packed             =   (MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(48) | 0x0019),                  ///< BGR10_Packed
    PixelType_Gvsp_RGB12_Packed             =   (MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(48) | 0x001A),                  ///< RGB12_Packed
    PixelType_Gvsp_BGR12_Packed             =   (MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(48) | 0x001B),                  ///< BGR12_Packed
    PixelType_Gvsp_RGB16_Packed             =   (MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(48) | 0x0033),                  ///< RGB16_Packed
    PixelType_Gvsp_BGR16_Packed             =   (MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(48) | 0x004B),                  ///< BGR16_Packed
    PixelType_Gvsp_RGBA16_Packed            =   (MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(64) | 0x0064),                  ///< RGBA16_Packed
    PixelType_Gvsp_BGRA16_Packed            =   (MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(64) | 0x0051),                  ///< BGRA16_Packed
    PixelType_Gvsp_RGB10V1_Packed           =   (MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(32) | 0x001C),                  ///< RGB10V1_Packed
    PixelType_Gvsp_RGB10V2_Packed           =   (MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(32) | 0x001D),                  ///< RGB10V2_Packed
    PixelType_Gvsp_RGB12V1_Packed           =   (MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(36) | 0X0034),                  ///< RGB12V1_Packed
    PixelType_Gvsp_RGB565_Packed            =   (MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(16) | 0x0035),                  ///< RGB565_Packed
    PixelType_Gvsp_BGR565_Packed            =   (MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(16) | 0X0036),                  ///< BGR565_Packed

    // YUV Packed buffer format defines 
    PixelType_Gvsp_YUV411_Packed            =   (MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(12) | 0x001E),                  ///< YUV411_Packed
    PixelType_Gvsp_YUV422_Packed            =   (MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(16) | 0x001F),                  ///< YUV422_Packed
    PixelType_Gvsp_YUV422_YUYV_Packed       =   (MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(16) | 0x0032),                  ///< YUV422_YUYV_Packed
    PixelType_Gvsp_YUV444_Packed            =   (MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(24) | 0x0020),                  ///< YUV444_Packed
    PixelType_Gvsp_YCBCR8_CBYCR             =   (MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(24) | 0x003A),                  ///< YCBCR8_CBYCR
    PixelType_Gvsp_YCBCR422_8               =   (MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(16) | 0x003B),                  ///< YCBCR422_8
    PixelType_Gvsp_YCBCR422_8_CBYCRY        =   (MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(16) | 0x0043),                  ///< YCBCR422_8_CBYCRY
    PixelType_Gvsp_YCBCR411_8_CBYYCRYY      =   (MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(12) | 0x003C),                  ///< YCBCR411_8_CBYYCRYY
    PixelType_Gvsp_YCBCR601_8_CBYCR         =   (MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(24) | 0x003D),                  ///< YCBCR601_8_CBYCR
    PixelType_Gvsp_YCBCR601_422_8           =   (MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(16) | 0x003E),                  ///< YCBCR601_422_8
    PixelType_Gvsp_YCBCR601_422_8_CBYCRY    =   (MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(16) | 0x0044),                  ///< YCBCR601_422_8_CBYCRY
    PixelType_Gvsp_YCBCR601_411_8_CBYYCRYY  =   (MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(12) | 0x003F),                  ///< YCBCR601_411_8_CBYYCRYY
    PixelType_Gvsp_YCBCR709_8_CBYCR         =   (MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(24) | 0x0040),                  ///< YCBCR709_8_CBYCR
    PixelType_Gvsp_YCBCR709_422_8           =   (MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(16) | 0x0041),                  ///< YCBCR709_422_8
    PixelType_Gvsp_YCBCR709_422_8_CBYCRY    =   (MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(16) | 0x0045),                  ///< YCBCR709_422_8_CBYCRY
    PixelType_Gvsp_YCBCR709_411_8_CBYYCRYY  =   (MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(12) | 0x0042),                  ///< YCBCR709_411_8_CBYYCRYY

    // YUV420
    PixelType_Gvsp_YUV420SP_NV12            =   (MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(12) | 0x8001),                  ///< YUV420SP_NV12
    PixelType_Gvsp_YUV420SP_NV21            =   (MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(12) | 0x8002),                  ///< YUV420SP_NV21

    // RGB Planar buffer format defines 
    PixelType_Gvsp_RGB8_Planar              =   (MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(24) | 0x0021),                  ///< RGB8_Planar
    PixelType_Gvsp_RGB10_Planar             =   (MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(48) | 0x0022),                  ///< RGB10_Planar
    PixelType_Gvsp_RGB12_Planar             =   (MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(48) | 0x0023),                  ///< RGB12_Planar
    PixelType_Gvsp_RGB16_Planar             =   (MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(48) | 0x0024),                  ///< RGB16_Planar

    // 自定义的图片格式
    PixelType_Gvsp_Jpeg                     =   (MV_GVSP_PIX_CUSTOM | MV_PIXEL_BIT_COUNT(24) | 0x0001),                  ///< Jpeg

    PixelType_Gvsp_Coord3D_ABC32f           =   (MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(96) | 0x00C0),                  ///< 0x026000C0X
    PixelType_Gvsp_Coord3D_ABC32f_Planar    =   (MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(96) | 0x00C1),                  ///< 0x026000C1X

    PixelType_Gvsp_Coord3D_AC32f            =   (MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(40) | 0x00C2),      ///< 该值被废弃，请参考PixelType_Gvsp_Coord3D_AC32f_64; the value is discarded
    PixelType_Gvsp_COORD3D_DEPTH_PLUS_MASK  =   (MV_GVSP_PIX_CUSTOM | MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(28) | 0x0001),     ///< 该值被废弃; the value is discarded    (已放入Chunkdata)

    PixelType_Gvsp_Coord3D_ABC32            =   (MV_GVSP_PIX_CUSTOM | MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(96) | 0x3001),          ///< Coord3D_ABC32
    PixelType_Gvsp_Coord3D_AB32f            =   (MV_GVSP_PIX_CUSTOM | MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(64) | 0x3002),          ///< Coord3D_AB32f
    PixelType_Gvsp_Coord3D_AB32             =   (MV_GVSP_PIX_CUSTOM | MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(64) | 0x3003),          ///< Coord3D_AB32
    PixelType_Gvsp_Coord3D_AC32f_64         =   (MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(64) | 0x00C2),                               ///< Coord3D_AC32f_64
    PixelType_Gvsp_Coord3D_AC32f_Planar     =   (MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(64) | 0x00C3),                               ///< Coord3D_AC32f_Planar
    PixelType_Gvsp_Coord3D_AC32             =   (MV_GVSP_PIX_CUSTOM | MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(64) | 0x3004),          ///< Coord3D_AC32
    PixelType_Gvsp_Coord3D_A32f             =   (MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(32) | 0x00BD),                                ///< Coord3D_A32f
    PixelType_Gvsp_Coord3D_A32              =   (MV_GVSP_PIX_CUSTOM | MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(32) | 0x3005),           ///< Coord3D_A32
    PixelType_Gvsp_Coord3D_C32f             =   (MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(32) | 0x00BF),                                ///< Coord3D_C32f
    PixelType_Gvsp_Coord3D_C32              =   (MV_GVSP_PIX_CUSTOM | MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(32) | 0x3006),           ///< Coord3D_C32
    PixelType_Gvsp_Coord3D_ABC16            =   (MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(48) | 0x00B9),                               ///< Coord3D_ABC16
    PixelType_Gvsp_Coord3D_C16              =   (MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(16) | 0x00B8),                                ///< Coord3D_C16

    PixelType_Gvsp_Float32                  =   (MV_GVSP_PIX_CUSTOM | MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(32) | 0x0001),  //0x81200001

    //无损压缩像素格式定义
    PixelType_Gvsp_HB_Mono8                    =   (MV_GVSP_PIX_CUSTOM | MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(8) | 0x0001),         ///< HB_Mono8
    PixelType_Gvsp_HB_Mono10                   =   (MV_GVSP_PIX_CUSTOM | MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(16) | 0x0003),        ///< HB_Mono10
    PixelType_Gvsp_HB_Mono10_Packed            =   (MV_GVSP_PIX_CUSTOM | MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(12) | 0x0004),        ///< HB_Mono10_Packed
    PixelType_Gvsp_HB_Mono12                   =   (MV_GVSP_PIX_CUSTOM | MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(16) | 0x0005),        ///< HB_Mono12
    PixelType_Gvsp_HB_Mono12_Packed            =   (MV_GVSP_PIX_CUSTOM | MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(12) | 0x0006),        ///< HB_Mono12_Packed
    PixelType_Gvsp_HB_Mono16                   =   (MV_GVSP_PIX_CUSTOM | MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(16) | 0x0007),        ///< HB_Mono16
    PixelType_Gvsp_HB_BayerGR8                 =   (MV_GVSP_PIX_CUSTOM | MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(8) | 0x0008),         ///< HB_BayerGR8
    PixelType_Gvsp_HB_BayerRG8                 =   (MV_GVSP_PIX_CUSTOM | MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(8) | 0x0009),         ///< HB_BayerRG8
    PixelType_Gvsp_HB_BayerGB8                 =   (MV_GVSP_PIX_CUSTOM | MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(8) | 0x000A),         ///< HB_BayerGB8
    PixelType_Gvsp_HB_BayerBG8                 =   (MV_GVSP_PIX_CUSTOM | MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(8) | 0x000B),         ///< HB_BayerBG8
    PixelType_Gvsp_HB_BayerRBGG8               =   (MV_GVSP_PIX_CUSTOM | MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(8) | 0x0046),         ///< HB_BayerRBGG8
    PixelType_Gvsp_HB_BayerBRGG8               =   (MV_GVSP_PIX_CUSTOM | MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(8) | 0x0047),         ///< HB_BayerBRGG8
    PixelType_Gvsp_HB_BayerGR10                =   (MV_GVSP_PIX_CUSTOM | MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(16) | 0x000C),        ///< HB_BayerGR10
    PixelType_Gvsp_HB_BayerRG10                =   (MV_GVSP_PIX_CUSTOM | MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(16) | 0x000D),        ///< HB_BayerRG10
    PixelType_Gvsp_HB_BayerGB10                =   (MV_GVSP_PIX_CUSTOM | MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(16) | 0x000E),        ///< HB_BayerGB10
    PixelType_Gvsp_HB_BayerBG10                =   (MV_GVSP_PIX_CUSTOM | MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(16) | 0x000F),        ///< HB_BayerBG10
    PixelType_Gvsp_HB_BayerGR12                =   (MV_GVSP_PIX_CUSTOM | MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(16) | 0x0010),        ///< HB_BayerGR12
    PixelType_Gvsp_HB_BayerRG12                =   (MV_GVSP_PIX_CUSTOM | MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(16) | 0x0011),        ///< HB_BayerRG12
    PixelType_Gvsp_HB_BayerGB12                =   (MV_GVSP_PIX_CUSTOM | MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(16) | 0x0012),        ///< HB_BayerGB12
    PixelType_Gvsp_HB_BayerBG12                =   (MV_GVSP_PIX_CUSTOM | MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(16) | 0x0013),        ///< HB_BayerBG12
    PixelType_Gvsp_HB_BayerGR10_Packed         =   (MV_GVSP_PIX_CUSTOM | MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(12) | 0x0026),        ///< HB_BayerGR10_Packed
    PixelType_Gvsp_HB_BayerRG10_Packed         =   (MV_GVSP_PIX_CUSTOM | MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(12) | 0x0027),        ///< HB_BayerRG10_Packed
    PixelType_Gvsp_HB_BayerGB10_Packed         =   (MV_GVSP_PIX_CUSTOM | MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(12) | 0x0028),        ///< HB_BayerGB10_Packed
    PixelType_Gvsp_HB_BayerBG10_Packed         =   (MV_GVSP_PIX_CUSTOM | MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(12) | 0x0029),        ///< HB_BayerBG10_Packed
    PixelType_Gvsp_HB_BayerGR12_Packed         =   (MV_GVSP_PIX_CUSTOM | MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(12) | 0x002A),        ///< HB_BayerGR12_Packed
    PixelType_Gvsp_HB_BayerRG12_Packed         =   (MV_GVSP_PIX_CUSTOM | MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(12) | 0x002B),        ///< HB_BayerRG12_Packed
    PixelType_Gvsp_HB_BayerGB12_Packed         =   (MV_GVSP_PIX_CUSTOM | MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(12) | 0x002C),        ///< HB_BayerGB12_Packed
    PixelType_Gvsp_HB_BayerBG12_Packed         =   (MV_GVSP_PIX_CUSTOM | MV_GVSP_PIX_MONO | MV_PIXEL_BIT_COUNT(12) | 0x002D),        ///< HB_BayerBG12_Packed
    PixelType_Gvsp_HB_YUV422_Packed            =   (MV_GVSP_PIX_CUSTOM | MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(16) | 0x001F),       ///< HB_YUV422_Packed
    PixelType_Gvsp_HB_YUV422_YUYV_Packed       =   (MV_GVSP_PIX_CUSTOM | MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(16) | 0x0032),       ///< HB_YUV422_YUYV_Packed
    PixelType_Gvsp_HB_RGB8_Packed              =   (MV_GVSP_PIX_CUSTOM | MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(24) | 0x0014),       ///< HB_RGB8_Packed
    PixelType_Gvsp_HB_BGR8_Packed              =   (MV_GVSP_PIX_CUSTOM | MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(24) | 0x0015),       ///< HB_BGR8_Packed
    PixelType_Gvsp_HB_RGBA8_Packed             =   (MV_GVSP_PIX_CUSTOM | MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(32) | 0x0016),       ///< HB_RGBA8_Packed
    PixelType_Gvsp_HB_BGRA8_Packed             =   (MV_GVSP_PIX_CUSTOM | MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(32) | 0x0017),       ///< HB_BGRA8_Packed
    PixelType_Gvsp_HB_RGB16_Packed             =   (MV_GVSP_PIX_CUSTOM | MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(48) | 0x0033),       ///< HB_RGB16_Packed
    PixelType_Gvsp_HB_BGR16_Packed             =   (MV_GVSP_PIX_CUSTOM | MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(48) | 0x004B),       ///< HB_BGR16_Packed
    PixelType_Gvsp_HB_RGBA16_Packed            =   (MV_GVSP_PIX_CUSTOM | MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(64) | 0x0064),       ///< HB_RGBA16_Packed
    PixelType_Gvsp_HB_BGRA16_Packed            =   (MV_GVSP_PIX_CUSTOM | MV_GVSP_PIX_COLOR | MV_PIXEL_BIT_COUNT(64) | 0x0051),       ///< HB_BGRA16_Packed

};
///@}

#ifdef WIN32
typedef __int64 int64_t;
typedef unsigned __int64 uint64_t;
#else
#include <stdint.h>
#endif

#endif /* _MV_PIXEL_TYPE_H_ */
