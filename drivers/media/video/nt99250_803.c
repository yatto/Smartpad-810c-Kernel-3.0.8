/*
o* Driver for MT9M001 CMOS Image Sensor from Micron
 *
 * Copyright (C) 2008, Guennadi Liakhovetski <kernel@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/videodev2.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/log2.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/circ_buf.h>
#include <linux/miscdevice.h>
#include <media/v4l2-common.h>
#include <media/v4l2-chip-ident.h>
#include <media/soc_camera.h>
#include <mach/rk29_camera.h>
#include <mach/board.h>
#include <mach/gpio.h>

static int debug=1;
module_param(debug, int, S_IRUGO|S_IWUSR);

#define dprintk(level, fmt, arg...) do {			\
	if (debug >= level) 					\
	printk(KERN_WARNING fmt , ## arg); } while (0)

#define SENSOR_TR(format, ...) printk(KERN_ERR format, ## __VA_ARGS__)
#define SENSOR_DG(format, ...) dprintk(0, format, ## __VA_ARGS__)


#define _CONS(a,b) a##b
#define CONS(a,b) _CONS(a,b)

#define __STR(x) #x
#define _STR(x) __STR(x)
#define STR(x) _STR(x)

#define MIN(x,y)   ((x<y) ? x: y)
#define MAX(x,y)    ((x>y) ? x: y)

/* Sensor Driver Configuration */
#define SENSOR_NAME nt99250
#define SENSOR_V4L2_IDENT V4L2_IDENT_NT99250
#define SENSOR_ID 0x0105
#define SENSOR_MIN_WIDTH    176
#define SENSOR_MIN_HEIGHT   144
#define SENSOR_MAX_WIDTH    1600
#define SENSOR_MAX_HEIGHT   1200
#define SENSOR_INIT_WIDTH	800			/* Sensor pixel size for sensor_init_data array */
#define SENSOR_INIT_HEIGHT  600
#define SENSOR_INIT_WINSEQADR sensor_svga
#define SENSOR_INIT_PIXFMT V4L2_MBUS_FMT_YUYV8_2X8

#define CONFIG_SENSOR_WhiteBalance	1
#define CONFIG_SENSOR_Brightness	0
#define CONFIG_SENSOR_Contrast      0
#define CONFIG_SENSOR_Saturation    0
#define CONFIG_SENSOR_Effect        1
#define CONFIG_SENSOR_Scene         1
#define CONFIG_SENSOR_DigitalZoom   0
#define CONFIG_SENSOR_Focus         0
#define CONFIG_SENSOR_Exposure      0
#define CONFIG_SENSOR_Flash         1
#define CONFIG_SENSOR_Mirror        0
#define CONFIG_SENSOR_Flip          0

#define CONFIG_SENSOR_I2C_SPEED     350000       /* Hz */
/* Sensor write register continues by preempt_disable/preempt_enable for current process not be scheduled */
#define CONFIG_SENSOR_I2C_NOSCHED   0
#define CONFIG_SENSOR_I2C_RDWRCHK   0

//#define SENSOR_BUS_PARAM  (SOCAM_MASTER | SOCAM_PCLK_SAMPLE_RISING|\  
//                          SOCAM_HSYNC_ACTIVE_HIGH | SOCAM_VSYNC_ACTIVE_LOW |\
//                          SOCAM_DATA_ACTIVE_HIGH | SOCAM_DATAWIDTH_8  |SOCAM_MCLK_24MHZ)

#define SENSOR_BUS_PARAM  (SOCAM_MASTER | SOCAM_PCLK_SAMPLE_FALLING|\  
                          SOCAM_HSYNC_ACTIVE_HIGH | SOCAM_VSYNC_ACTIVE_LOW |\
                          SOCAM_DATA_ACTIVE_HIGH | SOCAM_DATAWIDTH_8  |SOCAM_MCLK_24MHZ)

#define COLOR_TEMPERATURE_CLOUDY_DN  6500
#define COLOR_TEMPERATURE_CLOUDY_UP    8000
#define COLOR_TEMPERATURE_CLEARDAY_DN  5000
#define COLOR_TEMPERATURE_CLEARDAY_UP    6500
#define COLOR_TEMPERATURE_OFFICE_DN     3500
#define COLOR_TEMPERATURE_OFFICE_UP     5000
#define COLOR_TEMPERATURE_HOME_DN       2500
#define COLOR_TEMPERATURE_HOME_UP       3500

#define SENSOR_NAME_STRING(a) STR(CONS(SENSOR_NAME, a))
#define SENSOR_NAME_VARFUN(a) CONS(SENSOR_NAME, a)

#define SENSOR_AF_IS_ERR    (0x00<<0)
#define SENSOR_AF_IS_OK		(0x01<<0)
#define SENSOR_INIT_IS_ERR   (0x00<<28)
#define SENSOR_INIT_IS_OK    (0x01<<28)

static struct i2c_client * g_i2c_client;

struct reginfo
{
    u16 reg;
    u8 val;
};

#define MASK_NT99250_ORIENT	(0x01<<1 | 0x01<<0)
#ifndef NT99250_MIRROR
#define NT99250_MIRROR	1
#endif
#ifndef NT99250_FLIP
#define NT99250_FLIP	0
#endif

#ifndef VAL_306D
#define VAL_306D	0x01
#endif

/* init 800*600 SVGA */
static struct reginfo sensor_init_data[] =
{	{0x3021, 0x01},
	{0x3024, 0x0a},//02,00,08,0a,0b
	{0x32F0, 0x03},
	{0x301e, 0x54},
	{0x301f, 0x48},
	{0x3270, 0x00},
	{0x3271, 0x04},
	{0x3272, 0x0E},
	{0x3273, 0x28},
	{0x3274, 0x3F},
	{0x3275, 0x50},
	{0x3276, 0x6E},
	{0x3277, 0x88},
	{0x3278, 0xA0},
	{0x3279, 0xB3},
	{0x327A, 0xD2},
	{0x327B, 0xE8},
	{0x327C, 0xF5},
	{0x327D, 0xFF},
	{0x327E, 0xFF},
	{0x3302, 0x00},
	{0x3303, 0x2E},
	{0x3304, 0x00},
	{0x3305, 0xB7},
	{0x3306, 0x00},
	{0x3307, 0x1A},
	{0x3308, 0x07},
	{0x3309, 0xE7},
	{0x330A, 0x07},
	{0x330B, 0x44},
	{0x330C, 0x00},
	{0x330D, 0xD6},
	{0x330E, 0x01},
	{0x330F, 0x01},
	{0x3310, 0x07},
	{0x3311, 0x1A},
	{0x3312, 0x07},
	{0x3313, 0xE5},
	{0x3250, 0x01}, 	
	{0x3251, 0x87}, 
	{0x3252, 0x01}, 
	{0x3253, 0x88}, 
	{0x3254, 0x01}, 
	{0x3255, 0x89}, 
	{0x3256, 0x01}, 
	{0x3257, 0x33}, 
	{0x3258, 0x01}, 
	{0x3259, 0x34}, 
	{0x325A, 0x01}, 
	{0x325B, 0x35}, 
	{0x325C, 0x00}, 
	{0x325D, 0x00}, 
	{0x325E, 0x00}, 
	{0x325F, 0x00}, 
	{0x3260, 0x00}, 
	{0x3261, 0x00}, 
	{0x3262, 0x09}, 
	{0x3263, 0x08}, 
	{0x3264, 0x08}, 
	{0x3265, 0x17}, 
	{0x3266, 0x00}, 	
	
	{0x3102, 0x0b},
	{0x3103, 0x46},
	{0x3105, 0x33},
	{0x3107, 0x32},
	{0x310A, 0x03},
	{0x310B, 0x18},
	{0x310f, 0x08},
	{0x3110, 0x03},
	{0x3113, 0x0F},
	{0x3119, 0x17},
	{0x3114, 0x03},
	{0x3117, 0x03},
	{0x3118, 0x01},
	{0x3380, 0x03},
	{0x3044, 0x02},
	{0x3045, 0xd0},
	{0x3046, 0x02},
	{0x3047, 0xd0},
	{0x3048, 0x02},
	{0x3049, 0xd0},
	{0x304a, 0x02},
	{0x304b, 0xd0},
	{0x303e, 0x02},
	{0x303f, 0x2b},
	{0x3052, 0x80},
	{0x3059, 0x10},
	{0x305a, 0x28},
	{0x305b, 0x20},
	{0x305c, 0x04},
	{0x305d, 0x28},
	{0x305e, 0x04},
	{0x305f, 0x52},
	{0x3058, 0x01},
	{0x3080, 0x80},
	{0x3081, 0x80},
	{0x3082, 0x80},
	{0x3083, 0x40},
	{0x3084, 0x80},
	{0x3085, 0x40},
	{0x32b0, 0x00},
	{0x32b1, 0x90},
	{0x32BB, 0x0b},
	{0x32bd, 0x05},
	{0x32be, 0x05},
	{0x32cd, 0x01},
	{0x32d3, 0x13},
	{0x32d7, 0x82},
	{0x32d8, 0x3F},
	{0x32d9, 0x18},
	{0x32c5, 0x1f},
	{0x32f6, 0x0c},
	{0x306d, VAL_306D}, 
	{0x3300, 0x30},
	{0x3301, 0x80},
	{0x3320, 0x28},
	{0x3331, 0x04},
	{0x3332, 0x40},
	{0x3339, 0x10},
	{0x333a, 0x1a},
	{0x329C, 0x4b},
	{0x32bf, 0x52},
	{0x32c0, 0x10},
	{0x3200, 0x3e},
	{0x3201, 0x3f},
	{0x32b0, 0x00},//0x02
	{0x32b1, 0xc0},
	
	{0x32e0, 0x03}, 
	{0x32e1, 0x22}, 
	{0x32e2, 0x02}, 
	{0x32e3, 0x58}, 
	{0x32e4, 0x00}, 
	{0x32e5, 0xff}, 
	{0x32e6, 0x00}, 
	{0x32e7, 0x00}, 
	{0x301e, 0x00}, 
	{0x301f, 0x02},//0x20}, 
	{0x3022, (0x24 & ~(MASK_NT99250_ORIENT)) | (NT99250_MIRROR<<1) | (NT99250_FLIP<<0)}, 
	{0x3023, 0x64}, 
	{0x3002, 0x00}, 
	{0x3003, 0x02}, 
	{0x3004, 0x00}, 
	{0x3005, 0x04}, 
	{0x3006, 0x06}, 
	{0x3007, 0x43}, 
	{0x3008, 0x04}, 
	{0x3009, 0xb3}, 
	{0x300a, 0x09}, 
	{0x300b, 0x91},
	{0x300c, 0x02}, 
	{0x300d, 0x91}, 
	{0x300e, 0x06}, 
	{0x300f, 0x42}, 
	{0x3010, 0x02}, 
	{0x3011, 0x58}, 
	
	{0x32bb, 0x0b}, 
	{0x32bc, 0x3a},//brightness
	{0x32c1, 0x23},//0x24}, 
	{0x32c2, 0x28},//0x36},
	{0x32c8, 0x49},//0x62}, 
	{0x32c9, 0x3d},//0x52}, 
	{0x32c4, 0x00}, 
	
	{0x3290, 0x01},
	{0x3291, 0x68},
	{0x3296, 0x01},
	{0x3297, 0x75}, 
	{0x32A9, 0x11},
	{0x32AA, 0x01}, 
	{0x329b, 0x01},
	{0x32a2, 0x60},
	{0x32a4, 0xa0},
	{0x32a6, 0x60},
	{0x32a8, 0xa0},
	
	{0x3012, 0x02},
	{0x3013, 0xae},
	{0x301d, 0x08}, 
	{0x3201, 0x7f},
	{0x3021, 0x06},
	{0x3060, 0x01},
	{0x0000, 0x00},
};

static struct reginfo sensor_720p[]=
{
	{0x3300, 0x3f},
	{0x3301, 0xa0},
	{0x3331, 0x08},
	{0x3332, 0x80},
	{0x3320, 0x20},
	{0x32e0, 0x05},
	{0x32e1, 0x02},
	{0x32e2, 0x02},
	{0x32e3, 0xd0},
	{0x32e4, 0x00},
	{0x32e5, 0x40},
	{0x32e6, 0x00},
	{0x32e7, 0x00},
	//{0x301e, 0x00}, 
	//{0x301f, 0x20}, 
	//{0x3022, 0x24},
	{0x3023, 0x24},
	{0x3002, 0x00}, 
	{0x3003, 0x02}, 
	{0x3004, 0x00}, 
	{0x3005, 0xf4}, 
	{0x3006, 0x06}, 
	{0x3007, 0x43}, 
	{0x3008, 0x03}, 
	{0x3009, 0xc3}, 
	{0x300a, 0x09}, 
	{0x300b, 0x91}, 
	{0x300c, 0x02}, 
	{0x300d, 0xd9}, 
	{0x300e, 0x06}, 
	{0x300f, 0x42}, 
	{0x3010, 0x02}, 
	{0x3011, 0xd0},
	{0x32bb, 0x0b}, 
	{0x32c4, 0x00}, 
	{0x3021, 0x06}, 
	{0x3060, 0x01}, 
	{0x0, 0x0},   //end flag
};

/* 1600X1200 UXGA */
static struct reginfo sensor_uxga[] =
{
	{0x3300, 0x3f},
	{0x3301, 0xa0},
	{0x3331, 0x08},
	{0x3332, 0x80},
	{0x3320, 0x20},
	
	{0x32e0, 0x06},
	{0x32e1, 0x42},
	{0x32e2, 0x04},
	{0x32e3, 0xb0},
	{0x32e4, 0x00},
	{0x32e5, 0x00},
	{0x32e6, 0x00},
	{0x32e7, 0x00},
	//{0x301e, 0x00}, 
	//{0x301f, 0x20}, 
	//{0x3022, 0x24},
	{0x3023, 0x24},
	{0x3002, 0x00}, 
	{0x3003, 0x02}, 
	{0x3004, 0x00}, 
	{0x3005, 0x04}, 
	{0x3006, 0x06}, 
	{0x3007, 0x43}, 
	{0x3008, 0x00}, 
	{0x3009, 0xb3}, 
	{0x300a, 0x09}, 
	{0x300b, 0x91}, 
	{0x300c, 0x04}, 
	{0x300d, 0xb9}, 
	{0x300e, 0x06}, 
	{0x300f, 0x42}, 
	{0x3010, 0x04}, 
	{0x3011, 0xb0},
	{0x32bb, 0x0b}, 
	{0x32c4, 0x00}, 

	{0x306d, VAL_306D}, 

	{0x3021, 0x06}, 
	{0x3060, 0x01}, 

	{0x0, 0x0}, 
};

/* 1280X1024 SXGA */
static struct reginfo sensor_sxga[] =
{
	{0x3300, 0x3f},
	{0x3301, 0xa0},
	{0x3331, 0x0c},
	{0x3332, 0x80},
	{0x3320, 0x20},	
	
	{0x32e0, 0x05}, 
	{0x32e1, 0x02}, 
	{0x32e2, 0x04}, 
	{0x32e3, 0x00}, 
	{0x32e4, 0x00}, 
	{0x32e5, 0x40}, 
	{0x32e6, 0x00}, 
	{0x32e7, 0x2c}, 
	//{0x301e, 0x00}, 
	//{0x3022, 0x24}, 
	{0x3023, 0x24}, 
	{0x3002, 0x00}, 
	{0x3003, 0x02}, 
	{0x3004, 0x00}, 
	{0x3005, 0x04}, 
	{0x3006, 0x06}, 
	{0x3007, 0x43}, 
	{0x3008, 0x04}, 
	{0x3009, 0xb3}, 
	{0x300a, 0x09}, 
	{0x300b, 0x91}, 
	{0x300c, 0x04}, 
	{0x300d, 0xb9}, 
	{0x300e, 0x06}, 
	{0x300f, 0x42}, 
	{0x3010, 0x04}, 
	{0x3011, 0xb0}, 
	{0x32bb, 0x0b}, 

	{0x306d, VAL_306D}, 

	{0x3021, 0x06}, 
	{0x3060, 0x01}, 

	{0x0, 0x0}, 
};
static struct reginfo sensor_xga[] =
{
	{0x0, 0x0}
};
/* 800X600 SVGA*/
static struct reginfo sensor_svga[] =
{
	{0x3300, 0x30},
	{0x3301, 0x80},
	{0x3320, 0x28},
	{0x3331, 0x04},
	{0x3332, 0x40},

	{0x32e0, 0x03}, 
	{0x32e1, 0x22}, 
	{0x32e2, 0x02}, 
	{0x32e3, 0x58}, 
	{0x32e4, 0x00}, 
	{0x32e5, 0xff}, 
	{0x32e6, 0x00}, 
	{0x32e7, 0x00}, 
	{0x301e, 0x00}, 
	{0x301f, 0x02},//0x20}, 
	//{0x3022, 0x24}, 
	{0x3023, 0x64}, 
	{0x3002, 0x00}, 
	{0x3003, 0x02}, 
	{0x3004, 0x00}, 
	{0x3005, 0x04}, 
	{0x3006, 0x06}, 
	{0x3007, 0x43}, 
	{0x3008, 0x04}, 
	{0x3009, 0xb3}, 
	{0x300a, 0x09}, 
	{0x300b, 0x91},
	{0x300c, 0x02}, 
	{0x300d, 0x91}, 
	{0x300e, 0x06}, 
	{0x300f, 0x42}, 
	{0x3010, 0x02}, 
	{0x3011, 0x58}, 
	
	{0x306d, VAL_306D}, 

	{0x3021, 0x06},
	{0x3060, 0x01},
	
	{0x0, 0x0}, 
};

/* 640X480 VGA */
static struct reginfo sensor_vga[] =
{
	{0x3300, 0x30},
	{0x3301, 0x80},
	{0x3320, 0x28},
	{0x3331, 0x04},
	{0x3332, 0x40},
	
	{0x32e0, 0x02}, 
	{0x32e1, 0x82}, 
	{0x32e2, 0x01}, 
	{0x32e3, 0xe0}, 
	{0x32e4, 0x01}, 
	{0x32e5, 0x7f}, 
	{0x32e6, 0x00}, 
	{0x32e7, 0x40}, 
	//{0x301e, 0x00}, 
	//{0x301f, 0x20}, 
	//{0x3022, 0x24}, 
	{0x3023, 0x64}, 
	{0x3002, 0x00}, 
	{0x3003, 0x02}, 
	{0x3004, 0x00}, 
	{0x3005, 0x04}, 
	{0x3006, 0x06}, 
	{0x3007, 0x43}, 
	{0x3008, 0x04}, 
	{0x3009, 0xb3}, 
	{0x300a, 0x09}, 
	{0x300b, 0x91}, 
	{0x300c, 0x02}, 
	{0x300d, 0x91}, 
	{0x300e, 0x06}, 
	{0x300f, 0x42}, 
	{0x3010, 0x02}, 
	{0x3011, 0x58}, 
	{0x32bb, 0x0b}, 
	{0x32c4, 0x00}, 

	{0x306d, VAL_306D}, 

	{0x3021, 0x06},
	{0x3060, 0x01},
	{0x0000, 0x00}, 
};

#define CIF_10FPS 
/* 352X288 CIF */
static struct reginfo sensor_cif[] =
{
#ifdef CIF_10FPS
      {0x3300, 0x30},	//edge & denoise
      {0x3301, 0x80},	//edge
      {0x3320, 0x40},	//denoise
      {0x3331, 0x0c},	//pre-edge thr
      {0x3332, 0x40},	//pre-edge
      {0x329C, 0x4b},	//awb speed
      {0x32bf, 0x52},
      {0x32e0, 0x01}, 
      {0x32e1, 0x62}, 
      {0x32e2, 0x01}, 
      {0x32e3, 0x20}, 
      {0x32e4, 0x03}, 
      {0x32e5, 0x89}, 
      {0x32e6, 0x01}, 
      {0x32e7, 0x17}, 
      {0x301e, 0x00}, 
      {0x301f, 0x02}, 
      //{0x3022, 0x24}, 
      {0x3023, 0x64}, 
      {0x3002, 0x00}, 
      {0x3003, 0x04}, 
      {0x3004, 0x00}, 
      {0x3005, 0x04}, 
      {0x3006, 0x06}, 
      {0x3007, 0x43}, 
      {0x3008, 0x04}, 
      {0x3009, 0xb3}, 
      {0x300a, 0x09}, 
      {0x300b, 0x91}, 
      {0x300c, 0x02}, 
      {0x300d, 0xdf}, 
      {0x300e, 0x06}, 
      {0x300f, 0x42}, 
      {0x3010, 0x02}, 
      {0x3011, 0x58}, 
      {0x32bb, 0x0b}, 
      {0x32bc, 0x3a}, 
      {0x32c1, 0x22}, 
      {0x32c2, 0xdf}, 
      {0x32c8, 0x49}, 
      {0x32c9, 0x3d}, 
      {0x32c4, 0x00}, 
      {0x3201, 0x7f}, 
      {0x3021, 0x06}, 
      {0x3060, 0x01}, 

	{0x0, 0x0}, 
#elif defined CIF_15FPS
      {0x3300, 0x30},	//edge & denoise
      {0x3301, 0x80},	//edge
      {0x3320, 0x40},	//denoise
      {0x3331, 0x0c},	//pre-edge thr
      {0x3332, 0x40},	//pre-edge
      {0x329C, 0x4b},	//awb speed
      {0x32bf,0x52},
      {0x32e0, 0x01}, 
      {0x32e1, 0x62}, 
      {0x32e2, 0x01}, 
      {0x32e3, 0x20}, 
      {0x32e4, 0x01}, 
      {0x32e5, 0x44}, 
      {0x32e6, 0x01}, 
      {0x32e7, 0x17}, 
      {0x301e, 0x00}, 
      {0x301f, 0x02}, 
      //{0x3022, 0x24}, 
      {0x3023, 0x6c}, 
      {0x3002, 0x00}, 
      {0x3003, 0x04}, 
      {0x3004, 0x00}, 
      {0x3005, 0x04}, 
      {0x3006, 0x06}, 
      {0x3007, 0x43}, 
      {0x3008, 0x04}, 
      {0x3009, 0xb3}, 
      {0x300a, 0x06}, 
      {0x300b, 0x71}, 
      {0x300c, 0x02}, 
      {0x300d, 0xd7}, 
      {0x300e, 0x03}, 
      {0x300f, 0x20}, 
      {0x3010, 0x02}, 
      {0x3011, 0x58}, 
      {0x32bb, 0x0b}, 
      {0x32bc, 0x3a}, 
      {0x32c1, 0x22}, 
      {0x32c2, 0x94}, 
      {0x32c8, 0x6d}, 
      {0x32c9, 0x5a}, 
      {0x32c4, 0x00}, 
      {0x3201, 0x7f}, 
      {0x3021, 0x06}, 
      {0x3060, 0x01}, 
	
	{0x0, 0x0}, 
#elif defined CIF_20FPS
      {0x3300, 0x30},	//edge & denoise
      {0x3301, 0x80},	//edge
      {0x3320, 0x40},	//denoise
      {0x3331, 0x0c},	//pre-edge thr
      {0x3332, 0x40},	//pre-edge
      {0x329C, 0x4b},	//awb speed
      {0x32bf,0x52},
      {0x32e0, 0x01}, 
      {0x32e1, 0x62}, 
      {0x32e2, 0x01}, 
      {0x32e3, 0x20}, 
      {0x32e4, 0x00}, 
      {0x32e5, 0xd0}, 
      {0x32e6, 0x00}, 
      {0x32e7, 0xac}, 
      {0x301e, 0x00}, 
      {0x301f, 0x02}, 
      //{0x3022, 0x24}, 
      {0x3023, 0x6c}, 
      {0x3002, 0x00}, 
      {0x3003, 0xa4}, 
      {0x3004, 0x00}, 
      {0x3005, 0x7c}, 
      {0x3006, 0x05}, 
      {0x3007, 0xa3}, 
      {0x3008, 0x04}, 
      {0x3009, 0x3b}, 
      {0x300a, 0x05}, 
      {0x300b, 0xd1}, 
      {0x300c, 0x02}, 
      {0x300d, 0x5c}, 
      {0x300e, 0x02}, 
      {0x300f, 0x80}, 
      {0x3010, 0x01}, 
      {0x3011, 0xe0}, 
      {0x32bb, 0x0b}, 
      {0x32bc, 0x4b}, 
      {0x32c1, 0x22}, 
      {0x32c2, 0x5c}, 
      {0x32c8, 0x78}, 
      {0x32c9, 0x64}, 
      {0x32c4, 0x00}, 
      {0x3201, 0x7f}, 
      {0x3021, 0x06}, 
      {0x3060, 0x01}, 


	{0x0, 0x0}, 
#elif defined CIF_25FPS
	{0x3300,0x30},	//edge & denoise
	{0x3301,0x80},	//edge
	{0x3320,0x28},	//denoise
	{0x3331,0x04},	//pre-edge thr
	{0x3332,0x40},	//pre-edge
	{0x329C,0x4b},	//awb speed
	{0x32bf,0x52},
	{0x32e0, 0x01}, 
	{0x32e1, 0x62}, 
	{0x32e2, 0x01}, 
	{0x32e3, 0x20}, 
	{0x32e4, 0x00}, 
	{0x32e5, 0xd0}, 
	{0x32e6, 0x00}, 
	{0x32e7, 0xac}, 
	{0x301e, 0x00}, 
	{0x301f, 0x20}, 
	//{0x3022, 0x24}, 
	{0x3023, 0x6c}, 
	{0x3002, 0x00}, 
	{0x3003, 0xa4}, 
	{0x3004, 0x00}, 
	{0x3005, 0x7c}, 
	{0x3006, 0x05}, 
	{0x3007, 0xa3}, 
	{0x3008, 0x04}, 
	{0x3009, 0x3b}, 
	{0x300a, 0x05}, 
	{0x300b, 0xd1}, 
	{0x300c, 0x02}, 
	{0x300d, 0x85}, 
	{0x300e, 0x02}, 
	{0x300f, 0x80}, 
	{0x3010, 0x01}, 
	{0x3011, 0xe0}, 
	{0x32bb, 0x0b}, 
	{0x32c1, 0x22}, 
	{0x32c2, 0x85}, 
	{0x32c8, 0xa1}, 
	{0x32c9, 0x86}, 
	{0x32c4, 0x00}, 
	{0x3201, 0x7f}, 
	{0x3021, 0x06}, 
	{0x3060, 0x01}, 

	{0x0, 0x0}, 
#else
	{0x3300,0x30},	//edge & denoise
	{0x3301,0x80},	//edge
	{0x3320,0x28},	//denoise
	{0x3331,0x04},	//pre-edge thr
	{0x3332,0x40},	//pre-edge
	{0x329C,0x4b},	//awb speed
	{0x329C,0x4b},
	{0x32bf,0x52},
	{0x32e0,0x01},
	{0x32e1,0x62},
	{0x32e2,0x01},
	{0x32e3,0x20},
	{0x32e4,0x03},
	{0x32e5,0x89},
	{0x32e6,0x01},
	{0x32e7,0x17},
	{0x301e,0x00},
	{0x301f,0x20},
	//{0x3022,0x24},
	{0x3023,0x64},
	{0x3002,0x00},
	{0x3003,0x04},
	{0x3004,0x00},
	{0x3005,0x04},
	{0x3006,0x06},
	{0x3007,0x43},
	{0x3008,0x04},
	{0x3009,0xb3},
	{0x300a,0x09},
	{0x300b,0x91},
	{0x300c,0x02},//0x03},//0x02},
	{0x300d,0xae},//0xd4},//0x8d},
	{0x300e,0x06},
	{0x300f,0x42},
	{0x3010,0x02},
	{0x3011,0x58},
	{0x32bb,0x0b},
	{0x3201,0x7f},
	{0x3021,0x06},
	{0x3060,0x01},
	{0x32c1,0x42},//0x43},
	{0x32c2,0xae},//0xd4},

	    {0x0, 0x0}, 
#endif
};

/* 320*240 QVGA */
static  struct reginfo sensor_qvga[] =
{
	    {0x0, 0x0}, 
};

#define QCIF_10FPS 

/* 176X144 QCIF*/
static struct reginfo sensor_qcif[] =
{
#ifdef QCIF_10FPS
      {0x3300, 0x30},	//edge & denoise
      {0x3301, 0x80},	//edge
      {0x3320, 0x28},	//denoise
      {0x3331, 0x04},	//pre-edge thr
      {0x3332, 0x40},	//pre-edge
      {0x329C, 0x4b},	//awb speed
      {0x32bf, 0x52},
      {0x32e0, 0x00}, 
      {0x32e1, 0xb2}, 
      {0x32e2, 0x00}, 
      {0x32e3, 0x90}, 
      {0x32e4, 0x01}, 
      {0x32e5, 0x42}, 
      {0x32e6, 0x01}, 
      {0x32e7, 0x19}, 
      {0x301e, 0x04}, 
      {0x301f, 0x02}, 
      //{0x3022, 0x24}, 
      {0x3023, 0xfc}, 
      {0x3002, 0x00}, 
      {0x3003, 0x02}, 
      {0x3004, 0x00}, 
      {0x3005, 0x04}, 
      {0x3006, 0x06}, 
      {0x3007, 0x43}, 
      {0x3008, 0x04}, 
      {0x3009, 0xb3}, 
      {0x300a, 0x03}, 
      {0x300b, 0xe8}, 
      {0x300c, 0x03}, 
      {0x300d, 0x84}, 
      {0x300e, 0x01}, 
      {0x300f, 0x90}, 
      {0x3010, 0x01}, 
      {0x3011, 0x2c}, 
      {0x32bb, 0x0b}, 
      {0x32bc, 0x3a}, 
      {0x32c1, 0x23}, 
      {0x32c2, 0x84}, 
      {0x32c8, 0x5a}, 
      {0x32c9, 0x4b}, 
      {0x32c4, 0x00}, 
      {0x3201, 0x7f}, 
      {0x3021, 0x06}, 
      {0x3060, 0x01}, 

	{0x0, 0x0}, 
#elif defined QCIF_15FPS
      {0x3300, 0x30},	//edge & denoise
      {0x3301, 0x80},	//edge
      {0x3320, 0x28},	//denoise
      {0x3331, 0x04},	//pre-edge thr
      {0x3332, 0x40},	//pre-edge
      {0x329C, 0x4b},	//awb speed
      {0x32bf, 0x52},
      {0x32e0, 0x00}, 
      {0x32e1, 0xb2}, 
      {0x32e2, 0x00}, 
      {0x32e3, 0x90}, 
      {0x32e4, 0x01}, 
      {0x32e5, 0x42}, 
      {0x32e6, 0x01}, 
      {0x32e7, 0x19}, 
      {0x301e, 0x00}, 
      {0x301f, 0x02}, 
      //{0x3022, 0x24}, 
      {0x3023, 0xfc}, 
      {0x3002, 0x00}, 
      {0x3003, 0x04}, 
      {0x3004, 0x00}, 
      {0x3005, 0x04}, 
      {0x3006, 0x06}, 
      {0x3007, 0x43}, 
      {0x3008, 0x04}, 
      {0x3009, 0xb3}, 
      {0x300a, 0x03}, 
      {0x300b, 0xe8}, 
      {0x300c, 0x04}, 
      {0x300d, 0xb0}, 
      {0x300e, 0x01}, 
      {0x300f, 0x90}, 
      {0x3010, 0x01}, 
      {0x3011, 0x2c}, 
      {0x32bb, 0x0b}, 
      {0x32bc, 0x3a}, 
      {0x32c1, 0x24}, 
      {0x32c2, 0x38}, 
      {0x32c8, 0xb4}, 
      {0x32c9, 0x96}, 
      {0x32c4, 0x00}, 
      {0x3201, 0x7f}, 
      {0x3021, 0x06}, 
      {0x3060, 0x01},
      	
	{0x0, 0x0}, 
#elif defined QCIF_20FPS
      {0x3300, 0x30},	//edge & denoise
      {0x3301, 0x80},	//edge
      {0x3320, 0x28},	//denoise
      {0x3331, 0x04},	//pre-edge thr
      {0x3332, 0x40},	//pre-edge
      {0x329C, 0x4b},	//awb speed
      {0x32bf, 0x52},
      {0x32e0, 0x00}, 
      {0x32e1, 0xb2}, 
      {0x32e2, 0x00}, 
      {0x32e3, 0x90}, 
      {0x32e4, 0x01}, 
      {0x32e5, 0x42}, 
      {0x32e6, 0x01}, 
      {0x32e7, 0x19}, 
      {0x301e, 0x00}, 
      {0x301f, 0x02}, 
      //{0x3022, 0x24}, 
      {0x3023, 0xfc}, 
      {0x3002, 0x00}, 
      {0x3003, 0x04}, 
      {0x3004, 0x00}, 
      {0x3005, 0x04}, 
      {0x3006, 0x06}, 
      {0x3007, 0x43}, 
      {0x3008, 0x04}, 
      {0x3009, 0xb3}, 
      {0x300a, 0x03}, 
      {0x300b, 0xe8}, 
      {0x300c, 0x03}, 
      {0x300d, 0x84}, 
      {0x300e, 0x01}, 
      {0x300f, 0x90}, 
      {0x3010, 0x01}, 
      {0x3011, 0x2c}, 
      {0x32bb, 0x0b}, 
      {0x32bc, 0x3a}, 
      {0x32c1, 0x23}, 
      {0x32c2, 0x84}, 
      {0x32c8, 0xb4}, 
      {0x32c9, 0x96}, 
      {0x32c4, 0x00}, 
      {0x3201, 0x7f}, 
      {0x3021, 0x06}, 
      {0x3060, 0x01}, 

	{0x0, 0x0}, 
#elif defined QCIF_25FPS
      {0x3300, 0x30},	//edge & denoise
      {0x3301, 0x80},	//edge
      {0x3320, 0x28},	//denoise
      {0x3331, 0x04},	//pre-edge thr
      {0x3332, 0x40},	//pre-edge
      {0x329C, 0x4b},	//awb speed
      {0x32bf, 0x52},
      {0x32e0, 0x00}, 
      {0x32e1, 0xb2}, 
      {0x32e2, 0x00}, 
      {0x32e3, 0x90}, 
      {0x32e4, 0x01}, 
      {0x32e5, 0x42}, 
      {0x32e6, 0x01}, 
      {0x32e7, 0x19}, 
      {0x301e, 0x00}, 
      {0x301f, 0x02}, 
      //{0x3022, 0x24}, 
      {0x3023, 0xfc}, 
      {0x3002, 0x00}, 
      {0x3003, 0x04}, 
      {0x3004, 0x00}, 
      {0x3005, 0x04}, 
      {0x3006, 0x06}, 
      {0x3007, 0x43}, 
      {0x3008, 0x04}, 
      {0x3009, 0xb3}, 
      {0x300a, 0x03}, 
      {0x300b, 0xe8}, 
      {0x300c, 0x02}, 
      {0x300d, 0xd0}, 
      {0x300e, 0x01}, 
      {0x300f, 0x90}, 
      {0x3010, 0x01}, 
      {0x3011, 0x2c}, 
      {0x32bb, 0x0b}, 
      {0x32bc, 0x3a}, 
      {0x32c1, 0x22}, 
      {0x32c2, 0xd0}, 
      {0x32c8, 0xb4}, 
      {0x32c9, 0x96}, 
      {0x32c4, 0x00}, 
      {0x3201, 0x7f}, 
      {0x3021, 0x06}, 
      {0x3060, 0x01}, 

	{0x0, 0x0}, 
#endif
};
/* 160X120 QQVGA*/
static struct reginfo nt99250_qqvga[] =
{
    {0x0, 0x0},
};



static  struct reginfo nt99250_Sharpness_auto[] =
{

};

static  struct reginfo nt99250_Sharpness1[] =
{

};

static  struct reginfo nt99250_Sharpness2[][3] =
{
    //Sharpness 2

};

static  struct reginfo nt99250_Sharpness3[] =
{
    //default

};
static  struct reginfo nt99250_Sharpness4[]=
{
    //Sharpness 4

};

static  struct reginfo nt99250_Sharpness5[] =
{
    //Sharpness 5

};


static  struct reginfo sensor_ClrFmt_YUYV[]=
{
    {0x0000, 0x00}
};

static  struct reginfo sensor_ClrFmt_UYVY[]=
{
    {0x0000, 0x00}
};

#if CONFIG_SENSOR_WhiteBalance
static  struct reginfo sensor_WhiteB_Auto[]=
{
	{0x3201, 0x7f},  //AWB auto, bit[1]:0,auto
    {0x0000, 0x00}
};
/* Cloudy Colour Temperature : 6500K - 8000K  */
static  struct reginfo sensor_WhiteB_Cloudy[]=
{
	{0x3201, 0x6f},
	{0x3290, 0x01},
	{0x3291, 0x51},
	{0x3296, 0x01},
	{0x3297, 0x00},
    {0x0000, 0x00}
};
/* ClearDay Colour Temperature : 5000K - 6500K  */
static  struct reginfo sensor_WhiteB_ClearDay[]=
{
	{0x3201, 0x6f},
	{0x3290, 0x01},
	{0x3291, 0x38},
	{0x3296, 0x01},
	{0x3297, 0x68},
    {0x0000, 0x00}
};
/* Office Colour Temperature : 3500K - 5000K  */
static  struct reginfo sensor_WhiteB_TungstenLamp1[]=
{
	{0x3201, 0x6f},
	{0x3290, 0x01},
	{0x3291, 0x30},
	{0x3296, 0x01},
	{0x3297, 0xcb},
    {0x0000, 0x00}

};
/* Home Colour Temperature : 2500K - 3500K  */
static  struct reginfo sensor_WhiteB_TungstenLamp2[]=
{
	{0x3201, 0x6f},
	{0x3290, 0x01},
	{0x3291, 0x7a},
	{0x3296, 0x01},
	{0x3297, 0x6e},
    {0x0000, 0x00}
};
static struct reginfo *sensor_WhiteBalanceSeqe[] = {sensor_WhiteB_Auto, sensor_WhiteB_TungstenLamp1,sensor_WhiteB_TungstenLamp2,
    sensor_WhiteB_ClearDay, sensor_WhiteB_Cloudy,NULL,
};
#endif

#if CONFIG_SENSOR_Brightness
static  struct reginfo sensor_Brightness0[]=
{
    {0x0000, 0x00}
};

static  struct reginfo sensor_Brightness1[]=
{
    {0x0000, 0x00}
};

static  struct reginfo sensor_Brightness2[]=
{
    {0x0000, 0x00}
};

static  struct reginfo sensor_Brightness3[]=
{
    {0x0000, 0x00}
};

static  struct reginfo sensor_Brightness4[]=
{
    {0x0000, 0x00}
};

static  struct reginfo sensor_Brightness5[]=
{
    {0x0000, 0x00}
};
static struct reginfo *sensor_BrightnessSeqe[] = {sensor_Brightness0, sensor_Brightness1, sensor_Brightness2, sensor_Brightness3,
    sensor_Brightness4, sensor_Brightness5,NULL,
};

#endif

#if CONFIG_SENSOR_Effect
static  struct reginfo sensor_Effect_Normal[] =
{
	{0x32f1, 0x10},
	{0x32f6, 0x0c},
    {0x0000, 0x00}
};

static  struct reginfo sensor_Effect_WandB[] =
{
	{0x32f1, 0x11},
	{0x32f6, 0x0c},
    {0x0000, 0x00}
};

static  struct reginfo sensor_Effect_Sepia[] =
{
	{0x32f1, 0x12},
	{0x32f6, 0x0c},
    {0x0000, 0x00}
};

static  struct reginfo sensor_Effect_Negative[] =
{
	{0x32f1, 0x13},
	{0x32f6, 0x0c},
    {0x0000, 0x00}
};
static  struct reginfo sensor_Effect_Bluish[] =
{
	{0x32f1, 0x15},
	{0x32f4, 0xf0},
	{0x32f5, 0x80},
	{0x32f6, 0x0c},
    {0x0000, 0x00}
};

static  struct reginfo sensor_Effect_Green[] =
{
	{0x32f1, 0x15},
	{0x32f4, 0x60},
	{0x32f5, 0x20},
	{0x32f6, 0x0c},
    {0x0000, 0x00}
};
static struct reginfo *sensor_EffectSeqe[] = {sensor_Effect_Normal, sensor_Effect_WandB, sensor_Effect_Negative,sensor_Effect_Sepia,
    sensor_Effect_Bluish, sensor_Effect_Green,NULL,
};
#endif
#if CONFIG_SENSOR_Exposure
static  struct reginfo sensor_Exposure0[]=
{
	{0x32f2, 0x38},
	{0x0000, 0x00}
};

static  struct reginfo sensor_Exposure1[]=
{
	{0x32f2, 0x50},
    {0x0000, 0x00}
};

static  struct reginfo sensor_Exposure2[]=
{
	{0x32f2, 0x68},
    {0x0000, 0x00}
};

static  struct reginfo sensor_Exposure3[]=
{
	{0x32f2, 0x80},
    {0x0000, 0x00}
};

static  struct reginfo sensor_Exposure4[]=
{
	{0x32f2, 0x98},
    {0x0000, 0x00}
};

static  struct reginfo sensor_Exposure5[]=
{
	{0x32f2, 0xb0},
    {0x0000, 0x00}
};

static  struct reginfo sensor_Exposure6[]=
{
	{0x32f2, 0xc8},
    {0x0000, 0x00}
};

static struct reginfo *sensor_ExposureSeqe[] = {sensor_Exposure0, sensor_Exposure1, sensor_Exposure2, sensor_Exposure3,
    sensor_Exposure4, sensor_Exposure5,sensor_Exposure6,NULL,
};
#endif
#if CONFIG_SENSOR_Saturation
static  struct reginfo sensor_Saturation0[]=
{
    {0x0000, 0x00}
};

static  struct reginfo sensor_Saturation1[]=
{
    {0x0000, 0x00}
};

static  struct reginfo sensor_Saturation2[]=
{
    {0x0000, 0x00}
};
static struct reginfo *sensor_SaturationSeqe[] = {sensor_Saturation0, sensor_Saturation1, sensor_Saturation2, NULL,};

#endif
#if CONFIG_SENSOR_Contrast
static  struct reginfo sensor_Contrast0[]=
{
    {0x0000, 0x00}
};

static  struct reginfo sensor_Contrast1[]=
{
    {0x0000, 0x00}
};

static  struct reginfo sensor_Contrast2[]=
{
    {0x0000, 0x00}
};

static  struct reginfo sensor_Contrast3[]=
{
    {0x0000, 0x00}
};

static  struct reginfo sensor_Contrast4[]=
{
    {0x0000, 0x00}
};


static  struct reginfo sensor_Contrast5[]=
{
    {0x0000, 0x00}
};

static  struct reginfo sensor_Contrast6[]=
{
    {0x0000, 0x00}
};
static struct reginfo *sensor_ContrastSeqe[] = {sensor_Contrast0, sensor_Contrast1, sensor_Contrast2, sensor_Contrast3,
    sensor_Contrast4, sensor_Contrast5, sensor_Contrast6, NULL,
};

#endif
#if CONFIG_SENSOR_Mirror
static  struct reginfo sensor_MirrorOn[]=
{
	//{0x3022, 0x25},     
    {0x0000, 0x00}
};

static  struct reginfo sensor_MirrorOff[]=
{
	//{0x3022, 0x24},
    {0x0000, 0x00}
};
static struct reginfo *sensor_MirrorSeqe[] = {sensor_MirrorOff, sensor_MirrorOn,NULL,};
#endif
#if CONFIG_SENSOR_Flip
static  struct reginfo sensor_FlipOn[]=
{
	//{0x3022, 0x27},
    {0x0000, 0x00}
};

static  struct reginfo sensor_FlipOff[]=
{
	//{0x3022, 0x26},
    {0x0000, 0x00}
};
static struct reginfo *sensor_FlipSeqe[] = {sensor_FlipOff, sensor_FlipOn,NULL,};

#endif
#if CONFIG_SENSOR_Scene
static  struct reginfo sensor_SceneAuto[] =
{	
	{0x301e, 0x00},
	{0x32c1, 0x24},
	{0x32c2, 0x36},
	{0x32c5, 0x1f}, 
    	{0x0000, 0x00}
};

static  struct reginfo sensor_SceneNight[] =
{
	{0x301e, 0x04},
	{0x32c5, 0x18},    
    	{0x0000, 0x00}
};
static struct reginfo *sensor_SceneSeqe[] = {sensor_SceneAuto, sensor_SceneNight,NULL,};

#endif
#if CONFIG_SENSOR_DigitalZoom
static struct reginfo sensor_Zoom0[] =
{
    {0x0, 0x0},
};

static struct reginfo sensor_Zoom1[] =
{
     {0x0, 0x0},
};

static struct reginfo sensor_Zoom2[] =
{
    {0x0, 0x0},
};


static struct reginfo sensor_Zoom3[] =
{
    {0x0, 0x0},
};
static struct reginfo *sensor_ZoomSeqe[] = {sensor_Zoom0, sensor_Zoom1, sensor_Zoom2, sensor_Zoom3, NULL,};
#endif
static const struct v4l2_querymenu sensor_menus[] =
{
	#if CONFIG_SENSOR_WhiteBalance
    { .id = V4L2_CID_DO_WHITE_BALANCE,  .index = 0,  .name = "auto",  .reserved = 0, }, {  .id = V4L2_CID_DO_WHITE_BALANCE,  .index = 1, .name = "incandescent",  .reserved = 0,},
    { .id = V4L2_CID_DO_WHITE_BALANCE,  .index = 2,  .name = "fluorescent", .reserved = 0,}, {  .id = V4L2_CID_DO_WHITE_BALANCE, .index = 3,  .name = "daylight", .reserved = 0,},
    { .id = V4L2_CID_DO_WHITE_BALANCE,  .index = 4,  .name = "cloudy-daylight", .reserved = 0,},
    #endif

	#if CONFIG_SENSOR_Effect
    { .id = V4L2_CID_EFFECT,  .index = 0,  .name = "none",  .reserved = 0, }, {  .id = V4L2_CID_EFFECT,  .index = 1, .name = "mono",  .reserved = 0,},
    { .id = V4L2_CID_EFFECT,  .index = 2,  .name = "negative", .reserved = 0,}, {  .id = V4L2_CID_EFFECT, .index = 3,  .name = "sepia", .reserved = 0,},
    { .id = V4L2_CID_EFFECT,  .index = 4, .name = "posterize", .reserved = 0,} ,{ .id = V4L2_CID_EFFECT,  .index = 5,  .name = "aqua", .reserved = 0,},
    #endif

	#if CONFIG_SENSOR_Scene
    { .id = V4L2_CID_SCENE,  .index = 0, .name = "auto", .reserved = 0,} ,{ .id = V4L2_CID_SCENE,  .index = 1,  .name = "night", .reserved = 0,},
    #endif

	#if CONFIG_SENSOR_Flash
    { .id = V4L2_CID_FLASH,  .index = 0,  .name = "off",  .reserved = 0, }, {  .id = V4L2_CID_FLASH,  .index = 1, .name = "auto",  .reserved = 0,},
    { .id = V4L2_CID_FLASH,  .index = 2,  .name = "on", .reserved = 0,}, {  .id = V4L2_CID_FLASH, .index = 3,  .name = "torch", .reserved = 0,},
    #endif
};

static const struct v4l2_queryctrl sensor_controls[] =
{
	#if CONFIG_SENSOR_WhiteBalance
    {
        .id		= V4L2_CID_DO_WHITE_BALANCE,
        .type		= V4L2_CTRL_TYPE_MENU,
        .name		= "White Balance Control",
        .minimum	= 0,
        .maximum	= 4,
        .step		= 1,
        .default_value = 0,
    },
    #endif

	#if CONFIG_SENSOR_Brightness
	{
        .id		= V4L2_CID_BRIGHTNESS,
        .type		= V4L2_CTRL_TYPE_INTEGER,
        .name		= "Brightness Control",
        .minimum	= -3,
        .maximum	= 2,
        .step		= 1,
        .default_value = 0,
    },
    #endif

	#if CONFIG_SENSOR_Effect
	{
        .id		= V4L2_CID_EFFECT,
        .type		= V4L2_CTRL_TYPE_MENU,
        .name		= "Effect Control",
        .minimum	= 0,
        .maximum	= 5,
        .step		= 1,
        .default_value = 0,
    },
	#endif

	#if CONFIG_SENSOR_Exposure
	{
        .id		= V4L2_CID_EXPOSURE,
        .type		= V4L2_CTRL_TYPE_INTEGER,
        .name		= "Exposure Control",
        .minimum	= 0,
        .maximum	= 6,
        .step		= 1,
        .default_value = 0,
    },
	#endif

	#if CONFIG_SENSOR_Saturation
	{
        .id		= V4L2_CID_SATURATION,
        .type		= V4L2_CTRL_TYPE_INTEGER,
        .name		= "Saturation Control",
        .minimum	= 0,
        .maximum	= 2,
        .step		= 1,
        .default_value = 0,
    },
    #endif

	#if CONFIG_SENSOR_Contrast
	{
        .id		= V4L2_CID_CONTRAST,
        .type		= V4L2_CTRL_TYPE_INTEGER,
        .name		= "Contrast Control",
        .minimum	= -3,
        .maximum	= 3,
        .step		= 1,
        .default_value = 0,
    },
	#endif

	#if CONFIG_SENSOR_Mirror
	{
        .id		= V4L2_CID_HFLIP,
        .type		= V4L2_CTRL_TYPE_BOOLEAN,
        .name		= "Mirror Control",
        .minimum	= 0,
        .maximum	= 1,
        .step		= 1,
        .default_value = 1,
    },
    #endif

	#if CONFIG_SENSOR_Flip
	{
        .id		= V4L2_CID_VFLIP,
        .type		= V4L2_CTRL_TYPE_BOOLEAN,
        .name		= "Flip Control",
        .minimum	= 0,
        .maximum	= 1,
        .step		= 1,
        .default_value = 1,
    },
    #endif

	#if CONFIG_SENSOR_Scene
    {
        .id		= V4L2_CID_SCENE,
        .type		= V4L2_CTRL_TYPE_MENU,
        .name		= "Scene Control",
        .minimum	= 0,
        .maximum	= 1,
        .step		= 1,
        .default_value = 0,
    },
    #endif

	#if CONFIG_SENSOR_DigitalZoom
    {
        .id		= V4L2_CID_ZOOM_RELATIVE,
        .type		= V4L2_CTRL_TYPE_INTEGER,
        .name		= "DigitalZoom Control",
        .minimum	= -1,
        .maximum	= 1,
        .step		= 1,
        .default_value = 0,
    }, {
        .id		= V4L2_CID_ZOOM_ABSOLUTE,
        .type		= V4L2_CTRL_TYPE_INTEGER,
        .name		= "DigitalZoom Control",
        .minimum	= 0,
        .maximum	= 3,
        .step		= 1,
        .default_value = 0,
    },
    #endif

	#if CONFIG_SENSOR_Focus
	{
        .id		= V4L2_CID_FOCUS_RELATIVE,
        .type		= V4L2_CTRL_TYPE_INTEGER,
        .name		= "Focus Control",
        .minimum	= -1,
        .maximum	= 1,
        .step		= 1,
        .default_value = 0,
    }, {
        .id		= V4L2_CID_FOCUS_ABSOLUTE,
        .type		= V4L2_CTRL_TYPE_INTEGER,
        .name		= "Focus Control",
        .minimum	= 0,
        .maximum	= 255,
        .step		= 1,
        .default_value = 125,
    },
    #endif

	#if CONFIG_SENSOR_Flash
	{
        .id		= V4L2_CID_FLASH,
        .type		= V4L2_CTRL_TYPE_MENU,
        .name		= "Flash Control",
        .minimum	= 0,
        .maximum	= 3,
        .step		= 1,
        .default_value = 0,
    },
	#endif
};

static int sensor_probe(struct i2c_client *client, const struct i2c_device_id *did);
static int sensor_video_probe(struct soc_camera_device *icd, struct i2c_client *client);
static int sensor_g_control(struct v4l2_subdev *sd, struct v4l2_control *ctrl);
static int sensor_s_control(struct v4l2_subdev *sd, struct v4l2_control *ctrl);
static int sensor_g_ext_controls(struct v4l2_subdev *sd,  struct v4l2_ext_controls *ext_ctrl);
static int sensor_s_ext_controls(struct v4l2_subdev *sd,  struct v4l2_ext_controls *ext_ctrl);
static int sensor_suspend(struct soc_camera_device *icd, pm_message_t pm_msg);
static int sensor_resume(struct soc_camera_device *icd);
static int sensor_set_bus_param(struct soc_camera_device *icd,unsigned long flags);
static unsigned long sensor_query_bus_param(struct soc_camera_device *icd);
static int sensor_set_effect(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value);
static int sensor_set_whiteBalance(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value);
static int sensor_deactivate(struct i2c_client *client);

static struct soc_camera_ops sensor_ops =
{
    .suspend                     = sensor_suspend,
    .resume                       = sensor_resume,
    .set_bus_param		= sensor_set_bus_param,
    .query_bus_param	= sensor_query_bus_param,
    .controls		= sensor_controls,
    .menus                         = sensor_menus,
    .num_controls		= ARRAY_SIZE(sensor_controls),
    .num_menus		= ARRAY_SIZE(sensor_menus),
};

/* only one fixed colorspace per pixelcode */
struct sensor_datafmt {
	enum v4l2_mbus_pixelcode code;
	enum v4l2_colorspace colorspace;
};

/* Find a data format by a pixel code in an array */
static const struct sensor_datafmt *sensor_find_datafmt(
	enum v4l2_mbus_pixelcode code, const struct sensor_datafmt *fmt,
	int n)
{
	int i;
	for (i = 0; i < n; i++)
		if (fmt[i].code == code)
			return fmt + i;

	return NULL;
}

static const struct sensor_datafmt sensor_colour_fmts[] = {
//	JPG_FMT(SENSOR_NAME_STRING(UYVY), 16, V4L2_PIX_FMT_UYVY),
//	JPG_FMT(SENSOR_NAME_STRING(YUYV), 16, V4L2_PIX_FMT_YUYV),
    {V4L2_MBUS_FMT_UYVY8_2X8, V4L2_COLORSPACE_JPEG},
    {V4L2_MBUS_FMT_YUYV8_2X8, V4L2_COLORSPACE_JPEG}	
};

typedef struct sensor_info_priv_s
{
    int whiteBalance;
    int brightness;
    int contrast;
    int saturation;
    int effect;
    int scene;
    int digitalzoom;
    int focus;
    int flash;
    int exposure;
	bool snap2preview;
	bool video2preview;
    unsigned char mirror;                                        /* HFLIP */
    unsigned char flip;                                          /* VFLIP */
    unsigned int winseqe_cur_addr;
    struct sensor_datafmt fmt;
    unsigned int funmodule_state;
} sensor_info_priv_t;

struct sensor
{
    struct v4l2_subdev subdev;
    struct i2c_client *client;
    sensor_info_priv_t info_priv;
    int model;	/* V4L2_IDENT_OV* codes from v4l2-chip-ident.h */
#if CONFIG_SENSOR_I2C_NOSCHED
	atomic_t tasklock_cnt;
#endif
	struct rk29camera_platform_data *sensor_io_request;
    struct rk29camera_gpio_res *sensor_gpio_res;
};

static int sensor_read(struct i2c_client *client, u16 reg, u8 *val);
static int sensor_write(struct i2c_client *client, u16 reg, u8 val);

#ifdef DYNA_CHG_ORIENT
extern void set_cam_exist_flag(int);
extern int get_gsensor_curr_mode();
static u8 is_cam_active=0;
#ifdef CONFIG_SOC_CAMERA_FCAM
void nt99250_change_cam_orientation(struct i2c_client *client, int mode)
#else
void change_cam_orientation(struct i2c_client *client, int mode)
#endif
{
	if(!is_cam_active)
		return;
	int val, val_3021;
	const int tmp_dly=75;

	if(!client)
		client=g_i2c_client;

	sensor_read(g_i2c_client, 0x3022, &val);
	if( (!mode && (val&MASK_NT99250_ORIENT)!=0x00) || (mode && (val&MASK_NT99250_ORIENT)!=MASK_NT99250_ORIENT) ) {
        sensor_write(client, 0x3025, 0x01);	//halt sensor output
		msleep(tmp_dly);
		if(!mode && (val&MASK_NT99250_ORIENT)!=0x00) {
			printk("%s(): mode=%d, val=0x%02x, change cam orientation to horz\n", __FUNCTION__, mode, val);
			sensor_write(client, 0x3022, 0x24);
		}
		else if(mode && (val&MASK_NT99250_ORIENT)!=MASK_NT99250_ORIENT) {
			printk("%s(): mode=%d, val=0x%02x, change cam orientation to vert\n", __FUNCTION__, mode, val);
			sensor_write(client, 0x3022,  0x27);
		}
		
		msleep(tmp_dly);
        sensor_write(client, 0x3025, 0x00);	//resume sensor output
	}
}
#ifdef CONFIG_SOC_CAMERA_FCAM
EXPORT_SYMBOL(nt99250_change_cam_orientation);
#else
EXPORT_SYMBOL(change_cam_orientation);
#endif
#endif

static struct sensor* to_sensor(const struct i2c_client *client)
{
    return container_of(i2c_get_clientdata(client), struct sensor, subdev);
}

static int sensor_task_lock(struct i2c_client *client, int lock)
{
#if CONFIG_SENSOR_I2C_NOSCHED
	int cnt = 3;
    struct sensor *sensor = to_sensor(client);

	if (lock) {
		if (atomic_read(&sensor->tasklock_cnt) == 0) {
			while ((atomic_read(&client->adapter->bus_lock.count) < 1) && (cnt>0)) {
				SENSOR_TR("\n %s will obtain i2c in atomic, but i2c bus is locked! Wait...\n",SENSOR_NAME_STRING());
				msleep(35);
				cnt--;
			}
			if ((atomic_read(&client->adapter->bus_lock.count) < 1) && (cnt<=0)) {
				SENSOR_TR("\n %s obtain i2c fail in atomic!!\n",SENSOR_NAME_STRING());
				goto sensor_task_lock_err;
			}
			preempt_disable();
		}

		atomic_add(1, &sensor->tasklock_cnt);
	} else {
		if (atomic_read(&sensor->tasklock_cnt) > 0) {
			atomic_sub(1, &sensor->tasklock_cnt);

			if (atomic_read(&sensor->tasklock_cnt) == 0)
				preempt_enable();
		}
	}
#endif
	return 0;
sensor_task_lock_err:
	return -1;
}

/* sensor register write */
static int sensor_write(struct i2c_client *client, u16 reg, u8 val)
{
    int err,cnt;
    u8 buf[3];
    struct i2c_msg msg[1];

    buf[0] = reg >> 8;
    buf[1] = reg & 0xFF;
    buf[2] = val;

    msg->addr = client->addr;
    msg->flags = client->flags;
    msg->buf = buf;
    msg->len = sizeof(buf);
    msg->scl_rate = CONFIG_SENSOR_I2C_SPEED;         /* ddl@rock-chips.com : 100kHz */
    msg->read_type = 0;               /* fpga i2c:0==I2C_NORMAL : direct use number not enum for don't want include spi_fpga.h */

    cnt = 3;
    err = -EAGAIN;

    while ((cnt-- > 0) && (err < 0)) {                       /* ddl@rock-chips.com :  Transfer again if transent is failed   */
        err = i2c_transfer(client->adapter, msg, 1);

        if (err >= 0) {
            return 0;
        } else {
        	SENSOR_TR("\n %s write reg(0x%x, val:0x%x) failed, try to write again!\n",SENSOR_NAME_STRING(),reg, val);
	        udelay(10);
        }
    }

    return err;
}

/* sensor register read */
static int sensor_read(struct i2c_client *client, u16 reg, u8 *val)
{
    int err,cnt;
    u8 buf[2];
    struct i2c_msg msg[2];

    buf[0] = reg >> 8;
    buf[1] = reg & 0xFF;

    msg[0].addr = client->addr;
    msg[0].flags = client->flags;
    msg[0].buf = buf;
    msg[0].len = sizeof(buf);
    msg[0].scl_rate = CONFIG_SENSOR_I2C_SPEED;       /* ddl@rock-chips.com : 100kHz */
    msg[0].read_type = 2;   /* fpga i2c:0==I2C_NO_STOP : direct use number not enum for don't want include spi_fpga.h */

    msg[1].addr = client->addr;
    msg[1].flags = client->flags|I2C_M_RD;
    msg[1].buf = buf;
    msg[1].len = 1;
    msg[1].scl_rate = CONFIG_SENSOR_I2C_SPEED;                       /* ddl@rock-chips.com : 100kHz */
    msg[1].read_type = 2;                             /* fpga i2c:0==I2C_NO_STOP : direct use number not enum for don't want include spi_fpga.h */

    cnt = 3;
    err = -EAGAIN;
    while ((cnt-- > 0) && (err < 0)) {                       /* ddl@rock-chips.com :  Transfer again if transent is failed   */
        err = i2c_transfer(client->adapter, msg, 2);

        if (err >= 0) {
            *val = buf[0];
            return 0;
        } else {
        	SENSOR_TR("\n %s read reg(0x%x val:0x%x) failed, try to read again! \n",SENSOR_NAME_STRING(),reg, *val);
            udelay(10);
        }
    }

    return err;
}

/* write a array of registers  */
static int sensor_write_array(struct i2c_client *client, struct reginfo *regarray)
{
    int err = 0, cnt;
    int i = 0;
	char valchk;

	cnt = 0;
	if (sensor_task_lock(client, 1) < 0)
		goto sensor_write_array_end;

    while (regarray[i].reg != 0)
    {
#ifdef DYNA_CHG_ORIENT
		if(regarray[i].reg == 0x3022)
			err = sensor_write(client, regarray[i].reg, get_gsensor_curr_mode()?0x27:0x24);
		else
			err = sensor_write(client, regarray[i].reg, regarray[i].val);
#else
        err = sensor_write(client, regarray[i].reg, regarray[i].val);
#endif
        if (err < 0)
        {
            if (cnt-- > 0) {
			    SENSOR_TR("%s..write failed current reg:0x%x, Write array again !\n", SENSOR_NAME_STRING(),regarray[i].reg);
				i = 0;
				continue;
            } else {
                SENSOR_TR("%s..write array failed!!!\n", SENSOR_NAME_STRING());
                err = -EPERM;
				goto sensor_write_array_end;
            }
        } else {
        #if CONFIG_SENSOR_I2C_RDWRCHK
			sensor_read(client, regarray[i].reg, &valchk);
			if (valchk != regarray[i].val)
				SENSOR_TR("%s Reg:0x%x write(0x%x, 0x%x) fail\n",SENSOR_NAME_STRING(), regarray[i].reg, regarray[i].val, valchk);
		#endif
        }
        i++;
    }

sensor_write_array_end:
	sensor_task_lock(client,0);
	return err;
}
static int sensor_readchk_array(struct i2c_client *client, struct reginfo *regarray)
{
    int cnt;
    int i = 0;
	char valchk;

	cnt = 0;
	valchk = 0;
    while (regarray[i].reg != 0)
    {
		sensor_read(client, regarray[i].reg, &valchk);
		if (valchk != regarray[i].val)
			SENSOR_TR("%s Reg:0x%x read(0x%x, 0x%x) error\n",SENSOR_NAME_STRING(), regarray[i].reg, regarray[i].val, valchk);

        i++;
    }
    return 0;
}
static int sensor_ioctrl(struct soc_camera_device *icd,enum rk29sensor_power_cmd cmd, int on)
{
	struct soc_camera_link *icl = to_soc_camera_link(icd);
	int ret = 0;

    SENSOR_DG("%s %s  cmd(%d) on(%d)\n",SENSOR_NAME_STRING(),__FUNCTION__,cmd,on);
	switch (cmd)
	{
		case Sensor_PowerDown:
		{
			if (icl->powerdown) {
				ret = icl->powerdown(icd->pdev, on);
				if (ret == RK29_CAM_IO_SUCCESS) {
					if (on == 0) {
						mdelay(2);
						if (icl->reset)
							icl->reset(icd->pdev);
					}
				} else if (ret == RK29_CAM_EIO_REQUESTFAIL) {
					ret = -ENODEV;
					goto sensor_power_end;
				}
			}
			break;
		}
		case Sensor_Flash:
		{
			struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));
    		struct sensor *sensor = to_sensor(client);

			if (sensor->sensor_io_request && sensor->sensor_io_request->sensor_ioctrl) {
				sensor->sensor_io_request->sensor_ioctrl(icd->pdev,Cam_Flash, on);
			}
            break;
		}
		default:
		{
			SENSOR_TR("%s %s cmd(0x%x) is unknown!",SENSOR_NAME_STRING(),__FUNCTION__,cmd);
			break;
		}
	}
sensor_power_end:
	return ret;
}
static int sensor_init(struct v4l2_subdev *sd, u32 val)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct soc_camera_device *icd = client->dev.platform_data;
    struct sensor *sensor = to_sensor(client);
	const struct v4l2_queryctrl *qctrl;
    const struct sensor_datafmt *fmt;
    char value;
    int ret,pid = 0;

	printk("*** sensor_init() set power on\n");
    SENSOR_DG("\n%s..%s.. \n",SENSOR_NAME_STRING(),__FUNCTION__);
#if 1
	if (sensor_ioctrl(icd, Sensor_PowerDown, 0) < 0) {
		ret = -ENODEV;
		goto sensor_INIT_ERR;
	}
#endif
    /* soft reset */
	if (sensor_task_lock(client,1)<0)
		goto sensor_INIT_ERR;
#if 0	
    ret = sensor_write(client, 0x3012, 0x80);
    if (ret != 0)
    {
        SENSOR_TR("%s soft reset sensor failed\n",SENSOR_NAME_STRING());
        ret = -ENODEV;
		goto sensor_INIT_ERR;
    }
#endif
    mdelay(5);  //delay 5 microseconds
	/* check if it is an sensor sensor */

re:	
    ret = sensor_read(client, 0x307e, &value);
    if (ret != 0) {
        SENSOR_TR("read chip id high byte failed\n");
        ret = -ENODEV;
	printk("*** re-read\n");
        goto sensor_INIT_ERR;
    }

    pid |= (value << 8);

    ret = sensor_read(client, 0x307f, &value);
    if (ret != 0) {
        SENSOR_TR("read chip id low byte failed\n");
        ret = -ENODEV;
        goto sensor_INIT_ERR;
    }

    pid |= (value & 0xff);
    SENSOR_DG("\n %s  pid = 0x%x\n", SENSOR_NAME_STRING(), pid);
    if (pid == SENSOR_ID) {
        sensor->model = SENSOR_V4L2_IDENT;
    } else {
        SENSOR_TR("error: %s mismatched   pid = 0x%x\n", SENSOR_NAME_STRING(), pid);
        ret = -ENODEV;
        goto sensor_INIT_ERR;
    }

    ret = sensor_write_array(client, sensor_init_data);
    if (ret != 0)
    {
        SENSOR_TR("error: %s initial failed\n",SENSOR_NAME_STRING());
        goto sensor_INIT_ERR;
    }
	sensor_task_lock(client,0);
    //icd->user_width = SENSOR_INIT_WIDTH;
    //icd->user_height = SENSOR_INIT_HEIGHT;
    sensor->info_priv.winseqe_cur_addr  = (int)SENSOR_INIT_WINSEQADR;
    fmt = sensor_find_datafmt(SENSOR_INIT_PIXFMT,sensor_colour_fmts, ARRAY_SIZE(sensor_colour_fmts));
    if (!fmt) {
        SENSOR_TR("error: %s initial array colour fmts is not support!!",SENSOR_NAME_STRING());
        ret = -EINVAL;
        goto sensor_INIT_ERR;
    }
	sensor->info_priv.fmt = *fmt;

    /* sensor sensor information for initialization  */
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_DO_WHITE_BALANCE);
	if (qctrl)
    	sensor->info_priv.whiteBalance = qctrl->default_value;
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_BRIGHTNESS);
	if (qctrl)
    	sensor->info_priv.brightness = qctrl->default_value;
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_EFFECT);
	if (qctrl)
    	sensor->info_priv.effect = qctrl->default_value;
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_EXPOSURE);
	if (qctrl)
        sensor->info_priv.exposure = qctrl->default_value;

	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_SATURATION);
	if (qctrl)
        sensor->info_priv.saturation = qctrl->default_value;
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_CONTRAST);
	if (qctrl)
        sensor->info_priv.contrast = qctrl->default_value;
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_HFLIP);
	if (qctrl)
        sensor->info_priv.mirror = qctrl->default_value;
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_VFLIP);
	if (qctrl)
        sensor->info_priv.flip = qctrl->default_value;
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_SCENE);
	if (qctrl)
        sensor->info_priv.scene = qctrl->default_value;
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_ZOOM_ABSOLUTE);
	if (qctrl)
        sensor->info_priv.digitalzoom = qctrl->default_value;

    /* ddl@rock-chips.com : if sensor support auto focus and flash, programer must run focus and flash code  */
	#if CONFIG_SENSOR_Focus
    sensor_set_focus();
    qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_FOCUS_ABSOLUTE);
	if (qctrl)
        sensor->info_priv.focus = qctrl->default_value;
	#endif

	#if CONFIG_SENSOR_Flash	
	qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_FLASH);
	if (qctrl)
        sensor->info_priv.flash = qctrl->default_value;
    #endif

    SENSOR_DG("\n%s..%s.. icd->width = %d.. icd->height %d\n",SENSOR_NAME_STRING(),((val == 0)?__FUNCTION__:"sensor_reinit"),icd->user_width,icd->user_height);

#ifdef DYNA_CHG_ORIENT
	set_cam_exist_flag(1);
	is_cam_active=1;
#endif
    sensor->info_priv.funmodule_state |= SENSOR_INIT_IS_OK;
    return 0;
sensor_INIT_ERR:
    sensor->info_priv.funmodule_state &= ~SENSOR_INIT_IS_OK;
	sensor_task_lock(client,0);
	sensor_deactivate(client);
    return ret;
}

static int sensor_deactivate(struct i2c_client *client)
{
	struct soc_camera_device *icd = client->dev.platform_data;

    struct sensor *sensor = to_sensor(client);
	u8 reg_val;

	SENSOR_DG("\n%s..%s.. Enter\n",SENSOR_NAME_STRING(),__FUNCTION__);

	/* ddl@rock-chips.com : all sensor output pin must change to input for other sensor */
	//sensor_task_lock(client, 1);
	//sensor_read(client,0x3000,&reg_val);
    //sensor_write(client, 0x3000, reg_val&0xfc);
	//sensor_write(client, 0x3001, 0x00);
	//sensor_read(client,0x3002,&reg_val);
	//sensor_write(client, 0x3002, reg_val&0x1f);
	//sensor_task_lock(client, 0);
	sensor_ioctrl(icd, Sensor_PowerDown, 1);

	/* ddl@rock-chips.com : sensor config init width , because next open sensor quickly(soc_camera_open -> Try to configure with default parameters) */
	icd->user_width = SENSOR_INIT_WIDTH;
    icd->user_height = SENSOR_INIT_HEIGHT;
	msleep(100);

#ifdef DYNA_CHG_ORIENT	
    set_cam_exist_flag(0);
	is_cam_active=0;
	printk("*** cam is deactive\n");
#endif	

    sensor->info_priv.funmodule_state &= ~SENSOR_INIT_IS_OK;

	return 0;
}

static  struct reginfo sensor_power_down_sequence[]=
{
    {0x00,0x00}
};
static int sensor_suspend(struct soc_camera_device *icd, pm_message_t pm_msg)
{
    int ret;
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

    if (pm_msg.event == PM_EVENT_SUSPEND) {
        SENSOR_DG("\n %s Enter Suspend.. \n", SENSOR_NAME_STRING());
        ret = sensor_write_array(client, sensor_power_down_sequence) ;
        if (ret != 0) {
            SENSOR_TR("\n %s..%s WriteReg Fail.. \n", SENSOR_NAME_STRING(),__FUNCTION__);
            return ret;
        } else {
            ret = sensor_ioctrl(icd, Sensor_PowerDown, 1);
            if (ret < 0) {
			    SENSOR_TR("\n %s suspend fail for turn on power!\n", SENSOR_NAME_STRING());
                return -EINVAL;
            }
        }
    } else {
        SENSOR_TR("\n %s cann't suppout Suspend..\n",SENSOR_NAME_STRING());
        return -EINVAL;
    }
    return 0;
}

static int sensor_resume(struct soc_camera_device *icd)
{
	int ret;

    ret = sensor_ioctrl(icd, Sensor_PowerDown, 0);
    if (ret < 0) {
		SENSOR_TR("\n %s resume fail for turn on power!\n", SENSOR_NAME_STRING());
        return -EINVAL;
    }

	SENSOR_DG("\n %s Enter Resume.. \n", SENSOR_NAME_STRING());

    return 0;

}

static int sensor_set_bus_param(struct soc_camera_device *icd,
                                unsigned long flags)
{

    return 0;
}

static unsigned long sensor_query_bus_param(struct soc_camera_device *icd)
{
    struct soc_camera_link *icl = to_soc_camera_link(icd);
    unsigned long flags = SENSOR_BUS_PARAM;

    return soc_camera_apply_sensor_flags(icl, flags);
}

static int sensor_g_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct soc_camera_device *icd = client->dev.platform_data;
    struct sensor *sensor = to_sensor(client);

    mf->width	= icd->user_width;
	mf->height	= icd->user_height;
	mf->code	= sensor->info_priv.fmt.code;
	mf->colorspace	= sensor->info_priv.fmt.colorspace;
	mf->field	= V4L2_FIELD_NONE;

    return 0;
}

static bool sensor_fmt_capturechk(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
    bool ret = false;

	if ((mf->width == 1024) && (mf->height == 768)) {
		ret = true;
	} else if ((mf->width == 1280) && (mf->height == 1024)) {
		ret = true;
	} else if ((mf->width == 1600) && (mf->height == 1200)) {
		ret = true;
	} else if ((mf->width == 2048) && (mf->height == 1536)) {
		ret = true;
	} else if ((mf->width == 2592) && (mf->height == 1944)) {
		ret = true;
	}

	if (ret == true)
		SENSOR_DG("%s %dx%d is capture format\n", __FUNCTION__, mf->width, mf->height);


	return ret;
}

static bool sensor_fmt_videochk(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
    bool ret = false;

	if ((mf->width == 1280) && (mf->height == 720)) {
		ret = true;
	} else if ((mf->width == 1920) && (mf->height == 1080)) {
		ret = true;
	}

	if (ret == true)
		SENSOR_DG("%s %dx%d is video format\n", __FUNCTION__, mf->width, mf->height);
	return ret;
}

static int sensor_s_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    const struct sensor_datafmt *fmt;
    struct sensor *sensor = to_sensor(client);
	const struct v4l2_queryctrl *qctrl;
	struct soc_camera_device *icd = client->dev.platform_data;
    struct reginfo *winseqe_set_addr=NULL;
    int ret=0, set_w,set_h;

	fmt = sensor_find_datafmt(mf->code, sensor_colour_fmts,
				   ARRAY_SIZE(sensor_colour_fmts));
	if (!fmt) {
        ret = -EINVAL;
        goto sensor_s_fmt_end;
    }

	if (sensor->info_priv.fmt.code != mf->code) {
		switch (mf->code)
		{
			case V4L2_PIX_FMT_YUYV:
			{
				winseqe_set_addr = sensor_ClrFmt_YUYV;
				break;
			}
			case V4L2_PIX_FMT_UYVY:
			{
				winseqe_set_addr = sensor_ClrFmt_UYVY;
				break;
			}
			default:
				break;
		}
		if (winseqe_set_addr != NULL) {
            sensor_write_array(client, winseqe_set_addr);
			sensor->info_priv.fmt.code = mf->code;
            sensor->info_priv.fmt.colorspace= mf->colorspace;            
			SENSOR_DG("%s v4l2_mbus_code:%d set success!\n", SENSOR_NAME_STRING(),mf->code);
		} else {
			SENSOR_TR("%s v4l2_mbus_code:%d is invalidate!\n", SENSOR_NAME_STRING(),mf->code);
		}
	}

    set_w = mf->width;
    set_h = mf->height;

	if (((set_w <= 176) && (set_h <= 144)) && sensor_qcif[0].reg)
	{
		winseqe_set_addr = sensor_qcif;
        set_w = 176;
        set_h = 144;
	}
	else if (((set_w <= 320) && (set_h <= 240)) && sensor_qvga[0].reg)
    {
        winseqe_set_addr = sensor_qvga;
        set_w = 320;
        set_h = 240;
    }
    else if (((set_w <= 352) && (set_h<= 288)) && sensor_cif[0].reg)
    {
        winseqe_set_addr = sensor_cif;
        set_w = 352;
        set_h = 288;
    }
    else if (((set_w <= 640) && (set_h <= 480)) && sensor_vga[0].reg)
    {
        winseqe_set_addr = sensor_vga;
        set_w = 640;
        set_h = 480;
    }
    else if (((set_w <= 800) && (set_h <= 600)) && sensor_svga[0].reg)
    {
        winseqe_set_addr = sensor_svga;
        set_w = 800;
        set_h = 600;
    }
    else if (((set_w <= 1280) && (set_h <= 720)) && sensor_720p[0].reg)
    {
        winseqe_set_addr = sensor_720p;
        set_w = 1280;
        set_h = 720;
    }
	else if (((set_w <= 1024) && (set_h <= 768)) && sensor_xga[0].reg)
    {
        winseqe_set_addr = sensor_xga;
        set_w = 1024;
        set_h = 768;
    }
    else if (((set_w <= 1280) && (set_h <= 1024)) && sensor_sxga[0].reg)
    {
        winseqe_set_addr = sensor_sxga;
        set_w = 1280;
        set_h = 1024;
    }
    else if (((set_w <= 1600) && (set_h <= 1200)) && sensor_uxga[0].reg)
    {
        winseqe_set_addr = sensor_uxga;
        set_w = 1600;
        set_h = 1200;
    }
    else
    {
        winseqe_set_addr = SENSOR_INIT_WINSEQADR;               /* ddl@rock-chips.com : Sensor output smallest size if  isn't support app  */
        set_w = SENSOR_INIT_WIDTH;
        set_h = SENSOR_INIT_HEIGHT;
		SENSOR_TR("\n %s..%s Format is Invalidate. pix->width = %d.. pix->height = %d\n",SENSOR_NAME_STRING(),__FUNCTION__,mf->width,mf->height);
    }

    if ((int)winseqe_set_addr  != sensor->info_priv.winseqe_cur_addr) {
        #if CONFIG_SENSOR_Flash
        if (sensor_fmt_capturechk(sd,mf) == true) {      /* ddl@rock-chips.com : Capture */
            if ((sensor->info_priv.flash == 1) || (sensor->info_priv.flash == 2)) {
                sensor_ioctrl(icd, Sensor_Flash, Flash_On);
                SENSOR_DG("%s flash on in capture!\n", SENSOR_NAME_STRING());
            }           
        } else {                                        /* ddl@rock-chips.com : Video */
            if ((sensor->info_priv.flash == 1) || (sensor->info_priv.flash == 2)) {
                sensor_ioctrl(icd, Sensor_Flash, Flash_Off);
                SENSOR_DG("%s flash off in preivew!\n", SENSOR_NAME_STRING());
            }
        }
        #endif
        ret |= sensor_write_array(client, winseqe_set_addr);
        if (ret != 0) {
            SENSOR_TR("%s set format capability failed\n", SENSOR_NAME_STRING());
            #if CONFIG_SENSOR_Flash
            if (sensor_fmt_capturechk(sd,mf) == true) {
                if ((sensor->info_priv.flash == 1) || (sensor->info_priv.flash == 2)) {
                    sensor_ioctrl(icd, Sensor_Flash, Flash_Off);
                    SENSOR_TR("%s Capture format set fail, flash off !\n", SENSOR_NAME_STRING());
                }
            }
            #endif
            goto sensor_s_fmt_end;
        }

        sensor->info_priv.winseqe_cur_addr  = (int)winseqe_set_addr;

		if (sensor_fmt_capturechk(sd,mf) == true) {				    /* ddl@rock-chips.com : Capture */
			qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_EFFECT);
			sensor_set_effect(icd, qctrl,sensor->info_priv.effect);
			if (sensor->info_priv.whiteBalance != 0) {
				qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_DO_WHITE_BALANCE);
				sensor_set_whiteBalance(icd, qctrl,sensor->info_priv.whiteBalance);
			}
			sensor->info_priv.snap2preview = true;
		} else if (sensor_fmt_videochk(sd,mf) == true) {			/* ddl@rock-chips.com : Video */
			qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_EFFECT);
			sensor_set_effect(icd, qctrl,sensor->info_priv.effect);
			qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_DO_WHITE_BALANCE);
			sensor_set_whiteBalance(icd, qctrl,sensor->info_priv.whiteBalance);
			sensor->info_priv.video2preview = true;
		} else if ((sensor->info_priv.snap2preview == true) || (sensor->info_priv.video2preview == true)) {
			qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_EFFECT);
			sensor_set_effect(icd, qctrl,sensor->info_priv.effect);
			qctrl = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_DO_WHITE_BALANCE);
			sensor_set_whiteBalance(icd, qctrl,sensor->info_priv.whiteBalance);
			sensor->info_priv.video2preview = false;
			sensor->info_priv.snap2preview = false;
		}

        SENSOR_DG("\n%s..%s.. icd->width = %d.. icd->height %d\n",SENSOR_NAME_STRING(),__FUNCTION__,set_w,set_h);
    }
    else
    {
        SENSOR_DG("\n %s .. Current Format is validate. icd->width = %d.. icd->height %d\n",SENSOR_NAME_STRING(),set_w,set_h);
    }

	mf->width = set_w;
    mf->height = set_h;

sensor_s_fmt_end:
    return ret;
}

static int sensor_try_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct sensor *sensor = to_sensor(client);
    const struct sensor_datafmt *fmt;
    int ret = 0;
   
	fmt = sensor_find_datafmt(mf->code, sensor_colour_fmts,
				   ARRAY_SIZE(sensor_colour_fmts));
	if (fmt == NULL) {
		fmt = &sensor->info_priv.fmt;
        mf->code = fmt->code;
	} 

    if (mf->height > SENSOR_MAX_HEIGHT)
        mf->height = SENSOR_MAX_HEIGHT;
    else if (mf->height < SENSOR_MIN_HEIGHT)
        mf->height = SENSOR_MIN_HEIGHT;

    if (mf->width > SENSOR_MAX_WIDTH)
        mf->width = SENSOR_MAX_WIDTH;
    else if (mf->width < SENSOR_MIN_WIDTH)
        mf->width = SENSOR_MIN_WIDTH;

    mf->colorspace = fmt->colorspace;
    
    return ret;

}

 static int sensor_g_chip_ident(struct v4l2_subdev *sd, struct v4l2_dbg_chip_ident *id)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);

    if (id->match.type != V4L2_CHIP_MATCH_I2C_ADDR)
        return -EINVAL;

    if (id->match.addr != client->addr)
        return -ENODEV;

    id->ident = SENSOR_V4L2_IDENT;
    id->revision = 0;

    return 0;
}
#if CONFIG_SENSOR_Brightness
static int sensor_set_brightness(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (sensor_BrightnessSeqe[value - qctrl->minimum] != NULL)
        {
            if (sensor_write_array(client, sensor_BrightnessSeqe[value - qctrl->minimum]) != 0)
            {
                SENSOR_TR("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            SENSOR_DG("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
            return 0;
        }
    }
	SENSOR_TR("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
}
#endif
#if CONFIG_SENSOR_Effect
static int sensor_set_effect(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (sensor_EffectSeqe[value - qctrl->minimum] != NULL)
        {
            if (sensor_write_array(client, sensor_EffectSeqe[value - qctrl->minimum]) != 0)
            {
                SENSOR_TR("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            SENSOR_DG("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
            return 0;
        }
    }
	SENSOR_TR("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
}
#endif
#if CONFIG_SENSOR_Exposure
static int sensor_set_exposure(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (sensor_ExposureSeqe[value - qctrl->minimum] != NULL)
        {
            if (sensor_write_array(client, sensor_ExposureSeqe[value - qctrl->minimum]) != 0)
            {
                SENSOR_TR("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            SENSOR_DG("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
            return 0;
        }
    }
	SENSOR_TR("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
}
#endif
#if CONFIG_SENSOR_Saturation
static int sensor_set_saturation(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (sensor_SaturationSeqe[value - qctrl->minimum] != NULL)
        {
            if (sensor_write_array(client, sensor_SaturationSeqe[value - qctrl->minimum]) != 0)
            {
                SENSOR_TR("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            SENSOR_DG("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
            return 0;
        }
    }
    SENSOR_TR("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
}
#endif
#if CONFIG_SENSOR_Contrast
static int sensor_set_contrast(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (sensor_ContrastSeqe[value - qctrl->minimum] != NULL)
        {
            if (sensor_write_array(client, sensor_ContrastSeqe[value - qctrl->minimum]) != 0)
            {
                SENSOR_TR("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            SENSOR_DG("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
            return 0;
        }
    }
    SENSOR_TR("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
}
#endif
#if CONFIG_SENSOR_Mirror
static int sensor_set_mirror(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (sensor_MirrorSeqe[value - qctrl->minimum] != NULL)
        {
            if (sensor_write_array(client, sensor_MirrorSeqe[value - qctrl->minimum]) != 0)
            {
                SENSOR_TR("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            SENSOR_DG("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
            return 0;
        }
    }
    SENSOR_TR("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
}
#endif
#if CONFIG_SENSOR_Flip
static int sensor_set_flip(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (sensor_FlipSeqe[value - qctrl->minimum] != NULL)
        {
            if (sensor_write_array(client, sensor_FlipSeqe[value - qctrl->minimum]) != 0)
            {
                SENSOR_TR("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            SENSOR_DG("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
            return 0;
        }
    }
    SENSOR_TR("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
}
#endif
#if CONFIG_SENSOR_Scene
static int sensor_set_scene(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (sensor_SceneSeqe[value - qctrl->minimum] != NULL)
        {
            if (sensor_write_array(client, sensor_SceneSeqe[value - qctrl->minimum]) != 0)
            {
                SENSOR_TR("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            SENSOR_DG("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
            return 0;
        }
    }
    SENSOR_TR("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
}
#endif
#if CONFIG_SENSOR_WhiteBalance
static int sensor_set_whiteBalance(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));

    if ((value >= qctrl->minimum) && (value <= qctrl->maximum))
    {
        if (sensor_WhiteBalanceSeqe[value - qctrl->minimum] != NULL)
        {
            if (sensor_write_array(client, sensor_WhiteBalanceSeqe[value - qctrl->minimum]) != 0)
            {
                SENSOR_TR("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
                return -EINVAL;
            }
            SENSOR_DG("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
            return 0;
        }
    }
	SENSOR_TR("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
}
#endif
#if CONFIG_SENSOR_DigitalZoom
static int sensor_set_digitalzoom(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int *value)
{
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));
    struct sensor *sensor = to_sensor(client);
	const struct v4l2_queryctrl *qctrl_info;
    int digitalzoom_cur, digitalzoom_total;

	qctrl_info = soc_camera_find_qctrl(&sensor_ops, V4L2_CID_ZOOM_ABSOLUTE);
	if (qctrl_info)
		return -EINVAL;

    digitalzoom_cur = sensor->info_priv.digitalzoom;
    digitalzoom_total = qctrl_info->maximum;

    if ((*value > 0) && (digitalzoom_cur >= digitalzoom_total))
    {
        SENSOR_TR("%s digitalzoom is maximum - %x\n", SENSOR_NAME_STRING(), digitalzoom_cur);
        return -EINVAL;
    }

    if  ((*value < 0) && (digitalzoom_cur <= qctrl_info->minimum))
    {
        SENSOR_TR("%s digitalzoom is minimum - %x\n", SENSOR_NAME_STRING(), digitalzoom_cur);
        return -EINVAL;
    }

    if ((*value > 0) && ((digitalzoom_cur + *value) > digitalzoom_total))
    {
        *value = digitalzoom_total - digitalzoom_cur;
    }

    if ((*value < 0) && ((digitalzoom_cur + *value) < 0))
    {
        *value = 0 - digitalzoom_cur;
    }

    digitalzoom_cur += *value;

    if (sensor_ZoomSeqe[digitalzoom_cur] != NULL)
    {
        if (sensor_write_array(client, sensor_ZoomSeqe[digitalzoom_cur]) != 0)
        {
            SENSOR_TR("%s..%s WriteReg Fail.. \n",SENSOR_NAME_STRING(), __FUNCTION__);
            return -EINVAL;
        }
        SENSOR_DG("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, *value);
        return 0;
    }

    return -EINVAL;
}
#endif
#if CONFIG_SENSOR_Flash
static int sensor_set_flash(struct soc_camera_device *icd, const struct v4l2_queryctrl *qctrl, int value)
{
    
    if ((value >= qctrl->minimum) && (value <= qctrl->maximum)) {
        if (value == 3) {       /* ddl@rock-chips.com: torch */
            sensor_ioctrl(icd, Sensor_Flash, Flash_Torch);   /* Flash On */
        } else {
            sensor_ioctrl(icd, Sensor_Flash, Flash_Off);
        }
        SENSOR_DG("%s..%s : %x\n",SENSOR_NAME_STRING(),__FUNCTION__, value);
        return 0;
    }
    
	SENSOR_TR("\n %s..%s valure = %d is invalidate..    \n",SENSOR_NAME_STRING(),__FUNCTION__,value);
    return -EINVAL;
}
#endif

static int sensor_g_control(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct sensor *sensor = to_sensor(client);
    const struct v4l2_queryctrl *qctrl;

    qctrl = soc_camera_find_qctrl(&sensor_ops, ctrl->id);

    if (!qctrl)
    {
        SENSOR_TR("\n %s ioctrl id = %d  is invalidate \n", SENSOR_NAME_STRING(), ctrl->id);
        return -EINVAL;
    }

    switch (ctrl->id)
    {
        case V4L2_CID_BRIGHTNESS:
            {
                ctrl->value = sensor->info_priv.brightness;
                break;
            }
        case V4L2_CID_SATURATION:
            {
                ctrl->value = sensor->info_priv.saturation;
                break;
            }
        case V4L2_CID_CONTRAST:
            {
                ctrl->value = sensor->info_priv.contrast;
                break;
            }
        case V4L2_CID_DO_WHITE_BALANCE:
            {
                ctrl->value = sensor->info_priv.whiteBalance;
                break;
            }
        case V4L2_CID_EXPOSURE:
            {
                ctrl->value = sensor->info_priv.exposure;
                break;
            }
        case V4L2_CID_HFLIP:
            {
                ctrl->value = sensor->info_priv.mirror;
                break;
            }
        case V4L2_CID_VFLIP:
            {
                ctrl->value = sensor->info_priv.flip;
                break;
            }
        default :
                break;
    }
    return 0;
}



static int sensor_s_control(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct sensor *sensor = to_sensor(client);
    struct soc_camera_device *icd = client->dev.platform_data;
    const struct v4l2_queryctrl *qctrl;


    qctrl = soc_camera_find_qctrl(&sensor_ops, ctrl->id);

    if (!qctrl)
    {
        SENSOR_TR("\n %s ioctrl id = %d  is invalidate \n", SENSOR_NAME_STRING(), ctrl->id);
        return -EINVAL;
    }

    switch (ctrl->id)
    {
#if CONFIG_SENSOR_Brightness
        case V4L2_CID_BRIGHTNESS:
            {
                if (ctrl->value != sensor->info_priv.brightness)
                {
                    if (sensor_set_brightness(icd, qctrl,ctrl->value) != 0)
                    {
                        return -EINVAL;
                    }
                    sensor->info_priv.brightness = ctrl->value;
                }
                break;
            }
#endif
#if CONFIG_SENSOR_Exposure
        case V4L2_CID_EXPOSURE:
            {
                if (ctrl->value != sensor->info_priv.exposure)
                {
                    if (sensor_set_exposure(icd, qctrl,ctrl->value) != 0)
                    {
                        return -EINVAL;
                    }
                    sensor->info_priv.exposure = ctrl->value;
                }
                break;
            }
#endif
#if CONFIG_SENSOR_Saturation
        case V4L2_CID_SATURATION:
            {
                if (ctrl->value != sensor->info_priv.saturation)
                {
                    if (sensor_set_saturation(icd, qctrl,ctrl->value) != 0)
                    {
                        return -EINVAL;
                    }
                    sensor->info_priv.saturation = ctrl->value;
                }
                break;
            }
#endif
#if CONFIG_SENSOR_Contrast
        case V4L2_CID_CONTRAST:
            {
                if (ctrl->value != sensor->info_priv.contrast)
                {
                    if (sensor_set_contrast(icd, qctrl,ctrl->value) != 0)
                    {
                        return -EINVAL;
                    }
                    sensor->info_priv.contrast = ctrl->value;
                }
                break;
            }
#endif
#if CONFIG_SENSOR_WhiteBalance
        case V4L2_CID_DO_WHITE_BALANCE:
            {
                if (ctrl->value != sensor->info_priv.whiteBalance)
                {
                    if (sensor_set_whiteBalance(icd, qctrl,ctrl->value) != 0)
                    {
                        return -EINVAL;
                    }
                    sensor->info_priv.whiteBalance = ctrl->value;
                }
                break;
            }
#endif
#if CONFIG_SENSOR_Mirror
        case V4L2_CID_HFLIP:
            {
                if (ctrl->value != sensor->info_priv.mirror)
                {
                    if (sensor_set_mirror(icd, qctrl,ctrl->value) != 0)
                        return -EINVAL;
                    sensor->info_priv.mirror = ctrl->value;
                }
                break;
            }
#endif
#if CONFIG_SENSOR_Flip
        case V4L2_CID_VFLIP:
            {
                if (ctrl->value != sensor->info_priv.flip)
                {
                    if (sensor_set_flip(icd, qctrl,ctrl->value) != 0)
                        return -EINVAL;
                    sensor->info_priv.flip = ctrl->value;
                }
                break;
            }
#endif
        default:
            break;
    }

    return 0;
}
static int sensor_g_ext_control(struct soc_camera_device *icd , struct v4l2_ext_control *ext_ctrl)
{
    const struct v4l2_queryctrl *qctrl;
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));
    struct sensor *sensor = to_sensor(client);

    qctrl = soc_camera_find_qctrl(&sensor_ops, ext_ctrl->id);

    if (!qctrl)
    {
        SENSOR_TR("\n %s ioctrl id = %d  is invalidate \n", SENSOR_NAME_STRING(), ext_ctrl->id);
        return -EINVAL;
    }

    switch (ext_ctrl->id)
    {
        case V4L2_CID_SCENE:
            {
                ext_ctrl->value = sensor->info_priv.scene;
                break;
            }
        case V4L2_CID_EFFECT:
            {
                ext_ctrl->value = sensor->info_priv.effect;
                break;
            }
        case V4L2_CID_ZOOM_ABSOLUTE:
            {
                ext_ctrl->value = sensor->info_priv.digitalzoom;
                break;
            }
        case V4L2_CID_ZOOM_RELATIVE:
            {
                return -EINVAL;
            }
        case V4L2_CID_FOCUS_ABSOLUTE:
            {
                ext_ctrl->value = sensor->info_priv.focus;
                break;
            }
        case V4L2_CID_FOCUS_RELATIVE:
            {
                return -EINVAL;
            }
        case V4L2_CID_FLASH:
            {
                ext_ctrl->value = sensor->info_priv.flash;
                break;
            }
        default :
            break;
    }
    return 0;
}
static int sensor_s_ext_control(struct soc_camera_device *icd, struct v4l2_ext_control *ext_ctrl)
{
    const struct v4l2_queryctrl *qctrl;
    struct i2c_client *client = to_i2c_client(to_soc_camera_control(icd));
    struct sensor *sensor = to_sensor(client);
    int val_offset;

    qctrl = soc_camera_find_qctrl(&sensor_ops, ext_ctrl->id);
    
    if (!qctrl)
    {
        SENSOR_TR("\n %s ioctrl id = %d  is invalidate \n", SENSOR_NAME_STRING(), ext_ctrl->id);
        return -EINVAL;
    }

	val_offset = 0;
    switch (ext_ctrl->id)
    {
#if CONFIG_SENSOR_Scene
        case V4L2_CID_SCENE:
            {
                if (ext_ctrl->value != sensor->info_priv.scene)
                {
                    if (sensor_set_scene(icd, qctrl,ext_ctrl->value) != 0)
                        return -EINVAL;
                    sensor->info_priv.scene = ext_ctrl->value;
                }
                break;
            }
#endif
#if CONFIG_SENSOR_Effect
        case V4L2_CID_EFFECT:
            {
                if (ext_ctrl->value != sensor->info_priv.effect)
                {
                    if (sensor_set_effect(icd, qctrl,ext_ctrl->value) != 0)
                        return -EINVAL;
                    sensor->info_priv.effect= ext_ctrl->value;
                }
                break;
            }
#endif
#if CONFIG_SENSOR_DigitalZoom
        case V4L2_CID_ZOOM_ABSOLUTE:
            {
                if ((ext_ctrl->value < qctrl->minimum) || (ext_ctrl->value > qctrl->maximum))
                    return -EINVAL;

                if (ext_ctrl->value != sensor->info_priv.digitalzoom)
                {
                    val_offset = ext_ctrl->value -sensor->info_priv.digitalzoom;

                    if (sensor_set_digitalzoom(icd, qctrl,&val_offset) != 0)
                        return -EINVAL;
                    sensor->info_priv.digitalzoom += val_offset;

                    SENSOR_DG("%s digitalzoom is %x\n",SENSOR_NAME_STRING(),  sensor->info_priv.digitalzoom);
                }

                break;
            }
        case V4L2_CID_ZOOM_RELATIVE:
            {
                if (ext_ctrl->value)
                {
                    if (sensor_set_digitalzoom(icd, qctrl,&ext_ctrl->value) != 0)
                        return -EINVAL;
                    sensor->info_priv.digitalzoom += ext_ctrl->value;

                    SENSOR_DG("%s digitalzoom is %x\n", SENSOR_NAME_STRING(), sensor->info_priv.digitalzoom);
                }
                break;
            }
#endif
#if CONFIG_SENSOR_Focus
        case V4L2_CID_FOCUS_ABSOLUTE:
            {
                if ((ext_ctrl->value < qctrl->minimum) || (ext_ctrl->value > qctrl->maximum))
                    return -EINVAL;

                if (ext_ctrl->value != sensor->info_priv.focus)
                {
                    val_offset = ext_ctrl->value -sensor->info_priv.focus;

                    sensor->info_priv.focus += val_offset;
                }

                break;
            }
        case V4L2_CID_FOCUS_RELATIVE:
            {
                if (ext_ctrl->value)
                {
                    sensor->info_priv.focus += ext_ctrl->value;

                    SENSOR_DG("%s focus is %x\n", SENSOR_NAME_STRING(), sensor->info_priv.focus);
                }
                break;
            }
#endif
#if CONFIG_SENSOR_Flash
        case V4L2_CID_FLASH:
            {
                if (sensor_set_flash(icd, qctrl,ext_ctrl->value) != 0)
                    return -EINVAL;
                sensor->info_priv.flash = ext_ctrl->value;

                SENSOR_DG("%s flash is %x\n",SENSOR_NAME_STRING(), sensor->info_priv.flash);
                break;
            }
#endif
        default:
            break;
    }

    return 0;
}

static int sensor_g_ext_controls(struct v4l2_subdev *sd, struct v4l2_ext_controls *ext_ctrl)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct soc_camera_device *icd = client->dev.platform_data;
    int i, error_cnt=0, error_idx=-1;


    for (i=0; i<ext_ctrl->count; i++) {
        if (sensor_g_ext_control(icd, &ext_ctrl->controls[i]) != 0) {
            error_cnt++;
            error_idx = i;
        }
    }

    if (error_cnt > 1)
        error_idx = ext_ctrl->count;

    if (error_idx != -1) {
        ext_ctrl->error_idx = error_idx;
        return -EINVAL;
    } else {
        return 0;
    }
}

static int sensor_s_ext_controls(struct v4l2_subdev *sd, struct v4l2_ext_controls *ext_ctrl)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct soc_camera_device *icd = client->dev.platform_data;
    int i, error_cnt=0, error_idx=-1;


    for (i=0; i<ext_ctrl->count; i++) {
        if (sensor_s_ext_control(icd, &ext_ctrl->controls[i]) != 0) {
            error_cnt++;
            error_idx = i;
        }
    }

    if (error_cnt > 1)
        error_idx = ext_ctrl->count;

    if (error_idx != -1) {
        ext_ctrl->error_idx = error_idx;
        return -EINVAL;
    } else {
        return 0;
    }
}

/* Interface active, can use i2c. If it fails, it can indeed mean, that
 * this wasn't our capture interface, so, we wait for the right one */
static int sensor_video_probe(struct soc_camera_device *icd,
			       struct i2c_client *client)
{
    char value;
    int ret,pid = 0;
    struct sensor *sensor = to_sensor(client);

    /* We must have a parent by now. And it cannot be a wrong one.
     * So this entire test is completely redundant. */

	printk("*** nt99250 sensor_video_probe() set power on\n");
#if 1    
    if (!icd->dev.parent ||
	    to_soc_camera_host(icd->dev.parent)->nr != icd->iface)
		return -ENODEV;

	if (sensor_ioctrl(icd, Sensor_PowerDown, 0) < 0) {
		ret = -ENODEV;
		goto sensor_video_probe_err;
	}
#endif

#if 0
    /* soft reset */
    ret = sensor_write(client, 0x3012, 0x80);
    if (ret != 0)
    {
        SENSOR_TR("soft reset %s failed\n",SENSOR_NAME_STRING());
        return -ENODEV;
	}
    mdelay(5);          //delay 5 microseconds
#endif

re:
    /* check if it is an sensor sensor */
    ret = sensor_read(client, 0x307e, &value);
    if (ret != 0) {
        SENSOR_TR("read chip id high byte failed\n");
        ret = -ENODEV;
        goto sensor_video_probe_err;
    }

    pid |= (value << 8);

    ret = sensor_read(client, 0x307f, &value);
    if (ret != 0) {
        SENSOR_TR("read chip id low byte failed\n");
        ret = -ENODEV;
        goto sensor_video_probe_err;
    }

    pid |= (value & 0xff);
    SENSOR_DG("\n %s  pid = 0x%x\n", SENSOR_NAME_STRING(), pid);
    if (pid == SENSOR_ID) {
        sensor->model = SENSOR_V4L2_IDENT;
    } else {
        SENSOR_TR("error: %s mismatched   pid = 0x%x\n", SENSOR_NAME_STRING(), pid);
        ret = -ENODEV;
        goto sensor_video_probe_err;
    }

    return 0;

sensor_video_probe_err:
    return ret;
}
static long sensor_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct soc_camera_device *icd = client->dev.platform_data;
    struct sensor *sensor = to_sensor(client);
    int ret = 0,i;
    
	SENSOR_DG("\n%s..%s..cmd:%x \n",SENSOR_NAME_STRING(),__FUNCTION__,cmd);
	switch (cmd)
	{
		case RK29_CAM_SUBDEV_DEACTIVATE:
		{
			sensor_deactivate(client);
			break;
		}

		case RK29_CAM_SUBDEV_IOREQUEST:
		{
			sensor->sensor_io_request = (struct rk29camera_platform_data*)arg;           
            if (sensor->sensor_io_request != NULL) { 
                if (sensor->sensor_io_request->gpio_res[0].dev_name && 
                    (strcmp(sensor->sensor_io_request->gpio_res[0].dev_name, dev_name(icd->pdev)) == 0)) {
                    sensor->sensor_gpio_res = (struct rk29camera_gpio_res*)&sensor->sensor_io_request->gpio_res[0];
                } else if (sensor->sensor_io_request->gpio_res[1].dev_name && 
                    (strcmp(sensor->sensor_io_request->gpio_res[1].dev_name, dev_name(icd->pdev)) == 0)) {
                    sensor->sensor_gpio_res = (struct rk29camera_gpio_res*)&sensor->sensor_io_request->gpio_res[1];
                }
            } else {
                SENSOR_TR("%s %s RK29_CAM_SUBDEV_IOREQUEST fail\n",SENSOR_NAME_STRING(),__FUNCTION__);
                ret = -EINVAL;
                goto sensor_ioctl_end;
            }
            /* ddl@rock-chips.com : if gpio_flash havn't been set in board-xxx.c, sensor driver must notify is not support flash control 
               for this project */
            #if CONFIG_SENSOR_Flash	
        	if (sensor->sensor_gpio_res) {
                if (sensor->sensor_gpio_res->gpio_flash == INVALID_GPIO) {
                    for (i = 0; i < icd->ops->num_controls; i++) {
                		if (V4L2_CID_FLASH == icd->ops->controls[i].id) {
                			memset(&icd->ops->controls[i],0x00,sizeof(struct v4l2_queryctrl));                			
                		}
                    }
                    sensor->info_priv.flash = 0xff;
                    SENSOR_DG("%s flash gpio is invalidate!\n",SENSOR_NAME_STRING());
                }
        	}
            #endif
			break;
		}
		default:
		{
			SENSOR_TR("%s %s cmd(0x%x) is unknown !\n",SENSOR_NAME_STRING(),__FUNCTION__,cmd);
			break;
		}
	}

sensor_ioctl_end:
	return ret;

}
static int sensor_enum_fmt(struct v4l2_subdev *sd, unsigned int index,
			    enum v4l2_mbus_pixelcode *code)
{
	if (index >= ARRAY_SIZE(sensor_colour_fmts))
		return -EINVAL;

	*code = sensor_colour_fmts[index].code;
	return 0;
}
static struct v4l2_subdev_core_ops sensor_subdev_core_ops = {
	.init		= sensor_init,
	.g_ctrl		= sensor_g_control,
	.s_ctrl		= sensor_s_control,
	.g_ext_ctrls          = sensor_g_ext_controls,
	.s_ext_ctrls          = sensor_s_ext_controls,
	.g_chip_ident	= sensor_g_chip_ident,
	.ioctl = sensor_ioctl,
};

static struct v4l2_subdev_video_ops sensor_subdev_video_ops = {
	.s_mbus_fmt	= sensor_s_fmt,
	.g_mbus_fmt	= sensor_g_fmt,
	.try_mbus_fmt	= sensor_try_fmt,
	.enum_mbus_fmt	= sensor_enum_fmt,
};

static struct v4l2_subdev_ops sensor_subdev_ops = {
	.core	= &sensor_subdev_core_ops,
	.video = &sensor_subdev_video_ops,
};


static u32 cur_reg=0;
static ssize_t nt99250_show(struct class *cls, char *_buf)
{
	return printk("%s()\n", __FUNCTION__);
}

static u32 strtol(const char *nptr, int base)
{
	u32 ret;
	if(!nptr || (base!=16 && base!=10 && base!=8))
	{
		printk("%s(): NULL pointer input\n", __FUNCTION__);
		return -1;
	}
	for(ret=0; *nptr; nptr++)
	{
		if((base==16 && *nptr>='A' && *nptr<='F') || 
			(base==16 && *nptr>='a' && *nptr<='f') || 
			(base>=10 && *nptr>='0' && *nptr<='9') ||
			(base>=8 && *nptr>='0' && *nptr<='7') )
		{
			ret *= base;
			if(base==16 && *nptr>='A' && *nptr<='F')
				ret += *nptr-'A'+10;
			else if(base==16 && *nptr>='a' && *nptr<='f')
				ret += *nptr-'a'+10;
			else if(base>=10 && *nptr>='0' && *nptr<='9')
				ret += *nptr-'0';
			else if(base>=8 && *nptr>='0' && *nptr<='7')
				ret += *nptr-'0';
		}
		else
			return ret;
	}
	return ret;
}

static ssize_t nt99250_store(struct class *cls, const char *_buf, size_t _count)
{
	const char * p=_buf;
	u16 reg;
	u8 val;

	if(!strncmp(_buf, "get", strlen("get")))
	{
		p+=strlen("get");
		cur_reg=(u32)strtol(p, 16);
		sensor_read(g_i2c_client, cur_reg, &val);
		printk("%s(): get 0x%04x=0x%02x\n", __FUNCTION__, cur_reg, val);
	}
	else if(!strncmp(_buf, "put", strlen("put")))
	{
		p+=strlen("put");
		reg=strtol(p, 16);
		p=strchr(_buf, '=');
		if(p)
		{
			++ p;
			val=strtol(p, 16);
			sensor_write(g_i2c_client, reg, val);
			printk("%s(): set 0x%04x=0x%02x\n", __FUNCTION__, reg, val);
		}
		else
			printk("%s(): Bad string format input!\n", __FUNCTION__);
	}
	else
		printk("%s(): Bad string format input!\n", __FUNCTION__);
	
	return _count;
} 


static struct class *nt99250_class = NULL;
static CLASS_ATTR(nt99250, 0666, nt99250_show, nt99250_store);
static int sensor_probe(struct i2c_client *client,
			 const struct i2c_device_id *did)
{
    struct sensor *sensor;
    struct soc_camera_device *icd = client->dev.platform_data;
    struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
    struct soc_camera_link *icl;
    int ret;

    SENSOR_DG("\n%s..%s..%d..\n",__FUNCTION__,__FILE__,__LINE__);
    if (!icd) {
        dev_err(&client->dev, "%s: missing soc-camera data!\n",SENSOR_NAME_STRING());
        return -EINVAL;
    }

    icl = to_soc_camera_link(icd);
    if (!icl) {
        dev_err(&client->dev, "%s driver needs platform data\n", SENSOR_NAME_STRING());
        return -EINVAL;
    }

    if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
        dev_warn(&adapter->dev,
        	 "I2C-Adapter doesn't support I2C_FUNC_I2C\n");
        return -EIO;
    }

    sensor = kzalloc(sizeof(struct sensor), GFP_KERNEL);
    if (!sensor)
        return -ENOMEM;

    v4l2_i2c_subdev_init(&sensor->subdev, client, &sensor_subdev_ops);

    /* Second stage probe - when a capture adapter is there */
    icd->ops		= &sensor_ops;
    sensor->info_priv.fmt = sensor_colour_fmts[0];

	#if CONFIG_SENSOR_I2C_NOSCHED
	atomic_set(&sensor->tasklock_cnt,0);
	#endif

	nt99250_class = class_create(THIS_MODULE, "nt99250");
	if (IS_ERR(nt99250_class)) 
	{
		printk("Create class nt99250.\n");
		return -ENOMEM;
	}
	class_create_file(nt99250_class,&class_attr_nt99250);

	g_i2c_client=client;

    ret = sensor_video_probe(icd, client);
    if (ret) {
        icd->ops = NULL;
        i2c_set_clientdata(client, NULL);
        kfree(sensor);
    }
    SENSOR_DG("\n%s..%s..%d  ret = %x \n",__FUNCTION__,__FILE__,__LINE__,ret);

#ifdef DYNA_CHG_ORIENT	
	set_cam_exist_flag(1);
#endif
    return ret;
}

static int sensor_remove(struct i2c_client *client)
{
    struct sensor *sensor = to_sensor(client);
    struct soc_camera_device *icd = client->dev.platform_data;

    icd->ops = NULL;
    i2c_set_clientdata(client, NULL);
    client->driver = NULL;
    kfree(sensor);

    return 0;
}

static const struct i2c_device_id sensor_id[] = {
	{SENSOR_NAME_STRING(), 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sensor_id);

static struct i2c_driver sensor_i2c_driver = {
	.driver = {
		.name = SENSOR_NAME_STRING(),
	},
	.probe		= sensor_probe,
	.remove		= sensor_remove,
	.id_table	= sensor_id,
};

static int __init sensor_mod_init(void)
{
    SENSOR_DG("\n%s..%s.. \n",__FUNCTION__,SENSOR_NAME_STRING());
#ifdef CONFIG_SOC_CAMERA_FCAM
    return 0;
#else
    return i2c_add_driver(&sensor_i2c_driver);
#endif
}

static void __exit sensor_mod_exit(void)
{
    i2c_del_driver(&sensor_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION(SENSOR_NAME_STRING(Camera sensor driver));
MODULE_AUTHOR("ddl <kernel@rock-chips>");
MODULE_LICENSE("GPL");

#if 1
struct cam_sensor_info cam_sensor_info_nt99250={
    "nt99250",
    2,
    1,
    0x6c>>1,
    0,
    0x307e, 
    0x01,
    0x307f,
    0x05,

    &sensor_ops,
    &sensor_subdev_ops,
    sensor_deactivate,
    sensor_read,
    sensor_write,
#ifdef DYNA_CHG_ORIENT
#ifdef CONFIG_SOC_CAMERA_FCAM
    nt99250_change_cam_orientation,
#else
    change_cam_orientation,
#endif
#else
    0,
#endif
    sensor_video_probe,
};

EXPORT_SYMBOL_GPL(cam_sensor_info_nt99250);
#endif
