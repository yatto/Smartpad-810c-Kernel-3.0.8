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
	printk(fmt , ## arg); } while (0)

#define SENSOR_TR(format, ...) printk(KERN_ERR format, ## __VA_ARGS__)
#define SENSOR_DG(format, ...) dprintk(1, format, ## __VA_ARGS__)


#define _CONS(a,b) a##b
#define CONS(a,b) _CONS(a,b)

#define __STR(x) #x
#define _STR(x) __STR(x)
#define STR(x) _STR(x)

#define MIN(x,y)   ((x<y) ? x: y)
#define MAX(x,y)    ((x>y) ? x: y)

/* Sensor Driver Configuration */
#define SENSOR_NAME bcam
#define SENSOR_V4L2_IDENT V4L2_IDENT_BCAM
#define SENSOR_ID 0x2656
#define SENSOR_MIN_WIDTH    176
#define SENSOR_MIN_HEIGHT   144
#define SENSOR_MAX_WIDTH    1600
#define SENSOR_MAX_HEIGHT   1200
#define SENSOR_INIT_WIDTH	800//640			/* Sensor pixel size for sensor_init_data array */
#define SENSOR_INIT_HEIGHT  600//480
#define SENSOR_INIT_WINSEQADR sensor_vga
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
#define CONFIG_SENSOR_Flash         0
#define CONFIG_SENSOR_Mirror        0
#define CONFIG_SENSOR_Flip          0

#define CONFIG_SENSOR_I2C_SPEED     100000       /* Hz */
/* Sensor write register continues by preempt_disable/preempt_enable for current process not be scheduled */
#define CONFIG_SENSOR_I2C_NOSCHED   0
#define CONFIG_SENSOR_I2C_RDWRCHK   0

#define SENSOR_BUS_PARAM  (SOCAM_MASTER | SOCAM_PCLK_SAMPLE_RISING |\
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
static u32 cur_reg=0;
static struct cam_sensor_info * pinfo=0;
static int sensor_init(struct v4l2_subdev *sd, u32 val)
{
    return 0;
}

struct sensor_datafmt {
	enum v4l2_mbus_pixelcode code;
	enum v4l2_colorspace colorspace;
};

static const struct sensor_datafmt sensor_colour_fmts[] = {
//	JPG_FMT(SENSOR_NAME_STRING(UYVY), 16, V4L2_PIX_FMT_UYVY),
//	JPG_FMT(SENSOR_NAME_STRING(YUYV), 16, V4L2_PIX_FMT_YUYV),
    {V4L2_MBUS_FMT_UYVY8_2X8, V4L2_COLORSPACE_JPEG},
    {V4L2_MBUS_FMT_YUYV8_2X8, V4L2_COLORSPACE_JPEG}	
};

static struct v4l2_subdev_core_ops sensor_subdev_core_ops = {
	.init		= sensor_init,
};
static struct v4l2_subdev_ops sensor_subdev_ops = {
	.core	= &sensor_subdev_core_ops,
};

struct reginfo
{
    u16 reg;
    u8 val;
};

static int sensor_probe(struct i2c_client *client, const struct i2c_device_id *did);
static int sensor_video_probe(struct soc_camera_device *icd, struct i2c_client *client);

static struct soc_camera_ops sensor_ops;

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

static struct sensor* to_sensor(const struct i2c_client *client)
{
    return container_of(i2c_get_clientdata(client), struct sensor, subdev);
}

/* sensor register write */
static int sensor_write(struct i2c_client *client, u16 reg, u8 val)
{
    return pinfo->sensor_write(client, reg, val);
}

/* sensor register read */
static int sensor_read(struct i2c_client *client, u16 reg, u8 *val)
{
    return pinfo->sensor_read(client, reg, val);
}

static int sensor_deactivate(struct i2c_client *client)
{
	struct soc_camera_device *icd = client->dev.platform_data;

    SENSOR_DG("XXX%s()\n",__FUNCTION__);
	SENSOR_DG("\n%s..%s.. Enter\n",SENSOR_NAME_STRING(),__FUNCTION__);

    return pinfo->sensor_deactivate(client);

}

 static int sensor_g_chip_ident(struct v4l2_subdev *sd, struct v4l2_dbg_chip_ident *id)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);

    SENSOR_DG("XXX%s()\n",__FUNCTION__);

    if (id->match.type != V4L2_CHIP_MATCH_I2C_ADDR)
        return -EINVAL;

    if (id->match.addr != client->addr)
        return -ENODEV;

    id->ident = SENSOR_V4L2_IDENT;
    id->revision = 0;

    return 0;
}

extern int i2c1_prober_verify(u32 dev_addr, u16 reg, u32 reg_addr_len, u32 reg_val_len, u32 id);
extern struct cam_sensor_info cam_sensor_info_nt99250_back;
extern struct cam_sensor_info  cam_sensor_info_gt2005_back;
extern struct cam_sensor_info  cam_sensor_info_ov5642;
extern struct cam_sensor_info  cam_sensor_info_ov5640;
extern struct cam_sensor_info  cam_sensor_info_ov5640forgt2005;
extern struct cam_sensor_info  cam_sensor_info_ov5642forgt2005;
extern struct cam_sensor_info cam_sensor_info_ov2655_back;
static struct cam_sensor_info * all_cam_info[]= {
    &cam_sensor_info_nt99250_back,
    &cam_sensor_info_ov2655_back,
    &cam_sensor_info_ov5642,
    &cam_sensor_info_ov5640forgt2005,
    &cam_sensor_info_ov5640,
    &cam_sensor_info_ov5642forgt2005,
    &cam_sensor_info_gt2005_back
};

/* Interface active, can use i2c. If it fails, it can indeed mean, that
 * this wasn't our capture interface, so, we wait for the right one */
static int sensor_video_probe(struct soc_camera_device *icd,
			       struct i2c_client *client)
{
    char value;
    int ret,pid = 0, i;
    struct sensor *sensor = to_sensor(client);

    printk("%s %s\n", __FILE__, __func__);
    for(i=0; i<ARRAY_SIZE(all_cam_info); i++) {
        pinfo=all_cam_info[i];

        if(!pinfo->sensor_video_probe) {
            SENSOR_DG("XXX %s(): probe failed for %s\n\n", __func__, pinfo->pname);
            continue;
        }
        client->addr=pinfo->slave_addr;
        memcpy(&sensor_ops, pinfo->p_sensor_ops, sizeof(struct soc_camera_ops));
        memcpy(&sensor_subdev_ops, pinfo->p_sensor_subdev_ops, sizeof(struct v4l2_subdev_ops));
        ret=pinfo->sensor_video_probe(icd, client);
        if(!ret) {
            SENSOR_DG("OOO %s(): probe OK for %s\n\n", __func__, pinfo->pname);
            return 0;
        }
        else {
            SENSOR_DG("XXX %s(): probe failed for %s\n\n", __func__, pinfo->pname);
            continue;
        }

    }
    pinfo=0;
    SENSOR_DG("No cam found in back\n", __func__);
    return -ENODEV;
}

static long sensor_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct soc_camera_device *icd = client->dev.platform_data;  
    struct sensor *sensor = to_sensor(client);
    int ret = 0,i;
    
    SENSOR_DG("XXX%s()\n",__FUNCTION__);
        
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

static struct v4l2_subdev_video_ops sensor_subdev_video_ops;
static ssize_t bcam_show(struct device *dev, struct device_attribute *attr, char *_buf)
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

static ssize_t bcam_store(struct device *dev,
					struct device_attribute *attr,
					const char *_buf, size_t _count)
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

static ssize_t bname_show(struct device *dev, struct device_attribute *attr, char *_buf)
{
    if(pinfo) {
        strcpy(_buf, pinfo->pname);
        return strlen(pinfo->pname);
    }
    strcpy(_buf, "none");
    return 4;
}

static struct device *bcam_dev = NULL;
static struct class *  bcam_class = NULL;
static DEVICE_ATTR(bcam, 0666, bcam_show, bcam_store);
static DEVICE_ATTR(name, 0666, bname_show, NULL);
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

	bcam_class = class_create(THIS_MODULE, "bcam");
	if (IS_ERR(bcam_class)) 
	{
		printk("Create class bcam.\n");
		return -ENOMEM;
	}
	bcam_dev = device_create(bcam_class, NULL, MKDEV(0, 1), NULL, "dev");
	device_create_file(bcam_dev, &dev_attr_bcam);
	device_create_file(bcam_dev, &dev_attr_name);

    g_i2c_client=client;

    ret = sensor_video_probe(icd, client);
    if (ret) {
        icd->ops = NULL;
        i2c_set_clientdata(client, NULL);
        kfree(sensor);
    }
    SENSOR_DG("\n%s..%s..%d  ret = %x \n",__FUNCTION__,__FILE__,__LINE__,ret);
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
    printk("%s %s\n", __FILE__, __func__);
    return i2c_add_driver(&sensor_i2c_driver);
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

