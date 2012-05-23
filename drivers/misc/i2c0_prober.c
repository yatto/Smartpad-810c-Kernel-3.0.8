
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <mach/gpio.h>
#include <mach/irqs.h>
#include <mach/rk29_iomap.h>

#include <mach/board.h>
#include <mach/gpio.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#define DBG(fmt, args...)	printk(KERN_INFO "*** " fmt, ## args)

static u32 slave_addr=0, reg_size=1, i2c_speed=100000, try_num=3, reg_val_size=1;

static struct i2c_client * g_i2c_client;

#if 1
static int i2c0_prober_read_inter(u16 reg, u8 *val)
{
	struct i2c_client *client=g_i2c_client;
    int err,cnt;
    u8 buf[2];
	char reg_buf[2];
    struct i2c_msg msg[2];

    if(reg_size==1) {
        reg_buf[0] = reg;
    }
    else {
        reg_buf[0] = reg >> 8;
        reg_buf[1] = reg & 0xFF;
    }

    msg[0].addr = slave_addr;
    msg[0].flags = client->flags;
    msg[0].buf = &reg_buf;
    msg[0].len = reg_size;
    msg[0].scl_rate = i2c_speed;
    msg[0].read_type = 2;				/* fpga i2c:0==I2C_NO_STOP : direct use number not enum for don't want include spi_fpga.h */

    msg[1].addr = slave_addr;
    msg[1].flags = client->flags|I2C_M_RD;
    msg[1].buf = buf;
    msg[1].len = reg_val_size;
    msg[1].scl_rate = i2c_speed;
    msg[1].read_type = 2;				/* fpga i2c:0==I2C_NO_STOP : direct use number not enum for don't want include spi_fpga.h */

    cnt = try_num;
    err = -ENODEV;
    while ((cnt-- > 0) && (err < 0)) {
        err = i2c_transfer(client->adapter, msg, 2);

        if (err >= 0) {
            *val = buf[0];
			*(val+1)=buf[1];
            return 0;
        } else {
        	DBG("%s():Read failed -> reg(0x%04x val[0]:0x%02x, val[1]:0x%02x)\n", __func__, reg, val[0], val[1]);
            udelay(10);
        }
    }

    return err;
}

int i2c0_prober_read(u32 dev_addr, u16 reg, u8 *val, u32 reg_addr_len, u32 reg_val_len)
{
	slave_addr=dev_addr;
	reg_size=reg_addr_len;
	reg_val_size=reg_val_len;
	return i2c0_prober_read_inter(reg, val);
}
EXPORT_SYMBOL(i2c0_prober_read);

int i2c0_prober_verify(u32 dev_addr, u16 reg, u32 reg_addr_len, u32 reg_val_len, u32 id)
{
	u8 val[2];
	u32 read_id;
	int ret=i2c0_prober_read(dev_addr, reg, val, reg_addr_len, reg_val_len);
	if(!ret) {
        if(reg_val_len==1)
            read_id=val[0];
        else
		read_id=(val[0]<<8)|val[1];
		DBG("%s(): TARGET ID=0x%04x, READ ID=0x%04x\n", __FUNCTION__, id, read_id, id==read_id?"OK":"FAIL");
		ret = id==read_id?0:-ENODEV;
	}
	return ret;
}
EXPORT_SYMBOL(i2c0_prober_verify);
#endif

static ssize_t i2c0_prober_show(struct class *cls, char *_buf)
{
    char buf[80];
    sprintf(buf, "%s(): %s %s\n", __FUNCTION__, __DATE__, __TIME__);
	printk("%s(): %s \n", __FUNCTION__, buf);
    strcpy(_buf, buf);
	return strlen(buf);
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

static ssize_t i2c0_prober_store(struct class *cls, const char *_buf, size_t _count)
{
	const char * p=_buf;
	u32 reg, val;
	printk("%s()\n", __FUNCTION__);
	
	return _count;
} 

static ssize_t i2c0_prober_store_slave_addr(struct class *cls, const char *_buf, size_t _count)
{
	slave_addr=strtol(_buf, 16);
	printk("%s(): slave_addr=0x%02x\n", __FUNCTION__, slave_addr);
	return _count;
} 

static ssize_t i2c0_prober_show_slave_addr(struct class *cls, char *_buf)
{
	printk("%s(): slave_addr=0x%02x\n", __FUNCTION__, slave_addr);
	sprintf(_buf, "%02x", slave_addr);
	return strlen(_buf);
}

static struct class *i2c0_prober_class = NULL;
static CLASS_ATTR(i2c0_prober, 0666, i2c0_prober_show, i2c0_prober_store);
static CLASS_ATTR(slave_addr, 0666, i2c0_prober_show_slave_addr, i2c0_prober_store_slave_addr);
static __devinit int i2c0_prober_i2c_probe(struct i2c_client *i2c,
				      const struct i2c_device_id *id)
{
	g_i2c_client=i2c;	
	
	i2c0_prober_class = class_create(THIS_MODULE, "i2c0_prober");
	if (IS_ERR(i2c0_prober_class)) 
	{
		printk("Create class i2c0_prober.\n");
		return -ENOMEM;
	}
	class_create_file(i2c0_prober_class,&class_attr_i2c0_prober);
	class_create_file(i2c0_prober_class,&class_attr_slave_addr);


	DBG("%s(): addr=0x%02x\n", __FUNCTION__, g_i2c_client->addr);

	return 0;
}

static __devexit int i2c0_prober_i2c_remove(struct i2c_client *client)
{
	return 0;
}

void i2c0_prober_i2c_shutdown(struct i2c_client *client)
{
}

#ifdef CONFIG_PM
static int i2c0_prober_i2c_suspend(struct i2c_client *client, pm_message_t msg)
{
	return 0;
}

static int i2c0_prober_i2c_resume(struct i2c_client *client)
{
	return 0;
}
#else
#define i2c0_prober_i2c_suspend NULL
#define i2c0_prober_i2c_resume NULL
#endif

static const struct i2c_device_id i2c0_prober_i2c_id[] = {
	{ "i2c0_prober", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, i2c0_prober_i2c_id);
static struct i2c_driver i2c0_prober_i2c_driver = {
	.driver = {
		.name = "i2c0_prober",
		.owner = THIS_MODULE,
	},
	.probe = i2c0_prober_i2c_probe,
	.remove = __devexit_p(i2c0_prober_i2c_remove),
	.suspend = i2c0_prober_i2c_suspend,
	.resume = i2c0_prober_i2c_resume,
	.shutdown = i2c0_prober_i2c_shutdown,
	.id_table = i2c0_prober_i2c_id,
};

static int __init i2c0_prober_modinit(void)
{
//	return 0;
	return  i2c_add_driver(&i2c0_prober_i2c_driver);
}
module_init(i2c0_prober_modinit);

static void __exit i2c0_prober_exit(void)
{
	i2c_del_driver(&i2c0_prober_i2c_driver);
}
module_exit(i2c0_prober_exit);

MODULE_DESCRIPTION("ASoC i2c0_prober driver");
MODULE_AUTHOR("Mark Brown <broonie@opensource.wolfonmicro.com>");
MODULE_LICENSE("GPL");

