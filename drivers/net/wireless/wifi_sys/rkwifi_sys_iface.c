
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/err.h>

extern int rockchip_wifi_init_module(void);
extern void rockchip_wifi_exit_module(void);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 37))
static ssize_t wifi_chip_read(struct class *cls, struct class_attribute *attr, char *_buf)
#else
static ssize_t wifi_chip_read(struct class *cls, char *_buf)
#endif
{
    int count;

#ifdef CONFIG_BCM4329
    count = sprintf(_buf, "%s", "BCM4329");
    printk("Current WiFi chip is BCM4329.\n");
#endif

#ifdef CONFIG_RTL8192CU
    count = sprintf(_buf, "%s", "RTL8188");
    printk("Current WiFi chip is RTL8188.\n");
#endif
    
    return count;
}

/*
static ssize_t wifi_channel_write(struct class *cls, const char *_buf, size_t _count)
{
    int ret, channel;
    
    if (wifi_enabled == 0)
    {
        printk("WiFi is disabled.\n");
        return _count;
    }
    
    channel = simple_strtol(_buf, NULL, 10);
    
    ret = wifi_emi_set_channel(channel);
    if (ret != 0)
    {
        //printk("Set channel=%d fail.\n", channel);
    }
    else
    {
        //printk("Set channel=%d successfully.\n", channel);
        wifi_channel = channel;
    }
    
    return _count;
}
*/

static struct class *rkwifi_class = NULL;
static CLASS_ATTR(chip, 0666, wifi_chip_read, NULL);

static ssize_t wifi_read(struct class *cls, struct class_attribute *attr, char *_buf)
{
	int ret;
	int count;
	ret = rockchip_wifi_init_module();
	//      count = sprintf(_buf,"wifi value = %d", ret);
	if(ret >=0)
	{
		count = sprintf(_buf,"%s","sucessed\n");
		printk("sucessed!\n");
	}
	else
	{
		count = sprintf(_buf,"%s","failed\n");
		printk("failed!\n");
	}

	return count;
}
 
static CLASS_ATTR(wifi, 0666, wifi_read,NULL); 

int rkwifi_sysif_init(void)
{
    int ret;
    
    printk("Rockchip WiFi SYS interface (V1.00) ... \n");
    
    rkwifi_class = NULL;
    
    rkwifi_class = class_create(THIS_MODULE, "rkwifi");
    if (IS_ERR(rkwifi_class)) 
    {   
        printk("Create class rkwifi_class failed.\n");
        return -ENOMEM;
    }
    
    ret =  class_create_file(rkwifi_class, &class_attr_chip);
    ret =  class_create_file(rkwifi_class, &class_attr_wifi);
    
    return 0;
}

void rkwifi_sysif_exit(void)
{
    // need to remove the sys files and class
    class_remove_file(rkwifi_class, &class_attr_chip);
    class_remove_file(rkwifi_class, &class_attr_wifi);
    class_destroy(rkwifi_class);
    
    rkwifi_class = NULL;
}

module_init(rkwifi_sysif_init);
module_exit(rkwifi_sysif_exit);

MODULE_AUTHOR("Yongle Lai");
MODULE_DESCRIPTION("WiFi SYS @ Rockchip");
MODULE_LICENSE("GPL");

