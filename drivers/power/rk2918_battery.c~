/* drivers/power/rk2918_battery.c
 *
 * battery detect driver for the rk2918 
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <mach/gpio.h>
#include <linux/adc.h>
#include <mach/iomux.h>
#include <mach/board.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/slab.h>

#if 0
#define DBG(x...)   printk(x)
#else
#define DBG(x...)
#endif


int rk29_battery_dbg_level = 0;

#define adc_to_voltage(adc_val) ((adc_val * BAT_2V5_VALUE * (BAT_PULL_UP_R + BAT_PULL_DOWN_R)) / (1024 * BAT_PULL_DOWN_R))

#define BAT_ADC_TABLE_LEN       11
static int adc_raw_table_bat[BAT_ADC_TABLE_LEN] = 
{
#if defined(CONFIG_MACH_RK29_ACH7)
	3500, 3579, 3649, 3676, 3694, 3731, 3789, 3856, 3927, 4007, 4150
#elif defined(CONFIG_MACH_RK29_ACH8)
    3500, 3565, 3620, 3650, 3675, 3706, 3760, 3820, 3880, 3963, 4050
#elif defined(CONFIG_MACH_M1005HN) || defined(CONFIG_MACH_M1005HNW) || defined(CONFIG_MACH_M1005HD) || defined(CONFIG_MACH_M1006HD) || defined(CONFIG_MACH_M1007HD)
	7000, 7286, 7405, 7453, 7525, 7606, 7710, 7855, 7977, 8115, 8220
#elif defined (CONFIG_MACH_M727) || defined (CONFIG_MACH_M722HCN) || defined (CONFIG_MACH_M727HCN) || defined (CONFIG_MACH_M723HR)
	3500, 3602, 3665, 3686, 3710, 3745, 3800, 3865, 3932, 4005, 4060
#elif defined (CONFIG_MACH_M732) || defined (CONFIG_MACH_M732HCN)
	3500, 3578, 3635, 3653, 3673, 3709, 3765, 3829, 3898, 3980, 4065
#elif defined (CONFIG_MACH_M722)
	3500, 3625, 3675, 3708, 3728, 3748, 3795, 3855, 3915, 3990, 4080
#elif defined(CONFIG_MACH_M908HW) || defined(CONFIG_MACH_M908HD)
	3490, 3585, 3662, 3685, 3700, 3736, 3792, 3860, 3925, 4010, 4070
#elif defined (CONFIG_MACH_M803HC) || defined (CONFIG_MACH_M803HD)
	3500, 3650, 3715, 3738, 3765, 3805, 3857, 3923, 3977, 4030, 4120
#elif defined (CONFIG_MACH_M907HC) || defined (CONFIG_MACH_M907HD)
	3500, 3615, 3660, 3685, 3725, 3762, 3815, 3880, 3940, 4000, 4070
#elif defined (CONFIG_MACH_M735)
	3500, 3545, 3576, 3615, 3660, 3715, 3770, 3830, 3870, 3900, 4035
#elif defined (CONFIG_MACH_M726)
	3500, 3596, 3647, 3676, 3696, 3730, 3780, 3846, 3916, 3990, 4065
#elif defined (CONFIG_MACH_M805HC) || defined (CONFIG_MACH_M805HCX)|| defined(CONFIG_MACH_M805HD) 
	3500, 3610, 3670, 3700, 3730, 3772, 3813, 3872, 3931, 3982, 4050
#elif defined (CONFIG_MACH_M805HCZ) 
	3500, 3647, 3707, 3733, 3759, 3795, 3848, 3919, 3975, 4025, 4100
#else
    3490, 3597, 3628, 3641, 3660, 3697, 3747, 3809, 3879, 3945, 4165
#endif
};

static int adc_raw_table_ac[BAT_ADC_TABLE_LEN] = 
{
#if defined(CONFIG_MACH_RK29_ACH7)
	//3760, 3886, 3964, 3989, 4020, 4062, 4123, 4180, 4189, 4190, 4190      //实际测量值
	3691, 3760, 3800, 3827, 3845, 3885, 3950, 4007, 4078, 4158, 4300//4185//4301
#elif defined(CONFIG_MACH_RK29_ACH8)
    3500, 3565, 3620, 3650, 3675, 3706, 3760, 3820, 3880, 3963, 4050//4185//4301
#elif defined(CONFIG_MACH_M1005HN) || defined(CONFIG_MACH_M1005HNW) || defined(CONFIG_MACH_M1005HD) || defined(CONFIG_MACH_M1006HD) || defined(CONFIG_MACH_M1007HD)
	7320, 7565, 7665, 7715, 7785, 7865, 7960, 8090, 8200, 8340, 8460
#elif defined (CONFIG_MACH_M727) || defined (CONFIG_MACH_M722HCN) || defined (CONFIG_MACH_M727HCN) || defined (CONFIG_MACH_M723HR)
	3705, 3780, 3824, 3854, 3875, 3910, 3954, 4020, 4085, 4160, 4210
#elif defined (CONFIG_MACH_M732) || defined (CONFIG_MACH_M732HCN)
	3710, 3780, 3845, 3885, 3900, 3925, 3975, 4050, 4050, 4170, 4240
#elif defined (CONFIG_MACH_M722)
	3710, 3825, 3876, 3910, 3930, 3950, 3995, 4050, 4110, 4180, 4250
#elif defined(CONFIG_MACH_M908HW) || defined(CONFIG_MACH_M908HD)
	3680, 3795, 3875, 3895, 3910, 3946, 3996, 4065, 4115, 4190, 4240
#elif defined (CONFIG_MACH_M803HC) || defined (CONFIG_MACH_M803HD)
	3650, 3795, 3857, 3880, 3910, 3950, 3998, 4060, 4110, 4160, 4240
#elif defined (CONFIG_MACH_M907HC) || defined (CONFIG_MACH_M907HD)
	3730, 3835, 3880, 3900, 3935, 3970, 4020, 4085, 4130, 4190, 4250
#elif defined (CONFIG_MACH_M735)
	3690, 3714, 3740, 3771, 3805, 3855, 3915, 3940, 3995, 4027, 4080
#elif defined (CONFIG_MACH_M726)
	3710, 3795, 3840, 3868, 3887, 3925, 3970, 4030, 4100, 4160, 4220
#elif defined (CONFIG_MACH_M805HC) || defined (CONFIG_MACH_M805HCX) || defined(CONFIG_MACH_M805HD) 
	3740, 3840, 3890, 3920, 3950, 3985, 4020, 4080, 4125, 4170, 4240
#elif defined (CONFIG_MACH_M805HCZ) 
	3660, 3802, 3860, 3888, 3914, 3950, 4004, 4070, 4125, 4175, 4250
#else
    3600, 3760, 3800, 3827, 3845, 3885, 3950, 4007, 4078, 4140, 4200//4185//4301
#endif
};

static int shutdownvoltage = SHUTDOWNVOLTAGE;
static int gBatFullFlag =  0;

static int gBatLastStatus = 0;
static int gBatStatus =  POWER_SUPPLY_STATUS_UNKNOWN;
static int gBatHealth = POWER_SUPPLY_HEALTH_GOOD;
static int gBatLastPresent = 0;
static int gBatPresent = 1;
static int gBatLastVoltage =  0;
static int gBatVoltage =  BATT_NOMAL_VOL_VALUE;
static int gBatLastCapacity = 0;
static int gBatLastChargeCapacity = 100;          //记录充电时的电池容量
static int gBatCapacity = ((BATT_NOMAL_VOL_VALUE-BATT_ZERO_VOL_VALUE)*100/(BATT_MAX_VOL_VALUE-BATT_ZERO_VOL_VALUE));
static int gLastBatCapacity = ((BATT_NOMAL_VOL_VALUE-BATT_ZERO_VOL_VALUE)*100/(BATT_MAX_VOL_VALUE-BATT_ZERO_VOL_VALUE));
static int gBatStatusChangeCnt = 0;
static int gBatCapacityUpCnt = 0;
static int gBatCapacityDownCnt = 0;
static int gBatHighCapacityChargeCnt = 0;
static int gBatUsbChargeFlag = 0;
static int gBatUsbChargeCnt = 0;
static int shutdownflag = 0;

static int gBatVoltageSamples[NUM_VOLTAGE_SAMPLE+2]; //add 2 to handle one bug
static int gBatSlopeValue = 0;
static int gBatVoltageValue[2]={0,0};
static int *pSamples = &gBatVoltageSamples[0];		//采样点指针
static int gFlagLoop = 0;		//采样足够标志
//static int gNumSamples = 0;
static int gNumCharge = 0;
static int gMaxCharge = 0;
static int gNumLoader = 0;
static int gMaxLoader = 0;

static struct regulator *pChargeregulator;
static int gBatChargeStatus = 0;
static int openfailflag = 0;
static uint8_t openfailcount = 20;  //  文件打开失败的话，重复次数

static int last_BatChargeStatus = 0;

extern int dwc_vbus_status(void);
extern int get_msc_connect_flag(void);

struct rk2918_battery_data {
	int irq;
	spinlock_t lock;
	
	struct delayed_work	work;
	struct workqueue_struct *wq;
	
	struct work_struct 	timer_work;
	struct timer_list timer;
	struct power_supply battery;
	
#ifdef RK29_USB_CHARGE_SUPPORT
	struct power_supply usb;
#endif
	struct power_supply ac;
    
    int dc_det_pin;
    int batt_low_pin;
    int charge_set_pin;
	int charge_ok_pin;

    int dc_det_level;
    int batt_low_level;
    int charge_set_level;
	int charge_ok_level;
	int charge_cur_ctl;
	int charge_cur_ctl_level;
	
    int dc_det_irq;

	int adc_bat_divider;
	int bat_max;
	int bat_min;
	int adc_val;
	
	int full_times;
	
	struct adc_client *client; 
};


/* temporary variable used between rk2918_battery_probe() and rk2918_battery_open() */
static struct rk2918_battery_data *gBatteryData;

enum {
	BATTERY_STATUS          = 0,
	BATTERY_HEALTH          = 1,
	BATTERY_PRESENT         = 2,
	BATTERY_CAPACITY        = 3,
	BATTERY_AC_ONLINE       = 4,
	BATTERY_STATUS_CHANGED	= 5,
	AC_STATUS_CHANGED   	= 6,
	BATTERY_INT_STATUS	    = 7,
	BATTERY_INT_ENABLE	    = 8,
};

typedef enum {
	CHARGER_BATTERY = 0,
	CHARGER_USB,
	CHARGER_AC
} charger_type_t;

static struct device *battery_dev = NULL;

#define BATT_FILENAME "/data/bat_last_capacity.dat"
#include <linux/fs.h>

static void rk2918_batscan_timer(unsigned long data);

static ssize_t rk2918_battery_startget_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t len)
{
	static int only_one = 0;

	if (only_one != 0)
	{
		return len;
	}
	only_one = 1;
	
	if ((*buf=='A'))
	{
		rk2918_batscan_timer(NULL);
	}
	else
	{
		printk("%s error %s %d",__func__,buf,len);
		rk2918_batscan_timer(NULL);
	}

	return len;

}

static DEVICE_ATTR(startget,0666,NULL,rk2918_battery_startget_store);

static int rk2918_get_bat_capacity_raw(int BatVoltage);
int lastlost = 0;
static int rk2918_battery_load_capacity(void)
{
    int i;
    int tmp = 0;
	int  loadcapacity = 0;
	int  truecapacity = 0;
    char value[11];
    static char lastthree[6]= {0};
	char* p = value;
    struct file* fp = filp_open(BATT_FILENAME,O_RDONLY,0);
    
    //get true capacity
    for (i = 0; i < 20; i++)
    {
        tmp += adc_to_voltage(adc_sync_read(gBatteryData->client));
        mdelay(1);
    }
    tmp = tmp / 20;

	lastlost = tmp;
    truecapacity = rk2918_get_bat_capacity_raw(tmp);
    
	if(IS_ERR(fp))
    {
		printk("bryan---->open file /data/bat_last_capacity.dat failed\n");
		printk("truecapacity = %d\n", truecapacity);
		if(truecapacity>=100)
			truecapacity = 100;
	 	if(truecapacity==0)
			truecapacity=1;
		openfailflag = 1;

		if (openfailcount <= 5)
		{
			lastthree[openfailcount-1] = truecapacity;

			if (openfailcount == 1)
			{
				tmp = 0;
				for (i=0;i<5;i++)
				{
					tmp += lastthree[4-i];	
					printk("%s...............%d\n",__func__,tmp);
				}

				truecapacity = tmp/5;
				printk("%s...............%d\n",__func__,tmp);

			}
		}
		return truecapacity;
	}
	else
	{
		openfailflag = 0;
		openfailcount = 0;
	}
	kernel_read(fp,0,value,10);
    filp_close(fp,NULL);

	value[10]=0;
	while(*p)
	{
	    if(*p==0x0d)
	    {
			*p=0;
			break;
		}
		p++;
	}	
	sscanf(value,"%d",&loadcapacity);
	printk("bryan---->loadcapacity = %d, truecapacity = %d\n",loadcapacity, truecapacity);
	
    if ((loadcapacity <= 0) || (loadcapacity > 100))
	{
	    loadcapacity = truecapacity;
	}
	//如果从文件中读取的电压比实际的高很多的话，说明是长时间放置导致放电
	if (loadcapacity > truecapacity)
	{
	    if (loadcapacity - truecapacity > 20)
	    {
	        loadcapacity = truecapacity;
	    }
	}
	else
	{
		if ( ((truecapacity-loadcapacity) >= 20))
		{

			if (truecapacity < 30)
			{
				if (loadcapacity < 10)
				{
					loadcapacity = truecapacity/2;
				}
			}
			else
			{

				loadcapacity = truecapacity;	
				
			}

		}
	}
	    
	
	if (loadcapacity <= 0)
	{
		loadcapacity = 1;
	}
	if (loadcapacity >= 100)
	{
		loadcapacity = 100;
	}
	return loadcapacity;
}

static void rk2918_charge_enable(void)
{
    if (gBatteryData->charge_set_pin != INVALID_GPIO)
    {
        gpio_direction_output(gBatteryData->charge_set_pin, gBatteryData->charge_set_level);
    }
}

static void rk2918_charge_disable(void)
{
    if (gBatteryData->charge_set_pin != INVALID_GPIO)
    {
        gpio_direction_output(gBatteryData->charge_set_pin, 1 - gBatteryData->charge_set_level);
    }
}

extern int suspend_flag;
static void rk2918_get_charge_status(void)
{
    int charge_on = 0;
    
    if (gBatteryData->dc_det_pin != INVALID_GPIO)
    {
        if (gpio_get_value (gBatteryData->dc_det_pin) == gBatteryData->dc_det_level)
        {
            charge_on = 1;
        }
    }
    
#ifdef RK29_USB_CHARGE_SUPPORT
    if (charge_on == 0)
    {
        if (suspend_flag) return;
            
        if (1 == dwc_vbus_status())         //检测到USB插入，但是无法识别是否是充电器
        {                                   //通过延时检测PC识别标志，如果超时检测不到，说明是充电
            if (0 == get_msc_connect_flag())
            {                               //插入充电器时间大于一定时间之后，开始进入充电状态
                if (++gBatUsbChargeCnt >= NUM_USBCHARGE_IDENTIFY_TIMES)
                {
                    gBatUsbChargeCnt = NUM_USBCHARGE_IDENTIFY_TIMES + 1;
                    charge_on = 1;
                }
            }                               //否则，不进入充电模式
            #if defined(CONFIG_MACH_RK29_ACH8)
            charge_on = 1;
            #endif
        }                   
        else
        {
            gBatUsbChargeCnt = 0;
            if (2 == dwc_vbus_status()) 
            {
                charge_on = 1;
            }
        }
    }
#endif
        
    if (charge_on)
    {
        if(gBatChargeStatus !=1) 
        {            
            gBatChargeStatus = 1;
            gBatStatusChangeCnt = 0;        //状态变化开始计数
            rk2918_charge_enable();
        }
    } 
    else 
    {
        if(gBatChargeStatus != 0) 
        {
            gBatChargeStatus = 0;
            gBatStatusChangeCnt = 0;        //状态变化开始计数
            rk2918_charge_disable();
        }
    }
}

static void rk2918_get_bat_status(struct rk2918_battery_data *bat)
{
    rk2918_get_charge_status();
    
	if(gBatChargeStatus == 1)
	{
        if (gBatteryData->charge_ok_pin == INVALID_GPIO)
        {
            printk("dc_det_pin invalid!\n");
            return;
        }
    
		if (gBatStatus == POWER_SUPPLY_STATUS_FULL)
		{
			return;
		}
	    if ((gpio_get_value(gBatteryData->charge_ok_pin) == gBatteryData->charge_ok_level) && (gBatCapacity >= 80))
        {
            gBatteryData->full_times++;
            if (gBatteryData->full_times >= NUM_CHARGE_FULL_DELAY_TIMES)
		    {
		        gBatStatus = POWER_SUPPLY_STATUS_FULL;
		        gBatteryData->full_times = NUM_CHARGE_FULL_DELAY_TIMES + 1;
		    }
		    else
		    {
		        gBatStatus = POWER_SUPPLY_STATUS_CHARGING;
		    }
	    }
	    else
	    {
	        gBatStatus = POWER_SUPPLY_STATUS_CHARGING;
	        gBatteryData->full_times = 0;
	    }
	}
	else 
    {
	    gBatteryData->full_times = 0;
        gBatStatus = POWER_SUPPLY_STATUS_NOT_CHARGING;
	}
}

static void rk2918_get_bat_health(struct rk2918_battery_data *bat)
{
	gBatHealth = POWER_SUPPLY_HEALTH_GOOD;
}

static void rk2918_get_bat_present(struct rk2918_battery_data *bat)
{
	if(gBatVoltage < bat->bat_min)
	gBatPresent = 0;
	else
	gBatPresent = 1;
}

int capacitytmp = 0;
static int rk2918_get_bat_capacity_raw(int BatVoltage)
{
    int i = 0;
	int capacity = 0;
	int *p = adc_raw_table_bat;
    
    if (gBatChargeStatus == 1)
    {
        p = adc_raw_table_ac;
    }
	
	if(BatVoltage >= p[BAT_ADC_TABLE_LEN - 1])
	{
	    //当电压超过最大值
	    capacity = 100;
	}	
	else if(BatVoltage <= p[0])
	{
	    //当电压低于最小值
	    capacity = 0;
	}
	else
	{
    	//计算容量
    	for(i = 0; i < BAT_ADC_TABLE_LEN - 1; i++)
        {
    		
    		if((p[i] <= BatVoltage) && (BatVoltage < p[i+1]))
    		{
    			capacity = i * 10 + ((BatVoltage - p[i]) * 10) / (p[i+1] - p[i]);
    			break;
    		}
    	}
    }  
    return capacity;
}

s32 batteryspendcnt = 0;
unsigned long  last_batteryspendcnt = 0;

static int rk2918_battery_resume_get_Capacity(int deltatime)
{
	int i;
    int tmp = 0;
    int capacity = 0;

	for (i = 0; i < 20; i++)
    {
        tmp += adc_sync_read(gBatteryData->client);
        mdelay(1);
    }
    tmp = tmp / 20;
    //tmp = (tmp * BAT_2V5_VALUE * (BAT_PULL_UP_R + BAT_PULL_DOWN_R)) / (1024 * BAT_PULL_DOWN_R);
    tmp = adc_to_voltage(tmp);
    capacity = rk2918_get_bat_capacity_raw(tmp);
    //if last status is charging FULL, will return 100
	if(last_BatChargeStatus == POWER_SUPPLY_STATUS_FULL)
		return 100;
	//if last status is charging , and now still charging
    if ((gBatChargeStatus == 1) && (last_BatChargeStatus == 1))
    {
    	/*
        if (deltatime > (100 - gBatCapacity) * CHARGE_MIN_SECOND)
            deltatime = (100 - gBatCapacity) * CHARGE_MIN_SECOND;
        if (capacity > gBatCapacity + (deltatime / CHARGE_MIN_SECOND))       //采样电池容量偏差较大，将容量拉回
        {
            capacity = gBatCapacity + (deltatime / CHARGE_MIN_SECOND);
        }
		*/
		//
		if((gBatCapacity > 80))
		{
			capacity = gBatCapacity + deltatime/CHARGE_MID_SECOND;
		}
		else
		{
			/*some battery cannot arrive to 4.1V when charging full*/
			if((deltatime > (100 - gBatCapacity) * CHARGE_MID_SECOND))
			{
				capacity = 99;
				if (rk29_battery_dbg_level)
					printk("0000rk2918_battery_resume: last_BatChargeStatus: %d, gBatChargeStatus: %d, gBatVoltage = %d, gBatCapacity = %d, deltatime = %d, capacity = %d\n", 
	           			last_BatChargeStatus, gBatChargeStatus, gBatVoltage, gBatCapacity, deltatime, capacity);
				if (gBatStatus == POWER_SUPPLY_STATUS_FULL)
				{
					capacity = 100;
				}
   				return capacity;
			}
			/*if battery is not full after long charging*/
			if((capacity > 80))
			{
				if(capacity < gBatCapacity + deltatime/CHARGE_MID_SECOND)
					capacity = gBatCapacity + deltatime/CHARGE_MID_SECOND;
				if(capacity >= 100)
					capacity = 99;
				if (gBatStatus == POWER_SUPPLY_STATUS_FULL)
				{
					capacity = 100;
				}
				if (rk29_battery_dbg_level)
					printk("11111rk2918_battery_resume: last_BatChargeStatus: %d, gBatChargeStatus: %d, gBatVoltage = %d, gBatCapacity = %d, deltatime = %d, capacity = %d\n", 
		       			last_BatChargeStatus, gBatChargeStatus, gBatVoltage, gBatCapacity, deltatime, capacity);
				return capacity;
			}
			
			if(capacity > gBatCapacity)
	        {
	        	if((capacity-gBatCapacity)>15)
					gBatCapacity = capacity;
				else
				{
					if(gBatCapacity > 20)
					 	capacity=gBatCapacity;
					else
						gBatCapacity =(gBatCapacity + capacity)/2;
				}
	        }
	        else if (capacity < gBatCapacity)
	        {
	        	if((gBatCapacity - capacity)>10)
					gBatCapacity = capacity;
				else
					 capacity=gBatCapacity ;
	        }
		}
		if (capacity >= 100) 
			capacity = 99;
    }
    else
    {
    	/*
        if (deltatime > gBatCapacity * DISCHARGE_MIN_SECOND)
            deltatime = gBatCapacity * DISCHARGE_MIN_SECOND;            
        if (capacity < gBatCapacity - (deltatime / DISCHARGE_MIN_SECOND))    //采样电池容量偏差较大，将容量拉回
        {
            capacity = gBatCapacity - (deltatime / DISCHARGE_MIN_SECOND);
        }
		*/
		if (rk29_battery_dbg_level)
			printk("333rk2918_battery_resume: last_BatChargeStatus: %d, gBatChargeStatus: %d, gBatVoltage = %d, gBatCapacity = %d, deltatime = %d, capacity = %d\n", 
	           			last_BatChargeStatus, gBatChargeStatus, gBatVoltage, gBatCapacity, deltatime, capacity);
		/*to ensure battery show 100% after long charging during sleep and then pug out charger*/
		if((deltatime > (100 - gBatCapacity) * CHARGE_MID_SECOND) && (last_BatChargeStatus == 1))
		{
			if(capacity < 99)
				capacity = 99;
			if(capacity >= 100)
				capacity = 100;
			return capacity;
		}
		
        if (capacity < gBatCapacity)
        {
			if((gBatCapacity - capacity)>10)
				gBatCapacity = capacity;
			else
				 capacity=gBatCapacity ;
        }
        else if (capacity > gBatCapacity)
        {
        	if((capacity-gBatCapacity)>15)
				gBatCapacity = capacity;
			else
				 capacity=gBatCapacity ;
        }
    }
	
    if (capacity == 0) 
		capacity = 1;
    if (capacity >= 100) 
		capacity = 100;
    
    if (gBatStatus == POWER_SUPPLY_STATUS_FULL)
	{
	    capacity = 100;
	}
	if (rk29_battery_dbg_level)	
	    printk("rk2918_battery_resume: last_BatChargeStatus: %d, gBatChargeStatus: %d, gBatVoltage = %d, gBatCapacity = %d, deltatime = %d, ktmietmp.tv.sec = %lu, capacity = %d\n", 
	           last_BatChargeStatus, gBatChargeStatus, gBatVoltage, gBatCapacity, deltatime, batteryspendcnt, capacity);
    
    return capacity;
}

static int rk2918_get_bat_capacity_ext(int BatVoltage)
{
    int i = 0;
	int capacity = 0;
	static int lostcount = 0;
	
    capacity = rk2918_get_bat_capacity_raw(BatVoltage);
	capacitytmp = capacity;
	
	//充放电状态变化后，Buffer填满之前，不更新
    if (gBatStatusChangeCnt < NUM_VOLTAGE_SAMPLE)
    {
        capacity = gBatCapacity;
    }
        
    if (gBatChargeStatus == 1)
    {
        if ((capacity > gBatCapacity)  && (gBatCapacity < 100))
        {
            //实际采样到的电压比显示的电压大，逐级上升
            gBatHighCapacityChargeCnt = 0;
            if (gBatCapacityDownCnt == 0)
            {
                capacity = gBatCapacity + 1;
                gBatCapacityDownCnt = NUM_CHARGE_MIN_SAMPLE;
            }
            else
            {
                capacity = gBatCapacity;
            }
        }
        else if ((capacity <= gBatCapacity) && (gBatCapacity < 100))
        {
            capacity = gBatCapacity;
            
            //长时间内充电电压无变化，开始启动计时充电
            if ((gBatCapacity >= 80) && (++gBatHighCapacityChargeCnt > NUM_CHARGE_MAX_SAMPLE))
            {
                capacity = gBatCapacity + 1;
                gBatHighCapacityChargeCnt = (NUM_CHARGE_MAX_SAMPLE - NUM_CHARGE_MID_SAMPLE);
            }
        }
		else
        {
            capacity = gBatCapacity;
        }
        
		if (capacity == 0)
		{
		    capacity = 1;
		}
		
		if (capacity >= 100)
		{
		    capacity = 99;
		}

		if (gBatStatus == POWER_SUPPLY_STATUS_FULL)
		{
		    capacity = 100;
		}
    }    
    else
    {   
        //放电时,只允许电压下降
        if (capacity > gBatCapacity)
        {
            capacity = gBatCapacity;
        }
        
        if ((capacity < gBatCapacity) && (gBatCapacityDownCnt == 0))
        {
            capacity = gBatCapacity - 1;
            gBatCapacityDownCnt = NUM_DISCHARGE_MIN_SAMPLE;
        }
        else
        {
            capacity = gBatCapacity;
        }
		
	#if defined (CONFIG_MACH_M1005HN) || defined (CONFIG_MACH_M1005HNW) || defined (CONFIG_MACH_M1005HD)
		if (lostcount++ > NUM_SPEEDLOSE_SAMPLE)
		{
			lostcount = 0;
			if (((lastlost-gBatVoltage)>16)&&(gBatVoltage >= 6900))// start play game
			{
				shutdownvoltage = 6700; 
				printk("%s...lastlost=%d.enter game\n",__func__,gBatVoltage);
			}
			else
			{
				if ((gBatVoltage-lastlost)>16) //exit game
				{
					shutdownvoltage = 6900;
					printk("%s.. lastlost=%d..exit game\n",__func__,gBatVoltage);
				}
			}
			lastlost = gBatVoltage;
			
	
		}
	
		// <8%capacity enter game will make mistake
		if (((gBatVoltage<=shutdownvoltage) && (lastlost-gBatVoltage)<=6) || (gBatVoltage <= 6700))
		{	
			shutdownflag++;
			if(shutdownflag >= NUM_SHUTD0WN_SAMPLE)
			{
				capacity = 0;
				printk("%s.........%d.........%d\n",__func__,__LINE__,shutdownflag);
				shutdownflag = NUM_SHUTD0WN_SAMPLE+1;
			}
		}
	#else
		if (lostcount++ > NUM_SPEEDLOSE_SAMPLE)
		{
			lostcount = 0;
			if (((lastlost-gBatVoltage)>8)&&(gBatVoltage >= 3400))// start play game
			{
				shutdownvoltage = 3300; 
				printk("%s...lastlost=%d.enter game\n",__func__,gBatVoltage);
			}
			else
			{
				if ((gBatVoltage-lastlost)>8) //exit game
				{
					shutdownvoltage = 3400;
					printk("%s.. lastlost=%d..exit game\n",__func__,gBatVoltage);
				}
			}
			lastlost = gBatVoltage;
			
	
		}
	
		// <8%capacity enter game will make mistake
		if (((gBatVoltage<=shutdownvoltage) && (lastlost-gBatVoltage)<=3) || (gBatVoltage <= 3300))
		{	
			shutdownflag++;
			if(shutdownflag >= NUM_SHUTD0WN_SAMPLE)
			{
				capacity = 0;
				printk("%s.........%d.........%d\n",__func__,__LINE__,shutdownflag);
				shutdownflag = NUM_SHUTD0WN_SAMPLE+1;
			}
		}
	#endif
    }
    if (gBatCapacityDownCnt > 0)
    {
        gBatCapacityDownCnt --;
    }
    
	return capacity;
}

unsigned long AdcTestvalue = 0;
unsigned long AdcTestCnt = 0;
static void rk2918_get_bat_voltage(struct rk2918_battery_data *bat)
{
	int value;
	int i,*pSamp,*pStart = &gBatVoltageSamples[0],num = 0;
	int temp[2] = {0,0};
	
	value = gBatteryData->adc_val;
	AdcTestvalue = value;
    adc_async_read(gBatteryData->client);
    
	//*pSamples++ = (value * BAT_2V5_VALUE * (BAT_PULL_UP_R + BAT_PULL_DOWN_R)) / (1024 * BAT_PULL_DOWN_R);
	*pSamples++ = adc_to_voltage(value);
	num = pSamples - pStart;
	if (num >= NUM_VOLTAGE_SAMPLE)
	{
	    pSamples = pStart;
	    gFlagLoop = 1;
	}
	if (gFlagLoop == 1)
	{
	    num = NUM_VOLTAGE_SAMPLE;
	}
	value = 0;
	for (i = 0; i < num; i++)
	{
	    value += gBatVoltageSamples[i];
	}
	gBatVoltage = value / num;
	//gBatVoltage = (value * BAT_2V5_VALUE * 2) / 1024;
	
	/*消除毛刺电压*/
	if(gBatVoltage >= BATT_MAX_VOL_VALUE + 10)
		gBatVoltage = BATT_MAX_VOL_VALUE + 10;
	else if(gBatVoltage <= BATT_ZERO_VOL_VALUE - 10)
		gBatVoltage = BATT_ZERO_VOL_VALUE - 10;

    //充放电状态变化时,开始计数	
	if (++gBatStatusChangeCnt > NUM_VOLTAGE_SAMPLE)  
	    gBatStatusChangeCnt = NUM_VOLTAGE_SAMPLE + 1;
}

int first_flag = 1;
extern int time_chg_flag;
static void rk2918_get_bat_capacity(struct rk2918_battery_data *bat)
{
    s32 deltatime = 0;
    ktime_t ktmietmp;
    struct timespec ts;
    
	ktmietmp = ktime_get_real();
    ts = ktime_to_timespec(ktmietmp);
	deltatime = ts.tv_sec - batteryspendcnt;
	
	if (first_flag || (openfailflag && (openfailcount > 1))) 
	//多次打开，刚开始采样时不准，多次采样。。
	{
	     if(first_flag == 1)
	     		first_flag--;
	     openfailcount--;
	  printk("%s,first_flag=%d,openfailflag=%d,openfailcount=%d\n",__func__,first_flag,openfailflag,openfailcount);   		
	    gBatCapacity = rk2918_battery_load_capacity();
	    if (gBatCapacity == 0) gBatCapacity = 1;
	}
	else if ((deltatime > 1) && (first_flag == 0) &&(!time_chg_flag))//处理休眠之后的电量回复,如果超过十分钟
	{	
		//printk("---->time_chg_flag =%d\n", time_chg_flag);
        gBatCapacity = rk2918_battery_resume_get_Capacity(deltatime);
	}
	else
	{
        gBatCapacity = rk2918_get_bat_capacity_ext(gBatVoltage);
        
    }
	if(time_chg_flag)
		time_chg_flag = 0;
    batteryspendcnt = ts.tv_sec;
	
}

#define BATT_DBG_FILE "/sdcard/bat_dbg_record.dat"
static int buf_offset = 0;

static void rk2918_battery_timer_work(struct work_struct *work)
{		
	rk2918_get_bat_status(gBatteryData);
	rk2918_get_bat_health(gBatteryData);
	rk2918_get_bat_present(gBatteryData);
	rk2918_get_bat_voltage(gBatteryData);
	//to prevent gBatCapacity be changed sharply
	if (gBatCapacity < 0)
	{
		gBatCapacity = 0;
	}
	else
	{
		if (gBatCapacity > 100)
		{
			gBatCapacity = 100;
		}
	}
	rk2918_get_bat_capacity(gBatteryData);
	
	if (rk29_battery_dbg_level)
	{
    	if (++AdcTestCnt >= 20)
    	{
    	    AdcTestCnt = 0;
    	    printk("\nchg_ok_level =%d, chg_ok= %d, gBatStatus = %d, adc_val = %d, TrueBatVol = %d,gBatVol = %d, gBatCap = %d, captmp = %d, sec = %lu, time_chg_flag = %d, first_flag  = %d\n", 
    	            gBatteryData->charge_ok_level, gpio_get_value(gBatteryData->charge_ok_pin), gBatStatus, AdcTestvalue, adc_to_voltage(AdcTestvalue), 
    	            gBatVoltage, gBatCapacity, capacitytmp, batteryspendcnt, time_chg_flag, first_flag);
    	}
    }

	/*update battery parameter after adc and capacity has been changed*/
	if(((gBatStatus != gBatLastStatus) || (gBatPresent != gBatLastPresent) || (gBatCapacity != gBatLastCapacity))&&(suspend_flag==0))
	{
		//for debug
		if (rk29_battery_dbg_level)
		{
			char _tmp_buf[250];
			int buf_len = 0;
			struct file* fp;
			sprintf(_tmp_buf, "gBatStatus = %d, adc_val = %d, TrueBatVol = %d,gBatVol = %d, gBatCap = %d, captmp = %d, sec = %lu, inter_sec = %lu, time_chg_flag = %d, first_flag = %d\n", 
		    	            gBatStatus, AdcTestvalue, ((AdcTestvalue * BAT_2V5_VALUE * (BAT_PULL_UP_R + BAT_PULL_DOWN_R)) / (1024 * BAT_PULL_DOWN_R)), 
		    	            gBatVoltage, gBatCapacity, capacitytmp, batteryspendcnt, batteryspendcnt - last_batteryspendcnt, time_chg_flag, first_flag);
			buf_len = strlen(_tmp_buf);
			fp = filp_open(BATT_DBG_FILE,O_RDWR | O_APPEND | O_CREAT, 0);
			if(IS_ERR(fp))
			{
				printk("bryan---->open file /data/bat_dbg_record.dat failed\n");
			}
			else
			{
				kernel_write(fp, _tmp_buf, buf_len ,buf_offset);
			    filp_close(fp,NULL);
				buf_offset += buf_len; 
			}
			last_batteryspendcnt = batteryspendcnt;
		}
		
		gBatLastStatus = gBatStatus;
		gBatLastPresent = gBatPresent;
		gBatLastCapacity = gBatCapacity;
		power_supply_changed(&gBatteryData->battery);
	}
}


static void rk2918_batscan_timer(unsigned long data)
{
    gBatteryData->timer.expires  = jiffies + msecs_to_jiffies(TIMER_MS_COUNTS);
	add_timer(&gBatteryData->timer);
	schedule_work(&gBatteryData->timer_work);	
}

#ifdef RK29_USB_CHARGE_SUPPORT
static int rk2918_usb_get_property(struct power_supply *psy, 
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	charger_type_t charger;
	charger =  CHARGER_USB;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_USB)
			val->intval = get_msc_connect_flag();
		printk("%s:%d\n",__FUNCTION__,val->intval);
		break;

	default:
		return -EINVAL;
	}
	
	return 0;

}
#endif

static int rk2918_ac_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	int ret = 0;
	charger_type_t charger;
	charger =  CHARGER_USB;
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_MAINS)
		{
			if (gBatChargeStatus == 1)
				val->intval = 1;
			else
				val->intval = 0;	
		}
		DBG("%s:%d\n",__FUNCTION__,val->intval);
		break;
		
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int rk2918_battery_get_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *val)
{
	struct rk2918_battery_data *data = container_of(psy,
		struct rk2918_battery_data, battery);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = gBatStatus;
		DBG("gBatStatus=%d\n",val->intval);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = gBatHealth;
		DBG("gBatHealth=%d\n",val->intval);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = gBatPresent;
		DBG("gBatPresent=%d\n",val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		if(gBatVoltageValue[1] == 0)
		val ->intval = gBatVoltage;
		else
		val ->intval = gBatVoltageValue[1];
		DBG("gBatVoltage=%d\n",val->intval);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;	
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = gBatCapacity;
		DBG("gBatCapacity=%d%%\n",val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = data->bat_max;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = data->bat_min;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static enum power_supply_property rk2918_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
};

#ifdef RK29_USB_CHARGE_SUPPORT
static enum power_supply_property rk2918_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};
#endif

static enum power_supply_property rk2918_ac_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};


#ifdef CONFIG_PM
static int rk2918_battery_suspend(struct platform_device *dev, pm_message_t state)
{
	/* flush all pending status updates */
	printk("rk2918_battery_suspend!!!\n");
	struct rk2918_battery_platform_data *pdata = dev->dev.platform_data;

	if (pdata->charge_cur_ctl != INVALID_GPIO) {
		gpio_set_value(pdata->charge_cur_ctl, pdata->charge_cur_ctl_level); /* huge current charge */
	}

	flush_scheduled_work();
	last_BatChargeStatus = gBatChargeStatus;
	return 0;
}

static int rk2918_battery_resume(struct platform_device *dev)
{
	struct rk2918_battery_platform_data *pdata = dev->dev.platform_data;

	if (pdata->charge_cur_ctl != INVALID_GPIO) {
		gpio_set_value(pdata->charge_cur_ctl, !pdata->charge_cur_ctl_level); /* small current charge */
	}
	/* things may have changed while we were away */
	schedule_work(&gBatteryData->timer_work);
	printk("rk2918_battery_resume!!!\n");
	return 0;
}
#else
#define rk2918_battery_suspend NULL
#define rk2918_battery_resume NULL
#endif

static irqreturn_t rk2918_battery_interrupt(int irq, void *dev_id)
{
//    if ((rk2918_get_charge_status()) && (gBatFullFlag != 1))
//    {
//        gBatFullFlag = 1;
//    }

    return 0;
}

static irqreturn_t rk2918_dc_wakeup(int irq, void *dev_id)
{   
    gBatStatusChangeCnt = 0;        //状态变化开始计数
    
//    disable_irq_wake(gBatteryData->dc_det_irq);
    queue_delayed_work(gBatteryData->wq, &gBatteryData->work, 0);
	
    return IRQ_HANDLED;
}

static void rk2918_battery_work(struct work_struct *work)
{
    int ret;
    int irq_flag;
    
    rk28_send_wakeup_key();
    
    free_irq(gBatteryData->dc_det_irq, gBatteryData);
    irq_flag = (!gpio_get_value (gBatteryData->dc_det_pin)) ? IRQF_TRIGGER_RISING : IRQF_TRIGGER_FALLING;
	ret = request_irq(gBatteryData->dc_det_irq, rk2918_dc_wakeup, irq_flag, "rk2918_battery", gBatteryData);
	if (ret) {
		free_irq(gBatteryData->dc_det_irq, gBatteryData);
	}
	enable_irq_wake(gBatteryData->dc_det_irq);
}

static void rk2918_battery_callback(struct adc_client *client, void *param, int result)
{
    gBatteryData->adc_val = result;
	return;
}

#define POWER_ON_PIN    RK29_PIN4_PA4
static void rk2918_low_battery_check(void)
{
    int i;
    int tmp = 0;
    
    for (i = 0; i < 100; i++)
    {
        tmp += adc_sync_read(gBatteryData->client);
        mdelay(1);
    }
    tmp = tmp / 100;
    
    //tmp = (tmp * BAT_2V5_VALUE * (BAT_PULL_UP_R + BAT_PULL_DOWN_R)) / (1024 * BAT_PULL_DOWN_R);
    tmp = adc_to_voltage(tmp);
    rk2918_get_charge_status();
    gBatCapacity = rk2918_get_bat_capacity_raw(tmp);
    if (gBatCapacity == 0) gBatCapacity = 1;
    printk("rk2918_low_battery_check: gBatVoltage = %d, gBatCapacity = %d\n", gBatVoltage, gBatCapacity);
    
    if (gBatVoltage <= BATT_ZERO_VOL_VALUE + 50)
    {
        printk("low battery: powerdown\n");
        gpio_direction_output(POWER_ON_PIN, GPIO_LOW);
        tmp = 0;
        while(1)
        {
            if(gpio_get_value(POWER_ON_PIN) == GPIO_HIGH)
		    {
			    gpio_set_value(POWER_ON_PIN,GPIO_LOW);
		    }
		    mdelay(5);
		    if (++tmp > 50) break;
		}
    }
    gpio_direction_output(POWER_ON_PIN, GPIO_HIGH);
}

static ssize_t rk29_battery_dbg_show(struct device *dev, struct device_attribute *attr, char *_buf)
{
    return sprintf(_buf, "%d\n", rk29_battery_dbg_level);
}

static ssize_t rk29_battery_dbg_store(struct device *dev, struct device_attribute *attr, const char *_buf, size_t _count)
{
    rk29_battery_dbg_level = simple_strtoul(_buf, NULL, 16);
    printk("rk29_battery_dbg_level = %d\n",rk29_battery_dbg_level);
    
    return _count;
} 

static struct class *rk29_battery_dbg_class = NULL;
static DEVICE_ATTR(rk29_battery_dbg, 0666, rk29_battery_dbg_show, rk29_battery_dbg_store);

static int rk2918_battery_probe(struct platform_device *pdev)
{
	int ret;
	struct adc_client *client;
	struct rk2918_battery_data *data;
	struct rk2918_battery_platform_data *pdata = pdev->dev.platform_data;
	int irq_flag;
	int i = 0;
	
	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL) {
		ret = -ENOMEM;
		goto err_data_alloc_failed;
	}
	gBatteryData = data;

    //clear io
    data->dc_det_pin     = INVALID_GPIO;
    data->batt_low_pin   = INVALID_GPIO;
    data->charge_set_pin = INVALID_GPIO;
	data->charge_ok_pin  = INVALID_GPIO;
	
	if (pdata && pdata->io_init) {
		ret = pdata->io_init();
		if (ret) 
			goto err_free_gpio1;		
	}
	
	//dc det
	if (pdata->dc_det_pin != INVALID_GPIO)
	{
	#ifndef DC_DET_WITH_USB_INT
    	ret = gpio_request(pdata->dc_det_pin, NULL);
    	if (ret) {
    		printk("failed to request dc_det gpio\n");
    		goto err_free_gpio1;
    	}
	#endif
		if(pdata->dc_det_level)
    		gpio_pull_updown(pdata->dc_det_pin, 0);//important
    	else
			gpio_pull_updown(pdata->dc_det_pin, GPIOPullUp);//important
    	ret = gpio_direction_input(pdata->dc_det_pin);
    	if (ret) {
    		printk("failed to set gpio dc_det input\n");
    		goto err_free_gpio1;
    	}
    	data->dc_det_pin   = pdata->dc_det_pin;
    	data->dc_det_level = pdata->dc_det_level;
    }

	if (pdata->charge_cur_ctl != INVALID_GPIO) {

		ret = gpio_request(pdata->charge_cur_ctl, "DC_CURRENT_CONTROL");
		if (ret < 0) {
    		printk("failed to request charge current control gpio\n");
    		goto err_free_gpio2;
		}

		ret = gpio_direction_output(pdata->charge_cur_ctl, !pdata->charge_cur_ctl_level);
		if (ret < 0) {
			printk("rk29_battery: failed to set charge current control gpio\n");
    		goto err_free_gpio2;
		}
		gpio_pull_updown(pdata->charge_cur_ctl, !pdata->charge_cur_ctl_level);
		gpio_set_value(pdata->charge_cur_ctl, !pdata->charge_cur_ctl_level);
		data->charge_cur_ctl = pdata->charge_cur_ctl;
		data->charge_cur_ctl_level = pdata->charge_cur_ctl_level;
	}
	
	//charge set for usb charge
	if (pdata->charge_set_pin != INVALID_GPIO)
	{
    	ret = gpio_request(pdata->charge_set_pin, NULL);
    	if (ret) {
    		printk("failed to request dc_det gpio\n");
    		goto err_free_gpio2;
    	}
    	data->charge_set_pin = pdata->charge_set_pin;
    	data->charge_set_level = pdata->charge_set_level;
    	gpio_direction_output(pdata->charge_set_pin, 1 - pdata->charge_set_level);
    }
	
	//charge_ok
	if (pdata->charge_ok_pin != INVALID_GPIO)
	{
        ret = gpio_request(pdata->charge_ok_pin, NULL);
    	if (ret) {
    		printk("failed to request charge_ok gpio\n");
    		goto err_free_gpio3;
    	}
	
    	gpio_pull_updown(pdata->charge_ok_pin, GPIOPullUp);//important
    	ret = gpio_direction_input(pdata->charge_ok_pin);
    	if (ret) {
    		printk("failed to set gpio charge_ok input\n");
    		goto err_free_gpio3;
    	}
    	data->charge_ok_pin   = pdata->charge_ok_pin;
    	data->charge_ok_level = pdata->charge_ok_level;
    }
    
	client = adc_register(0, rk2918_battery_callback, NULL);
    if(!client)
		goto err_adc_register_failed;
    
	memset(gBatVoltageSamples, 0, sizeof(gBatVoltageSamples));
	spin_lock_init(&data->lock);
    data->adc_val = adc_sync_read(client);
	data->client = client;
    data->battery.properties = rk2918_battery_props;
	data->battery.num_properties = ARRAY_SIZE(rk2918_battery_props);
	data->battery.get_property = rk2918_battery_get_property;
	data->battery.name = "battery";
	data->battery.type = POWER_SUPPLY_TYPE_BATTERY;
	data->adc_bat_divider = 414;
	data->bat_max = BATT_MAX_VOL_VALUE;
	data->bat_min = BATT_ZERO_VOL_VALUE;
	DBG("bat_min = %d\n",data->bat_min);
	
#ifdef RK29_USB_CHARGE_SUPPORT
	data->usb.properties = rk2918_usb_props;
	data->usb.num_properties = ARRAY_SIZE(rk2918_usb_props);
	data->usb.get_property = rk2918_usb_get_property;
	data->usb.name = "usb";
	data->usb.type = POWER_SUPPLY_TYPE_USB;
#endif

	data->ac.properties = rk2918_ac_props;
	data->ac.num_properties = ARRAY_SIZE(rk2918_ac_props);
	data->ac.get_property = rk2918_ac_get_property;
	data->ac.name = "ac";
	data->ac.type = POWER_SUPPLY_TYPE_MAINS;	
	
	rk2918_low_battery_check();
    
	ret = power_supply_register(&pdev->dev, &data->ac);
	if (ret)
	{
		printk(KERN_INFO "fail to ac power_supply_register\n");
		goto err_ac_failed;
	}
#if 0
	ret = power_supply_register(&pdev->dev, &data->usb);
	if (ret)
	{
		printk(KERN_INFO "fail to usb power_supply_register\n");
		goto err_usb_failed;
	}
#endif
	ret = power_supply_register(&pdev->dev, &data->battery);
	if (ret)
	{
		printk(KERN_INFO "fail to battery power_supply_register\n");
		goto err_battery_failed;
	}
	platform_set_drvdata(pdev, data);
	
	INIT_WORK(&data->timer_work, rk2918_battery_timer_work);
	
//    irq_flag = (pdata->charge_ok_level) ? IRQF_TRIGGER_RISING : IRQF_TRIGGER_FALLING;
//	ret = request_irq(gpio_to_irq(pdata->charge_ok_pin), rk2918_battery_interrupt, irq_flag, "rk2918_battery", data);
//	if (ret) {
//		printk("failed to request irq\n");
//		goto err_irq_failed;
//	}
#ifndef DC_DET_WITH_USB_INT
    if (pdata->dc_det_pin != INVALID_GPIO)
    {
        irq_flag = (!gpio_get_value (pdata->dc_det_pin)) ? IRQF_TRIGGER_RISING : IRQF_TRIGGER_FALLING;
    	ret = request_irq(gpio_to_irq(pdata->dc_det_pin), rk2918_dc_wakeup, irq_flag, "rk2918_battery", data);
    	if (ret) {
    		printk("failed to request dc det irq\n");
    		goto err_dcirq_failed;
    	}
    	data->dc_det_irq = gpio_to_irq(pdata->dc_det_pin);
    	//data->wq = create_rt_workqueue("rk2918_battery");
    	data->wq = create_workqueue("rk2918_battery");
    	INIT_DELAYED_WORK(&data->work, rk2918_battery_work);
    	
    	enable_irq_wake(gpio_to_irq(pdata->dc_det_pin));
    }
#endif

	setup_timer(&data->timer, rk2918_batscan_timer, (unsigned long)data);
	//data->timer.expires  = jiffies + 2000;
	//add_timer(&data->timer);
   
    rk29_battery_dbg_class = class_create(THIS_MODULE, "rk29_battery");
	battery_dev = device_create(rk29_battery_dbg_class, NULL, MKDEV(0, 1), NULL, "battery");
	ret = device_create_file(battery_dev, &dev_attr_rk29_battery_dbg);
	if (ret)
	{
		printk("create file sys failed!!! \n");
		//goto err_dcirq_failed;
	}
	for(i = 0; i<10; i++)
	{
		ret = device_create_file(&pdev->dev, &dev_attr_startget);
		if (ret)
		{
			printk("make a mistake in creating devices  attr file, failed times: %d\n\n ", i+1);
			continue;
		}
		break;
	}
	printk(KERN_INFO "rk2918_battery: driver initialized\n");
	
	return 0;
	
err_dcirq_failed:
    free_irq(gpio_to_irq(pdata->dc_det_pin), data);
    
err_irq_failed:
	free_irq(gpio_to_irq(pdata->charge_ok_pin), data);
    
err_battery_failed:
//	power_supply_unregister(&data->usb);
//err_usb_failed:
err_ac_failed:
	power_supply_unregister(&data->ac);
	
err_adc_register_failed:
err_free_gpio3:
	gpio_free(pdata->charge_ok_pin);
err_free_gpio2:
	gpio_free(pdata->charge_cur_ctl);
err_free_gpio1:
    gpio_free(pdata->dc_det_pin);
    
err_data_alloc_failed:
	kfree(data);

    printk("rk2918_battery: error!\n");
    
	return ret;
}

static int rk2918_battery_remove(struct platform_device *pdev)
{
	struct rk2918_battery_data *data = platform_get_drvdata(pdev);
	struct rk2918_battery_platform_data *pdata = pdev->dev.platform_data;

	power_supply_unregister(&data->battery);
//	power_supply_unregister(&data->usb);
	power_supply_unregister(&data->ac);
	free_irq(data->irq, data);
	gpio_free(pdata->charge_ok_pin);
	gpio_free(pdata->dc_det_pin);
	kfree(data);
	gBatteryData = NULL;
	return 0;
}

static struct platform_driver rk2918_battery_driver = {
	.probe		= rk2918_battery_probe,
	.remove		= rk2918_battery_remove,
	.suspend	= rk2918_battery_suspend,
	.resume		= rk2918_battery_resume,
	.driver = {
		.name = "rk2918-battery",
		.owner	= THIS_MODULE,
	}
};

static int __init rk2918_battery_init(void)
{
	return platform_driver_register(&rk2918_battery_driver);
}

static void __exit rk2918_battery_exit(void)
{
	platform_driver_unregister(&rk2918_battery_driver);
}

module_init(rk2918_battery_init);
module_exit(rk2918_battery_exit);

MODULE_DESCRIPTION("Battery detect driver for the rk2918");
MODULE_AUTHOR("luowei lw@rock-chips.com");
MODULE_LICENSE("GPL");

