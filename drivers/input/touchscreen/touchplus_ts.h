#ifndef __TOUCHPLUS_H__
#define __TOUCHPLUS_H__ 
                                                                                       
/* IOCTLs for TOUCHPLUS device */
#define TOUCHPLUS_IOM       't'
#define TOUCHPLUS_IOC_CALI     _IO (TOUCHPLUS_IOM, 0x00)
#define TOUCHPLUS_IOC_READ_REG _IOR (TOUCHPLUS_IOM, 0x01, unsigned char)
#define TOUCHPLUS_IOC_WRITE_REG _IOW (TOUCHPLUS_IOM, 0x02, unsigned char)
#define TOUCHPLUS_IOC_READ_REGS _IOR (TOUCHPLUS_IOM, 0x03, char)
#define TOUCHPLUS_IOC_READ_RAW  _IOR (TOUCHPLUS_IOM, 0x04, char)
#define TOUCHPLUS_IOC_FW_DL     _IOW (TOUCHPLUS_IOM, 0x05, int)

/* TOUCHPLUS registers */
#define REGBASE_XY 0
#define REG_NFINGERS 0
#define REG_FINGER_POS 2

#define REGBASE_CFG 62
#define REG_INTMODE  62
#define REG_INTWIDTH  63
#define REG_POWERMODE  64
#define REG_VERSION  65 /* 65 ~ 68 */
#define REG_MAXTOUCH  69
#define REG_BUTTON  70
#define REG_TUNING  71
#define REG_SCANRATE  72
#define REG_IDLEPERIOD  73
#define REG_CMDID 74

#define REGBASE_RAW 75
#define REG_RAWIDX 75 /* MU raw data index */
#define REG_RAWDATA 76 /* X, Y raw data  76~218 */

/* TPMODE */
#define TPMODE_NORMAL 0
#define TPMODE_SC 1
#define TPMODE_MU 2

struct	tpreg
{
	__u8 idx;
	__u8 val;
};

struct	tpregs
{
	__u8 base;
	__u8 len;
	__u8 buf[250];
};


#endif
