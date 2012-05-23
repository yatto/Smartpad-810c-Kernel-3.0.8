#ifndef __LINUX_FT5X0X_TS_H__
#define __LINUX_FT5X0X_TS_H__


/* -- dirver configure -- */
#define CFG_MAX_TOUCH_POINTS  5

//#define CFG_EX_FUN	

#if defined (CONFIG_TOUCHSCREEN_1024X768)
#define SCREEN_MAX_X 1024
#define SCREEN_MAX_Y 768
#elif defined (CONFIG_TOUCHSCREEN_1024X600)
#define SCREEN_MAX_X 1024
#define SCREEN_MAX_Y 600
#elif defined (CONFIG_TOUCHSCREEN_800X600)
#define SCREEN_MAX_X 800
#define SCREEN_MAX_Y 600
#elif defined (CONFIG_TOUCHSCREEN_800X480)
#define SCREEN_MAX_X 800
#define SCREEN_MAX_Y 480
#endif

#define PRESS_MAX       255

#define POINT_READ_BUF  (3 + 6 * (CFG_MAX_TOUCH_POINTS))

#define FT5X0X_NAME	"ft5x0x"	//"ft5x0x_ts"

/*register address*/
#define FT5x0x_REG_FW_VER 		0xA6
#define FT5x0x_REG_POINT_RATE	0x88
#define FT5X0X_REG_THGROUP	0x80

int ft5x0x_i2c_Read(struct i2c_client *client,  char * writebuf, int writelen, 
							char *readbuf, int readlen);
int ft5x0x_i2c_Write(struct i2c_client *client, char *writebuf, int writelen);

void delay_qt_ms(unsigned long  w_ms);


#endif
