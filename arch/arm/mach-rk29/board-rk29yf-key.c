
#include <mach/key.h>
#include <mach/gpio.h>
#include <mach/board.h>


static struct rk29_keys_button key_button[] = KEYS_MAP;

struct rk29_keys_platform_data rk29_keys_pdata = {
	.buttons	= key_button,
	.nbuttons	= ARRAY_SIZE(key_button),
#if defined (ADC_KEY_CHN)
	.chn		= ADC_KEY_CHN,  //chn: 0-7, if do not use ADC,set 'chn' -1
#else
	.chn		= -1,  //chn: 0-7, if do not use ADC,set 'chn' -1
#endif
};
