/*
 * driver definition for sensor driver
 *
 * Coypright (c) 2017 huaqin
 */
#include <linux/types.h>
#include <linux/interrupt.h>



#define USE_COMMON_FP 1
#define USE_COMMON_PINCTRL 1

#if USE_COMMON_PINCTRL
static const char * const pinctrl_names[] = {
	//"commonfp_power_on",
	//"commonfp_power_off",
	"commonfp_reset_reset",
	"commonfp_reset_active",
	"commonfp_irq_active",
};
#endif


struct fp_common_resource{
	struct platform_device *pdev;	//platform handle
	unsigned int irq_num;					//softirq number
#if USE_COMMON_PINCTRL
	struct pinctrl *fp_pinctrl;
	struct pinctrl_state *states[ARRAY_SIZE(pinctrl_names)];
#else
	int rst_gpio;	//reset gpio number
	int irq_gpio;	//irq gpio number
#endif
	int pwr_gpio;	//power gpio number
};

int commonfp_power_on(void);
int commonfp_power_off(void);

int get_reset_gpio_number(void);
int get_irq_gpio_number(void);

int commonfp_hw_reset(int ns);
/* Huaqin modify for ZQL1650-143 by wangxiang at 2018/02/09 start */
int commonfp_request_irq(irq_handler_t handler, irq_handler_t thread_fn, unsigned long flags,
	    const char *name, void *dev);
/* Huaqin modify for ZQL1650-143 by wangxiang at 2018/02/09 end */
void commonfp_free_irq(void *dev_id);
void commonfp_irq_enable(void);
void commonfp_irq_disable(void);


