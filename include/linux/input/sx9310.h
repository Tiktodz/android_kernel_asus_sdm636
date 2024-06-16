/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */
#ifndef SX9310_H
#define SX9310_H

#include <linux/device.h>
#include <linux/slab.h>
#include <linux/interrupt.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/wakelock.h>
#include <linux/earlysuspend.h>
#include <linux/suspend.h>
#endif

/*
 *  I2C Registers
 */
#define SX9310_IRQSTAT_REG      0x00
#define SX9310_STAT0_REG        0x01
#define SX9310_STAT1_REG        0x02
#define SX9310_IRQ_ENABLE_REG   0x03
#define SX9310_IRQFUNC_REG      0x04

#define SX9310_CPS_CTRL0_REG    0x10
#define SX9310_CPS_CTRL1_REG    0x11
#define SX9310_CPS_CTRL2_REG    0x12
#define SX9310_CPS_CTRL3_REG    0x13
#define SX9310_CPS_CTRL4_REG    0x14
#define SX9310_CPS_CTRL5_REG    0x15
#define SX9310_CPS_CTRL6_REG    0x16
#define SX9310_CPS_CTRL7_REG    0x17
#define SX9310_CPS_CTRL8_REG    0x18
#define SX9310_CPS_CTRL9_REG    0x19
#define SX9310_CPS_CTRL10_REG   0x1A
#define SX9310_CPS_CTRL11_REG   0x1B
#define SX9310_CPS_CTRL12_REG   0x1C
#define SX9310_CPS_CTRL13_REG   0x1D
#define SX9310_CPS_CTRL14_REG   0x1E
#define SX9310_CPS_CTRL15_REG   0x1F
#define SX9310_CPS_CTRL16_REG   0x20
#define SX9310_CPS_CTRL17_REG   0x21
#define SX9310_CPS_CTRL18_REG   0x22
#define SX9310_CPS_CTRL19_REG   0x23
#define SX9310_SAR_CTRL0_REG    0x2A
#define SX9310_SAR_CTRL1_REG    0x2B
#define SX9310_SAR_CTRL2_REG    0x2C

#define SX9310_SOFTRESET_REG  0x7F

/*      Sensor Readback */
#define SX9310_CPSRD          0x30

#define SX9310_USEMSB         0x31
#define SX9310_USELSB         0x32

#define SX9310_AVGMSB         0x33
#define SX9310_AVGLSB         0x34

#define SX9310_DIFFMSB        0x35
#define SX9310_DIFFLSB        0x36
#define SX9310_OFFSETMSB                0x37
#define SX9310_OFFSETLSB                0x38

#define SX9310_SARMSB                   0x39
#define SX9310_SARLSB                   0x3A

/*      IrqStat 0:Inactive 1:Active     */
#define SX9310_IRQSTAT_RESET_FLAG      0x80
#define SX9310_IRQSTAT_TOUCH_FLAG      0x40
#define SX9310_IRQSTAT_RELEASE_FLAG    0x20
#define SX9310_IRQSTAT_COMPDONE_FLAG   0x10
#define SX9310_IRQSTAT_CONV_FLAG       0x08
#define SX9310_IRQSTAT_CLOSEALL_FLAG   0x04
#define SX9310_IRQSTAT_FARALL_FLAG     0x02
#define SX9310_IRQSTAT_SMARTSAR_FLAG   0x01

/* CpsStat  */
#define SX9310_TCHCMPSTAT_TCHCOMB_FLAG    0x08
#define SX9310_TCHCMPSTAT_TCHSTAT2_FLAG   0x04
#define SX9310_TCHCMPSTAT_TCHSTAT1_FLAG   0x02
#define SX9310_TCHCMPSTAT_TCHSTAT0_FLAG   0x01

/*      SoftReset */
#define SX9310_SOFTRESET  0xDE

/**************************************
*                       define platform data
*
**************************************/
struct smtc_reg_data {
        unsigned char reg;
        unsigned char val;
        };
typedef struct smtc_reg_data smtc_reg_data_t;
typedef struct smtc_reg_data *psmtc_reg_data_t;

struct _buttonInfo {
        /*! The Key to send to the input */
        int keycode0;
	int keycode1;
        /*! Mask to look for on Touch Status */
        int mask;
        /*! Current state of button. */
        int state;
        };

struct _totalButtonInformation {
        struct _buttonInfo *buttons;
        int buttonSize;
        struct input_dev *input;
        };

typedef struct _totalButtonInformation buttonInformation_t;
typedef struct _totalButtonInformation *pbuttonInformation_t;

/* Define Registers that need to be initialized to values different than
 * default
 */
static struct smtc_reg_data sx9310_i2c_reg_setup[] = {
                {
                .reg = SX9310_IRQ_ENABLE_REG,
                .val = 0x70,/*Huaqin modify sarsensor INTreg by chenyijun5 at 2018/03/16*/
                },
                {
                .reg = SX9310_IRQFUNC_REG,
                .val = 0x00,
                },
                {
                .reg = SX9310_CPS_CTRL1_REG,
                .val = 0x00,
                },
                {
                .reg = SX9310_CPS_CTRL2_REG,
                .val = 0x04,
                },
                {
                .reg = SX9310_CPS_CTRL3_REG,
                .val = 0x0E,
                },
		/*Huaqin modify sarsensor reg by chenyijun5 at 2018/03/12 start*/
                {
                .reg = SX9310_CPS_CTRL4_REG,
                .val = 0x0D,
                },
		/*Huaqin modify sarsensor reg by chenyijun5 at 2018/03/12 end*/
		/*Huaqin modify sarsensor reg by zhuqiang at 2018/05/10 start*/
                {
                .reg = SX9310_CPS_CTRL5_REG,
                .val = 0x43,
                },
		/*Huaqin modify sarsensor reg by zhuqiang at 2018/05/10 end*/
                {
                .reg = SX9310_CPS_CTRL6_REG,
                .val = 0x20,
                },
                {
                .reg = SX9310_CPS_CTRL7_REG,
                .val = 0x4C,
                },
		/*Huaqin modify sarsensor reg by chenyijun5 at 2018/03/12 start*/
                {
                .reg = SX9310_CPS_CTRL8_REG,
                .val = 0x80,
                },
		/*Huaqin modify sarsensor reg by chenyijun5 at 2018/03/12 end*/
                {
                .reg = SX9310_CPS_CTRL9_REG,
                .val = 0xAE,
                },
                {
                .reg = SX9310_CPS_CTRL10_REG,
                .val = 0x00,
                },
                {
                .reg = SX9310_CPS_CTRL11_REG,
                .val = 0x00,
                },
                {
                .reg = SX9310_CPS_CTRL12_REG,
                .val = 0x00,
                },
                {
                .reg = SX9310_CPS_CTRL13_REG,
                .val = 0x00,
                },
                {
                .reg = SX9310_CPS_CTRL14_REG,
                .val = 0x00,
                },
                {
                .reg = SX9310_CPS_CTRL15_REG,
                .val = 0x00,
                },
                {
                .reg = SX9310_CPS_CTRL16_REG,
                .val = 0x00,
                },
                {
                .reg = SX9310_CPS_CTRL17_REG,
                .val = 0x04,
                },
                {
                .reg = SX9310_CPS_CTRL18_REG,
                .val = 0x00,
                },
                {
                .reg = SX9310_CPS_CTRL19_REG,
                .val = 0x00,
                },
                {
                .reg = SX9310_SAR_CTRL0_REG,
                .val = 0x00,
                },
                {
                .reg = SX9310_SAR_CTRL1_REG,
                .val = 0x80,
                },
                {
                .reg = SX9310_SAR_CTRL2_REG,
                .val = 0x0C,
                },
		/*Huaqin modify sarsensor reg by zhuqiang at 2018/05/10 start*/
                {
                .reg = SX9310_CPS_CTRL0_REG,
                .val = 0x21,        //0x51 for cs0
                },
		/*Huaqin modify sarsensor reg by zhuqiang at 2018/05/10 end*/
        };

static struct _buttonInfo psmtcButtons[] = {
                {
                //.keycode = KEY_0,
                //.keycode = KEY_CAPSENSOR_SX9310_CS0,
		.keycode0 = KEY_SARSENSOR_NEAR,
		.keycode1 = KEY_SARSENSOR_FAR,
                .mask = SX9310_TCHCMPSTAT_TCHSTAT0_FLAG,
                },
                {
                //.keycode = KEY_1,
                //.keycode = KEY_CAPSENSOR_SX9310_CS1,
		.keycode0 = KEY_SARSENSOR_NEAR,
		.keycode1 = KEY_SARSENSOR_FAR,
                .mask = SX9310_TCHCMPSTAT_TCHSTAT1_FLAG,
                },
                {
                //.keycode = KEY_2,
                //.keycode = KEY_CAPSENSOR_SX9310_CS2,
		.keycode0 = KEY_SARSENSOR_NEAR,
		.keycode1 = KEY_SARSENSOR_FAR,
                .mask = SX9310_TCHCMPSTAT_TCHSTAT2_FLAG,
                },
                {
                //.keycode = KEY_3,
                //.keycode = KEY_CAPSENSOR_SX9310_CS1_CS2_COMB,
		.keycode0 = KEY_SARSENSOR_NEAR,
		.keycode1 = KEY_SARSENSOR_FAR,
                .mask = SX9310_TCHCMPSTAT_TCHCOMB_FLAG,
                },
        };

static struct _totalButtonInformation smtcButtonInformation = {
        .buttons = psmtcButtons,
        .buttonSize = ARRAY_SIZE(psmtcButtons),
        };

struct sx9310_platform_data {
        int i2c_reg_num;
        struct smtc_reg_data *pi2c_reg;

        pbuttonInformation_t pbuttonInformation;

        int   irq_gpio;

        int (*get_is_nirq_low)(void);

        int     (*init_platform_hw)(void);
        void    (*exit_platform_hw)(void);
        struct pinctrl *sar_pinctrl;
        };
typedef struct sx9310_platform_data sx9310_platform_data_t;
typedef struct sx9310_platform_data *psx9310_platform_data_t;

/***************************************
*               define data struct/interrupt
*
***************************************/
#define USE_THREADED_IRQ

#define MAX_NUM_STATUS_BITS (8)

typedef struct sx93XX sx93XX_t, *psx93XX_t;
struct sx93XX {
        void * bus; /* either i2c_client or spi_client */

        struct device *pdev; /* common device struction for linux */

        void *pDevice; /* device specific struct pointer */

        struct regulator *vdd;
        struct regulator *vdd1;
        struct regulator *vio;
        int power_enabled;

        /* Function Pointers */
        int (*init)(psx93XX_t this); /* (re)initialize device */

        /* since we are trying to avoid knowing registers, create a pointer to a
         * common read register which would be to read what the interrupt source
         * is from
         */
        int (*refreshStatus)(psx93XX_t this); /* read register status */

        int (*get_nirq_low)(void); /* get whether nirq is low (platform data) */

        /* array of functions to call for corresponding status bit */
        void (*statusFunc[MAX_NUM_STATUS_BITS])(psx93XX_t this);

#if defined(USE_THREADED_IRQ)
        struct mutex mutex;
#else
        spinlock_t          lock; /* Spin Lock used for nirq worker function */
#endif
        int irq; /* irq number used */

        /* whether irq should be ignored.. cases if enable/disable irq is not used
         * or does not work properly */
        char irq_disabled;

        u8 useIrqTimer; /* older models need irq timer for pen up cases */

        int irqTimeout; /* msecs only set if useIrqTimer is true */

        /* struct workqueue_struct    *ts_workq;  */  /* if want to use non default */
        struct delayed_work dworker; /* work struct for worker function */

#ifdef CONFIG_HAS_EARLYSUSPEND
        struct early_suspend early_suspend;  /* early suspend data  */
#endif
        };

void sx93XX_suspend(psx93XX_t this);
void sx93XX_resume(psx93XX_t this);
int sx93XX_init(psx93XX_t this);
int sx93XX_remove(psx93XX_t this);

#endif
