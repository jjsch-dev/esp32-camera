/*
 * This file is part of the OpenMV project.
 * author: Juan Schiavoni <juanjoseschiavoni@hotmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * OV7670 driver.
 *
 */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "sccb.h"
#include "ov7670.h"
#include "ov7670_regs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#else
#include "esp_log.h"
static const char* TAG = "ov7760";
#endif

static int ov7670_clkrc = 0x01;

/*
 * The default register settings, as obtained from OmniVision.  There
 * is really no making sense of most of these - lots of "reserved" values
 * and such.
 *
 * These settings give VGA YUYV.
 */
struct regval_list {
	uint8_t reg_num;
	uint8_t value;
};

struct windowing {           
    int hstart;
    int hstop; 
    int vstart;
    int vstop;
};

static struct regval_list ov7670_default_regs[] = {
    /* Sensor automatically sets output window when resolution changes. */    
    {TSLB, 0x04}, 
    
    /* Frame rate 30 fps at 12 Mhz clock */    
	{CLKRC, 0x01},  
	{DBLV,  0x4A},  

    {COM10, COM10_VSYNC_NEG | COM10_PCLK_MASK},

    /* Improve white balance */ 
	{COM4, 0x40},  
    
    /* Improve color */   
    {RSVD_B0, 0x84},  

    /* Enable 50/60 Hz auto detection */
    {COM11, COM11_EXP|COM11_HZAUTO}, 

    /* Disable some delays */
	{HSYST, 0},
    {HSYEN, 0},   

    {MVFP, MVFP_SUN}, 

	/* More reserved magic, some of which tweaks white balance */
	{AWBC1, 0x0a},		
    {AWBC2, 0xf0},
	{AWBC3, 0x34},		
    {AWBC4, 0x58},
	{AWBC5, 0x28},		
    {AWBC6, 0x3a},
	
    {AWBCTR3, 0x0a},		
    {AWBCTR2, 0x55},
	{AWBCTR1, 0x11},		
    {AWBCTR0, 0x9e}, 

    {COM8, COM8_FAST_AUTO|COM8_STEP_UNLIMIT|COM8_AGC_EN|COM8_AEC_EN|COM8_AWB_EN},

    /* End marker is FF because in ov7670 the address of GAIN 0 and default value too. */
    {0xFF, 0xFF},  
};

static struct regval_list ov7670_fmt_yuv422[] = {
	{ COM7,     0x0                         },  /* Selects YUV mode */
	{ RGB444,   0                           },  /* No RGB444 please */
	{ COM1,     0                           },  /* CCIR601 */
	{ COM15,    COM15_R00FF                 },
    { MVFP,     MVFP_SUN                    }, 
	{ COM9,     0x6A                        },  /* 128x gain ceiling; 0x8 is reserved bit */
	{ MTX1,     0x80                        },  /* "matrix coefficient 1" */
	{ MTX2,     0x80                        }, 	/* "matrix coefficient 2" */
	{ MTX3,     0                           },  /* vb */
	{ MTX4,     0x22                        }, 	/* "matrix coefficient 4" */
	{ MTX5,     0x5e                        },  /* "matrix coefficient 5" */
	{ MTX6,     0x80                        },  /* "matrix coefficient 6" */
	{ COM13,    COM13_UVSAT                 },
	{ 0xff,     0xff                        },  /* END MARKER */
};

static struct regval_list ov7670_fmt_rgb565[] = {
	{ COM7,     COM7_FMT_RGB565             },	/* Selects RGB mode */
	{ RGB444,   0                           },	/* No RGB444 please */
	{ COM1,     0x0                         },	/* CCIR601 */
	{ COM15,    COM15_RGB565 |COM15_R00FF   },
    { MVFP,     MVFP_SUN                    },   
	{ COM9,     0x6A                        }, 	/* 128x gain ceiling; 0x8 is reserved bit */
	{ MTX1,     0xb3                        }, 	/* "matrix coefficient 1" */
	{ MTX2,     0xb3                        }, 	/* "matrix coefficient 2" */
	{ MTX3,     0                           },	/* vb */
	{ MTX4,     0x3d                        }, 	/* "matrix coefficient 4" */
	{ MTX5,     0xa7                        }, 	/* "matrix coefficient 5" */
	{ MTX6,     0xe4                        }, 	/* "matrix coefficient 6" */
	{ COM13,    COM13_UVSAT                 },
	{ 0xff,     0xff                        },  /* END MARKER */
};

static struct regval_list ov7670_vga[] = {
    { COM3,                 0x00 },
    { COM14,                0x00 },
    { SCALING_XSC,          0x3A },
    { SCALING_YSC,          0x35 },
    { SCALING_DCWCTR,       0x11 },
    { SCALING_PCLK_DIV,     0xF0 },
    { SCALING_PCLK_DELAY,   0x02 },
    { 0xff, 0xff },
};

/* These values from Omnivision */
static struct windowing wind_vga = {
    .hstart = 158, 
    .hstop = 14,
    .vstart = 10,
    .vstop = 490
};

static struct regval_list ov7670_cif[] = {
    { COM3,                 0x08 },
    { COM14,                0x11 },
    { SCALING_XSC,          0x3A },
    { SCALING_YSC,          0x35 },
    { SCALING_DCWCTR,       0x11 },
    { SCALING_PCLK_DIV,     0xF1 },
    { SCALING_PCLK_DELAY,   0x02 },
    { 0xff, 0xff },
};

/* These values empirically determined */
static struct windowing wind_cif = {
    .hstart = 170, 
    .hstop = 90, 
    .vstart = 14,
    .vstop = 494
};

static struct regval_list ov7670_qvga[] = {
    { COM3,                 0x04 },
    { COM14,                0x19 },
    { SCALING_XSC,          0x3A },
    { SCALING_YSC,          0x35 },
    { SCALING_DCWCTR,       0x11 },
    { SCALING_PCLK_DIV,     0xF1 },
    { SCALING_PCLK_DELAY,   0x02 },
    { 0xff, 0xff },
};

/* These values empirically determined */
static struct windowing wind_qvga = {
    .hstart = 168, 
    .hstop = 24,  
    .vstart = 10,
    .vstop = 490
};

static struct regval_list ov7670_qcif[] = {
    { COM3,                 0x0C },
    { COM14,                0x11 },
    { SCALING_XSC,          0x3A },
    { SCALING_YSC,          0x35 },
    { SCALING_DCWCTR,       0x11 },
    { SCALING_PCLK_DIV,     0xF1 },
    { SCALING_PCLK_DELAY,   0x52 },
    { 0xff, 0xff },
};

/* These values empirically determined */
static struct windowing wind_qcif = {
    .hstart = 454,  
    .hstop = 22,   
    .vstart = 14,
    .vstop = 494
};

static struct regval_list ov7670_qqvga[] = {
	{ COM3,                 0x04 }, //DCW enable	
	{ COM14,                0x1a }, //pixel clock divided by 4, manual scaling enable, DCW and PCLK controlled by register	
	{ SCALING_XSC,          0x3a },	
	{ SCALING_YSC,          0x35 },
	{ SCALING_DCWCTR,       0x22 }, //downsample by 4	
	{ SCALING_PCLK_DIV,     0xf2 }, //pixel clock divided by 4	
	{ SCALING_PCLK_DELAY,   0x02 },
    { 0xff, 0xff },
};

/* These values empirically determined */
static struct windowing wind_qqvga = {
    .hstart = 190,
    .hstop = 46,
    .vstart = 10,
    .vstop = 490
};

/*
 * Write a list of register settings; ff/ff stops the process.
 */
static int ov7670_write_array(sensor_t *sensor, struct regval_list *vals)
{
int ret = 0;
	
	while ( (vals->reg_num != 0xff || vals->value != 0xff) && (ret == 0) ) {
        ret = SCCB_Write(sensor->slv_addr, vals->reg_num, vals->value);

	    ESP_LOGD(TAG, "reset reg %02X, W(%02X) R(%02X)", vals->reg_num, 
                        vals->value, SCCB_Read(sensor->slv_addr, vals->reg_num) );
		
		vals++;
	}

    return ret;
}

/*
 * Calculate the frame control registers.
 */
static int ov7670_frame_control(sensor_t *sensor, struct windowing *wind_select )
{
struct regval_list frame[7];

    frame[0].reg_num = HSTART;
    frame[0].value = (wind_select->hstart >> 3) & 0xFF;

    frame[1].reg_num = HSTOP;
    frame[1].value = (wind_select->hstop >> 3) & 0xFF;

    frame[2].reg_num = HREF;
    frame[2].value = (((wind_select->hstop & 0x07) << 3) | (wind_select->hstart & 0x07));
    
    frame[3].reg_num = VSTART;
    frame[3].value = (wind_select->vstart >> 2) & 0xFF;
    
    frame[4].reg_num = VSTOP;
    frame[4].value = (wind_select->vstop >> 2) & 0xFF;

    frame[5].reg_num = VREF;
    frame[5].value = (((wind_select->vstop & 0x02) << 2) | (wind_select->vstart & 0x02));

    /* End mark */
    frame[6].reg_num = 0xFF;
    frame[6].value = 0xFF;

    return ov7670_write_array(sensor, frame);
}

static int reset(sensor_t *sensor)
{
int ret;

    // Reset all registers
    SCCB_Write(sensor->slv_addr, COM7, COM7_RESET);

    // Delay 10 ms
    vTaskDelay(10 / portTICK_PERIOD_MS);

    ret = ov7670_write_array(sensor, ov7670_default_regs);

    // Delay
    vTaskDelay(30 / portTICK_PERIOD_MS);

    return ret;
}

static int set_pixformat(sensor_t *sensor, pixformat_t pixformat)
{
int ret;

    switch (pixformat) {
        case PIXFORMAT_RGB565:
        case PIXFORMAT_RGB888:
            ret = ov7670_write_array(sensor, ov7670_fmt_rgb565);
        break;
 
        case PIXFORMAT_YUV422:
        case PIXFORMAT_GRAYSCALE:
	    default:
            ret = ov7670_write_array(sensor, ov7670_fmt_yuv422);
        break;
    }

    vTaskDelay(30 / portTICK_PERIOD_MS);

    /*
	 * If we're running RGB565, we must rewrite clkrc after setting
	 * the other parameters or the image looks poor.  If we're *not*
	 * doing RGB565, we must not rewrite clkrc or the image looks
	 * *really* poor.
	 *
	 * (Update) Now that we retain clkrc state, we should be able
	 * to write it unconditionally, and that will make the frame
	 * rate persistent too.
	 */
    if (pixformat == PIXFORMAT_RGB565) {
        ret = SCCB_Write(sensor->slv_addr, CLKRC, ov7670_clkrc); 
    }

    return ret;
}

static int set_framesize(sensor_t *sensor, framesize_t framesize)
{
int ret;
struct regval_list *regs;
struct windowing *wind_select;

    // store clkrc before changing window settings...
    ov7670_clkrc =  SCCB_Read(sensor->slv_addr, CLKRC);
     
	switch (framesize){
        case FRAMESIZE_VGA:
            regs = ov7670_vga;
            wind_select = &wind_vga;
        break;
        case FRAMESIZE_CIF:
            regs = ov7670_cif;
            wind_select = &wind_cif;    
        break;
	    case FRAMESIZE_QVGA:
            regs = ov7670_qvga;
            wind_select = &wind_qvga;
        break;
        case FRAMESIZE_QCIF:
            regs = ov7670_qcif;
            wind_select = &wind_qcif;
        break;
	    case FRAMESIZE_QQVGA:
            regs = ov7670_qqvga;
            wind_select = &wind_qqvga;
        break; 

        default:
            return -1;   
    }

    if ((ret = ov7670_write_array(sensor, regs)) == 0) {
        if ((ret = ov7670_frame_control(sensor, wind_select)) == 0) {
            sensor->status.framesize = framesize;         
        }
    }
            
    vTaskDelay(30 / portTICK_PERIOD_MS);

    return ret;
}

static int set_colorbar(sensor_t *sensor, int enable)
{
int8_t ret = 0;
    // Read register scaling_xsc
    uint8_t reg = SCCB_Read(sensor->slv_addr, SCALING_XSC);

    // Pattern to set color bar bit[0]=0 in every case
    reg = SCALING_XSC_CBAR(reg);

    // Write pattern to SCALING_XSC
    ret = SCCB_Write(sensor->slv_addr, SCALING_XSC, reg);

    // Read register scaling_ysc
    reg = SCCB_Read(sensor->slv_addr, SCALING_YSC);

    // Pattern to set color bar bit[0]=0 in every case
    reg = SCALING_YSC_CBAR(reg, enable);

    // Write pattern to SCALING_YSC
    ret = ret | SCCB_Write(sensor->slv_addr, SCALING_YSC, reg);

    // return 0 or 0xFF
    return ret;
}

static int set_whitebal(sensor_t *sensor, int enable)
{
    // Read register COM8
    uint8_t reg = SCCB_Read(sensor->slv_addr, COM8);

    // Set white bal on/off
    reg = COM8_SET_AWB(reg, enable);

    // Write back register COM8
    return SCCB_Write(sensor->slv_addr, COM8, reg);
}

static int set_gain_ctrl(sensor_t *sensor, int enable)
{
    // Read register COM8
    uint8_t reg = SCCB_Read(sensor->slv_addr, COM8);

    // Set white bal on/off
    reg = COM8_SET_AGC(reg, enable);

    // Write back register COM8
    return SCCB_Write(sensor->slv_addr, COM8, reg);
}

static int set_exposure_ctrl(sensor_t *sensor, int enable)
{
    // Read register COM8
    uint8_t reg = SCCB_Read(sensor->slv_addr, COM8);

    // Set white bal on/off
    reg = COM8_SET_AEC(reg, enable);

    // Write back register COM8
    return SCCB_Write(sensor->slv_addr, COM8, reg);
}

static int set_hmirror(sensor_t *sensor, int enable)
{
    // Read register MVFP
    uint8_t reg = SCCB_Read(sensor->slv_addr, MVFP);

    // Set mirror on/off
    reg = MVFP_SET_MIRROR(reg, enable);

    // Write back register MVFP
    return SCCB_Write(sensor->slv_addr, MVFP, reg);
}

static int set_vflip(sensor_t *sensor, int enable)
{
    // Read register MVFP
    uint8_t reg = SCCB_Read(sensor->slv_addr, MVFP);

    // Set mirror on/off
    reg = MVFP_SET_FLIP(reg, enable);

    // Write back register MVFP
    return SCCB_Write(sensor->slv_addr, MVFP, reg);
}

static int init_status(sensor_t *sensor)
{
    sensor->status.awb = 0;
    sensor->status.aec = 0;
    sensor->status.agc = 0;
    sensor->status.hmirror = 0;
    sensor->status.vflip = 0;
    sensor->status.colorbar = 0;
    return 0;
}

static int set_dummy(sensor_t *sensor, int val){ return -1; }
static int set_gainceiling_dummy(sensor_t *sensor, gainceiling_t val){ return -1; }

int ov7670_init(sensor_t *sensor)
{
    // Set function pointers
    sensor->reset = reset;
    sensor->init_status = init_status;
    sensor->set_pixformat = set_pixformat;
    sensor->set_framesize = set_framesize;
    sensor->set_colorbar = set_colorbar;
    sensor->set_whitebal = set_whitebal;
    sensor->set_gain_ctrl = set_gain_ctrl;
    sensor->set_exposure_ctrl = set_exposure_ctrl;
    sensor->set_hmirror = set_hmirror;
    sensor->set_vflip = set_vflip;

    //not supported
    sensor->set_brightness= set_dummy;
    sensor->set_saturation= set_dummy;
    sensor->set_quality = set_dummy;
    sensor->set_gainceiling = set_gainceiling_dummy;
    sensor->set_aec2 = set_dummy;
    sensor->set_aec_value = set_dummy;
    sensor->set_special_effect = set_dummy;
    sensor->set_wb_mode = set_dummy;
    sensor->set_ae_level = set_dummy;
    sensor->set_dcw = set_dummy;
    sensor->set_bpc = set_dummy;
    sensor->set_wpc = set_dummy;
    sensor->set_awb_gain = set_dummy;
    sensor->set_agc_gain = set_dummy;
    sensor->set_raw_gma = set_dummy;
    sensor->set_lenc = set_dummy;
    sensor->set_sharpness = set_dummy;
    sensor->set_denoise = set_dummy;

    // Retrieve sensor's signature
    sensor->id.MIDH = SCCB_Read(sensor->slv_addr, REG_MIDH);
    sensor->id.MIDL = SCCB_Read(sensor->slv_addr, REG_MIDL);
    sensor->id.PID = SCCB_Read(sensor->slv_addr, REG_PID);
    sensor->id.VER = SCCB_Read(sensor->slv_addr, REG_VER);
    
    ESP_LOGD(TAG, "OV7670 Attached");
    
    return 0;
}
