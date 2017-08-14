/*
 * Weida HiTech WDT87xx TouchScreen I2C driver
 *
 * Copyright (c) 2015  Weida Hi-Tech Co., Ltd.
 * HN Chen <hn.chen@weidahitech.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 */

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/firmware.h>
#include <linux/input/mt.h>
#include <linux/acpi.h>
#include <asm/unaligned.h>
#include <linux/gpio.h>
#include <linux/version.h>
#include <linux/debugfs.h>
#include <linux/of_gpio.h>

#define dev_dbg(dev, format, arg...)				\
({								\
    dev_printk(KERN_DEBUG, dev, format, ##arg);	\
})

#define		EXYNOS5			0 	// exynos5410
#define		MINNOWBOARD		0	// minnowboard max
#define		ROCKCHIP		0	// RK3188
#define		MTK			0	
#define		ALLWINNER		0	// AW A83
#define		BYPASS_GET_PARAM	0
#define     SABRELITE       1

#if ROCKCHIP
#define		ROCKCHIP_3288_OF	0	// RK3288 device tree
#endif

#if ROCKCHIP	
#define		I2C_MASTER_CLK		400 * 1000
#endif

#define		I2C_RP_ST		0x01

#if MTK
#define		I2C_DMA_MAX_TRANS_SZ	255
#endif

#if MTK
#define		I2C_MASTER_CLK		400
#include <linux/dma-mapping.h>
#endif

#ifdef CONFIG_EARLYSUSPEND
#define		EARLY_SUSPEND		1
#else
#define		EARLY_SUSPEND		0
#endif

#if 	EARLY_SUSPEND
#include <linux/earlysuspend.h>
#endif

#if 	ALLWINNER
#include <linux/init-input.h>
#endif

/* the definition for EXYNOS5 */
#if EXYNOS5
#include <mach/hardware.h>
#include <plat/gpio-cfg.h>
#include <plat/irqs.h>
#endif

#define WDT87XX_NAME			"wdt87xx_i2c"
#define WDT87XX_DRV_VER			"0.9.25"
#define WDT87XX_FW_NAME			"wdt87xx_fw.bin"
#define WDT87XX_CFG_NAME		"wdt87xx_cfg.bin"

#define	ST_INIT				0x00
#define	ST_REPORT			0x01
#define	ST_PROG				0x02

#define	RPT_PARALLEL_74			0x00
#define	RPT_PARALLEL_54			0x01
#define	RPT_HYBRID_54			0x02
#define	RPT_HID_HYBRID			0x03

#define	PLT_WDT8756			0x00
#define	PLT_WDT8752			0x01

#define	RPT_ID_TOUCH			0x01
#define	RPT_ID_PEN			0x02
#define	RPT_ID_MOUSE			0x03

#define MODE_ACTIVE			0x01
#define MODE_READY			0x02
#define MODE_IDLE			0x03
#define MODE_SLEEP			0x04
#define MODE_STOP			0xFF

#define WDT_MAX_FINGER			10
#define WDT_RAW_BUF_COUNT		54
#define WDT_V1_RAW_BUF_COUNT		74
#define	WDT_HALF_RAW_BUF_COUNT		29
#define WDT_FIRMWARE_ID			0xa9e368f5


#define PG_SIZE				0x1000
#define MAX_RETRIES			3

#define MAX_UNIT_AXIS			0x7FFF

#define	PKT_TX_SIZE			16
#define PKT_READ_SIZE			72
#define PKT_WRITE_SIZE			80

#define	PKT_HYBRID_SIZE			31
#define	PKT_PEN_SIZE			10
#define	PKT_MOUSE_SIZE			8

/* the finger definition of the report event */
#define FINGER_EV_OFFSET_ID		0
#define FINGER_EV_OFFSET_X		1
#define FINGER_EV_OFFSET_Y		3
#define FINGER_EV_SIZE			5

#define FINGER_EV_V1_OFFSET_ID		0
#define FINGER_EV_V1_OFFSET_W		1
#define FINGER_EV_V1_OFFSET_P		2
#define FINGER_EV_V1_OFFSET_X		3
#define FINGER_EV_V1_OFFSET_Y		5
#define FINGER_EV_V1_SIZE		7

#define	PEN_EV_OFFSET_BTN		1
#define	PEN_EV_OFFSET_X			2
#define	PEN_EV_OFFSET_Y			4
#define	PEN_EV_OFFSET_P			6

#define MOUSE_EV_OFFSET_BTN		1
#define MOUSE_EV_OFFSET_X		2
#define MOUSE_EV_OFFSET_Y		4

/* The definition of a report packet */
#define TOUCH_PK_OFFSET_REPORT_ID	0
#define TOUCH_PK_OFFSET_EVENT		1
#define TOUCH_PK_OFFSET_SCAN_TIME	51
#define TOUCH_PK_OFFSET_FNGR_NUM	53

#define TOUCH_PK_V1_OFFSET_REPORT_ID	0
#define TOUCH_PK_V1_OFFSET_EVENT	1
#define TOUCH_PK_V1_OFFSET_SCAN_TIME	71
#define TOUCH_PK_V1_OFFSET_FNGR_NUM	73

#define TOUCH_PK_HALF_OFFSET_REPORT_ID	0
#define TOUCH_PK_HALF_OFFSET_EVENT	1
#define TOUCH_PK_HALF_OFFSET_SCAN_TIME	26
#define TOUCH_PK_HALF_OFFSET_FNGR_NUM	28

/* The definition of the firmware id string */
#define FW_ID_OFFSET_FW_ID		1
#define FW_ID_OFFSET_N_TCH_PKT		13
#define	FW_ID_OFFSET_N_BT_TCH		14

/* The definition of the controller parameters */
#define CTL_PARAM_OFFSET_FW_ID		0
#define CTL_PARAM_OFFSET_PLAT_ID	2
#define CTL_PARAM_OFFSET_XMLS_ID1	4
#define CTL_PARAM_OFFSET_XMLS_ID2	6
#define CTL_PARAM_OFFSET_PHY_CH_X	8
#define CTL_PARAM_OFFSET_PHY_CH_Y	10
#define CTL_PARAM_OFFSET_PHY_X0		12
#define CTL_PARAM_OFFSET_PHY_X1		14
#define CTL_PARAM_OFFSET_PHY_Y0		16
#define CTL_PARAM_OFFSET_PHY_Y1		18
#define CTL_PARAM_OFFSET_PHY_W		22
#define CTL_PARAM_OFFSET_PHY_H		24
#define CTL_PARAM_OFFSET_FACTOR		32
#define	CTL_PARAM_OFFSET_I2C_CFG	36

/* The definition of the device descriptor */
#define WDT_GD_DEVICE			1
#define DEV_DESC_OFFSET_VID		8
#define DEV_DESC_OFFSET_PID		10

/* Communication commands */
#define PACKET_SIZE			56
#define VND_REQ_READ			0x06
#define VND_READ_DATA			0x07
#define VND_REQ_WRITE			0x08

#define VND_CMD_START			0x00
#define VND_CMD_STOP			0x01
#define VND_CMD_RESET			0x09

#define VND_CMD_ERASE			0x1A

#define VND_GET_CHECKSUM		0x66

#define VND_SET_DATA			0x83
#define VND_SET_COMMAND_DATA		0x84
#define VND_SET_CHECKSUM_CALC		0x86
#define VND_SET_CHECKSUM_LENGTH		0x87

#define	VND1_SET_PARAMETER		0x90
#define VND1_SET_COMMAND		0x91

#define VND_CMD_SFLCK			0xFC
#define VND_CMD_SFUNL			0xFD

#define CMD_SFLCK_KEY			0xC39B
#define CMD_SFUNL_KEY			0x95DA

#define STRIDX_PLATFORM_ID		0x80
#define STRIDX_PARAMETERS		0x81

#define CMD_BUF_SIZE			8
#define PKT_BUF_SIZE			64

/* The definition of the command packet */
#define CMD_REPORT_ID_OFFSET		0x0
#define CMD_TYPE_OFFSET			0x1
#define CMD_INDEX_OFFSET		0x2
#define CMD_KEY_OFFSET			0x3
#define CMD_LENGTH_OFFSET		0x4
#define CMD_DATA_OFFSET			0x8

/* The definition of firmware chunk tags */
#define FOURCC_ID_RIFF			0x46464952
#define FOURCC_ID_WHIF			0x46494857
#define FOURCC_ID_FRMT			0x544D5246
#define FOURCC_ID_FRWR			0x52575246
#define FOURCC_ID_CNFG			0x47464E43

#define CHUNK_ID_FRMT			FOURCC_ID_FRMT
#define CHUNK_ID_FRWR			FOURCC_ID_FRWR
#define CHUNK_ID_CNFG			FOURCC_ID_CNFG

#define FW_FOURCC1_OFFSET		0
#define FW_SIZE_OFFSET			4
#define FW_FOURCC2_OFFSET		8
#define FW_PAYLOAD_OFFSET		40

#define FW_CHUNK_ID_OFFSET		0
#define FW_CHUNK_SIZE_OFFSET		4
#define FW_CHUNK_TGT_START_OFFSET	8
#define FW_CHUNK_PAYLOAD_LEN_OFFSET	12
#define FW_CHUNK_SRC_START_OFFSET	16
#define FW_CHUNK_VERSION_OFFSET		20
#define FW_CHUNK_ATTR_OFFSET		24
#define FW_CHUNK_PAYLOAD_OFFSET		32

/* Controller requires minimum 300us between commands */
#define WDT_COMMAND_DELAY_MS		2
#define WDT_FLASH_WRITE_DELAY_MS	4
#define WDT_FW_RESET_TIME		2500

/* The definition for WDT8752 */
#define	W8752_READ_OFFSET_MASK		0x10000
#define W8752_DEV_INFO_READ_OFFSET	0xC
#define	W8752_PKT_HEADER_SZ		4
#define	W8752_PKT_SIZE			60

#define W8752_MODE_SENSE		0x1
#define	W8752_MODE_DOZE			0x2
#define W8752_MODE_SLEEP		0x3
/* Communication commands of WDT8752 */
#define	W8755_FW_GET_DEV_INFO		0x73

#define CMD_SIZE_OFFSET			0x2
#define CMD_ID_OFFSET			0x4
#define CMD_DATA1_OFFSET		0x4
#define CMD_VALUE_OFFSET		0x5


#if	EXYNOS5
#define	FT_INT_PORT			GPIO_CTP_INT
#define	FT_WAKE_PORT			GPIO_CTP_RST
#define	EINT_NUM			IRQ_CTP_INT

#endif

#if MINNOWBOARD
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,18,0)
#define	DINT_GPIO_NUM			82
#else
#define DINT_GPIO_NUM			338
#endif
#endif

#if	ALLWINNER
static struct ctp_config_info config_info = {
	.input_type = CTP_TYPE,
	.name = NULL,
	.int_number = 0,
};

static const unsigned short weida_i2c[2] = {0x2C, I2C_CLIENT_END};
#endif


struct i2c_hid_desc {
	__le16 wHIDDescLength;
	__le16 bcdVersion;
	__le16 wReportDescLength;
	__le16 wReportDescRegister;
	__le16 wInputRegister;
	__le16 wMaxInputLength;
	__le16 wOutputRegister;
	__le16 wMaxOutputLength;
	__le16 wCommandRegister;
	__le16 wDataRegister;
	__le16 wVendorID;
	__le16 wProductID;
	__le16 wVersionID;
	__le32 reserved;
} __packed;
 
struct wdt87xx_param {
	u16	fw_id;
	u16	plat_id;
	u16	xmls_id1;
	u16	xmls_id2;
	u16	phy_ch_x;
	u16	phy_ch_y;
	u16	phy_w;
	u16	phy_h;
	u16	scaling_factor;
	u32	max_x;
	u32	max_y;
	u16	vendor_id;
	u16	product_id;
	u16 	i2c_cfg;
	u32	protocol_version;	
	u8	n_tch_pkt;
	u8	n_bt_tch;
} __packed;

struct wdt87xx_data;

typedef	int	(* LPFUNC_report_type) (struct wdt87xx_data *wdt);

struct wdt87xx_data {
	struct i2c_client		*client;
	struct input_dev		*input_mt;
	struct input_dev		*input_pen;
	struct input_dev		*input_mouse;
	/* Mutex for fw update to prevent concurrent access */
	struct mutex			fw_mutex;
	struct wdt87xx_param		param;
	struct i2c_hid_desc		hid_desc;
	u8				phys[32];
	u32				plt_id;
	u32				state;	
	bool				wake_irq_enabled;
		
	u32				report_type;
	u32 				dummy_bytes;
	u32				dev_status;
	LPFUNC_report_type		func_report_type[4];
	u32				fngr_left;
	int				pen_type;
	u32 				btn_state;
	u32				fngr_state;
#if 	EARLY_SUSPEND
	struct early_suspend	wdt_early_suspend;
#endif	
#if	MTK
	u8 				*p_dma_va;
	dma_addr_t 			p_dma_pa;
	struct mutex 			dma_mutex;
#endif
	struct dentry 	*dbg_root;
	struct dentry 	*dbg_st;
	u32				rpt_scantime;
};

static int wdt87xx_i2c_rx(struct i2c_client *client,
			    void *rxdata, size_t rxlen)
{
#if	MTK
	struct wdt87xx_data *wdt = i2c_get_clientdata(client);
#endif	
	struct i2c_msg msg = {
		.addr		= client->addr,
		.flags		= I2C_M_RD,
		.len		= rxlen,
		.buf		= rxdata,
#if ROCKCHIP
		.scl_rate	= I2C_MASTER_CLK,
#if !ROCKCHIP_3288_OF
		.udelay 	= client->udelay,
#endif
#endif
	};
	int ret;
	
#if 	MTK
	if (rxlen > I2C_DMA_MAX_TRANS_SZ)
		return -EINVAL;

	msg.ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG);
	msg.timing = I2C_MASTER_CLK;
	msg.buf = (u8 *) wdt->p_dma_pa;
	
	mutex_lock(&wdt->dma_mutex);
#endif

	ret = i2c_transfer(client->adapter, &msg, 1);
	
#if 	MTK
	memcpy(rxdata, wdt->p_dma_va, rxlen);
	mutex_unlock(&wdt->dma_mutex);
#endif	
	return (ret == 1) ? rxlen : ret;
}

static int wdt87xx_i2c_tx(struct i2c_client *client,
			    void *txdata, size_t txlen)
{
#if	MTK
	struct wdt87xx_data *wdt = i2c_get_clientdata(client);
#endif	
	struct i2c_msg msg = {
		.addr		= client->addr,
		.flags		= 0,
		.len		= txlen,
		.buf		= txdata,
#if ROCKCHIP
		.scl_rate	= I2C_MASTER_CLK,
#if !ROCKCHIP_3288_OF
		.udelay 	= client->udelay,
#endif
#endif
	};
	int ret;
	
#if 	MTK
	if (txlen > I2C_DMA_MAX_TRANS_SZ)
		return -EINVAL;

	msg.ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG);
	msg.timing = I2C_MASTER_CLK;
	msg.buf = (u8 *) wdt->p_dma_pa;
	
	memcpy(wdt->p_dma_va, txdata, txlen);
	
	mutex_lock(&wdt->dma_mutex);
#endif
	ret = i2c_transfer(client->adapter, &msg, 1);
	
#if 	MTK
	mutex_unlock(&wdt->dma_mutex);
#endif	
	return (ret == 1) ? txlen : ret;
}

#if 	MTK
static int wdt87xx_i2c_xfer(struct i2c_client *client,
			    void *txdata, size_t txlen,
			    void *rxdata, size_t rxlen)
{
	int ret;
	
	ret = wdt87xx_i2c_tx(client, txdata, txlen);
	if (ret < 0)
		return ret;

	udelay(100);		
	
	ret = wdt87xx_i2c_rx(client, rxdata, rxlen);
	if (ret < 0)
		return ret;
	
	return 0;	
}
#else
static int wdt87xx_i2c_xfer(struct i2c_client *client,
			    void *txdata, size_t txlen,
			    void *rxdata, size_t rxlen)
{
	struct i2c_msg msgs[] = {
		{
			.addr	= client->addr,
			.flags	= 0,
			.len	= txlen,
			.buf	= txdata,
#if ROCKCHIP
			.scl_rate	= I2C_MASTER_CLK,
#if !ROCKCHIP_3288_OF
			.udelay 	= client->udelay,
#endif
#endif
		},
		{
			.addr	= client->addr,
			.flags	= I2C_M_RD,
			.len	= rxlen,
			.buf	= rxdata,
#if ROCKCHIP
			.scl_rate	= I2C_MASTER_CLK,
#if !ROCKCHIP_3288_OF
			.udelay 	= client->udelay,
#endif
#endif
		},
	};
	int error;
	int ret;
	int array_sz = 1;
#if ROCKCHIP
	/* Repeat start isn't supported by RK */
	int flag = 0;
#else
	int flag = I2C_RP_ST;
#endif
	/* A special command for retrieve i2c cfg */
	if (msgs[0].buf[4] == 0xf4)
		flag &= ~I2C_RP_ST;	

	if (flag & I2C_RP_ST) {
		array_sz = 2;
		ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	} else {
		ret = i2c_transfer(client->adapter, &msgs[0], 1);
		if (ret == array_sz) {
			mdelay(WDT_COMMAND_DELAY_MS);
			ret = i2c_transfer(client->adapter, &msgs[1], 1);
		}
	} 

	if (ret != array_sz) {
		error = ret < 0 ? ret : -EIO;
		dev_err(&client->dev, "%s: i2c transfer failed: %d\n",
			__func__, error);
		return error;
	}

	return 0;
}
#endif

#if !BYPASS_GET_PARAM

static int wdt87xx_get_desc(struct i2c_client *client, u8 desc_idx,
			    u8 *buf, size_t len)
{
	u8 tx_buf[] = { 0x22, 0x00, 0x10, 0x0E, 0x23, 0x00 };
	struct wdt87xx_data *wdt = i2c_get_clientdata(client);
	int error;

	tx_buf[2] |= desc_idx & 0xF;

	error = wdt87xx_i2c_xfer(client, tx_buf, sizeof(tx_buf)
				 + wdt->dummy_bytes, buf, len);
        
	if (error) {
		dev_err(&client->dev, "get desc failed: %d\n", error);
		return error;
	}

	if (buf[0] != len) {
		dev_err(&client->dev, "unexpected response to get desc: %d\n",
			buf[0]);
		return -EINVAL;
	}

	mdelay(WDT_COMMAND_DELAY_MS);

	return 0;
}

static int wdt87xx_get_string(struct i2c_client *client, u8 str_idx,
			      u8 *buf, size_t len)
{
	u8 tx_buf[] = { 0x22, 0x00, 0x13, 0x0E, str_idx, 0x23, 0x00 };
	u8 rx_buf[PKT_WRITE_SIZE];
	struct wdt87xx_data *wdt = i2c_get_clientdata(client);
	size_t rx_len = len + 2;
	int error;

	if (rx_len > sizeof(rx_buf))
		return -EINVAL;

	error = wdt87xx_i2c_xfer(client, tx_buf, sizeof(tx_buf) +
				 wdt->dummy_bytes, rx_buf, rx_len);
	if (error) {
		dev_err(&client->dev, "get string failed: %d\n", error);
		return error;
	}

	if (rx_buf[1] != 0x03) {
		dev_err(&client->dev, "unexpected response to get string: %d\n",
			rx_buf[1]);
		return -EINVAL;
	}

	rx_len = min_t(size_t, len, rx_buf[0]);
	memcpy(buf, &rx_buf[2], rx_len);

	mdelay(WDT_COMMAND_DELAY_MS);

	return 0;
}
#endif

static int wdt87xx_get_feature(struct i2c_client *client,
			       u8 *buf, size_t buf_size)
{
	struct wdt87xx_data *wdt = i2c_get_clientdata(client);
	u8 tx_buf[PKT_TX_SIZE];
	u8 rx_buf[PKT_WRITE_SIZE];
	size_t tx_len = 0;
	size_t rx_len = buf_size + 2;
	int error;

	if (rx_len > sizeof(rx_buf))
		return -EINVAL;

	memset(tx_buf, 0, sizeof(tx_buf));

	/* Get feature command packet */
	tx_buf[tx_len++] = 0x22;
	tx_buf[tx_len++] = 0x00;
	if (buf[CMD_REPORT_ID_OFFSET] > 0xF) {
		tx_buf[tx_len++] = 0x30;
		tx_buf[tx_len++] = 0x02;
		tx_buf[tx_len++] = buf[CMD_REPORT_ID_OFFSET];
	} else {
		tx_buf[tx_len++] = 0x30 | buf[CMD_REPORT_ID_OFFSET];
		tx_buf[tx_len++] = 0x02;
	}
	tx_buf[tx_len++] = 0x23;
	tx_buf[tx_len++] = 0x00;
		
	tx_len += wdt->dummy_bytes;

	error = wdt87xx_i2c_xfer(client, tx_buf, tx_len, rx_buf, rx_len);
	if (error) {
		dev_err(&client->dev, "get feature failed: %d\n", error);
		return error;
	}

	rx_len = min_t(size_t, buf_size, get_unaligned_le16(rx_buf));
	memcpy(buf, &rx_buf[2], rx_len);
	
	mdelay(WDT_COMMAND_DELAY_MS);
	
	return 0;
}

static int wdt87xx_set_feature(struct i2c_client *client, u8 rpt_id,
			       const u8 *buf, size_t buf_size)
{
	struct wdt87xx_data *wdt = i2c_get_clientdata(client);
	u8 tx_buf[PKT_WRITE_SIZE];
	int tx_len = 0;
	int error;

	/* Set feature command packet */
	tx_buf[tx_len++] = 0x22;
	tx_buf[tx_len++] = 0x00;
	if (rpt_id > 0xF) {
		tx_buf[tx_len++] = 0x30;
		tx_buf[tx_len++] = 0x03;
		tx_buf[tx_len++] = rpt_id;
	} else {
		tx_buf[tx_len++] = 0x30 | rpt_id;
		tx_buf[tx_len++] = 0x03;
	}
	tx_buf[tx_len++] = 0x23;
	tx_buf[tx_len++] = 0x00;
	
	if ((wdt->plt_id == PLT_WDT8752 &&
	     wdt->param.protocol_version >= 0x1000007) ||
	     wdt->state != ST_PROG) {
		tx_buf[tx_len++] = ((buf_size + 2) & 0xFF);
		tx_buf[tx_len++] = (((buf_size + 2) & 0xFF00) >> 8);
	} else {
		tx_buf[tx_len++] = (buf_size & 0xFF);
		tx_buf[tx_len++] = ((buf_size & 0xFF00) >> 8);
	}	

	if (tx_len + buf_size > sizeof(tx_buf))
		return -EINVAL;

	memcpy(&tx_buf[tx_len], buf, buf_size);
	tx_len += buf_size;

	error = wdt87xx_i2c_tx(client, tx_buf, tx_len);
	if (error < 0) {
		dev_err(&client->dev, "set feature failed: %d\n", error);
		return error;
	}

	mdelay(WDT_COMMAND_DELAY_MS);
	
	return 0;
}

static int wdt87xx_send_command(struct i2c_client *client, int cmd, int value)
{
	u8 cmd_buf[CMD_BUF_SIZE];

	/* Set the command packet */
	cmd_buf[CMD_REPORT_ID_OFFSET] = VND_REQ_WRITE;
	cmd_buf[CMD_TYPE_OFFSET] = VND_SET_COMMAND_DATA;
	put_unaligned_le16((u16)cmd, &cmd_buf[CMD_INDEX_OFFSET]);

	switch (cmd) {
	case VND_CMD_START:
	case VND_CMD_STOP:
	case VND_CMD_RESET:
		/* Mode selector */
		put_unaligned_le32((value & 0xFF), &cmd_buf[CMD_LENGTH_OFFSET]);
		break;

	case VND_CMD_SFLCK:
		put_unaligned_le16(CMD_SFLCK_KEY, &cmd_buf[CMD_KEY_OFFSET]);
		break;

	case VND_CMD_SFUNL:
		put_unaligned_le16(CMD_SFUNL_KEY, &cmd_buf[CMD_KEY_OFFSET]);
		break;

	case VND_CMD_ERASE:
	case VND_SET_CHECKSUM_CALC:
	case VND_SET_CHECKSUM_LENGTH:
		put_unaligned_le32(value, &cmd_buf[CMD_KEY_OFFSET]);
		break;

	default:
		cmd_buf[CMD_REPORT_ID_OFFSET] = 0;
		dev_err(&client->dev, "Invalid command: %d\n", cmd);
		return -EINVAL;
	}

	return wdt87xx_set_feature(client, VND_REQ_WRITE,
				   cmd_buf, sizeof(cmd_buf));
}

static int wdt87xx_send_short_command(struct i2c_client *client,
				      int cmd, int value)
{
	u8 cmd_buf[CMD_BUF_SIZE];

	/* Set the command packet */
	cmd_buf[0] = 0;
	cmd_buf[1] = VND1_SET_COMMAND;
	cmd_buf[2] = 0;
	cmd_buf[3] = 0;
	cmd_buf[4] = cmd;
	cmd_buf[5] = value;	
	
	return wdt87xx_set_feature(client, VND_REQ_WRITE, cmd_buf, 6);
}

static int wdt87xx_sw_reset(struct i2c_client *client)
{
	int error;

	dev_dbg(&client->dev, "resetting device now\n");

	error = wdt87xx_send_command(client, VND_CMD_RESET, 0);
	if (error) {
		dev_err(&client->dev, "reset failed\n");
		return error;
	}

	/* Wait the device to be ready */
	msleep(WDT_FW_RESET_TIME);

	return 0;
}

static const void *wdt87xx_get_fw_chunk(const struct firmware *fw, u32 id)
{
	size_t pos = FW_PAYLOAD_OFFSET;
	u32 chunk_id, chunk_size;

	while (pos < fw->size) {
		chunk_id = get_unaligned_le32(fw->data +
					      pos + FW_CHUNK_ID_OFFSET);
		if (chunk_id == id)
			return fw->data + pos;

		chunk_size = get_unaligned_le32(fw->data +
						pos + FW_CHUNK_SIZE_OFFSET);
		pos += chunk_size + 2 * sizeof(u32); /* chunk ID + size */
	}

	return NULL;
}

#if BYPASS_GET_PARAM
static int wdt87xx_get_param(struct wdt87xx_data *wdt)
{
	struct i2c_client *client = wdt->client;
	struct wdt87xx_param *param = &wdt->param;
	
	wdt->dummy_bytes = 0;

	param->vendor_id = 0x2575;
	param->product_id = 0x0001;

	param->xmls_id1 = 0x1234;
	param->xmls_id2 = 0xabcd;
	param->phy_ch_x = 64;
	param->phy_ch_y = 40;
	param->phy_w = 450;
	param->phy_h = 280;

	/* Get the report mode */	
	param->i2c_cfg = 0;

	/* Get the scaling factor of pixel to logical coordinate */
	param->scaling_factor = 60;
	param->max_x = MAX_UNIT_AXIS;
	param->max_y = DIV_ROUND_CLOSEST(MAX_UNIT_AXIS * param->phy_h,
					 param->phy_w);

	param->fw_id = 0xF;
	param->n_tch_pkt = 0xA;
	param->n_bt_tch = 0x5;
	param->protocol_version = 0x01000000;

	dev_info(&client->dev,
		 "fw_id: 0x%x, i2c_cfg: 0x%x, xml_id1: %04x, xml_id2: %04x\n",
		 param->fw_id, param->i2c_cfg, param->xmls_id1, param->xmls_id2);

	return 0;
}
#else

static void wdt87xx_parse_param(struct wdt87xx_data *wdt, u8 *buf, int len)
{
	struct i2c_client *client = wdt->client;
	struct wdt87xx_param *param = &wdt->param;

	param->xmls_id1 = get_unaligned_le16(buf + CTL_PARAM_OFFSET_XMLS_ID1);
	param->xmls_id2 = get_unaligned_le16(buf + CTL_PARAM_OFFSET_XMLS_ID2);
	param->phy_ch_x = get_unaligned_le16(buf + CTL_PARAM_OFFSET_PHY_CH_X);
	param->phy_ch_y = get_unaligned_le16(buf + CTL_PARAM_OFFSET_PHY_CH_Y);
	param->phy_w = get_unaligned_le16(buf + CTL_PARAM_OFFSET_PHY_W) / 10;
	param->phy_h = get_unaligned_le16(buf + CTL_PARAM_OFFSET_PHY_H) / 10;

	/* Get the report mode */	
	if (len == 34)
		param->i2c_cfg = RPT_PARALLEL_74;
	else
		param->i2c_cfg = get_unaligned_le16(buf +
						    CTL_PARAM_OFFSET_I2C_CFG);

	dev_info(&client->dev, "dm_bt: 0x%x, len: %d\n", wdt->dummy_bytes, len);

	wdt->report_type = param->i2c_cfg & 0xF;
	if (wdt->report_type > RPT_HID_HYBRID)
		wdt->report_type = RPT_PARALLEL_74;
	
	/* Get the scaling factor of pixel to logical coordinate */
	param->scaling_factor =
			get_unaligned_le16(buf + CTL_PARAM_OFFSET_FACTOR);

	param->max_x = MAX_UNIT_AXIS;
	param->max_y = DIV_ROUND_CLOSEST(MAX_UNIT_AXIS * param->phy_h,
					 param->phy_w);
}

static int wdt8752_exec_read_pkt(struct i2c_client *client, u8 type,
				 u8 *data, size_t len, int offset)
{
	u8 pkt_buf[PKT_BUF_SIZE];
	int error;
	size_t size;

	/*
	 * Some vendor commands can read the data structure from controller,
	 * set the mask to indicate the offset.
	 */
	if (offset & W8752_READ_OFFSET_MASK)
		size = offset & 0xFF;
	else
		size = len;

	pkt_buf[CMD_REPORT_ID_OFFSET] = VND_REQ_READ;
	pkt_buf[CMD_TYPE_OFFSET] = type;
	put_unaligned_le16(size, &pkt_buf[CMD_SIZE_OFFSET]);

	error = wdt87xx_set_feature(client, VND_REQ_READ, pkt_buf,
				    W8752_PKT_HEADER_SZ);
	if (error)
		return error;

	pkt_buf[CMD_REPORT_ID_OFFSET] = VND_READ_DATA;
	pkt_buf[CMD_TYPE_OFFSET] = type;
	error = wdt87xx_get_feature(client, pkt_buf, PKT_BUF_SIZE);
	if (error)
		return error;

	if (pkt_buf[CMD_REPORT_ID_OFFSET] != VND_READ_DATA) {
		dev_err(&client->dev, "wrong id of fw response: 0x%x\n",
			pkt_buf[CMD_REPORT_ID_OFFSET]);
		return -EINVAL;
	}
	memcpy(data, &pkt_buf[CMD_DATA1_OFFSET], len);

	return 0;
}

static int wdt8752_dev_get_device_info(struct i2c_client *client, u8 *buf,
				       int size)
{
	return wdt8752_exec_read_pkt(client, W8755_FW_GET_DEV_INFO, buf,
				     size, W8752_READ_OFFSET_MASK);
}

static int wdt87xx_get_param_hid(struct wdt87xx_data *wdt)
{
	u8 buf[PKT_READ_SIZE];
	u8 in_buf[8] = { 0x06, 0xA2, 0x10, 0, 0, 0, 0, 0 };
	int error;
	struct i2c_client *client = wdt->client;
	struct wdt87xx_param *param = &wdt->param;

	buf[0] = 0x20;
	buf[1] = 0x00;

	error = wdt87xx_i2c_xfer(client, buf, 2, &wdt->hid_desc,
				 sizeof(wdt->hid_desc));
	if (error < 0) {
		dev_err(&client->dev, "failed to get hid desc\n");
		return error;
	}

	param->vendor_id = wdt->hid_desc.wVendorID;
	param->product_id = wdt->hid_desc.wProductID;

	error = wdt8752_dev_get_device_info(client, buf, 32);
	if (error < 0) {
		dev_err(&client->dev, "failed to get device info\n");
		return error;
	}

	param->protocol_version = get_unaligned_le32(&buf[0]);

	error = wdt87xx_set_feature(client, VND_REQ_WRITE, in_buf, 8);
	if (error < 0) 
		return error;

	buf[0] = 0x07;
	error = wdt87xx_get_feature(client, buf, 24);
	if (error < 0 || (buf[0] != 0x07))
		return error;

	wdt->dev_status = buf[4] | (buf[5] << 8) | (buf[6] << 16) |
			  (buf[11] << 24);

	return 0;
}

static int wdt87xx_get_param_private(struct wdt87xx_data *wdt)
{
	u8 buf[PKT_READ_SIZE];
	int error, str_len;
	struct i2c_client *client = wdt->client;
	struct wdt87xx_param *param = &wdt->param;

	error = wdt87xx_get_desc(client, WDT_GD_DEVICE, buf, 18);
	if (error < 0) {
		dev_err(&client->dev, "failed to get device desc\n");
		return error;
	}

	param->vendor_id = get_unaligned_le16(buf + DEV_DESC_OFFSET_VID);
	param->product_id = get_unaligned_le16(buf + DEV_DESC_OFFSET_PID);

	str_len = wdt87xx_get_string(client, STRIDX_PARAMETERS, buf, 38);
	if (str_len < 0) {
		dev_err(&client->dev, "failed to get parameters\n");
		return str_len;
	}

	wdt87xx_parse_param(wdt, buf, str_len);

	error = wdt87xx_get_string(client, STRIDX_PLATFORM_ID, buf, 8);
	if (error < 0) {
		dev_err(&client->dev, "failed to get platform id\n");
		return error;
	}

	param->plat_id = buf[1];

	return 0;
}

static int wdt87xx_get_param(struct wdt87xx_data *wdt)
{
	u8 buf[PKT_READ_SIZE];
	int error;
	struct i2c_client *client = wdt->client;
	struct wdt87xx_param *param = &wdt->param;

	wdt->dummy_bytes = 0;
	buf[0] = 0xf4;
	error = wdt87xx_get_feature(client, buf, 56);
	if (error)
		dev_err(&client->dev, "failed to get i2c cfg\n");
	else 
		if (buf[0] != 0xf4)
			dev_err(&client->dev, "wrong id of fw response: 0x%x\n",
				buf[0]);
		else
			wdt->dummy_bytes = buf[1];

	if (buf[0] == 0xf4 && (get_unaligned_le16(buf + 2) == 0x154f)) { 
		param->plat_id = buf[5];
		wdt87xx_parse_param(wdt, &buf[10],
				    get_unaligned_le16(buf + 12));
		wdt->plt_id = PLT_WDT8752;
		error = wdt87xx_get_param_hid(wdt);	
	} else
		error = wdt87xx_get_param_private(wdt);

	if (error < 0)
		return error;

	buf[0] = 0xf2;
	error = wdt87xx_get_feature(client, buf, 16);
	if (error) {
		dev_err(&client->dev, "failed to get firmware id\n");
		return error;
	}

	if (buf[0] != 0xf2) {
		dev_err(&client->dev, "wrong id of fw response: 0x%x\n", buf[0]);
		return -EINVAL;
	}

	param->fw_id = get_unaligned_le16(&buf[FW_ID_OFFSET_FW_ID]);
	param->n_tch_pkt = buf[FW_ID_OFFSET_N_TCH_PKT];
	param->n_bt_tch = buf[FW_ID_OFFSET_N_BT_TCH];
	
	dev_info(&client->dev,
		"fw_id: 0x%x, i2c_cfg: 0x%x, xml_id1: %04x, xml_id2: %04x\n",
		 param->fw_id, param->i2c_cfg, param->xmls_id1, param->xmls_id2);

	dev_info(&client->dev,
		"pid: %04x, vid: %04x, w: %d, h: %d, i_sz: %d, sts: 0x%x\n",
		 param->vendor_id, param->product_id, param->phy_w, 
		 param->phy_h, wdt->hid_desc.wMaxInputLength, wdt->dev_status);

	dev_info(&client->dev,
		"protocol_ver: 0x%08x, n_touch_pkt: %d, n_bytes_touch: %d\n",
		 param->protocol_version, param->n_tch_pkt, param->n_bt_tch); 
		
	return 0;
}
#endif

static int wdt87xx_validate_firmware(struct wdt87xx_data *wdt,
				     const struct firmware *fw)
{
	const void *fw_chunk;
	u32 data1, data2;
	u32 size;
	u8 fw_chip_id;
	u8 chip_id;

	data1 = get_unaligned_le32(fw->data + FW_FOURCC1_OFFSET);
	data2 = get_unaligned_le32(fw->data + FW_FOURCC2_OFFSET);
	if (data1 != FOURCC_ID_RIFF || data2 != FOURCC_ID_WHIF) {
		dev_err(&wdt->client->dev, "check fw tag failed\n");
		return -EINVAL;
	}

	size = get_unaligned_le32(fw->data + FW_SIZE_OFFSET);
	if (size != fw->size) {
		dev_err(&wdt->client->dev,
			"fw size mismatch: expected %d, actual %zu\n",
			size, fw->size);
		return -EINVAL;
	}

	/*
	 * Get the chip_id from the firmware. Make sure that it is the
	 * right controller to do the firmware and config update.
	 */
	fw_chunk = wdt87xx_get_fw_chunk(fw, CHUNK_ID_FRWR);
	if (!fw_chunk) {
		dev_err(&wdt->client->dev,
			"unable to locate firmware chunk\n");
		return -EINVAL;
	}

	fw_chip_id = (get_unaligned_le32(fw_chunk +
					 FW_CHUNK_VERSION_OFFSET) >> 12) & 0xF;
	chip_id = (wdt->param.fw_id >> 12) & 0xF;

	if (fw_chip_id != chip_id) {
		dev_err(&wdt->client->dev,
			"fw version mismatch: fw %d vs. chip %d\n",
			fw_chip_id, chip_id);
		return -ENODEV;
	}

	return 0;
}

static int wdt87xx_validate_fw_chunk(const void *data, int id)
{
	if (id == CHUNK_ID_FRWR) {
		u32 fw_id;

		fw_id = get_unaligned_le32(data + FW_CHUNK_PAYLOAD_OFFSET);
		if (fw_id != WDT_FIRMWARE_ID)
			return -EINVAL;
	}

	return 0;
}

static int wdt87xx_write_data(struct i2c_client *client, const char *data,
			      u32 address, int length)
{
	u16 packet_size;
	int count = 0;
	int error;
	u8 pkt_buf[PKT_BUF_SIZE];

	/* Address and length should be 4 bytes aligned */
	if ((address & 0x3) != 0 || (length & 0x3) != 0) {
		dev_err(&client->dev,
			"addr & len must be 4 bytes aligned %x, %x\n",
			address, length);
		return -EINVAL;
	}

	while (length) {
		packet_size = min(length, PACKET_SIZE);

		pkt_buf[CMD_REPORT_ID_OFFSET] = VND_REQ_WRITE;
		pkt_buf[CMD_TYPE_OFFSET] = VND_SET_DATA;
		put_unaligned_le16(packet_size, &pkt_buf[CMD_INDEX_OFFSET]);
		put_unaligned_le32(address, &pkt_buf[CMD_LENGTH_OFFSET]);
		memcpy(&pkt_buf[CMD_DATA_OFFSET], data, packet_size);

		error = wdt87xx_set_feature(client, VND_REQ_WRITE,
					    pkt_buf, sizeof(pkt_buf));
		if (error)
			return error;

		length -= packet_size;
		data += packet_size;
		address += packet_size;

		/* Wait for the controller to finish the write */
		mdelay(WDT_FLASH_WRITE_DELAY_MS);

		if ((++count % 32) == 0) {
			/* Delay for fw to clear watch dog */
			msleep(20);
		}
	}

	return 0;
}

static u16 misr(u16 cur_value, u8 new_value)
{
	u32 a, b;
	u32 bit0;
	u32 y;

	a = cur_value;
	b = new_value;
	bit0 = a ^ (b & 1);
	bit0 ^= a >> 1;
	bit0 ^= a >> 2;
	bit0 ^= a >> 4;
	bit0 ^= a >> 5;
	bit0 ^= a >> 7;
	bit0 ^= a >> 11;
	bit0 ^= a >> 15;
	y = (a << 1) ^ b;
	y = (y & ~1) | (bit0 & 1);

	return (u16)y;
}

static u16 wdt87xx_calculate_checksum(const u8 *data, size_t length)
{
	u16 checksum = 0;
	size_t i;

	for (i = 0; i < length; i++)
		checksum = misr(checksum, data[i]);

	return checksum;
}

static int wdt87xx_get_checksum(struct i2c_client *client, u16 *checksum,
				u32 address, int length)
{
	int error;
	int time_delay;
	u8 pkt_buf[PKT_BUF_SIZE];
	u8 cmd_buf[CMD_BUF_SIZE];

	error = wdt87xx_send_command(client, VND_SET_CHECKSUM_LENGTH, length);
	if (error) {
		dev_err(&client->dev, "failed to set checksum length\n");
		return error;
	}

	error = wdt87xx_send_command(client, VND_SET_CHECKSUM_CALC, address);
	if (error) {
		dev_err(&client->dev, "failed to set checksum address\n");
		return error;
	}

	/* Wait the operation to complete */
	time_delay = DIV_ROUND_UP(length, 1024);
	msleep(time_delay * 30);

	memset(cmd_buf, 0, sizeof(cmd_buf));
	cmd_buf[CMD_REPORT_ID_OFFSET] = VND_REQ_READ;
	cmd_buf[CMD_TYPE_OFFSET] = VND_GET_CHECKSUM;
	error = wdt87xx_set_feature(client, VND_REQ_READ,
				    cmd_buf, sizeof(cmd_buf));
	if (error) {
		dev_err(&client->dev, "failed to request checksum\n");
		return error;
	}

	memset(pkt_buf, 0, sizeof(pkt_buf));
	pkt_buf[CMD_REPORT_ID_OFFSET] = VND_READ_DATA;
	error = wdt87xx_get_feature(client, pkt_buf, sizeof(pkt_buf));
	if (error) {
		dev_err(&client->dev, "failed to read checksum\n");
		return error;
	}

	*checksum = get_unaligned_le16(&pkt_buf[CMD_DATA_OFFSET]);
	return 0;
}

static int wdt87xx_write_firmware(struct i2c_client *client, const void *chunk)
{
	u32 start_addr = get_unaligned_le32(chunk + FW_CHUNK_TGT_START_OFFSET);
	u32 size = get_unaligned_le32(chunk + FW_CHUNK_PAYLOAD_LEN_OFFSET);
	const void *data = chunk + FW_CHUNK_PAYLOAD_OFFSET;
	int error;
	int err1;
	int page_size;
	int retry = 0;
	u16 device_checksum, firmware_checksum;

	dev_dbg(&client->dev, "start 4k page program\n");

	error = wdt87xx_send_command(client, VND_CMD_STOP, MODE_STOP);
	if (error) {
		dev_err(&client->dev, "stop report mode failed\n");
		return error;
	}

	error = wdt87xx_send_command(client, VND_CMD_SFUNL, 0);
	if (error) {
		dev_err(&client->dev, "unlock failed\n");
		goto out_enable_reporting;
	}

	mdelay(10);

	while (size) {
		dev_dbg(&client->dev, "%s: %x, %x\n", __func__,
			start_addr, size);

		page_size = min_t(u32, size, PG_SIZE);
		size -= page_size;

		for (retry = 0; retry < MAX_RETRIES; retry++) {
			error = wdt87xx_send_command(client, VND_CMD_ERASE,
						     start_addr);
			if (error) {
				dev_err(&client->dev,
					"erase failed at %#08x\n", start_addr);
				break;
			}

			msleep(50);

			error = wdt87xx_write_data(client, data, start_addr,
						   page_size);
			if (error) {
				dev_err(&client->dev,
					"write failed at %#08x (%d bytes)\n",
					start_addr, page_size);
				break;
			}

			error = wdt87xx_get_checksum(client, &device_checksum,
						     start_addr, page_size);
			if (error) {
				dev_err(&client->dev,
					"failed to get cksum at %#08x\n",
					start_addr);
				break;
			}

			firmware_checksum =
				wdt87xx_calculate_checksum(data, page_size);

			if (device_checksum == firmware_checksum)
				break;

			dev_err(&client->dev,
				"checksum fail: %d vs %d, retry %d\n",
				device_checksum, firmware_checksum, retry);
		}

		if (retry == MAX_RETRIES) {
			dev_err(&client->dev, "page write failed\n");
			error = -EIO;
			goto out_lock_device;
		}

		start_addr = start_addr + page_size;
		data = data + page_size;
	}

out_lock_device:
	err1 = wdt87xx_send_command(client, VND_CMD_SFLCK, 0);
	if (err1)
		dev_err(&client->dev, "lock failed\n");

	mdelay(10);

out_enable_reporting:
	err1 = wdt87xx_send_command(client, VND_CMD_START, 0);
	if (err1)
		dev_err(&client->dev, "start to report failed\n");

	return error ? error : err1;
}

static int wdt87xx_load_chunk(struct i2c_client *client,
			      const struct firmware *fw, u32 ck_id)
{
	const void *chunk;
	int error;

	chunk = wdt87xx_get_fw_chunk(fw, ck_id);
	if (!chunk) {
		dev_err(&client->dev, "unable to locate chunk (type %d)\n",
			ck_id);
		return -EINVAL;
	}

	error = wdt87xx_validate_fw_chunk(chunk, ck_id);
	if (error) {
		dev_err(&client->dev, "invalid chunk (type %d): %d\n",
			ck_id, error);
		return error;
	}

	error = wdt87xx_write_firmware(client, chunk);
	if (error) {
		dev_err(&client->dev,
			"failed to write fw chunk (type %d): %d\n",
			ck_id, error);
		return error;
	}

	return 0;
}

static int wdt87xx_do_update_firmware(struct i2c_client *client,
				      const struct firmware *fw,
				      unsigned int chunk_id)
{
	struct wdt87xx_data *wdt = i2c_get_clientdata(client);
	int error;

	error = wdt87xx_validate_firmware(wdt, fw);
	if (error)
		return error;

	error = mutex_lock_interruptible(&wdt->fw_mutex);
	if (error)
		return error;

	disable_irq(client->irq);

	error = wdt87xx_load_chunk(client, fw, chunk_id);
	if (error) {
		dev_err(&client->dev,
			"firmware load failed (type: %d): %d\n",
			chunk_id, error);
		goto out;
	}

	error = wdt87xx_sw_reset(client);
	if (error) {
		dev_err(&client->dev, "soft reset failed: %d\n", error);
		goto out;
	}

	/* Refresh the parameters */
	error = wdt87xx_get_param(wdt);
	if (error)
		dev_err(&client->dev,
			    "failed to refresh system paramaters: %d\n", error);
out:
	enable_irq(client->irq);
	mutex_unlock(&wdt->fw_mutex);

	return error ? error : 0;
}

static int wdt87xx_update_firmware(struct device *dev, const char *fw_name,
				   unsigned int chunk_id)
{
	struct i2c_client *client = to_i2c_client(dev);
	const struct firmware *fw;
	int error;

	error = request_firmware(&fw, fw_name, dev);
	if (error) {
		dev_err(&client->dev, "unable to retrieve firmware %s: %d\n",
			fw_name, error);
		return error;
	}

	error = wdt87xx_do_update_firmware(client, fw, chunk_id);

	release_firmware(fw);

	return error ? error : 0;
}

static ssize_t config_csum_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wdt87xx_data *wdt = i2c_get_clientdata(client);
	u32 cfg_csum;

	cfg_csum = wdt->param.xmls_id1;
	cfg_csum = (cfg_csum << 16) | wdt->param.xmls_id2;

	return scnprintf(buf, PAGE_SIZE, "%x\n", cfg_csum);
}

static ssize_t fw_version_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wdt87xx_data *wdt = i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%x\n", wdt->param.fw_id);
}

static ssize_t plat_id_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wdt87xx_data *wdt = i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%x\n", wdt->param.plat_id);
}

static ssize_t update_config_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int error;

	error = wdt87xx_update_firmware(dev, WDT87XX_CFG_NAME, CHUNK_ID_CNFG);

	return error ? error : count;
}

static ssize_t update_fw_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	int error;

	error = wdt87xx_update_firmware(dev, WDT87XX_FW_NAME, CHUNK_ID_FRWR);

	return error ? error : count;
}

static DEVICE_ATTR(config_csum, S_IRUGO, config_csum_show, NULL);
static DEVICE_ATTR(fw_version, S_IRUGO, fw_version_show, NULL);
static DEVICE_ATTR(plat_id, S_IRUGO, plat_id_show, NULL);
static DEVICE_ATTR(update_config, S_IWUSR, NULL, update_config_store);
static DEVICE_ATTR(update_fw, S_IWUSR, NULL, update_fw_store);

static struct attribute *wdt87xx_attrs[] = {
	&dev_attr_config_csum.attr,
	&dev_attr_fw_version.attr,
	&dev_attr_plat_id.attr,
	&dev_attr_update_config.attr,
	&dev_attr_update_fw.attr,
	NULL
};

static const struct attribute_group wdt87xx_attr_group = {
	.attrs = wdt87xx_attrs,
};

static void wdt87xx_report_contact(struct wdt87xx_data *wdt,
				   struct wdt87xx_param *param,
				   u8 *buf)
{
	struct input_dev *input = wdt->input_mt;
	int fngr_id;
	u32 x, y;

	fngr_id = (buf[FINGER_EV_OFFSET_ID] >> 3) - 1;
	if (fngr_id < 0)
		return;

	/* Check if this is an active contact */
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0)
	if (!(buf[FINGER_EV_OFFSET_ID] & 0x1)) {
		input_mt_slot(input, fngr_id);
		input_mt_report_slot_state(input, MT_TOOL_FINGER, 0);	
		input_report_key(input, BTN_TOUCH, 0);
		wdt->fngr_state &= ~(0x1 << fngr_id);
		return;
	}
#else
	if (!(buf[FINGER_EV_V1_OFFSET_ID] & 0x1)) {
		wdt->fngr_state &= ~(0x1 << fngr_id);
		return;
	}
#endif

	x = get_unaligned_le16(buf + FINGER_EV_OFFSET_X);

	y = get_unaligned_le16(buf + FINGER_EV_OFFSET_Y);
	y = DIV_ROUND_CLOSEST(y * param->phy_h, param->phy_w);

	/* Refuse incorrect coordinates */
	if (x > param->max_x || y > param->max_y)
		return;

	dev_dbg(&wdt->client->dev, "tip on (%d), x(%d), y(%d)\n",
		fngr_id, x, y);

	input_mt_slot(input, fngr_id);
	input_mt_report_slot_state(input, MT_TOOL_FINGER, 1);
	input_report_abs(input, ABS_MT_POSITION_X, x);
	input_report_abs(input, ABS_MT_POSITION_Y, y);
	wdt->fngr_state |= (0x1 << fngr_id);
}

static void wdt87xx_report_contact_v1(struct wdt87xx_data *wdt,
				      struct wdt87xx_param *param,
				      u8 *buf)
{
	struct input_dev *input = wdt->input_mt;
	int fngr_id;
	u32 x, y, w;
	u8 p;

	fngr_id = (buf[FINGER_EV_V1_OFFSET_ID] >> 3) - 1;
	if (fngr_id < 0)
		return;

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0)
	if (!(buf[FINGER_EV_OFFSET_ID] & 0x1)) {
		input_mt_slot(input, fngr_id);
		input_mt_report_slot_state(input, MT_TOOL_FINGER, 0);	
		wdt->fngr_state &= ~(0x1 << fngr_id);
		return;
	}
#else
	if (!(buf[FINGER_EV_V1_OFFSET_ID] & 0x1)) {
		wdt->fngr_state &= ~(0x1 << fngr_id);
		return;
	}
#endif

	w = buf[FINGER_EV_V1_OFFSET_W];
	w *= param->scaling_factor;

	p = buf[FINGER_EV_V1_OFFSET_P];

	x = get_unaligned_le16(buf + FINGER_EV_V1_OFFSET_X);

	y = get_unaligned_le16(buf + FINGER_EV_V1_OFFSET_Y);
	y = DIV_ROUND_CLOSEST(y * param->phy_h, param->phy_w);

	/* Refuse incorrect coordinates */
	if (x > param->max_x || y > param->max_y)
		return;

	dev_dbg(&wdt->client->dev, "tip on (%d), x(%d), y(%d)\n",
		fngr_id, x, y);

	input_mt_slot(input, fngr_id);
	input_mt_report_slot_state(input, MT_TOOL_FINGER, 1);
	
#if ROCKCHIP
#else	
	input_report_abs(input, ABS_MT_TOUCH_MAJOR, w);
	input_report_abs(input, ABS_MT_PRESSURE, p);
#endif	
	input_report_abs(input, ABS_MT_POSITION_X, x);
	input_report_abs(input, ABS_MT_POSITION_Y, y);
	wdt->fngr_state |= (0x1 << fngr_id);
}

static int wdt87xx_report_pen(struct wdt87xx_data *wdt,
			       struct wdt87xx_param *param, u8 *buf)
{
	struct input_dev *input = wdt->input_pen;
	u32 x, y;
	u16 p;
	u8 tip, barrel, invt, eraser, in_rng, rubber;

	tip = (buf[PEN_EV_OFFSET_BTN] >> 0) & 0x1;
	barrel = (buf[PEN_EV_OFFSET_BTN] >> 1) & 0x1;
	invt = (buf[PEN_EV_OFFSET_BTN] >> 2) & 0x1;
	eraser = (buf[PEN_EV_OFFSET_BTN] >> 3) & 0x1;
	in_rng = (buf[PEN_EV_OFFSET_BTN] >> 4) & 0x1;
	rubber = eraser | invt;

	x = get_unaligned_le16(buf + PEN_EV_OFFSET_X);
	y = get_unaligned_le16(buf + PEN_EV_OFFSET_Y);
	y = DIV_ROUND_CLOSEST(y * param->phy_h, param->phy_w);
	p = get_unaligned_le16(buf + PEN_EV_OFFSET_P);

	/* Refuse incorrect coordinates */
	if (x > param->max_x || y > param->max_y)
		return 0;

	dev_info(&wdt->client->dev, "buf %x %x %x\n", buf[1], buf[2], buf[3]);

	if (wdt->pen_type == BTN_TOOL_RUBBER && !rubber) {
		input_report_key(input, BTN_TOUCH, 0);
		input_report_key(input, BTN_STYLUS, 0);
		input_report_key(input, BTN_STYLUS2, 0);
		input_report_key(input, BTN_TOOL_RUBBER, 0);
		input_report_abs(input, ABS_PRESSURE, 0);
		input_sync(input);
	}

	if (rubber)
		wdt->pen_type = BTN_TOOL_RUBBER;
	else
		wdt->pen_type = BTN_TOOL_PEN;

	input_report_key(input, wdt->pen_type, in_rng);
	if (in_rng) {	
		input_report_key(input, BTN_TOUCH, tip);
		input_report_key(input, BTN_STYLUS, barrel);
		input_report_key(input, BTN_STYLUS2, eraser);
		input_report_abs(input, ABS_X, x);
		input_report_abs(input, ABS_Y, y);
		input_report_abs(input, ABS_PRESSURE, p);
	}

	input_sync(input);

	return 0;
}

static int wdt87xx_report_mouse(struct wdt87xx_data *wdt,
			       struct wdt87xx_param *param, u8 *buf)
{
	struct input_dev *input = wdt->input_mouse;
	u32 x, y;
	u8	btn_left, btn_right;

	btn_left = buf[MOUSE_EV_OFFSET_BTN] & 0x1;
	btn_right = buf[MOUSE_EV_OFFSET_BTN] & 0x2;

	x = get_unaligned_le16(buf + MOUSE_EV_OFFSET_X);
	y = get_unaligned_le16(buf + MOUSE_EV_OFFSET_Y);
	y = DIV_ROUND_CLOSEST(y * param->phy_h, param->phy_w);

	/* Refuse incorrect coordinates */
	if (x > param->max_x || y > param->max_y)
		return 0;

	if (btn_left != (wdt->btn_state & 0x1))
		input_event(input, EV_MSC, MSC_SCAN, 0x90001);
	input_report_key(input, BTN_LEFT, btn_left);

	if (btn_right != (wdt->btn_state & 0x2))
		input_event(input, EV_MSC, MSC_SCAN, 0x90002);
	input_report_key(input, BTN_RIGHT, btn_right);

	input_report_abs(input, ABS_X, x);
	input_report_abs(input, ABS_Y, y);

	input_sync(input);

	wdt->btn_state = buf[MOUSE_EV_OFFSET_BTN] & 0x3;

	return 0;
}

static int wdt87xx_report_hid_hybrid(struct wdt87xx_data *wdt)
{
	struct i2c_client *client = wdt->client;
	int i, fngrs, pkt_size;
	int error;
	u8 raw_buf[WDT_V1_RAW_BUF_COUNT+2] = {0};

	error =  wdt87xx_i2c_rx(client, raw_buf,
				wdt->hid_desc.wMaxInputLength);
	pkt_size = get_unaligned_le16(raw_buf);
	if (error < 0 || !pkt_size) {
		dev_err(&client->dev, "read hid raw failed: (%d)\n", error);
		return 0;
	}

	if (raw_buf[2] == RPT_ID_PEN)
		if (pkt_size == PKT_PEN_SIZE)
			return wdt87xx_report_pen(wdt, &wdt->param, &raw_buf[2]);
		else 
			goto header_failed;
	else if (raw_buf[2] == RPT_ID_MOUSE && (wdt->dev_status & 0x300))	
		if (pkt_size == PKT_MOUSE_SIZE)
			return wdt87xx_report_mouse(wdt, &wdt->param, &raw_buf[2]);
		else
			goto header_failed;
	else if (raw_buf[2] == RPT_ID_TOUCH) {
		if (pkt_size != wdt->hid_desc.wMaxInputLength)
			goto header_failed;

		fngrs = raw_buf[pkt_size - 1];

		if (fngrs > WDT_MAX_FINGER) {
			dev_err(&client->dev, "exceed max fngrs: (%d)\n", fngrs);
			goto failed;
		}

		if (fngrs && wdt->fngr_left) {
			dev_err(&client->dev, "wrong fngrs: (%d), (%d)\n",
			fngrs, wdt->fngr_left);
			goto failed;
		}

		wdt->rpt_scantime = get_unaligned_le16(&raw_buf[pkt_size - 3]);

		if (!fngrs && !wdt->fngr_left)
			goto failed;

		if (!fngrs)
			fngrs = wdt->fngr_left;
		else
			wdt->rpt_scantime |= (fngrs << 16);
		
		if (fngrs > wdt->param.n_tch_pkt) {
			wdt->fngr_left = fngrs - wdt->param.n_tch_pkt;
			fngrs = wdt->param.n_tch_pkt;
		} else
			wdt->fngr_left = 0;

		if (wdt->param.n_bt_tch == 7) {
			for (i = 0; i < fngrs; i++)
				wdt87xx_report_contact_v1(wdt, &wdt->param,
					       &raw_buf[2 +
					       TOUCH_PK_HALF_OFFSET_EVENT +
					       i * FINGER_EV_V1_SIZE]);
		} else {
			for (i = 0; i < fngrs; i++)
				wdt87xx_report_contact(wdt, &wdt->param,
					       &raw_buf[2 +
					       TOUCH_PK_HALF_OFFSET_EVENT +
					       i * FINGER_EV_SIZE]);
		}

		if (!wdt->fngr_state && !wdt->fngr_left)
			wdt->rpt_scantime &= 0xFFFF;

		if (wdt->fngr_left)
			return 0;
	} else 
		goto header_failed;

	return 1;

header_failed:
	dev_err(&client->dev, "rpt_id (%d) pkt size (%d)\n", raw_buf[2], pkt_size);
failed:
	wdt->fngr_left = 0;
	fngrs = 0;
	wdt->rpt_scantime &= 0xFFFF;
	return 0;
}

static int wdt87xx_report_hybrid_54(struct wdt87xx_data *wdt)
{
	struct i2c_client *client = wdt->client;
	int i, fngrs;
	int error;
	u8 raw_buf[WDT_V1_RAW_BUF_COUNT] = {0};

	error =  wdt87xx_i2c_rx(client, raw_buf,
				WDT_HALF_RAW_BUF_COUNT);
	if (error < 0) {
		dev_err(&client->dev, "read top half raw data failed: %d\n",
			error);
		return 0;
	}

	fngrs = raw_buf[TOUCH_PK_HALF_OFFSET_FNGR_NUM];

	if (fngrs > 5) {	
		udelay(100);
		error = wdt87xx_i2c_rx(client,
				       &raw_buf[WDT_HALF_RAW_BUF_COUNT],
				       WDT_HALF_RAW_BUF_COUNT);
		if (error < 0) {
			dev_err(&client->dev, "read bottom half failed: %d\n",
				error);
			return 0;
		}
	}	

	if (!fngrs) {
		for (i = 0; i < WDT_MAX_FINGER; i++) {
			input_mt_slot(wdt->input_mt, i);
			input_mt_report_slot_state(wdt->input_mt, MT_TOOL_FINGER,
						   0);	
		}
		return 0;
	}

	for (i = 0; i < 5; i++)
		wdt87xx_report_contact(wdt, &wdt->param,
				       &raw_buf[TOUCH_PK_HALF_OFFSET_EVENT +
				       i * FINGER_EV_SIZE]);

	if (fngrs > 5)
		for (i = 0; i < 5; i++)
			wdt87xx_report_contact(wdt, &wdt->param,
				       	       &raw_buf[WDT_HALF_RAW_BUF_COUNT +
					       TOUCH_PK_HALF_OFFSET_EVENT +
					       i * FINGER_EV_SIZE]);	
	return 1;
}

static int wdt87xx_report_parallel_74(struct wdt87xx_data *wdt)
{
	struct i2c_client *client = wdt->client;
	int i, fngrs;
	int error;
	u8 raw_buf[WDT_V1_RAW_BUF_COUNT] = {0};

	error = wdt87xx_i2c_rx(client, raw_buf, WDT_V1_RAW_BUF_COUNT);

	if (error < 0) {
		dev_err(&client->dev, "read v1 raw data failed: %d\n", error);
		return 0;
	}

	fngrs = raw_buf[TOUCH_PK_V1_OFFSET_FNGR_NUM];
	if (!fngrs) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0)
		for (i = 0; i < WDT_MAX_FINGER; i++) {
			input_mt_slot(wdt->input_mt, i);
			input_mt_report_slot_state(wdt->input_mt,
						   MT_TOOL_FINGER, 0);	
		}
		wdt->fngr_state = 0;
		return 0;
	}
#else
		wdt->fngr_state = 0;
		return 0;
	}
#endif

	for (i = 0; i < WDT_MAX_FINGER; i++)
		wdt87xx_report_contact_v1(wdt, &wdt->param,
					  &raw_buf[TOUCH_PK_V1_OFFSET_EVENT +
					  i * FINGER_EV_V1_SIZE]);
	return 1;
}

static int wdt87xx_report_parallel_54(struct wdt87xx_data *wdt)
{
	struct i2c_client *client = wdt->client;
	int i, fngrs;
	int error;
	u8 raw_buf[WDT_RAW_BUF_COUNT] = {0};

	error = wdt87xx_i2c_rx(client, raw_buf, WDT_RAW_BUF_COUNT);
	if (error < 0) {
		dev_err(&client->dev, "read raw data failed: %d\n", error);
		return 0;
	}

	fngrs = raw_buf[TOUCH_PK_OFFSET_FNGR_NUM];
	if (!fngrs) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0)
		for (i = 0; i < WDT_MAX_FINGER; i++) {
			input_mt_slot(wdt->input_mt, i);
			input_mt_report_slot_state(wdt->input_mt,
						   MT_TOOL_FINGER, 0);	
		}
		wdt->fngr_state = 0;
		return 0;
	}
#else
		wdt->fngr_state = 0;
		return 0;
	}
#endif

	for (i = 0; i < WDT_MAX_FINGER; i++)
		wdt87xx_report_contact(wdt, &wdt->param,
				       &raw_buf[TOUCH_PK_OFFSET_EVENT +
				       i * FINGER_EV_SIZE]);
	return 1;
}

static irqreturn_t wdt87xx_ts_interrupt(int irq, void *dev_id)
{
	struct wdt87xx_data *wdt = dev_id;

	if (wdt->func_report_type[(wdt->report_type & 0xf)] (wdt)) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,7,0)
		input_mt_report_pointer_emulation(wdt->input_mt, 0);
#else
		input_mt_sync_frame(wdt->input_mt);
#endif
		input_sync(wdt->input_mt);
	}

	return IRQ_HANDLED;
}

static int wdt87xx_ts_create_input_device_mt(struct wdt87xx_data *wdt)
{
	struct device *dev = &wdt->client->dev;
	struct input_dev *input;
	unsigned int res = DIV_ROUND_CLOSEST(MAX_UNIT_AXIS, wdt->param.phy_w);
	int error;

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0)
	input = input_allocate_device();
#else
	input = devm_input_allocate_device(dev);
#endif
	if (!input) {
		dev_err(dev, "failed to allocate input device\n");
		return -ENOMEM;
	}
	wdt->input_mt = input;

	input->name = "WDT87xx Touchscreen";
	input->id.bustype = BUS_I2C;
	input->id.vendor = wdt->param.vendor_id;
	input->id.product = wdt->param.product_id;
	input->phys = wdt->phys;

	__set_bit(EV_ABS, input->evbit);

	input_set_abs_params(input, ABS_MT_POSITION_X, 0,
			     wdt->param.max_x, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0,
			     wdt->param.max_y, 0, 0);
	input_abs_set_res(input, ABS_MT_POSITION_X, res);
	input_abs_set_res(input, ABS_MT_POSITION_Y, res);

#if ROCKCHIP
#else
	if (wdt->report_type == RPT_PARALLEL_74 || wdt->param.n_bt_tch == 7) {
		input_set_abs_params(input, ABS_MT_TOUCH_MAJOR,
				     0, wdt->param.max_x, 0, 0);
		input_set_abs_params(input, ABS_MT_PRESSURE, 0, 0xFF, 0, 0);
	}
#endif	

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,7,0)
	__set_bit(INPUT_PROP_DIRECT, input->propbit);
	input_mt_init_slots(input, WDT_MAX_FINGER);
#else
	input_mt_init_slots(input, WDT_MAX_FINGER,
			    INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
#endif
	error = input_register_device(input);
	if (error) {
		dev_err(dev, "failed to register input mt: %d\n", error);
		return error;
	}

	return 0;
}

static int wdt87xx_ts_create_input_device_pen(struct wdt87xx_data *wdt)
{
	struct device *dev = &wdt->client->dev;
	struct input_dev *input;
	unsigned int res = DIV_ROUND_CLOSEST(MAX_UNIT_AXIS, wdt->param.phy_w);
	int error;

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0)
	input = input_allocate_device();
#else
	input = devm_input_allocate_device(dev);
#endif
	if (!input) {
		dev_err(dev, "failed to allocate input device\n");
		return -ENOMEM;
	}
	wdt->input_pen = input;

	input->name = "WDT87xx Pen";
	input->id.bustype = BUS_I2C;
	input->id.vendor = wdt->param.vendor_id;
	input->id.product = wdt->param.product_id;
	input->phys = wdt->phys;

	__set_bit(EV_ABS, input->evbit);
	__set_bit(EV_KEY, input->evbit);
	__set_bit(BTN_TOUCH, input->keybit);
	__set_bit(BTN_TOOL_PEN, input->keybit);
	__set_bit(BTN_TOOL_RUBBER, input->keybit);
	__set_bit(BTN_STYLUS, input->keybit);
	__set_bit(BTN_STYLUS2, input->keybit);

	input_set_abs_params(input, ABS_X, 0, wdt->param.max_x, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, wdt->param.max_y, 0, 0);
	input_abs_set_res(input, ABS_X, res);
	input_abs_set_res(input, ABS_Y, res);
	input_set_abs_params(input, ABS_PRESSURE, 0, 0x3FF, 0, 0);

	error = input_register_device(input);
	if (error) {
		dev_err(dev, "failed to register input pen: %d\n", error);
		return error;
	}

	return 0;
}

static int wdt87xx_ts_create_input_device_mouse(struct wdt87xx_data *wdt)
{
	struct device *dev = &wdt->client->dev;
	struct input_dev *input;
	unsigned int res = DIV_ROUND_CLOSEST(MAX_UNIT_AXIS, wdt->param.phy_w);
	int error;

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0)
	input = input_allocate_device();
#else
	input = devm_input_allocate_device(dev);
#endif
	if (!input) {
		dev_err(dev, "failed to allocate input device\n");
		return -ENOMEM;
	}
	wdt->input_mouse = input;

	input->name = "WDT87xx Mouse";
	input->id.bustype = BUS_I2C;
	input->id.vendor = wdt->param.vendor_id;
	input->id.product = wdt->param.product_id;
	input->phys = wdt->phys;

	__set_bit(EV_ABS, input->evbit);
	input_set_abs_params(input, ABS_X, 0, wdt->param.max_x, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, wdt->param.max_y, 0, 0);
	input_abs_set_res(input, ABS_X, res);
	input_abs_set_res(input, ABS_Y, res);

	input_set_capability(input, EV_KEY, BTN_LEFT);
	input_set_capability(input, EV_KEY, BTN_RIGHT);
	input_set_capability(input, EV_MSC, MSC_SCAN);

	error = input_register_device(input);
	if (error) {
		dev_err(dev, "failed to register input mouse: %d\n", error);
		return error;
	}

	return 0;
}

static int wdt87xx_mt_release_contacts(struct wdt87xx_data *wdt)
{
	struct input_dev *input = wdt->input_mt;
	int i;
	
	for (i = 0; i < WDT_MAX_FINGER; i++) {
		input_mt_slot(input, i);
		input_mt_report_slot_state(input, MT_TOOL_FINGER, 0);			
	}
	
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,7,0)
	input_mt_report_pointer_emulation(input, 0);
#else
	input_mt_sync_frame(input);
#endif
	input_sync(input);	

	return 0;
}

static int __maybe_unused wdt87xx_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wdt87xx_data *wdt = i2c_get_clientdata(client);
	int w8752_mode = W8752_MODE_DOZE;	
	int error;

	disable_irq(client->irq);

	dev_info(&client->dev, "enter wdt87xx suspend\n");

	if (device_may_wakeup(dev)) {
		dev_info(&client->dev, "wdt87xx suspend: wakeup\n");

		wdt->wake_irq_enabled = (enable_irq_wake(client->irq) == 0);
	} else
		w8752_mode = W8752_MODE_SLEEP;	

	if (wdt->plt_id == PLT_WDT8752) 	
		error = wdt87xx_send_short_command(client, 0x82, w8752_mode);
	else
		error = wdt87xx_send_command(client, VND_CMD_STOP, MODE_IDLE);
		
	if (error) {
		enable_irq(client->irq);
		dev_err(&client->dev,
			"failed to stop device when suspending: %d\n",
			error);
		return error;
	}

	return 0;
}

static int __maybe_unused wdt87xx_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct wdt87xx_data *wdt = i2c_get_clientdata(client);
	int error;

	if (device_may_wakeup(dev)) {
		dev_info(&client->dev, "wdt87xx resume: wakeup\n");

		if (wdt->wake_irq_enabled)
			disable_irq_wake(client->irq);
	} else if (wdt->plt_id == PLT_WDT8752) {
		/* WDT8752 should wakeup device by an operation first */
		wdt87xx_send_short_command(client, 0x82, W8752_MODE_SENSE);
		udelay(100);
	}
	
	/*
	 * The chip may have been reset while system is resuming,
	 * give it some time to settle.
	 */
	mdelay(100);

	if (wdt->plt_id == PLT_WDT8752)
		error = wdt87xx_send_short_command(client, 0x82,
						   W8752_MODE_SENSE);
	else
		error = wdt87xx_send_command(client, VND_CMD_START, 0);
		
	if (error)
		dev_err(&client->dev,
			"failed to start device when resuming: %d\n",
			error);

	enable_irq(client->irq);
	
	/* Release all slots on resume as start anew */
	wdt87xx_mt_release_contacts(wdt);
	
	dev_info(&client->dev, "leave wdt87xx resume\n");
	
	return 0;
}

#if	EARLY_SUSPEND
static void wdt87xx_early_suspend(struct early_suspend *handler)
{
	struct wdt87xx_data *wdt = container_of(handler, struct wdt87xx_data,
						wdt_early_suspend);
	dev_info(&wdt->client->dev, "early suspend\n");
	
	wdt87xx_suspend(&wdt->client->dev);
}

static void wdt87xx_late_resume(struct early_suspend *handler)
{
	struct wdt87xx_data *wdt = container_of(handler, struct wdt87xx_data,
						wdt_early_suspend);
	dev_info(&wdt->client->dev, "late resume\n");

	wdt87xx_resume(&wdt->client->dev);
}
#endif

static int wdt87xx_create_dbgfs(struct wdt87xx_data *wdt)
{
	wdt->dbg_root = debugfs_create_dir(WDT87XX_NAME, NULL);
	if (!wdt->dbg_root)
		return -EINVAL; 

	wdt->dbg_st = debugfs_create_u32("dbg_st", 0644, wdt->dbg_root,
					 &wdt->rpt_scantime); 
	
	if (!wdt->dbg_st)
		return -EINVAL;

	return 0;
}

#if	ROCKCHIP
#if	ROCKCHIP_3288_OF
static int wdt87xx_handle_of(struct i2c_client *client)
{
	int error;
	struct device *dev = &client->dev;
	struct device_node *np = dev->of_node;
	enum of_gpio_flags rst_flags;
	unsigned long irq_flags;
	int value, reset_pin;
	
	if (of_property_read_u32(np, "screen_max_x", &value))
		dev_info(&client->dev, "no screen_max_x defined\n");

	if (of_property_read_u32(np, "screen_max_y", &value))
		dev_info(&client->dev, "no screen_max_y defined\n");

	client->irq = of_get_named_gpio_flags(np, "touch_irq_gpio", 0,
					      (enum of_gpio_flags *)&irq_flags);
	reset_pin = of_get_named_gpio_flags(np, "touch_reset_gpio", 0,
					    &rst_flags);
	
	error = gpio_request(client->irq, "GPIO INT");
	if (error < 0) {
		dev_err(&client->dev, "%s: request gpio fail (%d)\n",
			__func__, error);
		return -EINVAL;
	}

	dev_info(&client->dev, "irq gpio num: (%d)\n", client->irq);

	client->irq = gpio_to_irq(client->irq);
	if (client->irq < 0) {
		dev_err(&client->dev, "%s: request irq fail (%d)\n",
			__func__, client->irq);
		return -EINVAL;
	}

	dev_info(&client->dev, "client irq num: (%d)\n", client->irq);

	return error;
}
#endif
#endif

static int wdt87xx_setup_irq(struct i2c_client *client)
{
	int error = 0;

#if SABRELITE
    dev_info(&client->dev, "%s: configured irq=%d\n", __func__, client->irq);
#endif

#if EXYNOS5

	client->irq = EINT_NUM;
	
	error = gpio_request(FT_INT_PORT, "EINT");
	if (error < 0) {		
		dev_err(&client->dev, "Request gpio fail (%d)\n", error);
		return error;
	}

	/* pull high to wait the irq notification */
	s3c_gpio_setpull(FT_INT_PORT, S3C_GPIO_PULL_UP);	
#endif

#if MINNOWBOARD
/*
 * once using the ACPI, we shall get the irq number from the client
 * if it is not in the ACPI, we shall get a irq number from the gpio
 */

	/* request a selected gpio */
	error = gpio_request(DINT_GPIO_NUM, "GPIO DINT");
	if (error < 0) {
		dev_err(&client->dev, "%s: request gpio fail (%d)\n",
			__func__, error);
		return -EINVAL;
	}

	/* request a irq no */
	client->irq = gpio_to_irq(DINT_GPIO_NUM);
	if (client->irq < 0) {
		dev_err(&client->dev, "%s: request irq fail (%d)\n",
			__func__, client->irq);
		return -EINVAL;
	}

	gpio_direction_input(DINT_GPIO_NUM);
#endif

#if ROCKCHIP
#if	ROCKCHIP_3288_OF
	return wdt87xx_handle_of(client);
#else
	client->irq = gpio_to_irq(client->irq);
	if (client->irq < 0) {
		dev_err(&client->dev, "%s: request irq fail (%d)\n",
			__func__, client->irq);
		return -EINVAL;
	}	
	dev_info(&client->dev, "client irq mapped: %d\n", client->irq);
#endif
#endif

#if ALLWINNER
	client->irq = gpio_to_irq(config_info.int_number);
	if (IS_ERR_VALUE(client->irq)) {
		dev_err(&client->dev, "%s: request irq fail (%d)\n",
			__func__, client->irq);
		return -EINVAL;
	}
	dev_info(&client->dev, "client irq mapped: %d\n", client->irq);
#endif
	return error;
}

static int wdt87xx_ts_probe(struct i2c_client *client,
			    const struct i2c_device_id *id)
{
	struct wdt87xx_data *wdt;
	int error;

    printk("Enter %s\n", __func__);

	dev_info(&client->dev, "adapter=%d, client irq: %d\n",
		client->adapter->nr, client->irq);

	/* Check if the I2C function is ok in this adaptor */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENXIO;

	wdt = devm_kzalloc(&client->dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;

	wdt->client = client;
	mutex_init(&wdt->fw_mutex);
	i2c_set_clientdata(client, wdt);

	snprintf(wdt->phys, sizeof(wdt->phys), "i2c-%u-%04x/input0",
		 client->adapter->nr, client->addr);

	wdt->func_report_type[0] = wdt87xx_report_parallel_74;
	wdt->func_report_type[1] = wdt87xx_report_parallel_54;
	wdt->func_report_type[2] = wdt87xx_report_hybrid_54;
	wdt->func_report_type[3] = wdt87xx_report_hid_hybrid;

	wdt->pen_type = BTN_TOOL_PEN;
	
#if	MTK
	mutex_init(&wdt->dma_mutex);
	
	client->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	wdt->p_dma_va = dma_alloc_coherent(&client->dev, I2C_DMA_MAX_TRANS_SZ,
					   &wdt->p_dma_pa, GFP_KERNEL);
	
	if (!wdt->p_dma_va) {
		dev_err(&client->dev, "Request dma buffer error\n");
		return -ENOMEM;
	}
	memset(wdt->p_dma_va, 0, I2C_DMA_MAX_TRANS_SZ);
#endif	

	error = wdt87xx_get_param(wdt);
	if (error)
		return error;

	error = wdt87xx_ts_create_input_device_mt(wdt);
	if (error)
		return error;

	if (wdt->plt_id == PLT_WDT8752 && wdt->report_type == RPT_HID_HYBRID) {
		error = wdt87xx_ts_create_input_device_pen(wdt);
		if (error)
			return error;

		if ((wdt->dev_status & 0x300)) {
			error = wdt87xx_ts_create_input_device_mouse(wdt);
			if (error)
				return error;
		}
	}

	error = wdt87xx_setup_irq(client);
	if (error)
		return error;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,39)
	set_irq_type(client->irq, IRQ_TYPE_EDGE_FALLING);	
#else
	irq_set_irq_type(client->irq, IRQ_TYPE_EDGE_FALLING);	
#endif

	error = devm_request_threaded_irq(&client->dev, client->irq,
					  NULL, wdt87xx_ts_interrupt,
					  IRQF_ONESHOT,
					  client->name, wdt);
	if (error) {
		dev_err(&client->dev, "request irq failed: %d\n", error);
		return error;
	}

	error = device_init_wakeup(&client->dev, true);
	if (error) {
		dev_err(&client->dev, "inii wakeup failed: %d\n", error);
		return error;
	}

	error = sysfs_create_group(&client->dev.kobj, &wdt87xx_attr_group);
	if (error) {
		dev_err(&client->dev, "create sysfs failed: %d\n", error);
		return error;
	}

	error = wdt87xx_create_dbgfs(wdt);
	if (error) {
		dev_err(&client->dev, "create debugfs failed: %d\n", error);
		return error;
	}

	wdt->state = ST_REPORT;
		
#if		EARLY_SUSPEND
	wdt->wdt_early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	wdt->wdt_early_suspend.suspend = wdt87xx_early_suspend;
	wdt->wdt_early_suspend.resume	= wdt87xx_late_resume;
	register_early_suspend(&wdt->wdt_early_suspend);
#endif

	return 0;
}

static int wdt87xx_ts_remove(struct i2c_client *client)
{
	struct wdt87xx_data *wdt = i2c_get_clientdata(client);

#if 	EARLY_SUSPEND
	unregister_early_suspend(&wdt->wdt_early_suspend);
#endif	
	if (wdt->dbg_root)
		debugfs_remove_recursive(wdt->dbg_root);

	sysfs_remove_group(&client->dev.kobj, &wdt87xx_attr_group);

#if	MTK
	dma_free_coherent(&client->dev, I2C_DMA_MAX_TRANS_SZ, wdt->p_dma_va,
			  wdt->p_dma_pa);
#endif		
	return 0;
}

static SIMPLE_DEV_PM_OPS(wdt87xx_pm_ops, wdt87xx_suspend, wdt87xx_resume);

static const struct i2c_device_id wdt87xx_dev_id[] = {
	{ WDT87XX_NAME, 0 },
	{ }
};

#ifdef	CONFIG_ACPI
MODULE_DEVICE_TABLE(i2c, wdt87xx_dev_id);

static const struct acpi_device_id wdt87xx_acpi_id[] = {
	{ "WDHT0001", 0 },
	{ }
};
MODULE_DEVICE_TABLE(acpi, wdt87xx_acpi_id);
#endif

#ifdef	CONFIG_OF
static struct of_device_id wdt87xx_of_ids[] = {
	{ .compatible = "wdt,wdt87xx_i2c" },
	{ }
};
#endif

static struct i2c_driver wdt87xx_driver = {
	.probe		= wdt87xx_ts_probe,
	.remove		= wdt87xx_ts_remove,
	.id_table	= wdt87xx_dev_id,
	.driver	= {
		.name	= WDT87XX_NAME,
#if !EARLY_SUSPEND		
		.pm     = &wdt87xx_pm_ops,
#endif
#ifdef	CONFIG_ACPI
		.acpi_match_table = ACPI_PTR(wdt87xx_acpi_id),
#endif	
#ifdef	CONFIG_OF
		.of_match_table = of_match_ptr(wdt87xx_of_ids),
#endif
	},

#if	ALLWINNER
	.address_list	= weida_i2c,
#endif
};

#if	ALLWINNER
static void ctp_print_info(struct ctp_config_info info,int debug_level)
{
	printk("info.ctp_used: %d\n", info.ctp_used);
	printk("info.twi_id: %d\n", info.twi_id);
	printk("info.screen_max_x: %d\n", info.screen_max_x);
	printk("info.screen_max_y: %d\n", info.screen_max_y);
	printk("info.revert_x_flag: %d\n", info.revert_x_flag);
	printk("info.revert_y_flag: %d\n", info.revert_y_flag);
	printk("info.exchange_x_y_flag: %d\n", info.exchange_x_y_flag);
	printk("info.irq_gpio_number: %d\n", info.irq_gpio.gpio);
	printk("info.wakeup_gpio_number: %d\n", info.wakeup_gpio.gpio);
	printk("info.int_number: %d\n", info.int_number);
}

static int ctp_get_system_config(void)
{   
        ctp_print_info(config_info, 0);

        if((config_info.screen_max_x == 0) || (config_info.screen_max_y == 0)) {
                printk("%s:read config error!\n",__func__);
                return -1;
        }
        return 0;
}

static struct i2c_board_info 	i2c_weida_info	__initdata = 
{
	I2C_BOARD_INFO(WDT87XX_NAME, 0x2C),
};
#endif

static int __init wdt87xx_driver_init(void)
{
#if	ALLWINNER
	struct i2c_adapter	*padapter;
	struct i2c_client	*pclient;
	int ret;      

	printk("enter %s\n", __func__);

	if (input_fetch_sysconfig_para(&(config_info.input_type))) {
		printk("%s: fetch sysconfig para err.\n", __func__);
		return 0;
	} else {
		ret = input_init_platform_resource(&(config_info.input_type));
		if (ret) 
			printk("%s:init platform_resource err.\n", __func__);    
	}

	if (!config_info.ctp_used) {
	        printk("ctp_used set to 0 !\n");
	        printk("please set ctp_used to 1 in sys_config.fex.\n");
	        return 0;
	}

        if(ctp_get_system_config()) {
                printk("%s: read config fail!\n", __func__);
                return -1;
        }

	input_set_power_enable(&(config_info.input_type), 1);

	mdelay(10);

	ret = i2c_add_driver(&wdt87xx_driver);
	if (ret) {
		printk("failed to add i2c driver\n");
		return ret;
	}

	padapter = i2c_get_adapter(config_info.twi_id);
	if (padapter == NULL) {
		printk("failed to get adapter (%d)\n", config_info.twi_id);
		return -ENODEV;
	}

	pclient = i2c_new_device(padapter, &i2c_weida_info);

	if (pclient == NULL) {
		printk("failed to get client");
		return -ENODEV;
	}

	return 0;
#else
	printk("Enter %s\n", __func__);
	return i2c_add_driver(&wdt87xx_driver);
#endif
}

static void __exit wdt87xx_driver_exit(void)
{
	i2c_del_driver(&wdt87xx_driver);

#if	ALLWINNER
	input_free_platform_resource(&(config_info.input_type));
#endif
}

module_init(wdt87xx_driver_init);
module_exit(wdt87xx_driver_exit);

MODULE_AUTHOR("HN Chen <hn.chen@weidahitech.com>");
MODULE_DESCRIPTION("WeidaHiTech WDT87XX Touchscreen driver");
MODULE_VERSION(WDT87XX_DRV_VER);
MODULE_LICENSE("GPL");

