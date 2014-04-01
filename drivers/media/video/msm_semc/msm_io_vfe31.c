/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 * Copyright (C) 2010 Sony Ericsson Mobile Communications AB.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <mach/gpio.h>
#include <mach/board.h>
#include <mach/camera.h>
#include <mach/clk.h>
#include <asm/mach-types.h>

#define CAMIF_CFG_RMSK             0x1fffff
#define CAM_SEL_BMSK               0x2
#define CAM_PCLK_SRC_SEL_BMSK      0x60000
#define CAM_PCLK_INVERT_BMSK       0x80000
#define CAM_PAD_REG_SW_RESET_BMSK  0x100000

#define EXT_CAM_HSYNC_POL_SEL_BMSK 0x10000
#define EXT_CAM_VSYNC_POL_SEL_BMSK 0x8000
#define MDDI_CLK_CHICKEN_BIT_BMSK  0x80

#define CAM_SEL_SHFT               0x1
#define CAM_PCLK_SRC_SEL_SHFT      0x11
#define CAM_PCLK_INVERT_SHFT       0x13
#define CAM_PAD_REG_SW_RESET_SHFT  0x14

#define EXT_CAM_HSYNC_POL_SEL_SHFT 0x10
#define EXT_CAM_VSYNC_POL_SEL_SHFT 0xF
#define MDDI_CLK_CHICKEN_BIT_SHFT  0x7

/* MIPI	CSI	controller registers */
#define	MIPI_PHY_CONTROL			0x00000000
#define	MIPI_PROTOCOL_CONTROL		0x00000004
#define	MIPI_INTERRUPT_STATUS		0x00000008
#define	MIPI_INTERRUPT_MASK			0x0000000C
#define	MIPI_CAMERA_CNTL			0x00000024
#define	MIPI_CALIBRATION_CONTROL	0x00000018
#define	MIPI_PHY_D0_CONTROL2		0x00000038
#define	MIPI_PHY_D1_CONTROL2		0x0000003C
#define	MIPI_PHY_D2_CONTROL2		0x00000040
#define	MIPI_PHY_D3_CONTROL2		0x00000044
#define	MIPI_PHY_CL_CONTROL			0x00000048
#define	MIPI_PHY_D0_CONTROL			0x00000034
#define	MIPI_PHY_D1_CONTROL			0x00000020
#define	MIPI_PHY_D2_CONTROL			0x0000002C
#define	MIPI_PHY_D3_CONTROL			0x00000030
#define	MIPI_PROTOCOL_CONTROL_SW_RST_BMSK			0x8000000
#define	MIPI_PROTOCOL_CONTROL_LONG_PACKET_HEADER_CAPTURE_BMSK	0x200000
#define	MIPI_PROTOCOL_CONTROL_DATA_FORMAT_BMSK			0x180000
#define	MIPI_PROTOCOL_CONTROL_DECODE_ID_BMSK			0x40000
#define	MIPI_PROTOCOL_CONTROL_ECC_EN_BMSK			0x20000
#define	MIPI_CALIBRATION_CONTROL_SWCAL_CAL_EN_SHFT		0x16
#define	MIPI_CALIBRATION_CONTROL_SWCAL_STRENGTH_OVERRIDE_EN_SHFT	0x15
#define	MIPI_CALIBRATION_CONTROL_CAL_SW_HW_MODE_SHFT		0x14
#define	MIPI_CALIBRATION_CONTROL_MANUAL_OVERRIDE_EN_SHFT	0x7
#define	MIPI_PROTOCOL_CONTROL_DATA_FORMAT_SHFT			0x13
#define	MIPI_PROTOCOL_CONTROL_DPCM_SCHEME_SHFT			0x1e
#define	MIPI_PHY_D0_CONTROL2_SETTLE_COUNT_SHFT			0x18
#define	MIPI_PHY_D0_CONTROL2_HS_TERM_IMP_SHFT			0x10
#define	MIPI_PHY_D0_CONTROL2_LP_REC_EN_SHFT				0x4
#define	MIPI_PHY_D0_CONTROL2_ERR_SOT_HS_EN_SHFT			0x3
#define	MIPI_PHY_D1_CONTROL2_SETTLE_COUNT_SHFT			0x18
#define	MIPI_PHY_D1_CONTROL2_HS_TERM_IMP_SHFT			0x10
#define	MIPI_PHY_D1_CONTROL2_LP_REC_EN_SHFT				0x4
#define	MIPI_PHY_D1_CONTROL2_ERR_SOT_HS_EN_SHFT			0x3
#define	MIPI_PHY_D2_CONTROL2_SETTLE_COUNT_SHFT			0x18
#define	MIPI_PHY_D2_CONTROL2_HS_TERM_IMP_SHFT			0x10
#define	MIPI_PHY_D2_CONTROL2_LP_REC_EN_SHFT				0x4
#define	MIPI_PHY_D2_CONTROL2_ERR_SOT_HS_EN_SHFT			0x3
#define	MIPI_PHY_D3_CONTROL2_SETTLE_COUNT_SHFT			0x18
#define	MIPI_PHY_D3_CONTROL2_HS_TERM_IMP_SHFT			0x10
#define	MIPI_PHY_D3_CONTROL2_LP_REC_EN_SHFT				0x4
#define	MIPI_PHY_D3_CONTROL2_ERR_SOT_HS_EN_SHFT			0x3
#define	MIPI_PHY_CL_CONTROL_HS_TERM_IMP_SHFT			0x18
#define	MIPI_PHY_CL_CONTROL_LP_REC_EN_SHFT				0x2
#define	MIPI_PHY_D0_CONTROL_HS_REC_EQ_SHFT				0x1c
#define	MIPI_PHY_D1_CONTROL_MIPI_CLK_PHY_SHUTDOWNB_SHFT		0x9
#define	MIPI_PHY_D1_CONTROL_MIPI_DATA_PHY_SHUTDOWNB_SHFT	0x8

static struct clk *camio_vfe_mdc_clk;
static struct clk *camio_mdc_clk;
static struct clk *camio_vfe_clk;
static struct clk *camio_vfe_camif_clk;
static struct clk *camio_vfe_pbdg_clk;
static struct clk *camio_cam_m_clk;
static struct clk *camio_camif_pad_pbdg_clk;
static struct clk *camio_csi_clk;
static struct clk *camio_csi_pclk;
static struct clk *camio_csi_vfe_clk;
static struct clk *camio_jpeg_clk;
static struct clk *camio_jpeg_pclk;
static struct clk *camio_vpe_clk;
static struct msm_camera_io_ext camio_ext;
static struct msm_camera_io_clk camio_clk;
static struct resource *camifpadio, *csiio;
void __iomem *camifpadbase, *csibase;

void msm_io_w(u32 data, void __iomem *addr)
{
	CDBG("%s: %08x %08x\n", __func__, (int) (addr), (data));
	writel((data), (addr));
}

void msm_io_w_mb(u32 data, void __iomem *addr)
{
	CDBG("%s: %08x %08x\n", __func__, (int) (addr), (data));
	wmb();
	writel((data), (addr));
	wmb();
}

u32 msm_io_r(void __iomem *addr)
{
	uint32_t data = readl(addr);
	CDBG("%s: %08x %08x\n", __func__, (int) (addr), (data));
	return data;
}

u32 msm_io_r_mb(void __iomem *addr)
{
	uint32_t data;
	rmb();
	data = readl(addr);
	rmb();
	CDBG("%s: %08x %08x\n", __func__, (int) (addr), (data));
	return data;
}

void msm_io_memcpy_toio(void __iomem *dest_addr,
	void __iomem *src_addr, u32 len)
{
	int i;
	u32 *d = (u32 *) dest_addr;
	u32 *s = (u32 *) src_addr;
	/* memcpy_toio does not work. Use writel for now */
	for (i = 0; i < len; i++)
		writel(*s++, d++);
}

void msm_io_dump(void __iomem *addr, int size)
{
	char line_str[128], *p_str;
	int i;
	u32 *p = (u32 *) addr;
	u32 data;
	CDBG("%s: %p %d\n", __func__, addr, size);
	line_str[0] = '\0';
	p_str = line_str;
	for (i = 0; i < size/4; i++) {
		if (i % 4 == 0) {
			sprintf(p_str, "%08x: ", (u32) p);
			p_str += 10;
		}
		data = readl(p++);
		sprintf(p_str, "%08x ", data);
		p_str += 9;
		if ((i + 1) % 4 == 0) {
			CDBG("%s\n", line_str);
			line_str[0] = '\0';
			p_str = line_str;
		}
	}
	if (line_str[0] != '\0')
		CDBG("%s\n", line_str);
}

void msm_io_memcpy(void __iomem *dest_addr, void __iomem *src_addr, u32 len)
{
	CDBG("%s: %p %p %d\n", __func__, dest_addr, src_addr, len);
	msm_io_memcpy_toio(dest_addr, src_addr, len / 4);
	msm_io_dump(dest_addr, len);
}

int msm_camio_clk_enable(enum msm_camio_clk_type clktype)
{
	int rc = 0;
	struct clk *clk = NULL;

	switch (clktype) {
	case CAMIO_VFE_MDC_CLK:
		camio_vfe_mdc_clk =
		clk = clk_get(NULL, "vfe_mdc_clk");
		break;

	case CAMIO_MDC_CLK:
		camio_mdc_clk =
		clk = clk_get(NULL, "mdc_clk");
		break;

	case CAMIO_VFE_CLK:
		camio_vfe_clk =
		clk = clk_get(NULL, "vfe_clk");
		msm_camio_clk_rate_set_2(clk, camio_clk.vfe_clk_rate);
		break;

	case CAMIO_VFE_CAMIF_CLK:
		camio_vfe_camif_clk =
		clk = clk_get(NULL, "vfe_camif_clk");
		break;

	case CAMIO_VFE_PBDG_CLK:
		camio_vfe_pbdg_clk =
		clk = clk_get(NULL, "vfe_pclk");
		break;

	case CAMIO_CAM_MCLK_CLK:
		camio_cam_m_clk =
		clk = clk_get(NULL, "cam_m_clk");
		msm_camio_clk_rate_set_2(clk, camio_clk.mclk_clk_rate);
		break;

	case CAMIO_CAMIF_PAD_PBDG_CLK:
		camio_camif_pad_pbdg_clk =
		clk = clk_get(NULL, "camif_pad_pclk");
		break;

	case CAMIO_CSI0_CLK:
		camio_csi_clk =
		clk = clk_get(NULL, "csi_clk");
		msm_camio_clk_rate_set_2(clk, 153600000);
		break;
	case CAMIO_CSI0_VFE_CLK:
		camio_csi_vfe_clk =
		clk = clk_get(NULL, "csi_vfe_clk");
		break;
	case CAMIO_CSI0_PCLK:
		camio_csi_pclk =
		clk = clk_get(NULL, "csi_pclk");
		break;

	case CAMIO_JPEG_CLK:
		camio_jpeg_clk =
		clk = clk_get(NULL, "jpeg_clk");
		clk_set_min_rate(clk, 144000000);
		break;
	case CAMIO_JPEG_PCLK:
		camio_jpeg_pclk =
		clk = clk_get(NULL, "jpeg_pclk");
		break;
	case CAMIO_VPE_CLK:
		camio_vpe_clk =
		clk = clk_get(NULL, "vpe_clk");
#ifdef CONFIG_MSM_VPE_STANDALONE
		msm_camio_clk_rate_set_2(clk, 153600000);
#else
		msm_camio_clk_set_min_rate(clk, 150000000);
#endif /* CONFIG_MSM_VPE_STANDALONE */
		break;
	default:
		break;
	}

	if (!IS_ERR(clk))
		clk_enable(clk);
	else
		rc = -1;
	return rc;
}

int msm_camio_clk_disable(enum msm_camio_clk_type clktype)
{
	int rc = 0;
	struct clk *clk = NULL;

	switch (clktype) {
	case CAMIO_VFE_MDC_CLK:
		clk = camio_vfe_mdc_clk;
		break;

	case CAMIO_MDC_CLK:
		clk = camio_mdc_clk;
		break;

	case CAMIO_VFE_CLK:
		clk = camio_vfe_clk;
		if (camio_clk.vfe_clk_rate == 192000000)
			msm_camio_clk_rate_set_2(clk, 153600000);
		break;

	case CAMIO_VFE_CAMIF_CLK:
		clk = camio_vfe_camif_clk;
		break;

	case CAMIO_VFE_PBDG_CLK:
		clk = camio_vfe_pbdg_clk;
		break;

	case CAMIO_CAM_MCLK_CLK:
		clk = camio_cam_m_clk;
		break;

	case CAMIO_CAMIF_PAD_PBDG_CLK:
		clk = camio_camif_pad_pbdg_clk;
		break;
	case CAMIO_CSI0_CLK:
		clk = camio_csi_clk;
		break;
	case CAMIO_CSI0_VFE_CLK:
		clk = camio_csi_vfe_clk;
		break;
	case CAMIO_CSI0_PCLK:
		clk = camio_csi_pclk;
		break;
	case CAMIO_JPEG_CLK:
		clk = camio_jpeg_clk;
		break;
	case CAMIO_JPEG_PCLK:
		clk = camio_jpeg_pclk;
		break;
	case CAMIO_VPE_CLK:
		clk = camio_vpe_clk;
		break;
	default:
		break;
	}

	if (!IS_ERR(clk)) {
		clk_disable(clk);
		clk_put(clk);
	} else
		rc = -1;

	return rc;
}

void msm_camio_clk_rate_set(int rate)
{
	struct clk *clk = camio_cam_m_clk;
	clk_set_rate(clk, rate);
}

void msm_camio_clk_rate_set_2(struct clk *clk, int rate)
{
	clk_set_rate(clk, rate);
}

void msm_camio_clk_set_min_rate(struct clk *clk, int rate)
{
	clk_set_min_rate(clk, rate);
}

#if defined(CONFIG_SEMC_CAMERA_MODULE) || defined(CONFIG_SEMC_SUB_CAMERA_MODULE)
void msm_camio_cam_mclk_enable(int rate)
{
	struct clk *clk = NULL;
	camio_cam_m_clk =
	clk = clk_get(NULL, "cam_m_clk");
	msm_camio_clk_rate_set_2(clk, rate);
	if (!IS_ERR(clk))
		clk_enable(clk);
}
#endif

static irqreturn_t msm_io_csi_irq(int irq_num, void *data)
{
	uint32_t irq;
	irq = msm_io_r(csibase + MIPI_INTERRUPT_STATUS);
	CDBG("%s MIPI_INTERRUPT_STATUS = 0x%x\n", __func__, irq);
	msm_io_w(irq, csibase + MIPI_INTERRUPT_STATUS);
	return IRQ_HANDLED;
}

int msm_camio_jpeg_clk_disable(void)
{
	msm_camio_clk_disable(CAMIO_JPEG_CLK);
	msm_camio_clk_disable(CAMIO_JPEG_PCLK);
	/* Need to add the code for remove PM QOS requirement */
	return 0;
}


int msm_camio_jpeg_clk_enable(void)
{
	msm_camio_clk_enable(CAMIO_JPEG_CLK);
	msm_camio_clk_enable(CAMIO_JPEG_PCLK);
	return 0;
}

int msm_camio_vpe_clk_disable(void)
{
	msm_camio_clk_disable(CAMIO_VPE_CLK);
	return 0;
}

int msm_camio_vpe_clk_enable(void)
{
	msm_camio_clk_enable(CAMIO_VPE_CLK);
	return 0;
}

int msm_camio_enable(struct platform_device *pdev)
{
	int rc = 0;
	uint32_t val;
	struct msm_camera_sensor_info *sinfo = pdev->dev.platform_data;
	msm_camio_clk_enable(CAMIO_VFE_PBDG_CLK);
#if defined(CONFIG_MACH_SEMC_ZEUS) || defined(CONFIG_MACH_SEMC_PHOENIX)
	msm_camio_clk_enable(CAMIO_CAM_MCLK_CLK);
#endif /* CONFIG_MACH_SEMC_ZEUS */
	if (!sinfo->csi_if)
		msm_camio_clk_enable(CAMIO_VFE_CAMIF_CLK);
	else {
		msm_camio_clk_enable(CAMIO_VFE_CLK);
		csiio = request_mem_region(camio_ext.csiphy,
			camio_ext.csisz, pdev->name);
		if (!csiio) {
			rc = -EBUSY;
			goto common_fail;
		}
		csibase = ioremap(camio_ext.csiphy,
			camio_ext.csisz);
		if (!csibase) {
			rc = -ENOMEM;
			goto csi_busy;
		}
		rc = request_irq(camio_ext.csiirq, msm_io_csi_irq,
			IRQF_TRIGGER_RISING, "csi", 0);
		if (rc < 0)
			goto csi_irq_fail;
		/* enable required clocks for CSI */
		msm_camio_clk_enable(CAMIO_CSI0_PCLK);
		msm_camio_clk_enable(CAMIO_CSI0_VFE_CLK);
		msm_camio_clk_enable(CAMIO_CSI0_CLK);

		msleep(10);
		val = (20 <<
			MIPI_PHY_D0_CONTROL2_SETTLE_COUNT_SHFT) |
			(0x0F << MIPI_PHY_D0_CONTROL2_HS_TERM_IMP_SHFT) |
			(0x0 << MIPI_PHY_D0_CONTROL2_LP_REC_EN_SHFT) |
			(0x1 << MIPI_PHY_D0_CONTROL2_ERR_SOT_HS_EN_SHFT);
		CDBG("%s MIPI_PHY_D0_CONTROL2 val=0x%x\n", __func__, val);
		msm_io_w(val, csibase + MIPI_PHY_D0_CONTROL2);
		msm_io_w(val, csibase + MIPI_PHY_D1_CONTROL2);
		msm_io_w(val, csibase + MIPI_PHY_D2_CONTROL2);
		msm_io_w(val, csibase + MIPI_PHY_D3_CONTROL2);

		val = (0x0F << MIPI_PHY_CL_CONTROL_HS_TERM_IMP_SHFT) |
			(0x0 << MIPI_PHY_CL_CONTROL_LP_REC_EN_SHFT);
		CDBG("%s MIPI_PHY_CL_CONTROL val=0x%x\n", __func__, val);
		msm_io_w(val, csibase + MIPI_PHY_CL_CONTROL);
	}
	return 0;
csi_irq_fail:
	iounmap(csibase);
csi_busy:
	release_mem_region(camio_ext.csiphy, camio_ext.csisz);
common_fail:
	msm_camio_clk_disable(CAMIO_VFE_PBDG_CLK);
	msm_camio_clk_disable(CAMIO_VFE_CLK);
	msm_camio_clk_disable(CAMIO_CAM_MCLK_CLK);
	return rc;
}

void msm_camio_disable(struct platform_device *pdev)
{
	struct msm_camera_sensor_info *sinfo = pdev->dev.platform_data;
	uint32_t val;
	if (!sinfo->csi_if) {
		msm_camio_clk_disable(CAMIO_VFE_CAMIF_CLK);
	} else {
		val = (20 <<
			MIPI_PHY_D0_CONTROL2_SETTLE_COUNT_SHFT) |
			(0x0F << MIPI_PHY_D0_CONTROL2_HS_TERM_IMP_SHFT) |
			(0x0 << MIPI_PHY_D0_CONTROL2_LP_REC_EN_SHFT) |
			(0x1 << MIPI_PHY_D0_CONTROL2_ERR_SOT_HS_EN_SHFT);
		CDBG("%s MIPI_PHY_D0_CONTROL2 val=0x%x\n", __func__, val);
		msm_io_w(val, csibase + MIPI_PHY_D0_CONTROL2);
		msm_io_w(val, csibase + MIPI_PHY_D1_CONTROL2);
		msm_io_w(val, csibase + MIPI_PHY_D2_CONTROL2);
		msm_io_w(val, csibase + MIPI_PHY_D3_CONTROL2);
		val = (0x0F << MIPI_PHY_CL_CONTROL_HS_TERM_IMP_SHFT) |
			(0x0 << MIPI_PHY_CL_CONTROL_LP_REC_EN_SHFT);
		CDBG("%s MIPI_PHY_CL_CONTROL val=0x%x\n", __func__, val);
		msm_io_w(val, csibase + MIPI_PHY_CL_CONTROL);
		msleep(10);
		free_irq(camio_ext.csiirq, 0);
		msm_camio_clk_disable(CAMIO_CSI0_PCLK);
		msm_camio_clk_disable(CAMIO_CSI0_VFE_CLK);
		msm_camio_clk_disable(CAMIO_CSI0_CLK);
		msm_camio_clk_disable(CAMIO_VFE_CLK);
		iounmap(csibase);
		release_mem_region(camio_ext.csiphy, camio_ext.csisz);
	}
	msm_camio_clk_disable(CAMIO_VFE_PBDG_CLK);
#if !defined(CONFIG_SEMC_CAMERA_MODULE) && \
	!defined(CONFIG_SEMC_SUB_CAMERA_MODULE)
	msm_camio_clk_disable(CAMIO_CAM_MCLK_CLK);
#endif
}

void msm_camio_camif_pad_reg_reset(void)
{
	uint32_t reg;

	msm_camio_clk_sel(MSM_CAMIO_CLK_SRC_INTERNAL);
	msleep(10);

	reg = (msm_io_r(camifpadbase)) & CAMIF_CFG_RMSK;
	reg |= 0x3;
	msm_io_w(reg, camifpadbase);
	msleep(10);

	reg = (msm_io_r(camifpadbase)) & CAMIF_CFG_RMSK;
	reg |= 0x10;
	msm_io_w(reg, camifpadbase);
	msleep(10);

	reg = (msm_io_r(camifpadbase)) & CAMIF_CFG_RMSK;
	/* Need to be uninverted*/
	reg &= 0x03;
	msm_io_w(reg, camifpadbase);
	msleep(10);
}

void msm_camio_vfe_blk_reset(void)
{
	return;


}

void msm_camio_camif_pad_reg_reset_2(void)
{
	uint32_t reg;
	uint32_t mask, value;
	reg = (msm_io_r(camifpadbase)) & CAMIF_CFG_RMSK;
	mask = CAM_PAD_REG_SW_RESET_BMSK;
	value = 1 << CAM_PAD_REG_SW_RESET_SHFT;
	msm_io_w((reg & (~mask)) | (value & mask), camifpadbase);
	mdelay(10);
	reg = (msm_io_r(camifpadbase)) & CAMIF_CFG_RMSK;
	mask = CAM_PAD_REG_SW_RESET_BMSK;
	value = 0 << CAM_PAD_REG_SW_RESET_SHFT;
	msm_io_w((reg & (~mask)) | (value & mask), camifpadbase);
	mdelay(10);
}

void msm_camio_clk_sel(enum msm_camio_clk_src_type srctype)
{
	struct clk *clk = NULL;

	clk = camio_vfe_clk;

	if (clk != NULL) {
		switch (srctype) {
		case MSM_CAMIO_CLK_SRC_INTERNAL:
			clk_set_flags(clk, 0x00000100 << 1);
			break;

		case MSM_CAMIO_CLK_SRC_EXTERNAL:
			clk_set_flags(clk, 0x00000100);
			break;

		default:
			break;
		}
	}
}
int msm_camio_probe_on(struct platform_device *pdev)
{
	struct msm_camera_sensor_info *sinfo = pdev->dev.platform_data;
	struct msm_camera_device_platform_data *camdev = sinfo->pdata;
	camdev->camera_gpio_on();
#if defined(CONFIG_MACH_SEMC_ZEUS) || defined(CONFIG_MACH_SEMC_PHOENIX)
	return msm_camio_clk_enable(CAMIO_CAM_MCLK_CLK);
#else
	return 0;
#endif /* CONFIG_MACH_SEMC_ZEUS */
}

int msm_camio_probe_off(struct platform_device *pdev)
{
	struct msm_camera_sensor_info *sinfo = pdev->dev.platform_data;
	struct msm_camera_device_platform_data *camdev = sinfo->pdata;
	camdev->camera_gpio_off();
#if !defined(CONFIG_SEMC_CAMERA_MODULE) && \
	!defined(CONFIG_SEMC_SUB_CAMERA_MODULE)
	return msm_camio_clk_disable(CAMIO_CAM_MCLK_CLK);
#else
	return 0;
#endif
}

int msm_camio_sensor_clk_on(struct platform_device *pdev)
{
	int rc = 0;
	struct msm_camera_sensor_info *sinfo = pdev->dev.platform_data;
	struct msm_camera_device_platform_data *camdev = sinfo->pdata;
	camio_clk = camdev->ioclk;
	camio_ext = camdev->ioext;
	camdev->camera_gpio_on();
#if !defined(CONFIG_SEMC_CAMERA_MODULE) && \
	!defined(CONFIG_SEMC_SUB_CAMERA_MODULE)
	msm_camio_clk_enable(CAMIO_CAM_MCLK_CLK);
#endif
	msm_camio_clk_enable(CAMIO_CAMIF_PAD_PBDG_CLK);
	if (!sinfo->csi_if) {
		camifpadio = request_mem_region(camio_ext.camifpadphy,
			camio_ext.camifpadsz, pdev->name);
		msm_camio_clk_enable(CAMIO_VFE_CLK);
		if (!camifpadio) {
			rc = -EBUSY;
			goto common_fail;
		}
		camifpadbase = ioremap(camio_ext.camifpadphy,
			camio_ext.camifpadsz);
		if (!camifpadbase) {
			CDBG("msm_camio_sensor_clk_on fail\n");
			rc = -ENOMEM;
			goto parallel_busy;
		}
	}
	return rc;
parallel_busy:
	release_mem_region(camio_ext.camifpadphy, camio_ext.camifpadsz);
	goto common_fail;
common_fail:
	msm_camio_clk_disable(CAMIO_CAM_MCLK_CLK);
	msm_camio_clk_disable(CAMIO_VFE_CLK);
	msm_camio_clk_disable(CAMIO_CAMIF_PAD_PBDG_CLK);
	camdev->camera_gpio_off();
	return rc;
}

int msm_camio_sensor_clk_off(struct platform_device *pdev)
{
	uint32_t rc = 0;
	struct msm_camera_sensor_info *sinfo = pdev->dev.platform_data;
	struct msm_camera_device_platform_data *camdev = sinfo->pdata;
	camdev->camera_gpio_off();
#if !defined(CONFIG_SEMC_CAMERA_MODULE) && \
	!defined(CONFIG_SEMC_SUB_CAMERA_MODULE)
	rc = msm_camio_clk_disable(CAMIO_CAM_MCLK_CLK);
#endif
	rc = msm_camio_clk_disable(CAMIO_CAMIF_PAD_PBDG_CLK);
	if (!sinfo->csi_if) {
		iounmap(camifpadbase);
		release_mem_region(camio_ext.camifpadphy, camio_ext.camifpadsz);
		rc = msm_camio_clk_disable(CAMIO_VFE_CLK);
	}
	return rc;
}

int msm_camio_csi_config(struct msm_camera_csi_params *csi_params)
{
	int rc = 0;
	uint32_t val = 0;

	CDBG("msm_camio_csi_config \n");

	/* SOT_ECC_EN enable error correction for SYNC (data-lane) */
	msm_io_w(0x4, csibase + MIPI_PHY_CONTROL);

	/* SW_RST to the CSI core */
	msm_io_w(MIPI_PROTOCOL_CONTROL_SW_RST_BMSK,
		csibase + MIPI_PROTOCOL_CONTROL);

	/* PROTOCOL CONTROL */
	val = MIPI_PROTOCOL_CONTROL_LONG_PACKET_HEADER_CAPTURE_BMSK |
		MIPI_PROTOCOL_CONTROL_DECODE_ID_BMSK |
		MIPI_PROTOCOL_CONTROL_ECC_EN_BMSK;
	val |= (uint32_t)(csi_params->data_format) <<
		MIPI_PROTOCOL_CONTROL_DATA_FORMAT_SHFT;
	val |= csi_params->dpcm_scheme <<
		MIPI_PROTOCOL_CONTROL_DPCM_SCHEME_SHFT;
	CDBG("%s MIPI_PROTOCOL_CONTROL val=0x%x\n", __func__, val);
	msm_io_w(val, csibase + MIPI_PROTOCOL_CONTROL);

	/* SW CAL EN */
	val = (0x1 << MIPI_CALIBRATION_CONTROL_SWCAL_CAL_EN_SHFT) |
		(0x1 <<
		MIPI_CALIBRATION_CONTROL_SWCAL_STRENGTH_OVERRIDE_EN_SHFT) |
		(0x1 << MIPI_CALIBRATION_CONTROL_CAL_SW_HW_MODE_SHFT) |
		(0x1 << MIPI_CALIBRATION_CONTROL_MANUAL_OVERRIDE_EN_SHFT);
	CDBG("%s MIPI_CALIBRATION_CONTROL val=0x%x\n", __func__, val);
	msm_io_w(val, csibase + MIPI_CALIBRATION_CONTROL);

	/* settle_cnt is very sensitive to speed!
	increase this value to run at higher speeds */
	val = (csi_params->settle_cnt <<
			MIPI_PHY_D0_CONTROL2_SETTLE_COUNT_SHFT) |
		(0x0F << MIPI_PHY_D0_CONTROL2_HS_TERM_IMP_SHFT) |
		(0x1 << MIPI_PHY_D0_CONTROL2_LP_REC_EN_SHFT) |
		(0x1 << MIPI_PHY_D0_CONTROL2_ERR_SOT_HS_EN_SHFT);
	CDBG("%s MIPI_PHY_D0_CONTROL2 val=0x%x\n", __func__, val);
	msm_io_w(val, csibase + MIPI_PHY_D0_CONTROL2);
	msm_io_w(val, csibase + MIPI_PHY_D1_CONTROL2);
	msm_io_w(val, csibase + MIPI_PHY_D2_CONTROL2);
	msm_io_w(val, csibase + MIPI_PHY_D3_CONTROL2);


	val = (0x0F << MIPI_PHY_CL_CONTROL_HS_TERM_IMP_SHFT) |
		(0x1 << MIPI_PHY_CL_CONTROL_LP_REC_EN_SHFT);
	CDBG("%s MIPI_PHY_CL_CONTROL val=0x%x\n", __func__, val);
	msm_io_w(val, csibase + MIPI_PHY_CL_CONTROL);

	val = 0 << MIPI_PHY_D0_CONTROL_HS_REC_EQ_SHFT;
	msm_io_w(val, csibase + MIPI_PHY_D0_CONTROL);

	val = (0x1 << MIPI_PHY_D1_CONTROL_MIPI_CLK_PHY_SHUTDOWNB_SHFT) |
		(0x1 << MIPI_PHY_D1_CONTROL_MIPI_DATA_PHY_SHUTDOWNB_SHFT);
	CDBG("%s MIPI_PHY_D1_CONTROL val=0x%x\n", __func__, val);
	msm_io_w(val, csibase + MIPI_PHY_D1_CONTROL);

	msm_io_w(0x00000000, csibase + MIPI_PHY_D2_CONTROL);
	msm_io_w(0x00000000, csibase + MIPI_PHY_D3_CONTROL);

	/* halcyon only supports 1 or 2 lane */
	switch (csi_params->lane_cnt) {
	case 1:
		msm_io_w(csi_params->lane_assign << 8 | 0x4,
			csibase + MIPI_CAMERA_CNTL);
		break;
	case 2:
		msm_io_w(csi_params->lane_assign << 8 | 0x5,
			csibase + MIPI_CAMERA_CNTL);
		break;
	case 3:
		msm_io_w(csi_params->lane_assign << 8 | 0x6,
			csibase + MIPI_CAMERA_CNTL);
		break;
	case 4:
		msm_io_w(csi_params->lane_assign << 8 | 0x7,
			csibase + MIPI_CAMERA_CNTL);
		break;
	}

	/* mask out ID_ERROR[19], DATA_CMM_ERR[11]
	and CLK_CMM_ERR[10] - de-featured */
	msm_io_w(0xFFF7F3FF, csibase + MIPI_INTERRUPT_MASK);
	/*clear IRQ bits*/
	msm_io_w(0xFFF7F3FF, csibase + MIPI_INTERRUPT_STATUS);

	return rc;
}
