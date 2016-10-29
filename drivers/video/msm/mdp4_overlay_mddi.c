/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
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
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>
#include <linux/fb.h>
#include <asm/system.h>
#include <asm/mach-types.h>
#include <mach/hardware.h>

#include "mdp.h"
#include "msm_fb.h"
#include "mdp4.h"

static int vsync_start_y_adjust = 4;

#define MAX_CONTROLLER	1

/*
 * VSYNC_EXPIRE_TICK == 0 means clock always on
 * VSYNC_EXPIRE_TICK == 4 is recommended
 */
#define VSYNC_EXPIRE_TICK 4

#define VSYNC_MIN_DIFF_MS 4

static struct vsycn_ctrl {
	struct device *dev;
	int inited;
	int update_ndx;
	int expire_tick;
	int blt_wait;
	u32 ov_koff;
	u32 ov_done;
	u32 dma_koff;
	u32 dma_done;
	u32 pan_display;
	uint32 rdptr_intr_tot;
	uint32 rdptr_sirq_tot;
	atomic_t suspend;
	int blt_change;
	int blt_free;
	int blt_end;
	int sysfs_created;
	struct mutex update_lock;
	struct completion ov_comp;
	struct completion dma_comp;
	spinlock_t spin_lock;
	struct msm_fb_data_type *mfd;
	struct mdp4_overlay_pipe *base_pipe;
	struct vsync_update vlist[2];
	int vsync_enabled;
	int clk_enabled;
	int clk_control;
	ktime_t vsync_time;
	u32 last_vsync_ms;
	struct work_struct clk_work;
	wait_queue_head_t wait_queue;
} vsync_ctrl_db[MAX_CONTROLLER];

/* MDP 4.0 versions earlier than 2.1 suffer underrun from DMA_P pipe on
 * MDDI panels. To overcome this MDDI overlay driver uses DMA_S pipe. */
static bool mddi_use_dmap(void)
{
	return (mdp_hw_revision >= MDP4_REVISION_V2_1);
}

static void vsync_irq_enable(int intr, int term)
{
	unsigned long flag;

	spin_lock_irqsave(&mdp_spin_lock, flag);
	/* no need to clrear other interrupts for comamnd mode */
	mdp_intr_mask |= intr;
	outp32(MDP_INTR_ENABLE, mdp_intr_mask);
	mdp_enable_irq(term);
	spin_unlock_irqrestore(&mdp_spin_lock, flag);
}

static void vsync_irq_disable(int intr, int term)
{
	unsigned long flag;

	spin_lock_irqsave(&mdp_spin_lock, flag);
	/* no need to clrear other interrupts for comamnd mode */
	mdp_intr_mask &= ~intr;
	outp32(MDP_INTR_ENABLE, mdp_intr_mask);
	mdp_disable_irq_nosync(term);
	spin_unlock_irqrestore(&mdp_spin_lock, flag);
}

static void mdp4_mddi_blt_ov_update(struct mdp4_overlay_pipe *pipe)
{
	uint32 off, addr;
	int bpp;
	char *overlay_base;

	if (pipe->ov_blt_addr == 0)
		return;

#ifdef BLT_RGB565
	bpp = 2; /* overlay ouput is RGB565 */
#else
	bpp = 3; /* overlay ouput is RGB888 */
#endif
	off = 0;
	if (pipe->ov_cnt & 0x01)
		off = pipe->src_height * pipe->src_width * bpp;
	addr = pipe->ov_blt_addr + off;
	/* overlay 0 */
	overlay_base = MDP_BASE + MDP4_OVERLAYPROC0_BASE;/* 0x10000 */
	outpdw(overlay_base + 0x000c, addr);
	outpdw(overlay_base + 0x001c, addr);
}

static void mdp4_mddi_blt_dmap_update(struct mdp4_overlay_pipe *pipe)
{
	uint32 off, addr;
	int bpp;

	if (pipe->ov_blt_addr == 0)
		return;

#ifdef BLT_RGB565
	bpp = 2; /* overlay ouput is RGB565 */
#else
	bpp = 3; /* overlay ouput is RGB888 */
#endif
	off = 0;
	if (pipe->dmap_cnt & 0x01)
		off = pipe->src_height * pipe->src_width * bpp;
	addr = pipe->dma_blt_addr + off;

	/* dmap */
	MDP_OUTP(MDP_BASE + 0x90008, addr);
}

static void mdp4_mddi_blt_dmas_update(struct mdp4_overlay_pipe *pipe)
{
	uint32 off, addr;
	int bpp;

	if (pipe->ov_blt_addr == 0)
		return;

#ifdef BLT_RGB565
	bpp = 2; /* overlay ouput is RGB565 */
#else
	bpp = 3; /* overlay ouput is RGB888 */
#endif
	off = 0;
	if (pipe->dmas_cnt & 0x01)
		off = pipe->src_height * pipe->src_width * bpp;
	addr = pipe->dma_blt_addr + off;

	/* dmas */
	MDP_OUTP(MDP_BASE + 0xa0008, addr);
}

static void mdp4_mddi_wait4dma(int cndx);
static void mdp4_mddi_wait4ov(int cndx);

static void mdp4_mddi_do_blt(struct msm_fb_data_type *mfd, int enable)
{
	unsigned long flags;
	int cndx = 0;
	struct vsycn_ctrl *vctrl;
	struct mdp4_overlay_pipe *pipe;
	int need_wait = 0;

	vctrl = &vsync_ctrl_db[cndx];
	pipe = vctrl->base_pipe;

	mdp4_allocate_writeback_buf(mfd, MDP4_MIXER0);

	if (mfd->ov0_wb_buf->write_addr == 0) {
		pr_err("%s: no blt_base assigned\n", __func__);
		return;
	}

	spin_lock_irqsave(&vctrl->spin_lock, flags);
	if (enable && pipe->ov_blt_addr == 0) {
		vctrl->blt_change++;
		if (vctrl->dma_koff != vctrl->dma_done) {
			INIT_COMPLETION(vctrl->dma_comp);
			need_wait = 1;
		}
	} else if (enable == 0 && pipe->ov_blt_addr) {
		vctrl->blt_change++;
		if (vctrl->ov_koff != vctrl->dma_done) {
			INIT_COMPLETION(vctrl->dma_comp);
			need_wait = 1;
		}
	}
	spin_unlock_irqrestore(&vctrl->spin_lock, flags);

	if (need_wait)
		mdp4_mddi_wait4dma(0);

	spin_lock_irqsave(&vctrl->spin_lock, flags);
	if (enable && pipe->ov_blt_addr == 0) {
		pipe->ov_blt_addr = mfd->ov0_wb_buf->write_addr;
		pipe->dma_blt_addr = mfd->ov0_wb_buf->read_addr;
		pipe->ov_cnt = 0;
		pipe->dmap_cnt = 0;
		pipe->dmas_cnt = 0;
		vctrl->ov_koff = vctrl->dma_koff;
		vctrl->ov_done = vctrl->dma_done;
		vctrl->blt_free = 0;
		vctrl->blt_wait = 0;
		vctrl->blt_end = 0;
		mdp4_stat.blt_mddi++;
	} else if (enable == 0 && pipe->ov_blt_addr) {
		pipe->ov_blt_addr = 0;
		pipe->dma_blt_addr =  0;
		vctrl->blt_end = 1;
		vctrl->blt_free = 4;	/* 4 commits to free wb buf */
	}

	pr_debug("%s: changed=%d enable=%d ov_blt_addr=%x\n", __func__,
		vctrl->blt_change, enable, (int)pipe->ov_blt_addr);

	spin_unlock_irqrestore(&vctrl->spin_lock, flags);
}

/*
 * mdp4_mddi_do_update:
 * called from thread context
 */
void mdp4_mddi_pipe_queue(int cndx, struct mdp4_overlay_pipe *pipe)
{
	struct vsycn_ctrl *vctrl;
	struct vsync_update *vp;
	struct mdp4_overlay_pipe *pp;
	int undx;

	if (cndx >= MAX_CONTROLLER) {
		pr_err("%s: out or range: cndx=%d\n", __func__, cndx);
		return;
	}

	vctrl = &vsync_ctrl_db[cndx];

	if (atomic_read(&vctrl->suspend)) {
		pr_err("%s: suspended, no more pipe queue\n", __func__);
		return;
	}

	mutex_lock(&vctrl->update_lock);
	undx =  vctrl->update_ndx;
	vp = &vctrl->vlist[undx];

	pp = &vp->plist[pipe->pipe_ndx - 1];	/* ndx start form 1 */

	pr_debug("%s: vndx=%d pipe_ndx=%d expire=%x pid=%d\n", __func__,
		undx, pipe->pipe_ndx, vctrl->expire_tick, current->pid);

	*pp = *pipe;	/* clone it */
	vp->update_cnt++;

	mutex_unlock(&vctrl->update_lock);
	mdp4_stat.overlay_play[pipe->mixer_num]++;
}

static void mdp4_mddi_pipe_clean(struct vsync_update *vp)
{
	struct mdp4_overlay_pipe *pipe;
	int i;

	pipe = vp->plist;
	for (i = 0; i < OVERLAY_PIPE_MAX; i++, pipe++) {
		if (pipe->pipe_used) {
			mdp4_overlay_iommu_pipe_free(pipe->pipe_ndx, 0);
			pipe->pipe_used = 0; /* clear */
		}
	}
	vp->update_cnt = 0;     /* empty queue */
}

static void mdp4_mddi_blt_ov_update(struct mdp4_overlay_pipe *pipe);
static int mdp4_mddi_clk_check(struct vsycn_ctrl *vctrl);

int mdp4_mddi_pipe_commit(int cndx, int wait)
{
	int  i, undx;
	int mixer = 0;
	struct vsycn_ctrl *vctrl;
	struct vsync_update *vp;
	struct mdp4_overlay_pipe *pipe;
	struct mdp4_overlay_pipe *real_pipe;
	unsigned long flags;
	int need_dma_wait = 0;
	int need_ov_wait = 0;
	int cnt = 0;

	vctrl = &vsync_ctrl_db[0];

	mutex_lock(&vctrl->update_lock);
	undx =  vctrl->update_ndx;
	vp = &vctrl->vlist[undx];
	pipe = vctrl->base_pipe;
	if (pipe == NULL) {
		pr_err("%s: NO base pipe\n", __func__);
		mutex_unlock(&vctrl->update_lock);
		return 0;
	}

	mixer = pipe->mixer_num;

	mdp_update_pm(vctrl->mfd, vctrl->vsync_time);

	/*
	 * allow stage_commit without pipes queued
	 * (vp->update_cnt == 0) to unstage pipes after
	 * overlay_unset
	 */

	vctrl->update_ndx++;
	vctrl->update_ndx &= 0x01;
	vp->update_cnt = 0;     /* reset */
	if (vctrl->blt_free) {
		vctrl->blt_free--;
		if (vctrl->blt_free == 0)
			mdp4_free_writeback_buf(vctrl->mfd, mixer);
	}

	if (mdp4_mddi_clk_check(vctrl) < 0) {
		mdp4_mddi_pipe_clean(vp);
		mutex_unlock(&vctrl->update_lock);
		return 0;
	}
	mutex_unlock(&vctrl->update_lock);

	/* free previous committed iommu back to pool */
	mdp4_overlay_iommu_unmap_freelist(mixer);

	spin_lock_irqsave(&vctrl->spin_lock, flags);
	if (pipe->ov_blt_addr) {
		/* Blt */
		if (vctrl->blt_wait) {
			INIT_COMPLETION(vctrl->dma_comp);
			need_dma_wait = 1;
		}
		if (vctrl->ov_koff != vctrl->ov_done) {
			INIT_COMPLETION(vctrl->ov_comp);
			need_ov_wait = 1;
		}
	} else {
		/* direct out */
		if (vctrl->dma_koff != vctrl->dma_done) {
			INIT_COMPLETION(vctrl->dma_comp);
			pr_debug("%s: wait, ok=%d od=%d dk=%d dd=%d cpu=%d\n",
			 __func__, vctrl->ov_koff, vctrl->ov_done,
			vctrl->dma_koff, vctrl->dma_done, smp_processor_id());
			need_dma_wait = 1;
		}
	}
	spin_unlock_irqrestore(&vctrl->spin_lock, flags);

	if (need_dma_wait) {
		pr_debug("%s: wait4dma\n", __func__);
		mdp4_mddi_wait4dma(0);
	}

	if (need_ov_wait) {
		pr_debug("%s: wait4ov\n", __func__);
		mdp4_mddi_wait4ov(0);
	}

	if (pipe->ov_blt_addr) {
		if (vctrl->blt_end) {
			vctrl->blt_end = 0;
			pipe->ov_blt_addr = 0;
			pipe->dma_blt_addr =  0;
		}
	}

	if (vctrl->blt_change) {
		mdp4_overlayproc_cfg(pipe);
		if (mddi_use_dmap())
			mdp4_overlay_dmap_xy(pipe);
		else
			mdp4_overlay_dmas_xy(pipe);
		vctrl->blt_change = 0;
	}

	pipe = vp->plist;
	for (i = 0; i < OVERLAY_PIPE_MAX; i++, pipe++) {
		if (pipe->pipe_used) {
			cnt++;
			real_pipe = mdp4_overlay_ndx2pipe(pipe->pipe_ndx);
			if (real_pipe && real_pipe->pipe_used) {
				/* pipe not unset */
				mdp4_overlay_vsync_commit(pipe);
			}
			/* free previous iommu to freelist
			* which will be freed at next
			* pipe_commit
			*/
			mdp4_overlay_iommu_pipe_free(pipe->pipe_ndx, 0);
			pipe->pipe_used = 0; /* clear */
		}
	}

	mdp4_mixer_stage_commit(mixer);

	pipe = vctrl->base_pipe;
	spin_lock_irqsave(&vctrl->spin_lock, flags);
	if (pipe->ov_blt_addr) {
		mdp4_mddi_blt_ov_update(pipe);
		pipe->ov_cnt++;
		vctrl->ov_koff++;
		INIT_COMPLETION(vctrl->ov_comp);
		vsync_irq_enable(INTR_OVERLAY0_DONE, MDP_OVERLAY0_TERM);
		mdp_pipe_kickoff_simplified(MDP_OVERLAY0_TERM);
		mdp4_stat.kickoff_ov0++;
	} else {
		INIT_COMPLETION(vctrl->dma_comp);
		if (mddi_use_dmap()) {
			vsync_irq_enable(INTR_DMA_P_DONE, MDP_DMAP_TERM);
			mdp4_stat.kickoff_dmap++;
			/* kickoff overlay engine */
			mdp4_stat.kickoff_ov0++;
			mdp_pipe_kickoff_simplified(MDP_OVERLAY0_TERM);
		} else {
			vsync_irq_enable(INTR_DMA_S_DONE, MDP_DMA_S_TERM);
			mdp_pipe_kickoff_simplified(MDP_DMA_S_TERM);
			mdp4_stat.kickoff_dmas++;
		}
		vctrl->dma_koff++;
	}
	pr_debug("%s: kickoff, pid=%d\n", __func__, current->pid);

	mb(); /* make sure kickoff ececuted */
	spin_unlock_irqrestore(&vctrl->spin_lock, flags);

	mdp4_stat.overlay_commit[pipe->mixer_num]++;

	if (wait) {
		mutex_unlock(&vctrl->mfd->dma->ov_mutex);
		/* In case of writeback, there is explicit need to synchronize the first
		 * rendering before writing to display. */
		if (pipe->ov_blt_addr)
			mdp4_mddi_wait4ov(0);
		else
			mdp4_mddi_wait4vsync(0);
		mutex_lock(&vctrl->mfd->dma->ov_mutex);
	}

	return cnt;
}

static void mdp4_overlay_update_mddi(struct msm_fb_data_type *mfd);

void mdp4_mddi_vsync_ctrl(struct fb_info *info, int enable)
{
	struct vsycn_ctrl *vctrl;
	unsigned long flags;
	int cndx = 0;
	int clk_set_on = 0;

	vctrl = &vsync_ctrl_db[cndx];

	mutex_lock(&vctrl->update_lock);
	pr_debug("%s: clk_enabled=%d vsync_enabled=%d req=%d\n", __func__,
		vctrl->clk_enabled, vctrl->vsync_enabled, enable);

	if (vctrl->vsync_enabled == enable) {
		mutex_unlock(&vctrl->update_lock);
		return;
	}

	vctrl->vsync_enabled = enable;

	if (enable) {
		spin_lock_irqsave(&vctrl->spin_lock, flags);
		vctrl->clk_control = 0;
		vctrl->expire_tick = 0;
		spin_unlock_irqrestore(&vctrl->spin_lock, flags);
		if (vctrl->clk_enabled == 0) {
			pr_debug("%s: SET_CLK_ON\n", __func__);
			mdp_clk_ctrl(1);
			vctrl->clk_enabled = 1;
			clk_set_on = 1;
			vctrl->last_vsync_ms =
				ktime_to_ms(ktime_get()) - VSYNC_MIN_DIFF_MS;
		}
		if (clk_set_on) {
			vsync_irq_enable(INTR_PRIMARY_RDPTR,
						MDP_PRIM_RDPTR_TERM);
		}
	} else {
		spin_lock_irqsave(&vctrl->spin_lock, flags);
		vctrl->expire_tick = VSYNC_EXPIRE_TICK;
		spin_unlock_irqrestore(&vctrl->spin_lock, flags);
	}
	mutex_unlock(&vctrl->update_lock);
}

void mdp4_mddi_wait4vsync(int cndx)
{
	struct vsycn_ctrl *vctrl;
	struct mdp4_overlay_pipe *pipe;

	if (cndx >= MAX_CONTROLLER) {
		pr_err("%s: out or range: cndx=%d\n", __func__, cndx);
		return;
	}

	vctrl = &vsync_ctrl_db[cndx];
	pipe = vctrl->base_pipe;

	if (atomic_read(&vctrl->suspend) > 0)
		return;

	wait_event_interruptible_timeout(vctrl->wait_queue, 1,
			msecs_to_jiffies(VSYNC_PERIOD * 8));

	mdp4_stat.wait4vsync0++;
}

static void mdp4_mddi_wait4dma(int cndx)
{
	struct vsycn_ctrl *vctrl;

	if (cndx >= MAX_CONTROLLER) {
		pr_err("%s: out or range: cndx=%d\n", __func__, cndx);
		return;
	}

	vctrl = &vsync_ctrl_db[cndx];

	if (atomic_read(&vctrl->suspend) > 0)
		return;

	wait_for_completion(&vctrl->dma_comp);
}

static void mdp4_mddi_wait4ov(int cndx)
{
	struct vsycn_ctrl *vctrl;

	if (cndx >= MAX_CONTROLLER) {
		pr_err("%s: out or range: cndx=%d\n", __func__, cndx);
		return;
	}

	vctrl = &vsync_ctrl_db[cndx];

	if (atomic_read(&vctrl->suspend) > 0)
		return;

	wait_for_completion(&vctrl->ov_comp);
}

/*
 * primary_rdptr_isr:
 * called from interrupt context
 */

static void primary_rdptr_isr(int cndx)
{
	struct vsycn_ctrl *vctrl;
	u32 cur_vsync_ms;
	int vsync_diff;
	vctrl = &vsync_ctrl_db[cndx];
	pr_debug("%s: ISR, tick=%d pan=%d cpu=%d\n", __func__,
		vctrl->expire_tick, vctrl->pan_display, smp_processor_id());
	vctrl->rdptr_intr_tot++;

	spin_lock(&vctrl->spin_lock);
	vctrl->vsync_time = ktime_get();
	cur_vsync_ms =  ktime_to_ms(vctrl->vsync_time);
	vsync_diff = (int)(cur_vsync_ms - vctrl->last_vsync_ms);

	if ((vsync_diff >= 0) && (vsync_diff < VSYNC_MIN_DIFF_MS)) {
		spin_unlock(&vctrl->spin_lock);
		return;
	}

	vctrl->last_vsync_ms = cur_vsync_ms;
	wake_up_interruptible_all(&vctrl->wait_queue);

	if (vctrl->expire_tick) {
		vctrl->expire_tick--;
		if (vctrl->expire_tick == 0) {
			if (vctrl->pan_display <= 0) {
				vctrl->clk_control = 1;
				schedule_work(&vctrl->clk_work);
			} else {
				/* wait one more vsycn */
				vctrl->expire_tick += 1;
			}
		}
	}
	spin_unlock(&vctrl->spin_lock);
}

void mdp4_dmap_done_mddi(int cndx)
{
	struct vsycn_ctrl *vctrl;
	struct mdp4_overlay_pipe *pipe;
	int diff;
	vctrl = &vsync_ctrl_db[cndx];
	pipe = vctrl->base_pipe;

	 /* blt enabled */
	spin_lock(&vctrl->spin_lock);
	vsync_irq_disable(INTR_DMA_P_DONE, MDP_DMAP_TERM);
	vctrl->dma_done++;

	if (vctrl->pan_display)
		vctrl->pan_display--;

	diff = vctrl->ov_done - vctrl->dma_done;
	pr_debug("%s: ov_koff=%d ov_done=%d dma_koff=%d dma_done=%d cpu=%d\n",
		__func__, vctrl->ov_koff, vctrl->ov_done, vctrl->dma_koff,
		vctrl->dma_done, smp_processor_id());
	complete(&vctrl->dma_comp);
	if (diff <= 0) {
		if (vctrl->blt_wait)
			vctrl->blt_wait = 0;
		spin_unlock(&vctrl->spin_lock);
		return;
	}

	/* kick dmap */
	mdp4_mddi_blt_dmap_update(pipe);
	pipe->dmap_cnt++;
	mdp4_stat.kickoff_dmap++;
	vctrl->dma_koff++;
	vsync_irq_enable(INTR_DMA_P_DONE, MDP_DMAP_TERM);
	mdp_pipe_kickoff_simplified(MDP_DMAP_TERM);
	mb(); /* make sure kickoff executed */
	spin_unlock(&vctrl->spin_lock);
}

void mdp4_dmas_done_mddi(int cndx)
{
	struct vsycn_ctrl *vctrl;
	struct mdp4_overlay_pipe *pipe;
	int diff;
	vctrl = &vsync_ctrl_db[cndx];
	pipe = vctrl->base_pipe;

	 /* blt enabled */
	spin_lock(&vctrl->spin_lock);
	vsync_irq_disable(INTR_DMA_S_DONE, MDP_DMA_S_TERM);
	vctrl->dma_done++;

	if (vctrl->pan_display)
		vctrl->pan_display--;

	diff = vctrl->ov_done - vctrl->dma_done;
	pr_debug("%s: ov_koff=%d ov_done=%d dma_koff=%d dma_done=%d cpu=%d\n",
		__func__, vctrl->ov_koff, vctrl->ov_done, vctrl->dma_koff,
		vctrl->dma_done, smp_processor_id());
	complete(&vctrl->dma_comp);
	if (diff <= 0) {
		if (vctrl->blt_wait)
			vctrl->blt_wait = 0;
		spin_unlock(&vctrl->spin_lock);
		return;
	}

	/* kick dmas */
	mdp4_mddi_blt_dmas_update(pipe);
	pipe->dmas_cnt++;
	mdp4_stat.kickoff_dmas++;
	vctrl->dma_koff++;
	vsync_irq_enable(INTR_DMA_S_DONE, MDP_DMA_S_TERM);
	mdp_pipe_kickoff_simplified(MDP_DMA_S_TERM);
	mb(); /* make sure kickoff executed */
	spin_unlock(&vctrl->spin_lock);
}

/*
 * mdp4_overlay0_done_mddi: called from isr
 */
void mdp4_overlay0_done_mddi(int cndx)
{
	struct vsycn_ctrl *vctrl;
	struct mdp4_overlay_pipe *pipe;
	int diff;

	vctrl = &vsync_ctrl_db[cndx];
	pipe = vctrl->base_pipe;

	spin_lock(&vctrl->spin_lock);
	vsync_irq_disable(INTR_OVERLAY0_DONE, MDP_OVERLAY0_TERM);
	vctrl->ov_done++;
	complete(&vctrl->ov_comp);
	diff = vctrl->ov_done - vctrl->dma_done;

	pr_debug("%s: ov_koff=%d ov_done=%d dma_koff=%d dma_done=%d cpu=%d\n",
		__func__, vctrl->ov_koff, vctrl->ov_done, vctrl->dma_koff,
		vctrl->dma_done, smp_processor_id());

	if (pipe->ov_blt_addr == 0) {
		/* blt disabled */
		spin_unlock(&vctrl->spin_lock);
		return;
	}

	if (diff > 1) {
		/*
		 * two overlay_done and none dma_done yet
		 * let dma_done kickoff dma
		 * and put pipe_commit to wait
		 */
		vctrl->blt_wait = 1;
		pr_debug("%s: blt_wait set\n", __func__);
		spin_unlock(&vctrl->spin_lock);
		return;
	}

	if (mddi_use_dmap()) {
		mdp4_mddi_blt_dmap_update(pipe);
		pipe->dmap_cnt++;
		mdp4_stat.kickoff_dmap++;
		vctrl->dma_koff++;
		vsync_irq_enable(INTR_DMA_P_DONE, MDP_DMAP_TERM);
		mdp_pipe_kickoff_simplified(MDP_DMAP_TERM);
	} else {
		mdp4_mddi_blt_dmas_update(pipe);
		pipe->dmas_cnt++;
		mdp4_stat.kickoff_dmas++;
		vctrl->dma_koff++;
		vsync_irq_enable(INTR_DMA_S_DONE, MDP_DMA_S_TERM);
		mdp_pipe_kickoff_simplified(MDP_DMA_S_TERM);
	}
	mb(); /* make sure kickoff executed */
	spin_unlock(&vctrl->spin_lock);
}

static void clk_ctrl_work(struct work_struct *work)
{
	unsigned long flags;
	struct vsycn_ctrl *vctrl =
		container_of(work, typeof(*vctrl), clk_work);

	mutex_lock(&vctrl->update_lock);
	spin_lock_irqsave(&vctrl->spin_lock, flags);
	if (vctrl->clk_control && vctrl->clk_enabled) {
		vsync_irq_disable(INTR_PRIMARY_RDPTR, MDP_PRIM_RDPTR_TERM);
		vctrl->clk_enabled = 0;
		vctrl->clk_control = 0;
		spin_unlock_irqrestore(&vctrl->spin_lock, flags);
		mdp_clk_ctrl(0);
		pr_debug("%s: SET_CLK_OFF, pid=%d\n", __func__, current->pid);
	} else {
		spin_unlock_irqrestore(&vctrl->spin_lock, flags);
	}
	mutex_unlock(&vctrl->update_lock);
}

ssize_t mdp4_mddi_show_event(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int cndx;
	struct vsycn_ctrl *vctrl;
	ssize_t ret = 0;
	u64 vsync_tick;
	ktime_t timestamp;

	cndx = 0;
	vctrl = &vsync_ctrl_db[0];
	timestamp = vctrl->vsync_time;

	ret = wait_event_interruptible(vctrl->wait_queue,
			!ktime_equal(timestamp, vctrl->vsync_time) &&
			vctrl->vsync_enabled);
	if (ret == -ERESTARTSYS)
		return ret;

	vsync_tick = ktime_to_ns(vctrl->vsync_time);
	ret = scnprintf(buf, PAGE_SIZE, "VSYNC=%llu\n", vsync_tick);
	buf[strlen(buf) + 1] = '\0';
	return ret;
}

void mdp4_mddi_rdptr_init(int cndx)
{
	struct vsycn_ctrl *vctrl;

	if (cndx >= MAX_CONTROLLER) {
		pr_err("%s: out or range: cndx=%d\n", __func__, cndx);
		return;
	}

	vctrl = &vsync_ctrl_db[cndx];
	if (vctrl->inited)
		return;

	vctrl->inited = 1;
	vctrl->update_ndx = 0;
	mutex_init(&vctrl->update_lock);
	init_completion(&vctrl->ov_comp);
	init_completion(&vctrl->dma_comp);
	spin_lock_init(&vctrl->spin_lock);
	init_waitqueue_head(&vctrl->wait_queue);
	atomic_set(&vctrl->suspend, 1);
	INIT_WORK(&vctrl->clk_work, clk_ctrl_work);
}

void mdp4_primary_rdptr(void)
{
	primary_rdptr_isr(0);
}

static void mdp4_mddi_vsync_enable(struct msm_fb_data_type *mfd,
		struct mdp4_overlay_pipe *pipe, int which)
{
	uint32 start_y, data, tear_en;

	tear_en = (1 << which);

	mdp_clk_ctrl(1);

	data = inpdw(MDP_BASE + 0x20c);
	if ((mfd->use_mdp_vsync) && (mfd->ibuf.vsync_enable) &&
	    (mfd->panel_info.lcd.vsync_enable)) {
		data |= tear_en;
		if (mfd->panel_info.lcd.primary_rdptr_irq) {
			MDP_OUTP(MDP_BASE + 0x21C,
				mfd->panel_info.lcd.primary_rdptr_irq);
		} else {
			MDP_OUTP(MDP_BASE + 0x21C, 1);
		}

		/*
		 * adjust start position and threshold to make sure
		 * write ptr follows read pts (TE is effective), and
		 * at the same write is not throttled(shorter dma
		 * time)
		 */
		if (vsync_start_y_adjust <= pipe->dst_y)
			start_y = pipe->dst_y - vsync_start_y_adjust;
		else
			start_y = (mfd->total_lcd_lines - 1) -
				(vsync_start_y_adjust - pipe->dst_y);

		if (mfd->panel_info.lcd.primary_start_pos)
			start_y = mfd->panel_info.lcd.primary_start_pos;

		if (which == 0)
			MDP_OUTP(MDP_BASE + 0x210, start_y); /* primary */
		else
			MDP_OUTP(MDP_BASE + 0x214, start_y); /* secondary */
	} else
		data &= ~tear_en;
	MDP_OUTP(MDP_BASE + 0x20c, data);
	mdp_clk_ctrl(0);
}

void mdp4_mddi_free_base_pipe(struct msm_fb_data_type *mfd)
{
	struct vsycn_ctrl *vctrl;
	struct mdp4_overlay_pipe *pipe;

	vctrl = &vsync_ctrl_db[0];
	pipe = vctrl->base_pipe;

	if (pipe == NULL)
		return ;
	/* adb stop */
	if (pipe->pipe_type == OVERLAY_TYPE_BF)
		mdp4_overlay_borderfill_stage_down(pipe);

	/* base pipe may change after borderfill_stage_down */
	pipe = vctrl->base_pipe;
	mdp4_mixer_stage_down(pipe, 1);
	mdp4_overlay_pipe_free(pipe, 1);
	vctrl->base_pipe = NULL;
}

void mdp4_mddi_base_swap(int cndx, struct mdp4_overlay_pipe *pipe)
{
	struct vsycn_ctrl *vctrl;

	if (cndx >= MAX_CONTROLLER) {
		pr_err("%s: out or range: cndx=%d\n", __func__, cndx);
		return;
	}

	vctrl = &vsync_ctrl_db[cndx];
	vctrl->base_pipe = pipe;
}

static void mdp4_overlay_setup_pipe_addr(struct msm_fb_data_type *mfd,
			struct mdp4_overlay_pipe *pipe)
{
	MDPIBUF *iBuf = &mfd->ibuf;
	struct fb_info *fbi;
	uint8 *src;

	/* whole screen for base layer */
	src = (uint8 *) iBuf->buf;
	fbi = mfd->fbi;

	pipe->src_height = fbi->var.yres;
	pipe->src_width = fbi->var.xres;
	pipe->src_h = fbi->var.yres;
	pipe->src_w = fbi->var.xres;
	pipe->dst_h = fbi->var.yres;
	pipe->dst_w = fbi->var.xres;
	pipe->srcp0_ystride = fbi->fix.line_length;
	pipe->src_y = 0;
	pipe->src_x = 0;
	pipe->dst_y = 0;
	pipe->dst_x = 0;
	pipe->srcp0_addr = (uint32)src;
}

static void mdp4_overlay_update_mddi(struct msm_fb_data_type *mfd)
{
	int ptype;
	struct mdp4_overlay_pipe *pipe;
	int ret;
	int cndx = 0;
	struct vsycn_ctrl *vctrl;
	uint16 mddi_ld_param, mddi_vdo_packet_reg;
	uint32 data;


	if (mfd->key != MFD_KEY)
		return;

	vctrl = &vsync_ctrl_db[cndx];

	if (vctrl->base_pipe == NULL) {
		ptype = mdp4_overlay_format2type(mfd->fb_imgType);
		if (ptype < 0)
			printk(KERN_INFO "%s: format2type failed\n", __func__);
		pipe = mdp4_overlay_pipe_alloc(ptype, MDP4_MIXER0);
		if (pipe == NULL) {
			printk(KERN_INFO "%s: pipe_alloc failed\n", __func__);
			return;
		}
		pipe->pipe_used++;
		pipe->mixer_stage  = MDP4_MIXER_STAGE_BASE;
		pipe->mixer_num  = MDP4_MIXER0;
		pipe->src_format = mfd->fb_imgType;
		mdp4_overlay_panel_mode(pipe->mixer_num, MDP4_PANEL_MDDI);
		ret = mdp4_overlay_format2pipe(pipe);
		if (ret < 0)
			printk(KERN_INFO "%s: format2type failed\n", __func__);

		vctrl->base_pipe = pipe; /* keep it */
		pipe->ov_blt_addr = 0;
		pipe->dma_blt_addr = 0;
	} else {
		pipe = vctrl->base_pipe;
	}

	/* TE enabled */
	if (mddi_use_dmap()) {
		mdp4_mddi_vsync_enable(mfd, pipe, 0);
	} else {
		mdp4_mddi_vsync_enable(mfd, pipe, 1);
	}

	mdp4_overlay_mdp_pipe_req(pipe, mfd);
	mdp4_calc_blt_mdp_bw(mfd, pipe);

	if (mddi_use_dmap()) {
		data = inpdw(MDP_BASE + 0x0028);
		data &= ~0x0300;	/* bit 8, 9, MASTER4 */
		if (mfd->fbi->var.xres == 540) /* qHD, 540x960 */
			data |= 0x0200;
		else
			data |= 0x0100;

		MDP_OUTP(MDP_BASE + 0x00028, data);
	}

	if (mddi_use_dmap()) {
		mddi_ld_param = 0; /* DMA_P */
	} else {
		mddi_ld_param = 1; /* DMA_S */
	}
	mddi_vdo_packet_reg = mfd->panel_info.mddi.vdopkt;

	MDP_OUTP(MDP_BASE + 0x00090, mddi_ld_param);

	if (mfd->panel_info.bpp == 24)
		MDP_OUTP(MDP_BASE + 0x00094,
		 (MDDI_VDO_PACKET_DESC_24 << 16) | mddi_vdo_packet_reg);
	else if (mfd->panel_info.bpp == 16)
		MDP_OUTP(MDP_BASE + 0x00094,
		 (MDDI_VDO_PACKET_DESC_16 << 16) | mddi_vdo_packet_reg);
	else
		MDP_OUTP(MDP_BASE + 0x00094,
		 (MDDI_VDO_PACKET_DESC << 16) | mddi_vdo_packet_reg);

	MDP_OUTP(MDP_BASE + 0x00098, 0x1);

	mdp4_overlay_solidfill_init(pipe);

	mdp4_overlay_setup_pipe_addr(mfd, pipe);

	mdp4_overlay_rgb_setup(pipe);

	mdp4_overlay_reg_flush(pipe, 1);

	mdp4_mixer_stage_up(pipe, 0);

	mdp4_overlayproc_cfg(pipe);

	if (mddi_use_dmap()) {
		mdp4_overlay_dmap_xy(pipe);

		mdp4_overlay_dmap_cfg(mfd, 0);
	} else {
		mdp4_overlay_dmas_xy(pipe);

		mdp4_overlay_dmas_cfg(mfd);
	}

	wmb();
}

void mdp4_mddi_blt_start(struct msm_fb_data_type *mfd)
{
	mdp4_mddi_do_blt(mfd, 1);
}

void mdp4_mddi_blt_stop(struct msm_fb_data_type *mfd)
{
	mdp4_mddi_do_blt(mfd, 0);
}

void mdp4_mddi_overlay_blt(struct msm_fb_data_type *mfd,
					struct msmfb_overlay_blt *req)
{
	mdp4_mddi_do_blt(mfd, req->enable);
}

int mdp4_mddi_on(struct platform_device *pdev)
{
	int ret = 0;
	int cndx = 0;
	struct msm_fb_data_type *mfd;
	struct vsycn_ctrl *vctrl;

	pr_debug("%s+: pid=%d\n", __func__, current->pid);

	mfd = (struct msm_fb_data_type *)platform_get_drvdata(pdev);
	mfd->cont_splash_done = 1;

	mutex_lock(&mfd->dma->ov_mutex);

	vctrl = &vsync_ctrl_db[cndx];
	vctrl->mfd = mfd;
	vctrl->dev = mfd->fbi->dev;
	vctrl->vsync_enabled = 0;

	mdp_clk_ctrl(1);
	mdp4_overlay_update_mddi(mfd);
	mdp_clk_ctrl(0);

	mdp4_iommu_attach();

	atomic_set(&vctrl->suspend, 0);

	if (!mddi_use_dmap())
		mdp4_mddi_blt_start(mfd);

	mutex_unlock(&mfd->dma->ov_mutex);

	pr_debug("%s-:\n", __func__);

	return ret;
}

int mdp4_mddi_off(struct platform_device *pdev)
{
	int ret = 0;
	int cndx = 0;
	struct msm_fb_data_type *mfd;
	struct vsycn_ctrl *vctrl;
	struct mdp4_overlay_pipe *pipe;
	struct vsync_update *vp;
	int undx;
	int need_wait, cnt;
	unsigned long flags;
	int mixer = 0;

	pr_debug("%s+: pid=%d\n", __func__, current->pid);

	mfd = (struct msm_fb_data_type *)platform_get_drvdata(pdev);

	mutex_lock(&mfd->dma->ov_mutex);

	vctrl = &vsync_ctrl_db[cndx];
	pipe = vctrl->base_pipe;
	if (pipe == NULL) {
		pr_err("%s: NO base pipe\n", __func__);
		mutex_unlock(&mfd->dma->ov_mutex);
		return ret;
	}

	need_wait = 0;
	mutex_lock(&vctrl->update_lock);
	wake_up_interruptible_all(&vctrl->wait_queue);

	pr_debug("%s: clk=%d pan=%d\n", __func__,
			vctrl->clk_enabled, vctrl->pan_display);
	if (vctrl->clk_enabled)
		need_wait = 1;
	mutex_unlock(&vctrl->update_lock);

	cnt = 0;
	if (need_wait) {
		while (vctrl->clk_enabled) {
			msleep(20);
			cnt++;
			if (cnt > 10)
				break;
		}
	}

	if (cnt > 10) {
		spin_lock_irqsave(&vctrl->spin_lock, flags);
		vctrl->clk_control = 0;
		vctrl->clk_enabled = 0;
		vctrl->expire_tick = 0;
		spin_unlock_irqrestore(&vctrl->spin_lock, flags);
		mdp_clk_ctrl(0);
		pr_err("%s: Error, SET_CLK_OFF by force\n", __func__);
	}

	if (vctrl->vsync_enabled) {
		vsync_irq_disable(INTR_PRIMARY_RDPTR, MDP_PRIM_RDPTR_TERM);
		vctrl->vsync_enabled = 0;
	}

	undx =  vctrl->update_ndx;
	vp = &vctrl->vlist[undx];
	if (vp->update_cnt) {
		/*
		 * pipe's iommu will be freed at next overlay play
		 * and iommu_drop statistic will be increased by one
		 */
		pr_warn("%s: update_cnt=%d\n", __func__, vp->update_cnt);
		mdp4_mddi_pipe_clean(vp);
	}

	if (pipe) {
		/* sanity check, free pipes besides base layer */
		mixer = pipe->mixer_num;
		mdp4_overlay_unset_mixer(mixer);
		if (mfd->ref_cnt == 0) {
			/* adb stop */
			if (pipe->pipe_type == OVERLAY_TYPE_BF)
				mdp4_overlay_borderfill_stage_down(pipe);

			/* base pipe may change after borderfill_stage_down */
			pipe = vctrl->base_pipe;
			mdp4_mixer_stage_down(pipe, 1);
			mdp4_overlay_pipe_free(pipe, 1);
			vctrl->base_pipe = NULL;
		} else {
			/* system suspending */
			mdp4_mixer_stage_down(pipe, 1);
			mdp4_overlay_iommu_pipe_free(pipe->pipe_ndx, 1);
		}
	}

	atomic_set(&vctrl->suspend, 1);

	/*
	 * clean up ion freelist
	 * there need two stage to empty ion free list
	 * therefore need call unmap freelist twice
	 */
	mdp4_overlay_iommu_unmap_freelist(mixer);
	mdp4_overlay_iommu_unmap_freelist(mixer);

	mutex_unlock(&mfd->dma->ov_mutex);

	pr_debug("%s-:\n", __func__);
	return ret;
}

static int mdp4_mddi_clk_check(struct vsycn_ctrl *vctrl)
{
	int clk_set_on = 0;
	unsigned long flags;

	if (atomic_read(&vctrl->suspend)) {
		pr_err("%s: suspended, no more pan display\n", __func__);
		return -EPERM;
	}

	spin_lock_irqsave(&vctrl->spin_lock, flags);
	vctrl->clk_control = 0;
	vctrl->pan_display++;
	if (!vctrl->clk_enabled) {
		clk_set_on = 1;
		vctrl->clk_enabled = 1;
		vctrl->expire_tick = VSYNC_EXPIRE_TICK;
	}
	spin_unlock_irqrestore(&vctrl->spin_lock, flags);

	if (clk_set_on) {
		pr_debug("%s: SET_CLK_ON\n", __func__);
		mdp_clk_ctrl(1);
		vsync_irq_enable(INTR_PRIMARY_RDPTR, MDP_PRIM_RDPTR_TERM);
	}

	return 0;
}

void mdp4_mddi_overlay(struct msm_fb_data_type *mfd)
{
	int cndx = 0;
	struct vsycn_ctrl *vctrl;
	struct mdp4_overlay_pipe *pipe;

	mutex_lock(&mfd->dma->ov_mutex);
	vctrl = &vsync_ctrl_db[cndx];

	if (!mfd->panel_power_on) {
		mutex_unlock(&mfd->dma->ov_mutex);
		return;
	}

	pipe = vctrl->base_pipe;
	if (pipe == NULL) {
		pr_err("%s: NO base pipe\n", __func__);
		mutex_unlock(&mfd->dma->ov_mutex);
		return;
	}

	if (pipe->mixer_stage == MDP4_MIXER_STAGE_BASE) {
		if (mddi_use_dmap()) {
			mdp4_mddi_vsync_enable(mfd, pipe, 0);
		} else {
			mdp4_mddi_vsync_enable(mfd, pipe, 1);
		}
		mdp4_overlay_setup_pipe_addr(mfd, pipe);
		mdp4_mddi_pipe_queue(0, pipe);
	}

	mdp4_overlay_mdp_perf_upd(mfd, 1);
	mdp4_mddi_pipe_commit(cndx, 0);
	mdp4_overlay_mdp_perf_upd(mfd, 0);
	mutex_unlock(&mfd->dma->ov_mutex);

}
