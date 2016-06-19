/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * Copyright (C) 2006-2007 - Motorola
 * Copyright (c) 2008-2010, The Linux Foundation. All rights reserved.
 *
 *  Date         Author           Comment
 * -----------  --------------   --------------------------------
 * 2006-Apr-28  Motorola         The kernel module for running the Bluetooth(R)
 *                               Sleep-Mode Protocol from the Host side
 * 2006-Sep-08  Motorola         Added workqueue for handling sleep work.
 * 2007-Jan-24  Motorola         Added mbm_handle_ioi() call to ISR.
 * 2009-Aug-10  Motorola         Changed "add_timer" to "mod_timer" to solve
 *                               race when flurry of queued work comes in.
*/

#include <linux/module.h>       /* kernel module definitions */
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/notifier.h>
#include <linux/proc_fs.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>

#include <linux/irq.h>
#include <linux/ioport.h>
#include <linux/param.h>
#include <linux/bitops.h>
#include <linux/termios.h>
#include <linux/pm.h>
#include <mach/gpio.h>
#include <linux/serial_core.h>
#include <mach/msm_serial_hs.h>

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h> /* event notifications */
#include "hci_uart.h"

#include <linux/barcode_emul.h>

#include <linux/jiffies.h>

#define BT_SLEEP_DBG
#ifndef BT_SLEEP_DBG
#define BT_DBG(fmt, arg...)
#endif
#undef  BT_DBG
#undef BT_ERR
#define BT_DBG(fmt, arg...) pr_err(fmt " [BT]\n", ##arg)
#define BT_ERR(fmt, arg...) pr_err(fmt " [BT]\n", ##arg)

/*
 * Defines
 */

#define VERSION	 "1.2"
#define PROC_DIR	"bluetooth/sleep"

#define POLARITY_LOW 0
#define POLARITY_HIGH 1

/* enable/disable wake-on-bluetooth */
#define BT_ENABLE_IRQ_WAKE 1

#define BT_BLUEDROID_SUPPORT 1

struct bluesleep_info {
	unsigned host_wake;
	unsigned ext_wake;
	unsigned host_wake_irq;
	struct uart_port *uport;
	struct wakeup_source bluesleep_wakeup_source;
	int irq_polarity;
	int has_ext_wake;
};

/* work function */
static void bluesleep_sleep_work(struct work_struct *work);
static void bluesleep_ext_wake_set_wq(struct work_struct *work);
static void bluesleep_sleep_wakeup_wq(struct work_struct *work);
static void bluesleep_start_wq(struct work_struct *work);
static void bluesleep_stop_wq(struct work_struct *work);
static void bluesleep_abnormal_stop_wq(struct work_struct *work);

/* work queue */
DECLARE_DELAYED_WORK(sleep_workqueue, bluesleep_sleep_work);
DECLARE_DELAYED_WORK(tx_timer_expired_workqueue, bluesleep_ext_wake_set_wq);
DECLARE_DELAYED_WORK(tx_data_wakeup_workqueue, bluesleep_sleep_wakeup_wq);
DECLARE_DELAYED_WORK(bluesleep_start_workqueue, bluesleep_start_wq);
DECLARE_DELAYED_WORK(bluesleep_stop_workqueue, bluesleep_stop_wq);
DECLARE_DELAYED_WORK(bluesleep_abnormal_stop_workqueue, bluesleep_abnormal_stop_wq);

/* Macros for handling sleep work */
#define bluesleep_rx_busy()     schedule_delayed_work(&sleep_workqueue, 0)
#define bluesleep_tx_busy()     schedule_delayed_work(&sleep_workqueue, 0)
#define bluesleep_rx_idle()     schedule_delayed_work(&sleep_workqueue, 0)
#define bluesleep_tx_idle()     schedule_delayed_work(&sleep_workqueue, 0)

#define bluesleep_tx_timer_expired()     schedule_delayed_work(&tx_timer_expired_workqueue, 0)
#define bluesleep_tx_data_wakeup()     schedule_delayed_work(&tx_data_wakeup_workqueue, 0)

#define bluesleep_start()     schedule_delayed_work(&bluesleep_start_workqueue, 0)
#define bluesleep_stop()     schedule_delayed_work(&bluesleep_stop_workqueue, 0)
#define bluesleep_abnormal_stop()     schedule_delayed_work(&bluesleep_abnormal_stop_workqueue, 0)

/* 10 second timeout */
#define TX_TIMER_INTERVAL  10

/* state variable names and bit positions */
#define BT_PROTO	 0x01
#define BT_TXDATA	 0x02
#define BT_ASLEEP	 0x04
#define BT_EXT_WAKE	0x08
#define BT_SUSPEND	0x10

#define PROC_BTWAKE	0
#define PROC_HOSTWAKE	1
#define PROC_PROTO	2
#define PROC_ASLEEP	3
#define PROC_LPM	4
#define PROC_BTWRITE	5

#if BT_BLUEDROID_SUPPORT
static bool has_lpm_enabled;
#else
/* global pointer to a single hci device. */
static struct hci_dev *bluesleep_hdev;
#endif

static struct platform_device *bluesleep_uart_dev;
static struct bluesleep_info *bsi;

/* module usage */
static atomic_t open_count = ATOMIC_INIT(1);

/*
 * Local function prototypes
 */
#if !BT_BLUEDROID_SUPPORT
static int bluesleep_hci_event(struct notifier_block *this,
			unsigned long event, void *data);
#endif

/*
 * Global variables
 */
/** Global state flags */
static unsigned long flags;

/** Tasklet to respond to change in hostwake line */
static struct tasklet_struct hostwake_task;

/** Transmission timer */
static void bluesleep_tx_timer_expire(unsigned long data);
static DEFINE_TIMER(tx_timer, bluesleep_tx_timer_expire, 0, 0);

/** Lock for state transitions */
static spinlock_t rw_lock;

#if !BT_BLUEDROID_SUPPORT
/** Notifier block for HCI events */
struct notifier_block hci_event_nblock = {
	.notifier_call = bluesleep_hci_event,
};
#endif

struct proc_dir_entry *bluetooth_dir, *sleep_dir;

/*
 * Local functions
 */
static void hsuart_power(int on)
{
	if (test_bit(BT_SUSPEND, &flags) && !on) {
		BT_DBG("hsuart_power OFF- it's suspend state. so return.");
		return;
	}

	if (!bsi->uport) {
		BT_DBG("hsuart_power called. But uport is null");
		return;
	}

	if (on) {
		BT_DBG("hsuart_power on");
		msm_hs_request_clock_on(bsi->uport);
		msm_hs_set_mctrl(bsi->uport, TIOCM_RTS);
	} else {
		BT_DBG("hsuart_power off");
		msm_hs_set_mctrl(bsi->uport, 0);
		msm_hs_request_clock_off(bsi->uport);
	}
}

/**
 * @return 1 if the Host can go to sleep, 0 otherwise.
 */
int bluesleep_can_sleep(void)
{
	/* check if WAKE_BT_GPIO and BT_WAKE_GPIO are both deasserted */
	return (!gpio_get_value(bsi->host_wake) &&
		(bsi->uport != NULL));
}

void bluesleep_sleep_wakeup(void)
{
	int ret;

	if (test_bit(BT_ASLEEP, &flags)) {
		BT_DBG("waking up...");
		/*Activating UART */
		hsuart_power(1);
		__pm_stay_awake(&bsi->bluesleep_wakeup_source);
		/* Start the timer */
		mod_timer(&tx_timer,
		          jiffies + msecs_to_jiffies(TX_TIMER_INTERVAL * 1000));
		if (bsi->has_ext_wake == 1) {
			ret = ice_gpiox_set(bsi->ext_wake, 1);
			if (ret)
				BT_ERR("(bluesleep_sleep_wakeup) failed to set ext_wake 1.");
		}
		set_bit(BT_EXT_WAKE, &flags);
		clear_bit(BT_ASLEEP, &flags);
		/*Activating UART */
	}
}

static void bluesleep_ext_wake_set_wq(struct work_struct *work)
{
	int ret;
	ret = ice_gpiox_set(bsi->ext_wake, 0);
	if (ret)
		BT_ERR("(bluesleep_ext_wake_set_wq) failed to set ext_wake 0.");
}

static void bluesleep_sleep_wakeup_wq(struct work_struct *work)
{
	int ret;

	if (test_bit(BT_ASLEEP, &flags)) {
		BT_DBG("(wq)waking up...");

		__pm_stay_awake(&bsi->bluesleep_wakeup_source);
		/* Start the timer */
		mod_timer(&tx_timer,
		          jiffies + msecs_to_jiffies(TX_TIMER_INTERVAL * 1000));
		if (bsi->has_ext_wake == 1) {
			ret = ice_gpiox_set(bsi->ext_wake, 1);
			if (ret)
				BT_ERR("(bluesleep_sleep_wakeup_wq) failed to set ext_wake 1.");
		}
		set_bit(BT_EXT_WAKE, &flags);
		clear_bit(BT_ASLEEP, &flags);
	}
}




/**
 * @brief@  main sleep work handling function which update the flags
 * and activate and deactivate UART ,check FIFO.
 */
static void bluesleep_sleep_work(struct work_struct *work)
{
	int ret;

	if (bluesleep_can_sleep()) {
		/* already asleep, this is an error case */
		if (test_bit(BT_ASLEEP, &flags)) {
			BT_DBG("already asleep");
			return;
		}

		if (msm_hs_tx_empty(bsi->uport)) {
			BT_DBG("going to sleep...");
			set_bit(BT_ASLEEP, &flags);
			/*Deactivating UART */
			hsuart_power(0);

			/*Deactivating UART */
			/* UART clk is not turned off immediately. Release
			 * wakelock after 500 ms.
			 */
			__pm_wakeup_event(
				&bsi->bluesleep_wakeup_source, msecs_to_jiffies(500));
		} else {
			mod_timer(&tx_timer,
			          jiffies + msecs_to_jiffies(TX_TIMER_INTERVAL * 1000));
			return;
		}
	} else if (!test_bit(BT_EXT_WAKE, &flags)
			&& !test_bit(BT_ASLEEP, &flags)) {
		mod_timer(&tx_timer,
		          jiffies + msecs_to_jiffies(TX_TIMER_INTERVAL * 1000));
		if (bsi->has_ext_wake == 1) {
			ret = ice_gpiox_set(bsi->ext_wake, 1);
			if (ret)
				BT_ERR("(bluesleep_sleep_work) failed to set ext_wake 1.");
		}
		set_bit(BT_EXT_WAKE, &flags);
	} else {
		bluesleep_sleep_wakeup();
	}
}

/**
 * A tasklet function that runs in tasklet context and reads the value
 * of the HOST_WAKE GPIO pin and further defer the work.
 * @param data Not used.
 */
static void bluesleep_hostwake_task(unsigned long data)
{
	BT_DBG("hostwake line change");

	spin_lock(&rw_lock);
	if ((gpio_get_value(bsi->host_wake) == bsi->irq_polarity))
		bluesleep_rx_busy();
	else
		bluesleep_rx_idle();
	spin_unlock(&rw_lock);

}

/**
 * Handles proper timer action when outgoing data is delivered to the
 * HCI line discipline. Sets BT_TXDATA.
 */
static void bluesleep_outgoing_data(void)
{
	/* log data passing by */
	set_bit(BT_TXDATA, &flags);

	BT_DBG("bluesleep_outgoing_data.");

	/* if the tx side is sleeping... */
	if (!test_bit(BT_EXT_WAKE, &flags)) {
		BT_DBG("tx was sleeping");

		/*
		** Uart Clk should be enabled promptly
		** before bluedroid write TX data.
		*/
		if (test_bit(BT_ASLEEP, &flags)) {
			hsuart_power(1);
		}

		bluesleep_tx_data_wakeup();
	}
}

#if BT_BLUEDROID_SUPPORT
struct uart_port *bluesleep_get_uart_port(void)
{
	struct uart_port *uport = NULL;

	uport = msm_hs_get_port_by_id(0);

	return uport;
}

#else
/**
 * Handles HCI device events.
 * @param this Not used.
 * @param event The event that occurred.
 * @param data The HCI device associated with the event.
 * @return <code>NOTIFY_DONE</code>.
 */
static int bluesleep_hci_event(struct notifier_block *this,
				unsigned long event, void *data)
{
	struct hci_dev *hdev = (struct hci_dev *) data;
	struct hci_uart *hu;
	struct uart_state *state;

	if (!hdev)
		return NOTIFY_DONE;

	switch (event) {
	case HCI_DEV_REG:
		if (!bluesleep_hdev) {
			bluesleep_hdev = hdev;
			hu  = (struct hci_uart *) hdev->driver_data;
			state = (struct uart_state *) hu->tty->driver_data;
			bsi->uport = state->uart_port;
			/* if bluetooth started, start bluesleep*/
			bluesleep_start();
		}
		break;
	case HCI_DEV_UNREG:
		bluesleep_stop();
		bluesleep_hdev = NULL;
		bsi->uport = NULL;
		/* if bluetooth stopped, stop bluesleep also */
		break;
	case HCI_DEV_WRITE:
		bluesleep_outgoing_data();
		break;
	}

	return NOTIFY_DONE;
}
#endif

/**
 * Handles transmission timer expiration.
 * @param data Not used.
 */
static void bluesleep_tx_timer_expire(unsigned long data)
{
	/* were we silent during the last timeout? */
	if (!test_bit(BT_TXDATA, &flags)) {
		BT_DBG("Tx has been idle");
		if (bsi->has_ext_wake == 1)
		{
			bluesleep_tx_timer_expired();
		}
		clear_bit(BT_EXT_WAKE, &flags);
		bluesleep_tx_idle();
	} else {
		BT_DBG("Tx data during last period");
		mod_timer(&tx_timer,
		          jiffies + msecs_to_jiffies(TX_TIMER_INTERVAL * 1000));
	}

	/* clear the incoming data flag */
	clear_bit(BT_TXDATA, &flags);
}

/**
 * Schedules a tasklet to run when receiving an interrupt on the
 * <code>HOST_WAKE</code> GPIO pin.
 * @param irq Not used.
 * @param dev_id Not used.
 */
static irqreturn_t bluesleep_hostwake_isr(int irq, void *dev_id)
{
	/* schedule a tasklet to handle the change in the host wake line */
	tasklet_schedule(&hostwake_task);
	return IRQ_HANDLED;
}

/**
 * Starts the Sleep-Mode Protocol on the Host.
 */
static void bluesleep_start_wq(struct work_struct *work)
{
	int retval;
	int ret;

	if (test_bit(BT_PROTO, &flags)) {
		return;
	}

	if (!atomic_dec_and_test(&open_count)) {
		atomic_inc(&open_count);
		BT_ERR("(bluesleep_start_wq) atomic inc. Bluesleep Not Started");
		return;
	}

	/* start the timer */
	mod_timer(&tx_timer, jiffies + msecs_to_jiffies(TX_TIMER_INTERVAL * 1000));

	/* assert BT_WAKE */
	if (bsi->has_ext_wake == 1) {
		ret = ice_gpiox_set(bsi->ext_wake, 1);
		if (ret)
			BT_ERR("(bluesleep_start_wq) failed to set ext_wake 1.");
	}
	set_bit(BT_EXT_WAKE, &flags);
#if BT_ENABLE_IRQ_WAKE
	retval = enable_irq_wake(bsi->host_wake_irq);
	if (retval < 0) {
		BT_ERR("Couldn't enable BT_HOST_WAKE as wakeup interrupt");
		goto fail;
	}
#endif
	set_bit(BT_PROTO, &flags);
	__pm_stay_awake(&bsi->bluesleep_wakeup_source);
	return;
fail:
	del_timer(&tx_timer);
	atomic_inc(&open_count);

	return;
}

/**
 * Stops the Sleep-Mode Protocol on the Host.
 */
static void bluesleep_stop_wq(struct work_struct *work)
{
	int ret;

	if (!test_bit(BT_PROTO, &flags)) {
		BT_ERR("(bluesleep_stop_wq) proto is not set. Failed to stop bluesleep");
		bsi->uport = NULL;
		return;
	}
	/* assert BT_WAKE */
	if (bsi->has_ext_wake == 1) {
		ret = ice_gpiox_set(bsi->ext_wake, 1);
		if (ret)
			BT_ERR("(bluesleep_stop_wq) failed to set ext_wake 1.");
	}
	set_bit(BT_EXT_WAKE, &flags);
	del_timer(&tx_timer);
	clear_bit(BT_PROTO, &flags);

	if (test_bit(BT_ASLEEP, &flags)) {
		clear_bit(BT_ASLEEP, &flags);
		hsuart_power(1);
	}

	atomic_inc(&open_count);

#if BT_ENABLE_IRQ_WAKE
	if (disable_irq_wake(bsi->host_wake_irq))
		BT_ERR("Couldn't disable hostwake IRQ wakeup mode\n");
#endif
	__pm_wakeup_event(
		&bsi->bluesleep_wakeup_source, msecs_to_jiffies(500));

	bsi->uport = NULL;
}

/**
 * Stops the Sleep-Mode Protocol on the Host when abnormal.
 */
static void bluesleep_abnormal_stop_wq(struct work_struct *work)
{
	int ret;

	BT_ERR("bluesleep_abnormal_stop_wq");

	/* assert BT_WAKE */
	if (bsi->has_ext_wake == 1) {
		ret = ice_gpiox_set(bsi->ext_wake, 1);
		if (ret)
			BT_ERR("(bluesleep_abnormal_stop_wq) failed to set ext_wake 1.");
	}
	set_bit(BT_EXT_WAKE, &flags);
	del_timer(&tx_timer);
	clear_bit(BT_PROTO, &flags);

	if (test_bit(BT_ASLEEP, &flags)) {
		clear_bit(BT_ASLEEP, &flags);
		hsuart_power(1);
	}

	atomic_inc(&open_count);

#if BT_ENABLE_IRQ_WAKE
	if (disable_irq_wake(bsi->host_wake_irq))
		BT_ERR("Couldn't disable hostwake IRQ wakeup mode\n");
#endif
	__pm_wakeup_event(
		&bsi->bluesleep_wakeup_source, msecs_to_jiffies(500));

	bsi->uport = NULL;
}

void bluesleep_setup_uart_port(struct platform_device *uart_dev)
{
	bluesleep_uart_dev = uart_dev;
}

static int bluesleep_probe(struct platform_device *pdev)
{
	int ret;
	struct resource *res;

	bsi = kzalloc(sizeof(struct bluesleep_info), GFP_KERNEL);
	if (!bsi)
		return -ENOMEM;

	res = platform_get_resource_byname(pdev, IORESOURCE_IO,
				"gpio_host_wake");
	if (!res) {
		BT_ERR("couldn't find host_wake gpio\n");
		ret = -ENODEV;
		goto free_bsi;
	}
	bsi->host_wake = res->start;

	ret = gpio_request(bsi->host_wake, "bt_host_wake");
	if (ret)
		goto free_bsi;

	/* configure host_wake as input */
	gpio_tlmm_config(GPIO_CFG(bsi->host_wake, 0, GPIO_CFG_INPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
	ret = gpio_direction_input(bsi->host_wake);
	if (ret < 0) {
		BT_ERR("gpio-keys: failed to configure input direction for GPIO %d, error %d",
				bsi->host_wake, ret);
		gpio_free(bsi->host_wake);
		goto free_bsi;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_IO,
				"gpio_ext_wake");

	if (!res)
		bsi->has_ext_wake = 0;
	else
		bsi->has_ext_wake = 1;

	if (bsi->has_ext_wake) {
		bsi->ext_wake = res->start;
		/* uses FPGA
		ret = gpio_request(bsi->ext_wake, "bt_ext_wake");
		if (ret)
			goto free_bt_host_wake;
		*/

		/* configure ext_wake as output mode*/
		/* uses FPGA
		ret = gpio_direction_output(bsi->ext_wake, 1);
		if (ret < 0) {
			BT_ERR("gpio-keys: failed to configure output direction for GPIO %d, error %d",
				  bsi->ext_wake, ret);
			gpio_free(bsi->ext_wake);
			goto free_bt_host_wake;
		}
		*/
	} else
		set_bit(BT_EXT_WAKE, &flags);

	res = platform_get_resource_byname(pdev, IORESOURCE_IRQ,
						"host_wake");
	if (!res) {
		BT_ERR("couldn't find host_wake irq\n");
		ret = -ENODEV;
		goto free_bt_host_wake;
	}
	bsi->host_wake_irq = res->start;
	if (bsi->host_wake_irq < 0) {
		BT_ERR("couldn't find host_wake irq\n");
		ret = -ENODEV;
		goto free_bt_ext_wake;
	}
	if (res->flags & IORESOURCE_IRQ_LOWEDGE)
		bsi->irq_polarity = POLARITY_LOW;/*low edge (falling edge)*/
	else
		bsi->irq_polarity = POLARITY_HIGH;/*anything else*/

	wakeup_source_init(&bsi->bluesleep_wakeup_source, "bluesleep");
	clear_bit(BT_SUSPEND, &flags);

	if (bsi->irq_polarity == POLARITY_LOW) {
		ret = request_irq(bsi->host_wake_irq, bluesleep_hostwake_isr,
				IRQF_DISABLED | IRQF_TRIGGER_FALLING,
				"bluetooth hostwake", NULL);
	} else {
		ret = request_irq(bsi->host_wake_irq, bluesleep_hostwake_isr,
				IRQF_DISABLED | IRQF_TRIGGER_RISING,
				"bluetooth hostwake", NULL);
	}
	if (ret  < 0) {
		BT_ERR("Couldn't acquire BT_HOST_WAKE IRQ");
		goto free_bt_ext_wake;
	}

	return 0;

free_bt_ext_wake:
	//gpio_free(bsi->ext_wake);
free_bt_host_wake:
	gpio_free(bsi->host_wake);
free_bsi:
	kfree(bsi);
	return ret;
}

static int bluesleep_remove(struct platform_device *pdev)
{
	free_irq(bsi->host_wake_irq, NULL);
	gpio_free(bsi->host_wake);
	//gpio_free(bsi->ext_wake);
	wakeup_source_trash(&bsi->bluesleep_wakeup_source);
	kfree(bsi);
	return 0;
}


static int bluesleep_resume(struct platform_device *pdev)
{
	if (test_bit(BT_SUSPEND, &flags)) {
		if (!has_lpm_enabled) {
			gpio_tlmm_config(GPIO_CFG(bsi->host_wake, 0, GPIO_CFG_INPUT,
						GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
		}

		if ((bsi->uport != NULL) &&
			(gpio_get_value(bsi->host_wake) == bsi->irq_polarity)) {
				BT_DBG("bluesleep resume form BT event...");
				hsuart_power(1);
		}
		clear_bit(BT_SUSPEND, &flags);
	}
	return 0;
}

static int bluesleep_suspend(struct platform_device *pdev, pm_message_t state)
{
	if (!has_lpm_enabled) {
		gpio_tlmm_config(GPIO_CFG(bsi->host_wake, 0, GPIO_CFG_INPUT,
						GPIO_CFG_PULL_DOWN, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
	}

	set_bit(BT_SUSPEND, &flags);
	return 0;
}

static struct platform_driver bluesleep_driver = {
	.probe = bluesleep_probe,
	.remove = bluesleep_remove,
	.suspend = bluesleep_suspend,
	.resume = bluesleep_resume,
	.driver = {
		.name = "bluesleep",
		.owner = THIS_MODULE,
	},
};

static int bluesleep_proc_show(struct seq_file *m, void *v)
{
	switch ((long)m->private) {
	case PROC_BTWAKE:
		seq_printf(m, "btwake:%u\n",
				test_bit(BT_EXT_WAKE, &flags) ? 1 : 0);
		break;
	case PROC_HOSTWAKE:
		seq_printf(m, "hostwake: %u\n", gpio_get_value(bsi->host_wake));
		break;
	case PROC_PROTO:
		seq_printf(m, "proto: %u\n",
				test_bit(BT_PROTO, &flags) ? 1 : 0);
		break;
	case PROC_ASLEEP:
		seq_printf(m, "asleep: %u\n",
				test_bit(BT_ASLEEP, &flags) ? 1 : 0);
		break;
#if BT_BLUEDROID_SUPPORT
	case PROC_LPM:
		seq_printf(m, "lpm: %u\n",
				has_lpm_enabled ? 1 : 0);
		break;
	case PROC_BTWRITE:
		seq_printf(m, "lpm: %s\n",
				"unsupported to read\n");
		break;
#endif
	default:
		return 0;
	}
	return 0;
}

static ssize_t bluesleep_proc_write(struct file *file, const char *buf,
	size_t count, loff_t *pos)
{
	void *data = PDE(file->f_path.dentry->d_inode)->data;
//	void *data = PDE_DATA(file_inode(file));
	char lbuf[32];
	int ret;

	if (count >= sizeof(lbuf))
		count = sizeof(lbuf)-1;

	if (copy_from_user(lbuf, buf, count))
		return -EFAULT;
	lbuf[count] = 0;

	switch ((long)data) {
	case PROC_BTWAKE:
		if (lbuf[0] == '0') {
			if (bsi->has_ext_wake == 1) {
				ret = ice_gpiox_set(bsi->ext_wake, 0);
				if (ret)
					BT_ERR("(bluepower_write_proc_btwake) failed to set ext_wake 0.");
			}
			clear_bit(BT_EXT_WAKE, &flags);
		} else if (lbuf[0] == '1') {
			if (bsi->has_ext_wake == 1) {
				ret = ice_gpiox_set(bsi->ext_wake, 1);
				if (ret)
					BT_ERR("(bluepower_write_proc_btwake) failed to set ext_wake 1.");
			}
			set_bit(BT_EXT_WAKE, &flags);
		}
		break;
	case PROC_PROTO:
		if (lbuf[0] == '0')
			bluesleep_stop();
		else
			bluesleep_start();
		break;
#if BT_BLUEDROID_SUPPORT
	case PROC_LPM:
		if (lbuf[0] == '0') {
			BT_ERR("(bluesleep_write_proc_lpm) Unreg HCI notifier.");
			/* HCI_DEV_UNREG */
			bluesleep_stop();
			has_lpm_enabled = false;
			//bsi->uport = NULL;
		} else if (lbuf[0] == '1') {
			BT_ERR("(bluesleep_write_proc_lpm) Reg HCI notifier.");
			/* HCI_DEV_REG */
			if (!has_lpm_enabled) {
				has_lpm_enabled = true;
				bsi->uport = bluesleep_get_uart_port();
				/* if bluetooth started, start bluesleep*/
				bluesleep_start();
			}
		} else if (lbuf[0] == '2') {
			BT_ERR("(bluesleep_write_proc_lpm) don`t control ext_wake & uart clk");
			if(has_lpm_enabled) {
				has_lpm_enabled = false;
				bluesleep_abnormal_stop();
			}
		}
		break;
	case PROC_BTWRITE:
		/* HCI_DEV_WRITE */
		if (lbuf[0] != '0') {
			bluesleep_outgoing_data();
		}
		break;
#endif
	default:
		return 0;
	}

	return count;
}

static int bluesleep_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, bluesleep_proc_show, PDE(inode)->data);
}

static const struct file_operations bluesleep_proc_readwrite_fops = {
	.owner	= THIS_MODULE,
	.open	= bluesleep_proc_open,
	.read   = seq_read,
	.write  = bluesleep_proc_write,
};
static const struct file_operations bluesleep_proc_read_fops = {
	.owner	= THIS_MODULE,
	.open	= bluesleep_proc_open,
	.read   = seq_read,
};

/**
 * Initializes the module.
 * @return On success, 0. On error, -1, and <code>errno</code> is set
 * appropriately.
 */
static int __init bluesleep_init(void)
{
	int retval;
	struct proc_dir_entry *ent;

	BT_INFO("BlueSleep Mode Driver Ver %s", VERSION);

#if BT_BLUEDROID_SUPPORT
	has_lpm_enabled = false;
#endif

	retval = platform_driver_register(&bluesleep_driver);
	if (retval)
		return retval;

	if (bsi == NULL)
		return 0;

#if !BT_BLUEDROID_SUPPORT
	bluesleep_hdev = NULL;
#endif

	bluetooth_dir = proc_mkdir("bluetooth", NULL);
	if (bluetooth_dir == NULL) {
		BT_ERR("Unable to create /proc/bluetooth directory");
		return -ENOMEM;
	}

	sleep_dir = proc_mkdir("sleep", bluetooth_dir);
	if (sleep_dir == NULL) {
		BT_ERR("Unable to create /proc/%s directory", PROC_DIR);
		return -ENOMEM;
	}

	/* Creating read/write "btwake" entry */
	ent = proc_create_data("btwake", 0, sleep_dir,
			&bluesleep_proc_readwrite_fops, (void *)PROC_BTWAKE);
	if (ent == NULL) {
		BT_ERR("Unable to create /proc/%s/btwake entry", PROC_DIR);
		retval = -ENOMEM;
		goto fail;
	}

	/* read only proc entries */
	ent = proc_create_data("hostwake", 0, sleep_dir,
				&bluesleep_proc_read_fops,
				(void *)PROC_HOSTWAKE);
	if (ent == NULL) {
		BT_ERR("Unable to create /proc/%s/hostwake entry", PROC_DIR);
		retval = -ENOMEM;
		goto fail;
	}

	/* read/write proc entries */
	ent = proc_create_data("proto", 0, sleep_dir,
			&bluesleep_proc_readwrite_fops,
			(void *)PROC_PROTO);
	if (ent == NULL) {
		BT_ERR("Unable to create /proc/%s/proto entry", PROC_DIR);
		retval = -ENOMEM;
		goto fail;
	}

	/* read only proc entries */
	ent = proc_create_data("asleep", 0,
			sleep_dir, &bluesleep_proc_read_fops,
			(void *)PROC_ASLEEP);
	if (ent == NULL) {
		BT_ERR("Unable to create /proc/%s/asleep entry", PROC_DIR);
		retval = -ENOMEM;
		goto fail;
	}

#if BT_BLUEDROID_SUPPORT
	/* read/write proc entries */
	ent = proc_create_data("lpm", 0, sleep_dir,
			&bluesleep_proc_readwrite_fops,
			(void *)PROC_LPM);
	if (ent == NULL) {
		BT_ERR("Unable to create /proc/%s/lpm entry", PROC_DIR);
		retval = -ENOMEM;
		goto fail;
	}

	/* read/write proc entries */
	ent = proc_create_data("btwrite", 0, sleep_dir,
			&bluesleep_proc_readwrite_fops,
			(void *)PROC_BTWRITE);
	if (ent == NULL) {
		BT_ERR("Unable to create /proc/%s/btwrite entry", PROC_DIR);
		retval = -ENOMEM;
		goto fail;
	}
#endif

	flags = 0; /* clear all status bits */

	/* Initialize spinlock. */
	spin_lock_init(&rw_lock);

	/* Initialize timer */
	init_timer(&tx_timer);
	tx_timer.function = bluesleep_tx_timer_expire;
	tx_timer.data = 0;

	/* initialize host wake tasklet */
	tasklet_init(&hostwake_task, bluesleep_hostwake_task, 0);

	/* assert bt wake */
	/* block code for FPGA to be set-up
	if (bsi->has_ext_wake == 1) {
		ret = ice_gpiox_set(bsi->ext_wake, 1);
		if (ret)
			BT_ERR("(bluesleep_init) failed to set ext_wake 1.");
	}
	*/
	set_bit(BT_EXT_WAKE, &flags);
#if !BT_BLUEDROID_SUPPORT
	hci_register_notifier(&hci_event_nblock);
#endif

	return 0;

fail:
#if BT_BLUEDROID_SUPPORT
	remove_proc_entry("btwrite", sleep_dir);
	remove_proc_entry("lpm", sleep_dir);
#endif
	remove_proc_entry("asleep", sleep_dir);
	remove_proc_entry("proto", sleep_dir);
	remove_proc_entry("hostwake", sleep_dir);
	remove_proc_entry("btwake", sleep_dir);
	remove_proc_entry("sleep", bluetooth_dir);
	remove_proc_entry("bluetooth", 0);
	return retval;
}

/**
 * Cleans up the module.
 */
static void __exit bluesleep_exit(void)
{
	int ret;

	if (bsi == NULL)
		return;

	/* assert bt wake */
	if (bsi->has_ext_wake == 1) {
		ret = ice_gpiox_set(bsi->ext_wake, 1);
		if (ret)
			BT_ERR("(bluesleep_exit) failed to set ext_wake 1.");
	}
	set_bit(BT_EXT_WAKE, &flags);
	if (test_bit(BT_PROTO, &flags)) {
		if (disable_irq_wake(bsi->host_wake_irq))
			BT_ERR("Couldn't disable hostwake IRQ wakeup mode\n");
		free_irq(bsi->host_wake_irq, NULL);
		del_timer(&tx_timer);
		if (test_bit(BT_ASLEEP, &flags))
			hsuart_power(1);
	}

#if !BT_BLUEDROID_SUPPORT
	hci_unregister_notifier(&hci_event_nblock);
#endif
	platform_driver_unregister(&bluesleep_driver);

#if BT_BLUEDROID_SUPPORT
	remove_proc_entry("btwrite", sleep_dir);
	remove_proc_entry("lpm", sleep_dir);
#endif
	remove_proc_entry("asleep", sleep_dir);
	remove_proc_entry("proto", sleep_dir);
	remove_proc_entry("hostwake", sleep_dir);
	remove_proc_entry("btwake", sleep_dir);
	remove_proc_entry("sleep", bluetooth_dir);
	remove_proc_entry("bluetooth", 0);
}

module_init(bluesleep_init);
module_exit(bluesleep_exit);

MODULE_DESCRIPTION("Bluetooth Sleep Mode Driver ver %s " VERSION);
#ifdef MODULE_LICENSE
MODULE_LICENSE("GPL");
#endif
