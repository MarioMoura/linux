// SPDX-License-Identifier: GPL-2.0
/*
 * HiSilicon SD5116 watchdog driver
 *
 * The watchdog lives in the system controller. The boot firmware starts it
 * and there is no known way to stop it, so it is reported as running and
 * the watchdog core keeps it alive until userspace takes over.
 *
 * Register usage was determined by observing the vendor firmware.
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/mfd/syscon.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/watchdog.h>

#define SD5116_WDT_CFG		0x54
#define SD5116_WDT_CFG_TIMEOUT	GENMASK(18, 10)	/* seconds - 1 */
#define SD5116_WDT_FEED		0x70
#define SD5116_WDT_FEED_KEY1	0x55aa5a5a
#define SD5116_WDT_FEED_KEY2	0xaa55a5a5

#define SD5116_WDT_MAX_TIMEOUT	512U
#define SD5116_WDT_DEF_TIMEOUT	60

struct sd5116_wdt {
	struct watchdog_device wdd;
	struct regmap *regmap;
};

static int sd5116_wdt_ping(struct watchdog_device *wdd)
{
	struct sd5116_wdt *wdt = watchdog_get_drvdata(wdd);
	int ret;

	ret = regmap_write(wdt->regmap, SD5116_WDT_FEED, SD5116_WDT_FEED_KEY1);
	if (ret)
		return ret;

	return regmap_write(wdt->regmap, SD5116_WDT_FEED, SD5116_WDT_FEED_KEY2);
}

static int sd5116_wdt_set_timeout(struct watchdog_device *wdd,
				  unsigned int timeout)
{
	struct sd5116_wdt *wdt = watchdog_get_drvdata(wdd);
	unsigned int hw_timeout = min(timeout, SD5116_WDT_MAX_TIMEOUT);
	int ret;

	ret = regmap_update_bits(wdt->regmap, SD5116_WDT_CFG,
				 SD5116_WDT_CFG_TIMEOUT,
				 FIELD_PREP(SD5116_WDT_CFG_TIMEOUT, hw_timeout - 1));
	if (ret)
		return ret;

	wdd->timeout = timeout;

	return sd5116_wdt_ping(wdd);
}

static int sd5116_wdt_start(struct watchdog_device *wdd)
{
	return sd5116_wdt_set_timeout(wdd, wdd->timeout);
}

static int sd5116_wdt_restart(struct watchdog_device *wdd,
			      unsigned long action, void *data)
{
	sd5116_wdt_set_timeout(wdd, wdd->min_timeout);
	mdelay(5000);

	return 0;
}

static const struct watchdog_info sd5116_wdt_info = {
	.identity = "SD5116 Watchdog",
	.options = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE,
};

static const struct watchdog_ops sd5116_wdt_ops = {
	.owner = THIS_MODULE,
	.start = sd5116_wdt_start,
	.ping = sd5116_wdt_ping,
	.set_timeout = sd5116_wdt_set_timeout,
	.restart = sd5116_wdt_restart,
};

static int sd5116_wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct watchdog_device *wdd;
	struct sd5116_wdt *wdt;
	int ret;

	wdt = devm_kzalloc(dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;

	wdt->regmap = syscon_node_to_regmap(dev->parent->of_node);
	if (IS_ERR(wdt->regmap))
		return dev_err_probe(dev, PTR_ERR(wdt->regmap),
				     "failed to get system controller regmap\n");

	wdd = &wdt->wdd;
	wdd->info = &sd5116_wdt_info;
	wdd->ops = &sd5116_wdt_ops;
	wdd->parent = dev;
	wdd->min_timeout = 1;
	wdd->max_hw_heartbeat_ms = SD5116_WDT_MAX_TIMEOUT * 1000;
	wdd->timeout = SD5116_WDT_DEF_TIMEOUT;
	watchdog_init_timeout(wdd, 0, dev);
	watchdog_set_drvdata(wdd, wdt);
	watchdog_set_restart_priority(wdd, 128);

	/* Started by the boot firmware and cannot be stopped */
	ret = sd5116_wdt_start(wdd);
	if (ret)
		return ret;
	set_bit(WDOG_HW_RUNNING, &wdd->status);

	return devm_watchdog_register_device(dev, wdd);
}

static const struct of_device_id sd5116_wdt_of_match[] = {
	{ .compatible = "hisilicon,sd5116-wdt" },
	{ }
};
MODULE_DEVICE_TABLE(of, sd5116_wdt_of_match);

static struct platform_driver sd5116_wdt_driver = {
	.probe = sd5116_wdt_probe,
	.driver = {
		.name = "sd5116-wdt",
		.of_match_table = sd5116_wdt_of_match,
	},
};
module_platform_driver(sd5116_wdt_driver);

MODULE_AUTHOR("Mario Moura <mm@mariomoura.com>");
MODULE_DESCRIPTION("HiSilicon SD5116 watchdog driver");
MODULE_LICENSE("GPL");
