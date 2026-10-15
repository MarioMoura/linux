// SPDX-License-Identifier: GPL-2.0
/*
 * HiSilicon SD5116 clock gates
 *
 * The gates live in the system controller. Their bits were determined from
 * the vendor firmware.
 */

#include <linux/clk-provider.h>
#include <linux/mfd/syscon.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include <dt-bindings/clock/hisilicon,sd5116-clock.h>

#define SD5116_CLK_EN		0x2c

struct sd5116_gate {
	struct clk_hw hw;
	struct regmap *regmap;
	u32 mask;
};

static const struct {
	const char *name;
	u32 mask;
} sd5116_gates[] = {
	[SD5116_CLK_I2C] = { "i2c", BIT(26) },
};

#define to_sd5116_gate(_hw) container_of(_hw, struct sd5116_gate, hw)

static int sd5116_gate_enable(struct clk_hw *hw)
{
	struct sd5116_gate *gate = to_sd5116_gate(hw);

	return regmap_set_bits(gate->regmap, SD5116_CLK_EN, gate->mask);
}

static void sd5116_gate_disable(struct clk_hw *hw)
{
	struct sd5116_gate *gate = to_sd5116_gate(hw);

	regmap_clear_bits(gate->regmap, SD5116_CLK_EN, gate->mask);
}

static int sd5116_gate_is_enabled(struct clk_hw *hw)
{
	struct sd5116_gate *gate = to_sd5116_gate(hw);

	return regmap_test_bits(gate->regmap, SD5116_CLK_EN, gate->mask);
}

static const struct clk_ops sd5116_gate_ops = {
	.enable = sd5116_gate_enable,
	.disable = sd5116_gate_disable,
	.is_enabled = sd5116_gate_is_enabled,
};

static int sd5116_clk_probe(struct platform_device *pdev)
{
	static const struct clk_parent_data parent = { .index = 0 };
	struct device *dev = &pdev->dev;
	struct clk_hw_onecell_data *data;
	struct regmap *regmap;
	int i, ret;

	regmap = syscon_node_to_regmap(dev->parent->of_node);
	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap),
				     "failed to get system controller regmap\n");

	data = devm_kzalloc(dev, struct_size(data, hws, ARRAY_SIZE(sd5116_gates)),
			    GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	data->num = ARRAY_SIZE(sd5116_gates);

	for (i = 0; i < ARRAY_SIZE(sd5116_gates); i++) {
		struct clk_init_data init = {
			.name = sd5116_gates[i].name,
			.ops = &sd5116_gate_ops,
			.parent_data = &parent,
			.num_parents = 1,
		};
		struct sd5116_gate *gate;

		gate = devm_kzalloc(dev, sizeof(*gate), GFP_KERNEL);
		if (!gate)
			return -ENOMEM;

		gate->regmap = regmap;
		gate->mask = sd5116_gates[i].mask;
		gate->hw.init = &init;

		ret = devm_clk_hw_register(dev, &gate->hw);
		if (ret)
			return ret;
		data->hws[i] = &gate->hw;
	}

	return devm_of_clk_add_hw_provider(dev, of_clk_hw_onecell_get, data);
}

static const struct of_device_id sd5116_clk_of_match[] = {
	{ .compatible = "hisilicon,sd5116-clk" },
	{ }
};
MODULE_DEVICE_TABLE(of, sd5116_clk_of_match);

static struct platform_driver sd5116_clk_driver = {
	.probe = sd5116_clk_probe,
	.driver = {
		.name = "sd5116-clk",
		.of_match_table = sd5116_clk_of_match,
	},
};
module_platform_driver(sd5116_clk_driver);

MODULE_DESCRIPTION("HiSilicon SD5116 clock gates");
MODULE_LICENSE("GPL");
