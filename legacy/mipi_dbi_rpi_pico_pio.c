/*
 * Copyright (c) 2024, Zheng Hua
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT raspberrypi_pico_mipi_dbi_pio

#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/mipi_dbi.h>
#include <zephyr/drivers/dma.h>

#include <zephyr/drivers/misc/pio_rpi_pico/pio_rpi_pico.h>

#include <hardware/pio.h>
#include <hardware/clocks.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mipi_dbi_rpi_pico_pio, CONFIG_MIPI_DBI_LOG_LEVEL);

/* The MIPI DBI spec allows 8, 9, and 16 bits */
#define MIPI_DBI_MAX_DATA_BUS_WIDTH 16

#define BULK_DATA_BOUNDARY (4)

struct stream {
	const struct device *dma_dev;
	uint32_t channel; /* stores the channel for dma */
	struct dma_config dma_cfg;
	struct dma_block_config dma_blk_cfg;
};

struct pio_mipi_dbi_config {
	const struct device *piodev;
	const struct pinctrl_dev_config *pcfg;

	/* Parallel 8080 data GPIOs */
	const uint8_t data_pin_start;

	/* Read (type B) GPIO, Write(Type A) GPIO */
	const struct gpio_dt_spec rdwr;

	/* Write (type B) GPIO */
	// const struct gpio_dt_spec wr;
	const uint8_t enwr_pin;

	/* Chip-select GPIO */
	const struct gpio_dt_spec cs;

	/* Command/Data GPIO */
	const struct gpio_dt_spec rs;

	/* Reset GPIO */
	const struct gpio_dt_spec reset;
};

struct pio_mipi_dbi_data {
	struct k_mutex lock;

	/* Frequency of Write (type B) or Enable/strobe GPIO (type A) */
	uint32_t frequency;
	uint8_t data_bus_width;

	/* PIO specific data */
	PIO pio;
	size_t sm;
	float clock_div;
	pio_sm_config sm_cfg;
	bool pio_initialized;

	struct stream dma_tx;
	struct k_sem transfer_done;
};

RPI_PICO_PIO_DEFINE_PROGRAM(mipi_dbi_wr, 0, 1,
	    //     .wrap_target
    0x6010, //  0: out    pins, 16        side 0
    0xb042, //  1: nop                    side 1
            //     .wrap
);

static int pio_mipi_dbi_configure(struct pio_mipi_dbi_data * data, uint data_pin_start,
				uint data_bus_width, uint clk_pin, float clk_div)
{
	pio_sm_config *cfg = &data->sm_cfg;
	static uint32_t offset = 0;
	PIO pio = data->pio;
	uint sm = data->sm;

	if (!data->pio_initialized) {
		if (!pio_can_add_program(pio, RPI_PICO_PIO_GET_PROGRAM(mipi_dbi_wr))) {
			return -EBUSY;
		}
		offset = pio_add_program(pio, RPI_PICO_PIO_GET_PROGRAM(mipi_dbi_wr));

		pio_sm_set_consecutive_pindirs(pio, sm, clk_pin, 1, true);
		*cfg = pio_get_default_sm_config();
		sm_config_set_wrap(cfg,
				offset + RPI_PICO_PIO_GET_WRAP_TARGET(mipi_dbi_wr),
				offset + RPI_PICO_PIO_GET_WRAP(mipi_dbi_wr));
		sm_config_set_sideset(cfg, 1, false, false);
		sm_config_set_sideset_pins(cfg, clk_pin);
		sm_config_set_fifo_join(cfg, PIO_FIFO_JOIN_TX);
		sm_config_set_clkdiv(cfg, clk_div);

		data->pio_initialized = true;
	}

	pio_sm_set_consecutive_pindirs(pio, sm, data_pin_start, data_bus_width, true);
	sm_config_set_out_pins(cfg, data_pin_start, data_bus_width);
	sm_config_set_out_shift(cfg, false, true, data_bus_width);
	pio_sm_init(pio, sm, offset, cfg);
	pio_sm_set_enabled(pio, sm, true);

	return 0;
}

static inline void pio_mipi_dbi_put(PIO pio, uint sm, uint16_t x) {
    while (pio_sm_is_tx_fifo_full(pio, sm))
        ;
    *(volatile uint16_t*)&pio->txf[sm] = x;
}

static inline void pio_mipi_dbi_wait_idle(PIO pio, uint sm) {
    uint32_t sm_stall_mask = 1u << (sm + PIO_FDEBUG_TXSTALL_LSB);
    pio->fdebug = sm_stall_mask;
    while (!(pio->fdebug & sm_stall_mask))
        ;
}

static int pio_mipi_dbi_cpu_wr(const struct device *dev, const uint8_t *data_buf, size_t len)
{
	struct pio_mipi_dbi_data *data = dev->data;

	pio_mipi_dbi_wait_idle(data->pio, data->sm);

	while (len--)
		pio_mipi_dbi_put(data->pio, data->sm, *data_buf++);

	pio_mipi_dbi_wait_idle(data->pio, data->sm);

	return 0;
}

static void pio_mipi_dbi_dma_callback(const struct device *dev, void *arg,
				      uint32_t channel, int status)
{
	const struct device *pio_mipi_dbi_dev = (void *)arg;
	struct pio_mipi_dbi_data *data = pio_mipi_dbi_dev->data;

	k_sem_give(&data->transfer_done);
}

static int pio_mipi_dbi_dma_wr(const struct device *dev, const uint8_t *data_buf,
			       uint8_t data_size, size_t len)
{
	// const struct pio_mipi_dbi_config *config = dev->config;
	struct pio_mipi_dbi_data *data = dev->data;
	struct stream *stream = &data->dma_tx;
	struct dma_block_config *blk_cfg;
	PIO pio = data->pio;

	blk_cfg = &stream->dma_blk_cfg;

	/* tx direction has memory as source and periph as dest. */
	blk_cfg->source_address = (uint32_t)data_buf;
	blk_cfg->dest_address = (uint32_t)&pio->txf[data->sm];
	blk_cfg->block_size = len / data_size;

	stream->dma_cfg.head_block = &stream->dma_blk_cfg;
	stream->dma_cfg.user_data = (void *)dev;
	stream->dma_cfg.source_data_size = data_size;
	stream->dma_cfg.dest_data_size = data_size;

	dma_config(stream->dma_dev, stream->channel, &stream->dma_cfg);
	dma_start(stream->dma_dev, stream->channel);

	k_sem_take(&data->transfer_done, K_FOREVER);
	return 0;
}

static int pio_mipi_dbi_write_buf_rs(const struct device *dev, const uint8_t *data_buf,
				     uint8_t data_size, size_t len, uint8_t rs)
{
	const struct pio_mipi_dbi_config *config = dev->config;
	gpio_pin_set_dt(&config->rs, rs);

	if (len > BULK_DATA_BOUNDARY)
		return pio_mipi_dbi_dma_wr(dev, data_buf, data_size, len);

	return pio_mipi_dbi_cpu_wr(dev, data_buf, len);
}

static int pio_mipi_dbi_write_helper(const struct device *dev,
					 const struct mipi_dbi_config *dbi_config, bool cmd_present,
					 uint8_t cmd, const uint8_t *data_buf, size_t len)
{
	const struct pio_mipi_dbi_config *config = dev->config;
	struct pio_mipi_dbi_data *data = dev->data;
	int ret;

	ret = k_mutex_lock(&data->lock, K_FOREVER);
	if (ret < 0) {
		return ret;
	}

	if ((dbi_config->mode == MIPI_DBI_MODE_6800_BUS_8_BIT) ||
	    (dbi_config->mode == MIPI_DBI_MODE_8080_BUS_8_BIT)) {
		data->data_bus_width = 8;
	} else {
		data->data_bus_width = 16;
	}

	pio_mipi_dbi_configure(data, config->data_pin_start,
			data->data_bus_width, config->enwr_pin, data->clock_div);

	/*
	 * Before writing, 8080 bus need to assert the RD pin and
	 * 6800 bus need to de-assert the WR pin.
	 */
	if (dbi_config->mode == MIPI_DBI_MODE_6800_BUS_8_BIT ||
	    dbi_config->mode == MIPI_DBI_MODE_6800_BUS_16_BIT) {
		gpio_pin_set_dt(&config->rdwr, 0);
	} else {
		gpio_pin_set_dt(&config->rdwr, 1);
	}

	switch (dbi_config->mode) {
	case MIPI_DBI_MODE_8080_BUS_8_BIT:
	case MIPI_DBI_MODE_6800_BUS_8_BIT:
	case MIPI_DBI_MODE_8080_BUS_16_BIT:
	case MIPI_DBI_MODE_6800_BUS_16_BIT:
		gpio_pin_set_dt(&config->cs, 1);
		if (cmd_present)
			pio_mipi_dbi_write_buf_rs(dev, &cmd, sizeof(cmd), 1, 0);
		if (len > 0)
			pio_mipi_dbi_write_buf_rs(dev, (void *)data_buf,
				(data->data_bus_width / 8), len, 1);
		gpio_pin_set_dt(&config->cs, 0);
		break;

	default:
		LOG_ERR("MIPI DBI mode %u is not supported.", dbi_config->mode);
		ret = -ENOTSUP;
	}

	k_mutex_unlock(&data->lock);
	return ret;
}

static int pio_mipi_dbi_command_write(const struct device *dev,
					  const struct mipi_dbi_config *dbi_config, uint8_t cmd,
					  const uint8_t *data_buf, size_t len)
{
	return pio_mipi_dbi_write_helper(dev, dbi_config, true, cmd, data_buf, len);
}

static int pio_mipi_dbi_write_display(const struct device *dev,
					  const struct mipi_dbi_config *dbi_config,
					  const uint8_t *framebuf,
					  struct display_buffer_descriptor *desc,
					  enum display_pixel_format pixfmt)
{
	ARG_UNUSED(pixfmt);
	return pio_mipi_dbi_write_helper(dev, dbi_config, false, 0x0, framebuf, desc->buf_size);
}

static int pio_mipi_dbi_reset(const struct device *dev, k_timeout_t delay)
{
	const struct pio_mipi_dbi_config *config = dev->config;
	int ret;

	LOG_DBG("Performing hw reset.");

	ret = gpio_pin_set_dt(&config->reset, 1);
	if (ret < 0) {
		return ret;
	}
	k_sleep(delay);
	return gpio_pin_set_dt(&config->reset, 0);
}

static int pio_mipi_dbi_init(const struct device *dev)
{
	const struct pio_mipi_dbi_config *config = dev->config;
	struct pio_mipi_dbi_data *data = dev->data;
	const char *failed_pin = NULL;
	size_t sm;
	int ret;

	data->pio = pio_rpi_pico_get_pio(config->piodev);

	data->clock_div = ((float)clock_get_hz(clk_sys) / 2.f / data->frequency);
	if (data->clock_div < 1.f) {
		LOG_WRN("mipi-max-frequency is too fast, using 1.f as clock divider.\n");
		data->clock_div = 1.f;
	}

	ret = pio_rpi_pico_allocate_sm(config->piodev, &sm);
	if (ret < 0) {
		LOG_ERR("Failed to allocate PIO state machine!\n");
		return ret;
	}
	data->sm = sm;

	if (gpio_is_ready_dt(&config->rs)) {
		ret = gpio_pin_configure_dt(&config->rs, GPIO_OUTPUT_ACTIVE);
		if (ret < 0) {
			failed_pin = "rs";
			goto fail;
		}
		gpio_pin_set_dt(&config->rs, 0);
	}

	if (gpio_is_ready_dt(&config->rdwr)) {
		gpio_pin_configure_dt(&config->rdwr, GPIO_OUTPUT_ACTIVE);
	}

	if (gpio_is_ready_dt(&config->cs)) {
		ret = gpio_pin_configure_dt(&config->cs, GPIO_OUTPUT_ACTIVE);
		if (ret < 0) {
			failed_pin = "cs";
			goto fail;
		}
		gpio_pin_set_dt(&config->cs, 0);
	}

	if (gpio_is_ready_dt(&config->reset)) {
		ret = gpio_pin_configure_dt(&config->reset, GPIO_OUTPUT_ACTIVE);
		if (ret < 0) {
			failed_pin = "reset";
			goto fail;
		}
		gpio_pin_set_dt(&config->reset, 0);
	}

	k_sem_init(&data->transfer_done, 0, 1);

	LOG_DBG("Bus clock: %dMHz\n", (int)((float)clock_get_hz(clk_sys) \
				      / 2.f / data->clock_div / MHZ(1)));
	return pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
fail:
	LOG_ERR("Failed to configure %s GPIO pin.", failed_pin);
	return ret;
}

static DEVICE_API(mipi_dbi, pio_mipi_dbi_driver_api) = {
	.reset = pio_mipi_dbi_reset,
	.command_write = pio_mipi_dbi_command_write,
	.write_display = pio_mipi_dbi_write_display
};

#define MIPI_DBI_RPI_PICO_PIO_INIT(n)                                                              \
	PINCTRL_DT_INST_DEFINE(n);								   \
	static const struct pio_mipi_dbi_config pio_mipi_dbi_config_##n = {			   \
		.piodev = DEVICE_DT_GET(DT_INST_PARENT(n)),					   \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),					   \
		.data_pin_start = DT_INST_PROP_OR(n, data_pin_start, 0),                           \
		.enwr_pin = DT_INST_PROP(n, enwr_pin),                                             \
		.rdwr = GPIO_DT_SPEC_INST_GET_OR(n, rdwr_gpios, {}),                               \
		.cs = GPIO_DT_SPEC_INST_GET_OR(n, cs_gpios, {}),                                   \
		.rs = GPIO_DT_SPEC_INST_GET_OR(n, rs_gpios, {}),                             	   \
		.reset = GPIO_DT_SPEC_INST_GET_OR(n, reset_gpios, {}),                             \
	};                                                                                         \
	static struct pio_mipi_dbi_data pio_mipi_dbi_data_##n = {                                  \
		.frequency = DT_PROP(DT_CHOSEN(zephyr_display), mipi_max_frequency),               \
		.dma_tx = {					                                   \
			.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(n, tx)),		   \
			.channel = DT_INST_DMAS_CELL_BY_NAME(n, tx, channel),		           \
			.dma_blk_cfg = {							   \
				.source_addr_adj = DMA_ADDR_ADJ_INCREMENT,			   \
				.dest_addr_adj   = DMA_ADDR_ADJ_NO_CHANGE,			   \
			},									   \
			.dma_cfg = {						                   \
				.channel_direction = MEMORY_TO_PERIPHERAL,		           \
				.dma_callback = pio_mipi_dbi_dma_callback,		           \
				.block_count = 1,					           \
				.dma_slot = DT_INST_DMAS_CELL_BY_NAME(n, tx, slot)	           \
			}								           \
		},                                                                                 \
	};					                                                   \
	DEVICE_DT_INST_DEFINE(n, pio_mipi_dbi_init, NULL, &pio_mipi_dbi_data_##n,                  \
			      &pio_mipi_dbi_config_##n, POST_KERNEL,                               \
			      CONFIG_MIPI_DBI_INIT_PRIORITY, &pio_mipi_dbi_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MIPI_DBI_RPI_PICO_PIO_INIT)
