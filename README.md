# Pico DM QD3503728 Zephyr Porting

## How to

### Setup zephyr

```bash
```

### Copy files

```bash
TODO: add dts
cp ./Kconfig.rpi_pico ~/zephyrproject/zephyr/drivers/mipi_dbi/Kconfig.rpi_pico
cp ./mipi_dbi_rpi_pico_pio.c ~/zephyrproject/zephyr/drivers/mipi_dbi/mipi_dbi_rpi_pico_pio.c
cp ./raspberrypi,pico-mipi-dbi-pio.yaml ~/zephyrproject/zephyr/dts/bindings/mipi-dbi/raspberrypi,pico-mipi-dbi-pio.yaml
```

### Build lvgl demo

```bash
west build -b rpi_pico  --pristine=always  samples/modules/lvgl/demos -- -DOPENOCD=/usr/local/bin/openocd -DOPENOCD_DEFAULT_PATH=/usr/local/share/openocd/scripts -DRPI_PICO_DEBUG_ADAPTER=cmsis-dap
```

### Depoly firmware

```bash
west flash
```
