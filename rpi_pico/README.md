#  树莓派 Pico 的移植

我们需要回退到 lvgl 8.4.0时的版本，因为后面zephyr将lvgl更新到了v9.2.0，以RP2040的性能，在不超频的情况下已经带不动了。

## 参考步骤

### 回退版本，打补丁
```bash
cp 0001-add-support-for-pico_dm_qd3503728-on-rpi_pico.patch ~/zephyrproject/zephyr

cd ~/zephyrproject/zephyr
git checkout 802eac71f0020646a2db6f4f12b5ceb358294064
west update

git apply 0001-add-support-for-pico_dm_qd3503728-on-rpi_pico.patch
```

### 编译

```bash
west build -b rpi_pico --shield pico_dm_qd3503728 --pristine=always  samples/modules/lvgl/demos -- -DOPENOCD=/usr/local/bin/openocd -DOPENOCD_DEFAULT_PATH=/usr/local/share/openocd/scripts -DRPI_PICO_DEBUG_ADAPTER=cmsis-dap
```

### 烧录
```bash
# 通过 CMSIS-DAP 烧录
west flash --runner openocd

# 通过 UF2 文件烧录
west flash --runner uf2
```

## LVGL Benchmark 的额外改动

在运行 LVGL Benchmark 的时候，我们发现，当运行到齿轮图片显示的测试项时，程序会卡住，经过debug发现，程序尝试向其所在的地址区域进行了写入，而 pico 的链接脚本将这幅图片的link address指定在flash地址中，而RP2040上的XIP flash地址不可写，触发了hardfault，导致程序卡住。

参考如下改动

```diff
diff --git a/demos/benchmark/assets/img_benchmark_cogwheel_rgb.c b/demos/benchmark/assets/img_benchmark_cogwheel_rgb.c       
index 39572d224..babfacccd 100644
--- a/demos/benchmark/assets/img_benchmark_cogwheel_rgb.c
+++ b/demos/benchmark/assets/img_benchmark_cogwheel_rgb.c
@@ -7,7 +7,7 @@
 #endif

 #ifndef LV_ATTRIBUTE_IMG_IMG_COGWHEEL_RGB
-#define LV_ATTRIBUTE_IMG_IMG_COGWHEEL_RGB
+#define LV_ATTRIBUTE_IMG_IMG_COGWHEEL_RGB __attribute__((__section__(".data")))
 #endif

 const LV_ATTRIBUTE_MEM_ALIGN LV_ATTRIBUTE_IMG_IMG_COGWHEEL_RGB uint8_t img_benchmark_cogwheel_rgb_map[] = {
```

若要运行此demo，请修改 `samples/module/lvgl/demos/prj.conf`

```bash
# Benchmark Demo
# CONFIG_LV_USE_FONT_COMPRESSED=y
CONFIG_LV_Z_DEMO_BENCHMARK=y
# CONFIG_LV_Z_DEMO_WIDGETS=y
# CONFIG_LV_Z_DEMO_STRESS=y

```

### 性能得分

在默认125MHz的频率下，平均fps为 37
