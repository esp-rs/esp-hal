# Migration Guide from v1.0.0-beta.0 to v1.0.0-beta.1

## LCD_CAM Camera changes

The data and ctrl pins of the camera have been split out into individual `with_*` methods.

```diff
- camera.with_ctrl_pins(vsync_pin, href_pin);
+ camera.with_vsync(vsync_pin).with_h_enable(href_pin);
```

```diff
- camera.with_ctrl_pins_and_de(vsync_pin, hsync_pin, href_pin);
+ camera.with_vsync(vsync_pin).with_hsync(hsync_pin).with_h_enable(href_pin);
+ config.with_vh_de_mode_en(false); // Needed to enable HCYNC pin
```

```diff
- let data_pins = RxEightBits::new(
-     peripherals.GPIO11,
-     peripherals.GPIO9,
-     peripherals.GPIO8,
-     ....
- );
  let mut camera = Camera::new(
      lcd_cam.cam,
      peripherals.DMA_CH0,
-     data_pins,
-     config,
+     config.with_enable_2byte_mode(false) // If you were previously using RxEightBits (This is the default)
+     config.with_enable_2byte_mode(true) // If you were previously using RxSixteenBits
  )?;

+ camera.with_data0(peripherals.GPIO11).with_data1(peripherals.GPIO9).with_dataX();

```
