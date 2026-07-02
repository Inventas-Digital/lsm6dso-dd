# lsm6dsox

An async, `no_std` Rust driver for the STMicroelectronics **LSM6DSO / LSM6DSOX**
6-axis IMU.

This driver uses a cutting-edge version of the
[`device-driver`](https://github.com/diondokter/device-driver) crate. This makes it possible
to describe the register map declaratively in [`lsm6dsox.ddsl`](lsm6dsox.ddsl) and
generate the typed register access functions at build time.

## Usage sketch

```rust
let mut imu = Lsm6dsox::new(i2c, 0x6A);
imu.sw_reset().await?;
let who = imu.read_who_am_i().await?;      // expect 0x6C on LSM6DSOX

imu.set_acc_odr(AccelOdr::Rate104Hz).await?;
imu.set_acc_full_scale(AccelFullScale::Scale4G).await?;

let raw = imu.read_acc().await?;            // [i16; 3]
let g = raw[0] as f32 * AccelFullScale::Scale4G.sensitivity_g();
```

## Implementation status

- [x] Raw accel/gyro/temperature read
- [x] I²C
- [x] FIFO batching
- [x] Sensor hub (I²C master)
- [ ] SPI
- [ ] Filter-chain configuration
- [ ] Finite state machine
- [ ] OIS (optical image stabilization) output
- [ ] Machine learning core

## License

Licensed under either of [Apache License, Version 2.0](LICENSE-APACHE) or
[MIT license](LICENSE-MIT) at your option.
