#![cfg_attr(not(test), no_std)]

#[cfg(feature = "defmt")]
macro_rules! debug {
    ($($arg:tt)*) => { defmt::debug!($($arg)*) };
}
#[cfg(all(not(feature = "defmt"), test))]
macro_rules! debug {
    ($($arg:tt)*) => { println!("[DEBUG] {}", format_args!($($arg)*)) };
}
#[cfg(all(not(feature = "defmt"), not(test)))]
macro_rules! debug {
    ($($arg:tt)*) => {{ let _ = format_args!($($arg)*); }};
}

use device_driver;
use device_driver::Block;
use device_driver::FieldsetMetadata;
use device_driver::RegisterInterfaceBase;
use embedded_hal_async;

#[cfg(feature = "defmt")]
device_driver::compile!(
    options: "--rust-defmt-feature=defmt",
    manifest: "lsm6dsox.ddsl"
);

#[cfg(not(feature = "defmt"))]
device_driver::compile!(
    options: "",
    manifest: "lsm6dsox.ddsl"
);

impl FifoEntryValue {
    pub fn as_timestamp(&self) -> u32 {
        u32::from_le_bytes([self.x_low(), self.x_high(), self.y_low(), self.y_high()])
    }

    pub fn x(&self) -> i16 {
        i16::from_le_bytes([self.x_low(), self.x_high()])
    }

    pub fn y(&self) -> i16 {
        i16::from_le_bytes([self.y_low(), self.y_high()])
    }

    pub fn z(&self) -> i16 {
        i16::from_le_bytes([self.z_low(), self.z_high()])
    }
}

pub const TEMPERATURE_SENSITIVITY_C_PER_LSB: f32 = 1.0 / 256.0;
pub const TEMPERATURE_OFFSET_C: f32 = 25.0;

impl AccelFullScale {
    pub const fn sensitivity_g(self) -> f32 {
        match self {
            AccelFullScale::Scale2G => 0.000_061,
            AccelFullScale::Scale4G => 0.000_122,
            AccelFullScale::Scale8G => 0.000_244,
            AccelFullScale::Scale16G => 0.000_488,
        }
    }
}

impl GyroFullScale {
    pub const fn sensitivity_dps(self, fs_125: bool) -> f32 {
        if fs_125 {
            0.004_375 // ±125 dps
        } else {
            match self {
                GyroFullScale::Scale250Dps => 0.008_75,
                GyroFullScale::Scale500Dps => 0.017_5,
                GyroFullScale::Scale1000Dps => 0.035,
                GyroFullScale::Scale2000Dps => 0.070,
            }
        }
    }
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(thiserror::Error, Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Lsm6dsoxError<I2cError> {
    #[error("i2c error: {0:?}")]
    I2c(I2cError),
    #[error("invalid range error")]
    InvalidRange,
    #[error("register write exceeds the maximum contiguous write length")]
    WriteTooLong,
}

#[derive(Debug)]
pub struct DeviceInterface<I2c: embedded_hal_async::i2c::I2c> {
    pub i2c: I2c,
    pub addr: u8,
}

pub struct SensorHubSlave {
    pub address: u8,
    pub register: u8,
    pub num_reads: u8,
    pub batch: bool,
}

pub struct Lsm6dsox<I2c: embedded_hal_async::i2c::I2c> {
    pub device: Device<DeviceInterface<I2c>>,
}

impl<I2c: embedded_hal_async::i2c::I2c> Lsm6dsox<I2c> {
    pub fn new(i2c: I2c, addr: u8) -> Self {
        Self {
            device: Device::new(DeviceInterface { i2c, addr }),
        }
    }

    pub async fn configure_sensor_hub(
        &mut self,
        slaves: &[SensorHubSlave],
        shub_odr: ShubOdr,
    ) -> Result<(), crate::Lsm6dsoxError<I2c::Error>> {
        self.device
            .multi_write()
            .with(|d| d.slv_0_add().plan())
            .with(|d| d.slv_0_subadd().plan())
            .with(|d| d.slv_0_config().plan())
            .with(|d| d.slv_1_add().plan())
            .with(|d| d.slv_1_subadd().plan())
            .with(|d| d.slv_1_config().plan())
            .with(|d| d.slv_2_add().plan())
            .with(|d| d.slv_2_subadd().plan())
            .with(|d| d.slv_2_config().plan())
            .with(|d| d.slv_3_add().plan())
            .with(|d| d.slv_3_subadd().plan())
            .with(|d| d.slv_3_config().plan())
            .execute_async(|(a0, s0, c0, a1, s1, c1, a2, s2, c2, a3, s3, c3)| {
                // shub_odr lives only in slave 0's config, so always set it.
                c0.set_shub_odr(shub_odr);

                if let Some(slave) = slaves.get(0) {
                    a0.set_slave_0_add(slave.address);
                    a0.set_rw_0(true);
                    s0.set_slave_reg(slave.register);
                    c0.set_slave_numop(slave.num_reads);
                    c0.set_batch_ext_sens_en(slave.batch);
                }
                if let Some(slave) = slaves.get(1) {
                    a1.set_slave_add(slave.address);
                    a1.set_r_1(true);
                    s1.set_slave_reg(slave.register);
                    c1.set_slave_numop(slave.num_reads);
                    c1.set_batch_ext_sens_en(slave.batch);
                }
                if let Some(slave) = slaves.get(2) {
                    a2.set_slave_add(slave.address);
                    a2.set_r_1(true);
                    s2.set_slave_reg(slave.register);
                    c2.set_slave_numop(slave.num_reads);
                    c2.set_batch_ext_sens_en(slave.batch);
                }
                if let Some(slave) = slaves.get(3) {
                    a3.set_slave_add(slave.address);
                    a3.set_r_1(true);
                    s3.set_slave_reg(slave.register);
                    c3.set_slave_numop(slave.num_reads);
                    c3.set_batch_ext_sens_en(slave.batch);
                }
            })
            .await?;

        let aux_sens = match slaves.len() {
            1 => AuxSensors::One,
            2 => AuxSensors::Two,
            3 => AuxSensors::Three,
            _ => AuxSensors::Four,
        };
        self.device
            .master_config()
            .modify_async(|r| {
                r.set_aux_sens_on(aux_sens);
                r.set_write_once(true);
            })
            .await?;

        Ok(())
    }

    pub async fn read_who_am_i(&mut self) -> Result<[u8; 1], crate::Lsm6dsoxError<I2c::Error>> {
        Ok(self.device.who_am_i().read_async().await?.into())
    }

    pub async fn read_status_master(
        &mut self,
    ) -> Result<StatusMasterMainpageFields, crate::Lsm6dsoxError<I2c::Error>> {
        self.device.status_master().read_async().await
    }

    pub async fn read_status_master_mainpage(
        &mut self,
    ) -> Result<StatusMasterMainpageFields, crate::Lsm6dsoxError<I2c::Error>> {
        self.device.status_master_mainpage().read_async().await
    }

    pub async fn read_fifo_status(
        &mut self,
    ) -> Result<FifoStatusFields, crate::Lsm6dsoxError<I2c::Error>> {
        self.device.fifo_status().read_async().await
    }

    pub async fn set_timestamp_enabled(
        &mut self,
        enable: bool,
    ) -> Result<(), crate::Lsm6dsoxError<I2c::Error>> {
        self.device
            .ctrl_10_c()
            .modify_async(|r| r.set_timestamp_en(enable))
            .await
    }

    pub async fn set_start_config(
        &mut self,
        trigger: ShubTrigger,
    ) -> Result<(), crate::Lsm6dsoxError<I2c::Error>> {
        self.device
            .master_config()
            .modify_async(|r| r.set_start_config(trigger))
            .await
    }

    pub async fn set_shub_pu_en(
        &mut self,
        enable: bool,
    ) -> Result<(), crate::Lsm6dsoxError<I2c::Error>> {
        self.device
            .master_config()
            .modify_async(|r| r.set_shub_pu_en(enable))
            .await
    }

    pub async fn set_master_on(
        &mut self,
        enable: bool,
    ) -> Result<(), crate::Lsm6dsoxError<I2c::Error>> {
        self.device
            .master_config()
            .modify_async(|r| r.set_master_on(enable))
            .await
    }

    pub async fn set_aux_sens_on(
        &mut self,
        sensors: AuxSensors,
    ) -> Result<(), crate::Lsm6dsoxError<I2c::Error>> {
        self.device
            .master_config()
            .modify_async(|r| r.set_aux_sens_on(sensors))
            .await
    }

    pub async fn reset_sensor_hub_master(
        &mut self,
    ) -> Result<(), crate::Lsm6dsoxError<I2c::Error>> {
        self.device
            .master_config()
            .modify_async(|r| r.set_rst_master_regs(true))
            .await?;

        self.device
            .master_config()
            .modify_async(|r| r.set_rst_master_regs(false))
            .await?;

        Ok(())
    }

    pub async fn set_acc_batch_rate(
        &mut self,
        rate: BatchDataRate,
    ) -> Result<(), crate::Lsm6dsoxError<I2c::Error>> {
        self.device
            .fifo_ctrl_3()
            .modify_async(|r| r.set_bdr_xl(rate))
            .await
    }

    pub async fn set_gyro_batch_rate(
        &mut self,
        rate: BatchDataRate,
    ) -> Result<(), crate::Lsm6dsoxError<I2c::Error>> {
        self.device
            .fifo_ctrl_3()
            .modify_async(|r| r.set_bdr_gy(rate))
            .await
    }

    pub async fn set_fifo_mode(
        &mut self,
        mode: FifoMode,
    ) -> Result<(), crate::Lsm6dsoxError<I2c::Error>> {
        self.device
            .fifo_ctrl_4()
            .modify_async(|r| r.set_fifo_mode(mode))
            .await
    }

    pub async fn set_ts_decimation(
        &mut self,
        decimation: TimestampDecimation,
    ) -> Result<(), crate::Lsm6dsoxError<I2c::Error>> {
        self.device
            .fifo_ctrl_4()
            .modify_async(|r| r.set_odr_ts_batch(decimation))
            .await
    }

    pub async fn set_temp_batch_rate(
        &mut self,
        rate: TemperatureBatchRate,
    ) -> Result<(), crate::Lsm6dsoxError<I2c::Error>> {
        self.device
            .fifo_ctrl_4()
            .modify_async(|r| r.set_odr_t_batch(rate))
            .await
    }

    pub async fn set_pass_through_mode(
        &mut self,
        enable: bool,
    ) -> Result<(), crate::Lsm6dsoxError<I2c::Error>> {
        self.device
            .master_config()
            .modify_async(|r| r.set_pass_through_mode(enable))
            .await
    }

    pub async fn sw_reset(&mut self) -> Result<(), crate::Lsm6dsoxError<I2c::Error>> {
        self.device
            .ctrl_3_c()
            .modify_async(|r| r.set_sw_reset(true))
            .await?;

        Ok(())
    }

    pub async fn set_shub_reg_access(
        &mut self,
        enable: bool,
    ) -> Result<(), crate::Lsm6dsoxError<I2c::Error>> {
        self.device
            .func_cfg_access()
            .modify_async(|r| {
                r.set_shub_reg_access(enable);
            })
            .await?;
        Ok(())
    }

    pub async fn set_acc_odr(
        &mut self,
        odr: AccelOdr,
    ) -> Result<(), crate::Lsm6dsoxError<I2c::Error>> {
        self.device
            .ctrl_1_xl()
            .modify_async(|r| r.set_odr_xl(odr))
            .await
    }

    pub async fn set_gyro_odr(
        &mut self,
        odr: GyroOdr,
    ) -> Result<(), crate::Lsm6dsoxError<I2c::Error>> {
        self.device
            .ctrl_2_g()
            .modify_async(|r| r.set_odr(odr))
            .await
    }

    pub async fn set_acc_full_scale(
        &mut self,
        full_scale: AccelFullScale,
    ) -> Result<(), crate::Lsm6dsoxError<I2c::Error>> {
        self.device
            .ctrl_1_xl()
            .modify_async(|r| r.set_fs_xl(full_scale))
            .await
    }

    pub async fn set_gyro_full_scale(
        &mut self,
        full_scale: GyroFullScale,
        fs_125: bool,
    ) -> Result<(), crate::Lsm6dsoxError<I2c::Error>> {
        self.device
            .ctrl_2_g()
            .modify_async(|r| {
                r.set_fs_g(full_scale);
                r.set_fs_125(fs_125);
            })
            .await
    }

    pub async fn drain_fifo(
        &mut self,
        buf: &mut [u8],
    ) -> Result<u16, crate::Lsm6dsoxError<I2c::Error>> {
        let fifo_status = self.device.fifo_status().read_async().await?;
        let count = fifo_status.diff_fifo();
        let word_count = (count as usize).min(buf.len() / 7);
        let buf = &mut buf[..word_count * 7];
        self.device.fifo_out().read_async(buf).await?;
        Ok(word_count as u16)
    }

    pub async fn process_fifo<F, EntryError>(
        &mut self,
        buf: &mut [u8],
        num_entries: u16,
        mut on_entry: F,
    ) -> Result<(), EntryError>
    where
        F: FnMut(FifoEntryValue) -> Result<u16, EntryError>,
    {
        let word_count = (num_entries as usize).min(buf.len() / 7);
        let buf = &mut buf[..word_count * 7];
        let (chunks, _remainder) = buf.as_chunks::<7>();

        for chunk in chunks {
            on_entry(FifoEntryValue::from(*chunk))?;
        }

        Ok(())
    }

    pub async fn with_passthrough<F, T, E>(&mut self, body: F) -> Result<T, E>
    where
        E: From<crate::Lsm6dsoxError<I2c::Error>>,
        F: AsyncFnOnce(&mut I2c) -> Result<T, E>,
    {
        self.device
            .master_config()
            .modify_async(|r| {
                r.set_pass_through_mode(true);
            })
            .await?;

        let out = body(&mut self.device.interface.i2c).await;
        let disable = self
            .device
            .master_config()
            .write_async(|r| {
                r.set_pass_through_mode(false);
            })
            .await;
        out.and_then(|value| disable.map(|()| value).map_err(E::from))
    }

    pub async fn with_shub_reg_access<F, T, E>(&mut self, body: F) -> Result<T, E>
    where
        E: From<crate::Lsm6dsoxError<I2c::Error>>,
        F: AsyncFnOnce(&mut Self) -> Result<T, E>,
    {
        self.set_shub_reg_access(true).await?;
        let out = body(self).await;
        let disable = self.set_shub_reg_access(false).await;
        out.and_then(|value| disable.map(|()| value).map_err(E::from))
    }

    pub async fn read_temperature(&mut self) -> Result<i16, crate::Lsm6dsoxError<I2c::Error>> {
        Ok(self.device.out_temp().read_async().await?.temp())
    }

    pub async fn read_gyro(&mut self) -> Result<[i16; 3], crate::Lsm6dsoxError<I2c::Error>> {
        let (x, y, z) = self
            .device
            .multi_read()
            .with(|d| d.outx_g().plan())
            .with(|d| d.outy_g().plan())
            .with(|d| d.outz_g().plan())
            .execute_async()
            .await?;
        Ok([x.value(), y.value(), z.value()])
    }

    pub async fn read_acc(&mut self) -> Result<[i16; 3], crate::Lsm6dsoxError<I2c::Error>> {
        let (x, y, z) = self
            .device
            .multi_read()
            .with(|d| d.outx_xl().plan())
            .with(|d| d.outy_xl().plan())
            .with(|d| d.outz_xl().plan())
            .execute_async()
            .await?;
        Ok([x.value(), y.value(), z.value()])
    }

    pub async fn read_sensor_hub_values<const N: usize>(
        &mut self,
        buf: &mut [u8; N],
        read_offset: u8,
    ) -> Result<(), crate::Lsm6dsoxError<I2c::Error>> {
        if read_offset as usize + buf.len() > 18 {
            return Err(Lsm6dsoxError::InvalidRange);
        }

        self.with_shub_reg_access(async |imu| {
            let shub_first_addr = imu.device.sensor_hub_1().address();
            imu.device
                .interface
                .i2c
                .write_read(
                    imu.device.interface.addr,
                    &[shub_first_addr + read_offset],
                    &mut buf[..N],
                )
                .await
                .map_err(|e| Lsm6dsoxError::I2c(e))?;

            Ok(())
        })
        .await?;

        Ok(())
    }
}

impl<I2c: embedded_hal_async::i2c::I2c> device_driver::BufferInterfaceBase
    for DeviceInterface<I2c>
{
    type Error = Lsm6dsoxError<I2c::Error>;
    type AddressType = u8;
}

impl<I2c: embedded_hal_async::i2c::I2c> device_driver::AsyncBufferInterface
    for DeviceInterface<I2c>
{
    async fn read(&mut self, address: u8, buf: &mut [u8]) -> Result<usize, Self::Error> {
        self.i2c
            .write_read(self.addr, &[address], buf)
            .await
            .map_err(Lsm6dsoxError::I2c)?;
        debug!(
            "AsyncBufferInterface::read address=0x{:X} data={:?}",
            address, buf
        );
        Ok(buf.len())
    }

    async fn write(&mut self, _address: u8, _buf: &[u8]) -> Result<usize, Self::Error> {
        unreachable!("FIFO_OUT buffer is read-only")
    }

    async fn flush(&mut self, _address: u8) -> Result<(), Self::Error> {
        Ok(())
    }
}

impl<I2c: embedded_hal_async::i2c::I2c> RegisterInterfaceBase for DeviceInterface<I2c> {
    type AddressType = u8;
    type Error = Lsm6dsoxError<I2c::Error>;
}

impl<I2c: embedded_hal_async::i2c::I2c> device_driver::AsyncRegisterInterface
    for DeviceInterface<I2c>
{
    async fn write_register(
        &mut self,
        address: Self::AddressType,
        data: &mut [u8],
        _metadata: &FieldsetMetadata,
    ) -> Result<(), Self::Error> {
        debug!("write_register address=0x{:X} data={:?}", address, data);

        // One register address byte plus the largest contiguous register block:
        // the 12-byte SLV0_ADD..SLV3_CONFIG sensor-hub block (0x15..=0x20).
        const MAX_WRITE_LEN: usize = 1 + 12;

        let mut buf = [0u8; MAX_WRITE_LEN];
        let len = data.len() + 1;
        buf[0] = address;
        buf.get_mut(1..len)
            .ok_or(Lsm6dsoxError::WriteTooLong)?
            .copy_from_slice(data);

        self.i2c
            .write(self.addr, &buf[..len])
            .await
            .map_err(Lsm6dsoxError::I2c)
    }

    async fn read_register(
        &mut self,
        address: Self::AddressType,
        data: &mut [u8],
        _metadata: &FieldsetMetadata,
    ) -> Result<(), Self::Error> {
        let result = self
            .i2c
            .write_read(self.addr, &[address], data)
            .await
            .map_err(Lsm6dsoxError::I2c);
        debug!("read_register address=0x{:X} data={:?}", address, data);
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal_async::i2c::Operation;

    #[derive(Debug)]
    enum I2cError {}

    impl embedded_hal_async::i2c::Error for I2cError {
        fn kind(&self) -> embedded_hal_async::i2c::ErrorKind {
            embedded_hal_async::i2c::ErrorKind::Other
        }
    }

    struct I2cMock {
        log: Vec<String>,
        responses: std::collections::VecDeque<Vec<u8>>,
    }

    impl I2cMock {
        fn new() -> Self {
            Self {
                log: Vec::new(),
                responses: std::collections::VecDeque::new(),
            }
        }

        fn push_response(&mut self, data: impl Into<Vec<u8>>) {
            self.responses.push_back(data.into());
        }
    }

    impl embedded_hal_async::i2c::ErrorType for I2cMock {
        type Error = I2cError;
    }

    impl embedded_hal_async::i2c::I2c for I2cMock {
        async fn transaction(
            &mut self,
            address: u8,
            operations: &mut [Operation<'_>],
        ) -> Result<(), Self::Error> {
            for op in operations.iter_mut() {
                match op {
                    Operation::Write(data) => {
                        let entry = format!("write  addr={:#04x} data={:02x?}", address, data);
                        self.log.push(entry);
                    }
                    Operation::Read(data) => {
                        if let Some(response) = self.responses.pop_front() {
                            let len = data.len().min(response.len());
                            data[..len].copy_from_slice(&response[..len]);
                        }
                        let entry =
                            format!("read   addr={:#04x} data={:02x?}", address, &data as &[u8]);
                        self.log.push(entry);
                    }
                }
            }
            Ok(())
        }
    }

    #[test]
    fn test_drain_fifo() {
        pollster::block_on(async {
            const FIFO_WORDS: u16 = 256;
            let mut mock = I2cMock::new();
            mock.push_response([0x00, 0x01]);
            let entry = [0x08, 0x64, 0x00, 0xC8, 0x00, 0x2C, 0x01];
            let mut fifo_response = Vec::with_capacity(FIFO_WORDS as usize * 7);
            for _ in 0..FIFO_WORDS {
                fifo_response.extend_from_slice(&entry);
            }
            mock.push_response(fifo_response);
            let mut lsm6dso = Lsm6dsox::new(mock, 0x6A);
            let mut buf = [0u8; FIFO_WORDS as usize * 7];
            let count = lsm6dso.drain_fifo(&mut buf).await.unwrap();

            assert_eq!(count, FIFO_WORDS);
        });
    }

    #[test]
    fn test_read_who_am_i() {
        pollster::block_on(async {
            let mut mock = I2cMock::new();
            const WHO_AM_I_RESPONSE: [u8; 1] = [0x6C];
            mock.push_response(&WHO_AM_I_RESPONSE);
            let mut lsm6dso = Lsm6dsox::new(mock, 0x6A);
            let result = lsm6dso.read_who_am_i().await.unwrap();
            assert_eq!(result, WHO_AM_I_RESPONSE);
        });
    }

    #[test]
    fn test_set_acc_full_scale() {
        pollster::block_on(async {
            const CTRL1_XL: u8 = 0x10;
            let mut mock = I2cMock::new();
            // Value returned for the read phase of the read-modify-write.
            mock.push_response([0x00]);

            let mut lsm6dso = Lsm6dsox::new(mock, 0x6A);
            lsm6dso
                .set_acc_full_scale(AccelFullScale::Scale4G)
                .await
                .unwrap();

            let log = &lsm6dso.device.interface.i2c.log;
            assert_eq!(
                log,
                &[
                    format!("write  addr=0x6a data=[{CTRL1_XL:02x}]"),
                    "read   addr=0x6a data=[00]".to_string(),
                    "write  addr=0x6a data=[10, 08]".to_string(),
                ],
            );
        });
    }

    #[test]
    fn test_configure_sensor_hub_writes_full_block() {
        pollster::block_on(async {
            let mut mock = I2cMock::new();
            mock.push_response([0x00]);

            let mut lsm6dso = Lsm6dsox::new(mock, 0x6A);
            let slaves = [SensorHubSlave {
                address: 0x1E,
                register: 0x28,
                num_reads: 6,
                batch: true,
            }];
            lsm6dso
                .configure_sensor_hub(&slaves, ShubOdr::Rate104Hz)
                .await
                .unwrap();

            let log = &lsm6dso.device.interface.i2c.log;
            assert!(
                log.iter().any(|e| e
                    == "write  addr=0x6a data=[15, 3d, 28, 0e, 00, 00, 00, 00, 00, 00, 00, 00, 00]"),
                "expected full 12-byte SLV block write; log was:\n{log:#?}",
            );
        });
    }
}
