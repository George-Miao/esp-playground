use core::convert::Infallible;

use embedded_hal::delay::DelayNs;
use esp_backtrace as _;
use esp_hal::{
    DriverMode,
    delay::Delay,
    gpio::{Flex, Output, Pull},
    spi::{
        DataMode, Error,
        master::{Address, Command, Spi},
    },
};

const MSB_MASK: u8 = 0b1000_0000;

#[repr(u8)]
#[derive(Clone, Copy)]
pub enum Instruction {
    NOP        = 0x00,
    SWRESET    = 0x01, // Software Reset
    SLPOUT     = 0x11, // Sleep Out
    PTLON      = 0x12, // Partial Display Mode On
    NORON      = 0x13, // Normal Display Mode On
    INVOFF     = 0x20, // Display Inversion Off
    INVON      = 0x21, // Display Inversion On
    ALLPOFF    = 0x22, // All Pixels Off
    ALLPON     = 0x23, // All Pixels On
    GAMSET     = 0x26, // Gamma Set
    DISPOFF    = 0x28, // Display Off
    DISPON     = 0x29, // Display On
    TEOFF      = 0x34, // Tearing Effect Line Off (kinda vsync)
    TEON       = 0x35, // Tearing Effect Line On (kinda vsync)
    MADCTL     = 0x36, // Display data access control
    IDMOFF     = 0x38, // Idle Mode Off
    IDMON      = 0x39, // Idle Mode On
    COLMOD     = 0x3A, // Interface Pixel Format
    GSL        = 0x45, // Get Scan Line
    // Command2_BK0
    PVGAMCTRL  = 0xB0, // Positive Voltage Gamma Control
    NVGAMCTRL  = 0xB1, // Negative Voltage Gamma Control
    // DGMEN = 0xB8,     // Digital Gamma Enable
    DGMLUTR    = 0xB9, // Digital Gamma LUT for Red
    DGMLUTB    = 0xBA, // Digital Gamma LUT for Blue
    LNESET     = 0xC0, // Display Line Setting
    PORCTRL    = 0xC1, // Porch Control
    INVSET     = 0xC2, // Inversion Selection & Frame Rate Control
    RGBCTRL    = 0xC3, // RGB Control
    PARCTRL    = 0xC5, // Partial Mode Control
    SDIR       = 0xC7, // X-direction Control
    // PDOSET = 0xC8,  // Pseudo-Dot Inversion Driving Setting
    COLCTRL    = 0xCD, // Colour Control
    SRECTRL    = 0xE0, // Sunlight Readable Enhancement
    NRCTRL     = 0xE1, // Noise Reduce Control
    SECTRL     = 0xE2, // Sharpness Control
    CCCTRL     = 0xE3, // Color Calibration Control
    SKCTRL     = 0xE4, // Skin Tone Preservation Control
    // Command2_BK1
    // VHRS = 0xB0, // Vop amplitude
    // VCOMS = 0xB1,   // VCOM amplitude
    VGHSS      = 0xB2, // VGH voltage
    TESTCMD    = 0xB3, // TEST command
    VGLS       = 0xB5, // VGL voltage
    VRHDV      = 0xB6, // VRH_DV voltage
    PWCTRL1    = 0xB7, // Power Control 1
    PWCTRL2    = 0xB8, // Power Control 2
    // PCLKS1 = 0xBA,  // Power pumping clock selection 1
    PCLKS2     = 0xBC, // Power pumping clock selection 2
    // PDR1 = 0xC1,   // Source pre-drive timing set 1
    // PDR2 = 0xC2,   // Source pre-drive timing set 2
    // Command2_BK3
    NVMEN      = 0xC8, // NVM enable
    NVMSET     = 0xCA, // NVM manual control
    PROMACT    = 0xCC, // NVM program active
    // Other
    CND2BKxSEL = 0xFF, // Command2 BKx Select
}

impl Instruction {
    fn ser(&self) -> Command {
        ser(true, *self as u8)
    }
}

fn ser(is_command: bool, byte: u8) -> Command {
    // First bit: 0 for command, 1 for parameter
    let first_bit = (!is_command as u16) << 15;
    // 1-bit C/D followed by 8-bit data
    let data = (byte as u16) << 7 | first_bit;

    Command::_9Bit(data, DataMode::Single)
}

pub struct St7701<'a, S> {
    spi: S,
    rst: Output<'a>,
}

pub struct ManualSpi<'a> {
    pub cs: Output<'a>,
    pub sda: Flex<'a>,
    pub scl: Output<'a>,
}

impl ManualSpi<'_> {
    pub fn read_command(&mut self, command: u8, buf: &mut [u8]) {
        self.while_cs(|s| {
            s.sda.set_as_output();
            s.write_byte(true, command).unwrap();
            s.sda.set_as_open_drain(Pull::None);
            (0..buf.len()).for_each(|i| {
                buf[i] = s.read_byte();
            });
        });
    }

    fn read_byte(&mut self) -> u8 {
        let mut data = 0;
        for _ in 0..u8::BITS {
            self.scl.set_low();
            data <<= 1;
            if self.sda.is_high() {
                data |= 1;
            }
            self.scl.set_high();
            Delay::new().delay_ns(100);
        }

        data
    }
}

impl<'a, S> St7701<'a, S> {
    pub fn new(spi: S, rst: Output<'a>) -> Self {
        Self { spi, rst }
    }

    pub fn into_parts(self) -> (S, Output<'a>) {
        (self.spi, self.rst)
    }

    pub fn spi(&mut self) -> &mut S {
        &mut self.spi
    }
}

pub trait SpiProvider {
    type Error;

    fn write_byte(&mut self, is_command: bool, byte: u8) -> Result<(), Self::Error>;

    fn write_command(&mut self, command: u8) -> Result<(), Self::Error> {
        self.while_cs(|s| s.write_byte(true, command))
    }

    fn write_param(&mut self, param: u8) -> Result<(), Self::Error> {
        self.while_cs(|s| s.write_byte(false, param))
    }

    fn write_data(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        self.while_cs(|s| data.iter().try_for_each(|byte| s.write_byte(false, *byte)))
    }

    fn while_cs<F, R>(&mut self, func: F) -> R
    where
        F: FnOnce(&mut Self) -> R,
    {
        func(self)
    }
}

impl<Dm: DriverMode> SpiProvider for Spi<'_, Dm> {
    type Error = Error;

    fn write_byte(&mut self, is_command: bool, byte: u8) -> Result<(), Self::Error> {
        self.half_duplex_write(
            DataMode::Single,
            ser(is_command, byte),
            Address::None,
            0,
            &[],
        )
    }

    fn write_command(&mut self, instruction: u8) -> Result<(), Self::Error> {
        self.half_duplex_write(
            DataMode::Single,
            ser(true, instruction),
            Address::None,
            0,
            &[],
        )
    }

    fn write_data(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        for byte in data {
            self.half_duplex_write(DataMode::Single, ser(false, *byte), Address::None, 0, &[])?;
        }

        Ok(())
    }
}

impl SpiProvider for ManualSpi<'_> {
    type Error = Infallible;

    fn while_cs<F, R>(&mut self, func: F) -> R
    where
        F: FnOnce(&mut Self) -> R,
    {
        self.cs.set_low();
        Delay::new().delay_ms(1);
        let result = func(self);
        Delay::new().delay_ms(1);
        self.cs.set_high();
        result
    }

    fn write_byte(&mut self, is_command: bool, byte: u8) -> Result<(), Self::Error> {
        self.sda.set_as_output();

        let mut data = byte;

        self.scl.set_low();
        // First bit: 0 for command, 1 for parameter
        if is_command {
            self.sda.set_low()
        } else {
            self.sda.set_high()
        }
        self.scl.set_high();

        for _ in 0..u8::BITS {
            Delay::new().delay_ns(100);

            self.scl.set_low();

            if data & MSB_MASK == MSB_MASK {
                self.sda.set_high();
            } else {
                self.sda.set_low();
            }

            self.scl.set_high();

            data <<= 1;
        }

        Delay::new().delay_ns(100);

        self.scl.set_high();

        Ok(())
    }
}

impl<S: SpiProvider> St7701<'_, S> {
    pub fn reset(&mut self, delay: &mut impl DelayNs) {
        self.rst.set_high();
        delay.delay_ms(100);
        self.rst.set_low();
        delay.delay_ms(100);
        self.rst.set_high();
        delay.delay_ms(100);
    }

    pub fn init2(&mut self, delay: &mut impl DelayNs) -> Result<(), S::Error> {
        self.reset(delay);

        self.spi.write_command(Instruction::SWRESET as u8)?;
        delay.delay_ms(150);

        self.spi.write_command(Instruction::SLPOUT as u8)?;
        delay.delay_ms(150);

        self.spi.write_command(Instruction::INVOFF as u8)?;

        // number of scan line = ((0x3B | 0b0111_1111 = 59) + 1) * 8 = 480
        self.spi.write_command(Instruction::LNESET as u8)?;
        self.spi.write_data(&[0x3B, 0x00])?;

        self.spi.write_command(Instruction::PORCTRL as u8)?;
        self.spi.write_data(&[0x8D, 0x05])?;

        self.spi.write_command(Instruction::MADCTL as u8)?;
        self.spi.write_data(&[0x00])?;

        self.spi.write_command(Instruction::COLMOD as u8)?;
        self.spi.write_data(&[0b0101_0000])?;

        self.spi.write_command(Instruction::INVON as u8)?;
        delay.delay_ms(10);

        self.spi.write_command(Instruction::NORON as u8)?;
        delay.delay_ms(10);

        self.spi.write_command(Instruction::DISPON as u8)?;
        delay.delay_ms(10);

        Ok(())
    }

    pub fn init1(&mut self, delay: &mut impl DelayNs) -> Result<(), S::Error> {
        self.reset(delay);

        self.spi.write_command(0xFF)?; // BK0
        self.spi.write_param(0x77)?;
        self.spi.write_param(0x01)?;
        self.spi.write_param(0x00)?;
        self.spi.write_param(0x00)?;
        self.spi.write_param(0x10)?;

        self.spi.write_command(0xC0)?; // Line set
        self.spi.write_param(0x3B)?; //Scan line
        self.spi.write_param(0x00)?;

        self.spi.write_command(0xC1)?;
        self.spi.write_param(0x0B)?; //VBP
        self.spi.write_param(0x02)?;

        self.spi.write_command(0xC2)?;
        self.spi.write_param(0x07)?;
        self.spi.write_param(0x02)?;

        self.spi.write_command(0xCC)?;
        self.spi.write_param(0x10)?;

        // self.spi.write_command( 0xCD)?;
        // self.spi.write_param( 0x08)?; //18BIT

        // self.spi.write_command(  );//R?GB format
        // self.spi.write_param( 0x08)?;

        self.spi.write_command(0xB0)?; // IPS
        self.spi.write_param(0x00)?; // 255
        self.spi.write_param(0x11)?; // 251
        self.spi.write_param(0x16)?; // 247  down
        self.spi.write_param(0x0e)?; // 239
        self.spi.write_param(0x11)?; // 231
        self.spi.write_param(0x06)?; // 203
        self.spi.write_param(0x05)?; // 175
        self.spi.write_param(0x09)?; // 147
        self.spi.write_param(0x08)?; // 108
        self.spi.write_param(0x21)?; // 80
        self.spi.write_param(0x06)?; // 52
        self.spi.write_param(0x13)?; // 24
        self.spi.write_param(0x10)?; // 16
        self.spi.write_param(0x29)?; // 8    down
        self.spi.write_param(0x31)?; // 4
        self.spi.write_param(0x18)?; // 0

        self.spi.write_command(0xB1)?; //  IPS
        self.spi.write_param(0x00)?; //  255
        self.spi.write_param(0x11)?; //  251
        self.spi.write_param(0x16)?; //  247   down
        self.spi.write_param(0x0e)?; //  239
        self.spi.write_param(0x11)?; //  231
        self.spi.write_param(0x07)?; //  203
        self.spi.write_param(0x05)?; //  175
        self.spi.write_param(0x09)?; //  147
        self.spi.write_param(0x09)?; //  108
        self.spi.write_param(0x21)?; //  80
        self.spi.write_param(0x05)?; //  52
        self.spi.write_param(0x13)?; //  24
        self.spi.write_param(0x11)?; //  16
        self.spi.write_param(0x2a)?; //  8  down
        self.spi.write_param(0x31)?; //  4
        self.spi.write_param(0x18)?; //  0

        self.spi.write_command(0xFF)?;
        self.spi.write_param(0x77)?;
        self.spi.write_param(0x01)?;
        self.spi.write_param(0x00)?;
        self.spi.write_param(0x00)?;
        self.spi.write_param(0x11)?;

        self.spi.write_command(0xB0)?; //VOP  3.5375+ *x 0.0125
        self.spi.write_param(0x6d)?; //5D

        self.spi.write_command(0xB1)?; //VCOM amplitude setting
        self.spi.write_param(0x37)?; //

        self.spi.write_command(0xB2)?; //VGH Voltage setting
        self.spi.write_param(0x81)?; //12V

        self.spi.write_command(0xB3)?;
        self.spi.write_param(0x80)?;

        self.spi.write_command(0xB5)?; //VGL Voltage setting
        self.spi.write_param(0x43)?; //-8.3V

        self.spi.write_command(0xB7)?;
        self.spi.write_param(0x85)?;

        self.spi.write_command(0xB8)?;
        self.spi.write_param(0x20)?;

        self.spi.write_command(0xC1)?;
        self.spi.write_param(0x78)?;

        self.spi.write_command(0xC2)?;
        self.spi.write_param(0x78)?;

        self.spi.write_command(0xD0)?;
        self.spi.write_param(0x88)?;

        self.spi.write_command(0xE0)?;
        self.spi.write_param(0x00)?;
        self.spi.write_param(0x00)?;
        self.spi.write_param(0x02)?;

        self.spi.write_command(0xE1)?;
        self.spi.write_param(0x03)?;
        self.spi.write_param(0xA0)?;
        self.spi.write_param(0x00)?;
        self.spi.write_param(0x00)?;
        self.spi.write_param(0x04)?;
        self.spi.write_param(0xA0)?;
        self.spi.write_param(0x00)?;
        self.spi.write_param(0x00)?;
        self.spi.write_param(0x00)?;
        self.spi.write_param(0x20)?;
        self.spi.write_param(0x20)?;

        self.spi.write_command(0xE2)?;
        self.spi.write_param(0x00)?;
        self.spi.write_param(0x00)?;
        self.spi.write_param(0x00)?;
        self.spi.write_param(0x00)?;
        self.spi.write_param(0x00)?;
        self.spi.write_param(0x00)?;
        self.spi.write_param(0x00)?;
        self.spi.write_param(0x00)?;
        self.spi.write_param(0x00)?;
        self.spi.write_param(0x00)?;
        self.spi.write_param(0x00)?;
        self.spi.write_param(0x00)?;
        self.spi.write_param(0x00)?;

        self.spi.write_command(0xE3)?;
        self.spi.write_param(0x00)?;
        self.spi.write_param(0x00)?;
        self.spi.write_param(0x11)?;
        self.spi.write_param(0x00)?;

        self.spi.write_command(0xE4)?;
        self.spi.write_param(0x22)?;
        self.spi.write_param(0x00)?;

        self.spi.write_command(0xE5)?;
        self.spi.write_param(0x05)?;
        self.spi.write_param(0xEC)?;
        self.spi.write_param(0xA0)?;
        self.spi.write_param(0xA0)?;
        self.spi.write_param(0x07)?;
        self.spi.write_param(0xEE)?;
        self.spi.write_param(0xA0)?;
        self.spi.write_param(0xA0)?;
        self.spi.write_param(0x00)?;
        self.spi.write_param(0x00)?;
        self.spi.write_param(0x00)?;
        self.spi.write_param(0x00)?;
        self.spi.write_param(0x00)?;
        self.spi.write_param(0x00)?;
        self.spi.write_param(0x00)?;
        self.spi.write_param(0x00)?;

        self.spi.write_command(0xE6)?;
        self.spi.write_param(0x00)?;
        self.spi.write_param(0x00)?;
        self.spi.write_param(0x11)?;
        self.spi.write_param(0x00)?;

        self.spi.write_command(0xE7)?;
        self.spi.write_param(0x22)?;
        self.spi.write_param(0x00)?;

        self.spi.write_command(0xE8)?;
        self.spi.write_param(0x06)?;
        self.spi.write_param(0xED)?;
        self.spi.write_param(0xA0)?;
        self.spi.write_param(0xA0)?;
        self.spi.write_param(0x08)?;
        self.spi.write_param(0xEF)?;
        self.spi.write_param(0xA0)?;
        self.spi.write_param(0xA0)?;
        self.spi.write_param(0x00)?;
        self.spi.write_param(0x00)?;
        self.spi.write_param(0x00)?;
        self.spi.write_param(0x00)?;
        self.spi.write_param(0x00)?;
        self.spi.write_param(0x00)?;
        self.spi.write_param(0x00)?;
        self.spi.write_param(0x00)?;

        self.spi.write_command(0xEB)?;
        self.spi.write_param(0x00)?;
        self.spi.write_param(0x00)?;
        self.spi.write_param(0x40)?;
        self.spi.write_param(0x40)?;
        self.spi.write_param(0x00)?;
        self.spi.write_param(0x00)?;
        self.spi.write_param(0x00)?;

        self.spi.write_command(0xED)?;
        self.spi.write_param(0xFF)?;
        self.spi.write_param(0xFF)?;
        self.spi.write_param(0xFF)?;
        self.spi.write_param(0xBA)?;
        self.spi.write_param(0x0A)?;
        self.spi.write_param(0xBF)?;
        self.spi.write_param(0x45)?;
        self.spi.write_param(0xFF)?;
        self.spi.write_param(0xFF)?;
        self.spi.write_param(0x54)?;
        self.spi.write_param(0xFB)?;
        self.spi.write_param(0xA0)?;
        self.spi.write_param(0xAB)?;
        self.spi.write_param(0xFF)?;
        self.spi.write_param(0xFF)?;
        self.spi.write_param(0xFF)?;

        self.spi.write_command(0xEF)?;
        self.spi.write_param(0x10)?;
        self.spi.write_param(0x0D)?;
        self.spi.write_param(0x04)?;
        self.spi.write_param(0x08)?;
        self.spi.write_param(0x3F)?;
        self.spi.write_param(0x1F)?;

        self.spi.write_command(0xFF)?;
        self.spi.write_param(0x77)?;
        self.spi.write_param(0x01)?;
        self.spi.write_param(0x00)?;
        self.spi.write_param(0x00)?;
        self.spi.write_param(0x13)?;

        self.spi.write_command(0xEF)?;
        self.spi.write_param(0x08)?;

        self.spi.write_command(0xFF)?;
        self.spi.write_param(0x77)?;
        self.spi.write_param(0x01)?;
        self.spi.write_param(0x00)?;
        self.spi.write_param(0x00)?;
        self.spi.write_param(0x00)?;

        self.spi.write_command(0x11)?;

        delay.delay_ms(120);

        self.spi.write_command(0x29)?;

        self.spi.write_command(0x36)?;
        self.spi.write_param(0x08)?;

        self.spi.write_command(0x3A)?;
        self.spi.write_param(0x77)?;

        Ok(())
    }

    pub fn init3(&mut self, delay: &mut impl DelayNs) -> Result<(), S::Error> {
        self.reset(delay);

        self.spi.write_command(0xFF)?;
        self.spi.write_data(&[0x77, 0x01, 0x00, 0x00, 0x10])?;

        self.spi.write_command(0xC0)?;
        self.spi.write_data(&[0x3B, 0x00])?;
        self.spi.write_command(0xC1)?;
        self.spi.write_data(&[0x0B, 0x02])?; // VBP
        self.spi.write_command(0xC2)?;
        self.spi.write_data(&[0x00, 0x02])?;

        self.spi.write_command(0xCC)?;
        self.spi.write_data(&[0x10])?;
        self.spi.write_command(0xCD)?;
        self.spi.write_data(&[0x08])?;

        self.spi.write_command(0xB0)?; // Positive Voltage Gamma Control
        self.spi.write_data(&[
            0x02, 0x13, 0x1B, 0x0D, 0x10, 0x05, 0x08, 0x07, 0x07, 0x24, 0x04, 0x11, 0x0E, 0x2C,
            0x33, 0x1D,
        ])?;

        self.spi.write_command(0xB1)?; // Negative Voltage Gamma Control
        self.spi.write_data(&[
            0x05, 0x13, 0x1B, 0x0D, 0x11, 0x05, 0x08, 0x07, 0x07, 0x24, 0x04, 0x11, 0x0E, 0x2C,
            0x33, 0x1D,
        ])?;

        self.spi.write_command(0xFF)?;
        self.spi.write_data(&[0x77, 0x01, 0x00, 0x00, 0x11])?;

        self.spi.write_command(0xB0)?;
        self.spi.write_data(&[0x5d])?; // 5d
        self.spi.write_command(0xB1)?;
        self.spi.write_data(&[0x43])?; // VCOM amplitude setting
        self.spi.write_command(0xB2)?;
        self.spi.write_data(&[0x81])?; // VGH Voltage setting, 12V
        self.spi.write_command(0xB3)?;
        self.spi.write_data(&[0x80])?;

        self.spi.write_command(0xB5)?;
        self.spi.write_data(&[0x43])?; // VGL Voltage setting, -8.3V

        self.spi.write_command(0xB7)?;
        self.spi.write_data(&[0x85])?;
        self.spi.write_command(0xB8)?;
        self.spi.write_data(&[0x20])?;

        self.spi.write_command(0xC1)?;
        self.spi.write_data(&[0x78])?;
        self.spi.write_command(0xC2)?;
        self.spi.write_data(&[0x78])?;

        self.spi.write_command(0xD0)?;
        self.spi.write_data(&[0x88])?;

        self.spi.write_command(0xE0)?;
        self.spi.write_data(&[0x00, 0x00, 0x02])?;

        self.spi.write_command(0xE1)?;
        self.spi.write_data(&[
            0x03, 0xA0, 0x00, 0x00, 0x04, 0xA0, 0x00, 0x00, 0x00, 0x20, 0x20,
        ])?;

        self.spi.write_command(0xE2)?;
        self.spi.write_data(&[
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        ])?;

        self.spi.write_command(0xE3)?;
        self.spi.write_data(&[0x00, 0x00, 0x11, 0x00])?;

        self.spi.write_command(0xE4)?;
        self.spi.write_data(&[0x22, 0x00])?;

        self.spi.write_command(0xE5)?;
        self.spi.write_data(&[
            0x05, 0xEC, 0xA0, 0xA0, 0x07, 0xEE, 0xA0, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00,
        ])?;

        self.spi.write_command(0xE6)?;
        self.spi.write_data(&[0x00, 0x00, 0x11, 0x00])?;

        self.spi.write_command(0xE7)?;
        self.spi.write_data(&[0x22, 0x00])?;

        self.spi.write_command(0xE8)?;
        self.spi.write_data(&[
            0x06, 0xED, 0xA0, 0xA0, 0x08, 0xEF, 0xA0, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00,
        ])?;

        self.spi.write_command(0xEB)?;
        self.spi
            .write_data(&[0x00, 0x00, 0x40, 0x40, 0x00, 0x00, 0x00])?;

        self.spi.write_command(0xED)?;
        self.spi.write_data(&[
            0xFF, 0xFF, 0xFF, 0xBA, 0x0A, 0xBF, 0x45, 0xFF, 0xFF, 0x54, 0xFB, 0xA0, 0xAB, 0xFF,
            0xFF, 0xFF,
        ])?;

        self.spi.write_command(0xEF)?;
        self.spi.write_data(&[0x10, 0x0D, 0x04, 0x08, 0x3F, 0x1F])?;

        self.spi.write_command(0xFF)?;
        self.spi.write_data(&[0x77, 0x01, 0x00, 0x00, 0x13])?;

        self.spi.write_command(0xEF)?;
        self.spi.write_data(&[0x08])?;

        self.spi.write_command(0xFF)?;
        self.spi.write_data(&[0x77, 0x01, 0x00, 0x00, 0x00])?;

        self.spi.write_command(0x36)?;
        self.spi.write_data(&[0x08])?;
        self.spi.write_command(0x3A)?;
        self.spi.write_data(&[0x60])?; // 0x70 RGB888, 0x60 RGB666, 0x50 RGB565

        self.spi.write_command(0x11)?; // Sleep Out

        Delay::new().delay_ms(100);

        self.spi.write_command(0x29)?; // Display On

        Delay::new().delay_ms(50);

        Ok(())
    }
}
