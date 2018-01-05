/*
 * name:        LPS25HB
 * description: 260-1260 hPa absolute digital output barometer
 * manuf:       STMicroelectronics
 * version:     Version 0.1
 * url:         http://www.st.com/resource/en/datasheet/lps25hb.pdf
 * date:        2018-01-04
 * author       https://chisl.io/
 * file:        LPS25HB.hpp
 */

/*                                                                                                       *
 *                                   THIS FILE IS AUTOMATICALLY CREATED                                  *
 *                                    D O     N O T     M O D I F Y  !                                   *
 *                                                                                                       */

#include <cinttypes>

/* Derive from class LPS25HB_Base and implement the read and write functions! */

/* LPS25HB: 260-1260 hPa absolute digital output barometer */
class LPS25HB_Base
{
public:
	/* Pure virtual functions that need to be implemented in derived class: */
	virtual uint8_t read8(uint16_t address, uint16_t n=8) = 0;  // 8 bit read
	virtual void write(uint16_t address, uint8_t value, uint16_t n=8) = 0;  // 8 bit write
	virtual uint16_t read16(uint16_t address, uint16_t n=16) = 0;  // 16 bit read
	virtual void write(uint16_t address, uint16_t value, uint16_t n=16) = 0;  // 16 bit write
	virtual uint32_t read32(uint16_t address, uint16_t n=32) = 0;  // 32 bit read
	virtual void write(uint16_t address, uint32_t value, uint16_t n=32) = 0;  // 32 bit write
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                            REG REF_P                                             *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG REF_P:
	 * 8.1-3
	 * Reference pressure
	 * The Reference pressure value is a 24-bit data subtracted from the sensor output measurement and
	 * it is composed of REF_P_H (0Ah), REF_P_L (09h) and REF_P_XL (08h). The value is expressed as
	 * 2’s complement.
	 * The reference pressure value is subtracted from the sensor output measurement, to detect a
	 * measured pressure beyond programmed limits (refer to INTERRUPT_CFG (24h) register), and is
	 * used for the Autozero function.
	 */
	struct REF_P
	{
		static const uint16_t __address = 8;
		
		/* Bits REFL: */
		/*
		 * The Reference pressure value is a 24-bit data and it is composed of Section 10.11:
		 * "REF_P_H_17h", Section 10.10: "REF_P_L_16h" and Section 10.9: "REF_P_XL (15h)".
		 */
		struct REFL
		{
			static const uint32_t mask = 0b111111111111111111111111; // [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23]
		};
	};
	
	/* Set register REF_P */
	void setREF_P(uint32_t value)
	{
		write(REF_P::__address, value, 24);
	}
	
	/* Get register REF_P */
	uint32_t getREF_P()
	{
		return read32(REF_P::__address, 24);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                           REG WHO_AM_I                                            *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG WHO_AM_I:
	 * 8.4
	 * Device Who am I
	 */
	struct WHO_AM_I
	{
		static const uint16_t __address = 15;
		
		/* Bits WHO_AM_I: */
		struct WHO_AM_I_
		{
			/* Mode:r */
			static const uint8_t dflt = 0b10111101; // 8'b10111101
			static const uint8_t mask = 0b11111111; // [0,1,2,3,4,5,6,7]
		};
	};
	
	/* Set register WHO_AM_I */
	void setWHO_AM_I(uint8_t value)
	{
		write(WHO_AM_I::__address, value, 8);
	}
	
	/* Get register WHO_AM_I */
	uint8_t getWHO_AM_I()
	{
		return read8(WHO_AM_I::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                           REG RES_CONF                                            *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG RES_CONF:
	 * 8.5
	 * Pressure and temperature resolution
	 */
	struct RES_CONF
	{
		static const uint16_t __address = 16;
		
		/* Bits reserved_0: */
		struct reserved_0
		{
			static const uint8_t mask = 0b11110000; // [4,5,6,7]
		};
		/* Bits AVGT: */
		/* Temperature internal average configuration.  */
		struct AVGT
		{
			static const uint8_t dflt = 0b11; // 2'b11
			static const uint8_t mask = 0b00001100; // [2,3]
			static const uint8_t INTERNAL_AVG_8 = 0b00; // 8 Nr. internal average
			static const uint8_t INTERNAL_AVG_16 = 0b01; // 16 Nr. internal average
			static const uint8_t INTERNAL_AVG_32 = 0b10; // 32 Nr. internal average
			static const uint8_t INTERNAL_AVG_64 = 0b11; // 64 Nr. internal average
		};
		/* Bits AVGP: */
		/* Temperature internal average configuration.  */
		struct AVGP
		{
			static const uint8_t dflt = 0b11; // 2'b11
			static const uint8_t mask = 0b00000011; // [0,1]
			static const uint8_t INTERNAL_AVG_8 = 0b00; // 8 Nr. internal average
			static const uint8_t INTERNAL_AVG_32 = 0b01; // 32 Nr. internal average
			static const uint8_t INTERNAL_AVG_128 = 0b10; // 128 Nr. internal average
			static const uint8_t INTERNAL_AVG_512 = 0b11; // 512 Nr. internal average
		};
	};
	
	/* Set register RES_CONF */
	void setRES_CONF(uint8_t value)
	{
		write(RES_CONF::__address, value, 8);
	}
	
	/* Get register RES_CONF */
	uint8_t getRES_CONF()
	{
		return read8(RES_CONF::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                          REG CTRL_REG1                                           *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG CTRL_REG1:
	 * 8.6
	 * Control register 1.
	 */
	struct CTRL_REG1
	{
		static const uint16_t __address = 32;
		
		/* Bits PD: */
		/*
		 * Power-down control.
		 * The PD bit allows the turn on of the device. The device is in power-down mode when PD is set
		 * to ‘0’ (default value after boot). The device is active when PD is set to ‘1’.
		 */
		struct PD
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b10000000; // [7]
			static const uint8_t POWER_DOWN_MODE = 0b0; // power-down mode
			static const uint8_t ACTIVE_MODE = 0b1; // active mode
		};
		/* Bits ODR: */
		/*
		 * Output data rate selection.
		 * When ODR[2,0] are set to ‘000’ the device enables one-shot mode. When ‘ONESHOT’ bit in
		 * CTRL_REG2 (21h) is set to ‘1’, a new set of data for pressure and temperature is acquired.
		 */
		struct ODR
		{
			static const uint8_t dflt = 0b000; // 3'b0
			static const uint8_t mask = 0b01110000; // [4,5,6]
			static const uint8_t ONE_SHOT = 0b00; // Power down / One shot mode enabled
			static const uint8_t F_1_HZ = 0b01; // p:  1 Hz,   T:  1 Hz
			static const uint8_t F_7_HZ = 0b10; // p:  7 Hz,   T:  7 Hz
			static const uint8_t F_12_5_HZ = 0b11; // p: 12.5 Hz, T: 12.5 Hz
			static const uint8_t F_25_HZ = 0b100; // p: 12.5 Hz, T: 12.5 Hz
			static const uint8_t reserved_0 = 0b101; // 
		};
		/* Bits DIFF_EN: */
		/*
		 * Interrupt generation enable.
		 * The DIFF_EN bit is used to enable the computing of differential pressure output.
		 * It is recommended to enable DIFF_EN after the configuration of REF_P_H (0Ah),
		 * REF_P_L (09h), REF_P_XL (08h), THS_P_H (31h) and THS_P_L (30h).
		 */
		struct DIFF_EN
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00001000; // [3]
			static const uint8_t DISABLE = 0b0; // interrupt generation disabled
			static const uint8_t ENABLE = 0b1; // interrupt generation enabled
		};
		/* Bits BDU: */
		/*
		 * Block data update.
		 * The BDU bit is used to inhibit the update of the output registers between the reading of the upper
		 * and lower register parts. In default mode (BDU = ‘0’), the lower and upper register parts are
		 * updated continuously. When the BDU is activated (BDU = ‘1’), the content of the output registers
		 * is not updated until both MSB and LSB are read, avoiding the reading of values related to different
		 * samples.
		 */
		struct BDU
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000100; // [2]
			static const uint8_t CONTINUOUS_UPDATE = 0b0; // continuous update;
			static const uint8_t NOT_UPDATED_UNTIL_READ = 0b1; // output registers not updated until MSB and LSB have been read
		};
		/* Bits RESET_AZ: */
		/*
		 * Reset Autozero function.
		 * The RESET_AZ bit is used to reset the AutoZero function. Resetting REF_P_H (0Ah), REF_P_L (09h) and
		 * REF_P_XL (08h) sets the pressure reference registers RPDS_H (3Ah) and RPDS_L (39h) to the default value.
		 * RESET_AZ is self-cleared. For the AutoZero function please refer to CTRL_REG2 (21h).
		 */
		struct RESET_AZ
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000010; // [1]
			static const uint8_t NORMAL_MODE = 0b0; // 
			static const uint8_t RESET_AUTOZERO_FUNCTION = 0b1; // 
		};
		/* Bits SIM: */
		/* SPI Serial Interface Mode selection.  */
		struct SIM
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000001; // [0]
			static const uint8_t SPI_4_WIRE = 0b0; // 4-wire interface
			static const uint8_t SPI_3_WIRE = 0b1; // 3-wire interface
		};
	};
	
	/* Set register CTRL_REG1 */
	void setCTRL_REG1(uint8_t value)
	{
		write(CTRL_REG1::__address, value, 8);
	}
	
	/* Get register CTRL_REG1 */
	uint8_t getCTRL_REG1()
	{
		return read8(CTRL_REG1::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                          REG CTRL_REG2                                           *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG CTRL_REG2:
	 * 8.7
	 * Control register 2
	 */
	struct CTRL_REG2
	{
		static const uint16_t __address = 33;
		
		/* Bits BOOT: */
		/*
		 * Reboot memory content.
		 * The bit is self-cleared when the BOOT is completed.
		 * The BOOT bit is used to refresh the content of the internal registers stored in the Flash
		 * memory block. At device power-up the content of the Flash memory block is transferred to
		 * the internal registers related to the trimming functions to allow correct behavior of the
		 * device itself. If for any reason the content of the trimming registers is modified, it is
		 * sufficient to use this bit to restore the correct values. When the BOOT bit is set to ‘1’,
		 * the content of the internal Flash is copied inside the corresponding internal registers and
		 * is used to calibrate the device. These values are factory trimmed and they are different for
		 * every device. They allow correct behavior of the device and normally they should not be changed.
		 * The boot process takes 2.2 msec. At the end of the boot process, the BOOT bit is set to ‘0’
		 * automatically.
		 */
		struct BOOT
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b10000000; // [7]
			static const uint8_t NORMAL_MODE = 0b0; // normal mode
			static const uint8_t REBOOT_MEMORY = 0b1; // reboot memory content
		};
		/* Bits FIFO_EN: */
		/* FIFO enable.  */
		struct FIFO_EN
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b01000000; // [6]
			static const uint8_t DISABLE = 0b0; // 
			static const uint8_t ENABLE = 0b1; // 
		};
		/* Bits STOP_ON_FTH: */
		/* Enable the FTH_FIFO bit in FIFO_STATUS (2Fh) for monitoring of FIFO level.  */
		struct STOP_ON_FTH
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00100000; // [5]
			static const uint8_t DISABLE = 0b0; // 
			static const uint8_t ENABLE = 0b1; // 
		};
		/* Bits FIFO_MEAN_DEC: */
		/*
		 * Enable to decimate the output pressure to 1Hz with FIFO Mean mode.
		 * FIFO_MEAN_DEC bit is to decimate the output pressure to 1Hz with FIFO Mean mode.
		 * When this bit is ‘1’, the output is decimated to 1 Hz as the moving average is being
		 * taken at the rate of the ODR. Otherwise, averaged pressure data will be updated
		 * according to the ODR defined.
		 */
		struct FIFO_MEAN_DEC
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00010000; // [4]
			static const uint8_t DISABLE = 0b0; // 
			static const uint8_t ENABLE = 0b1; // 
		};
		/* Bits I2C_DIS: */
		/* Disable I2C interface.  */
		struct I2C_DIS
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00001000; // [3]
			static const uint8_t ENABLE = 0b0; // I2C enabled
			static const uint8_t DISABLE = 0b1; // I2C disabled
		};
		/* Bits SWRESET: */
		/*
		 * Software reset.
		 * SWRESET is the software reset bit. The device is reset to the power-on configuration after
		 * SWRESET bit is set to '1'. The software reset process takes 4 μsec. When BOOT follows,
		 * the recommended sequence is SWRESET first and then BOOT.
		 */
		struct SWRESET
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000100; // [2]
			static const uint8_t NORMAL_MODE = 0b0; // normal mode
			static const uint8_t SOFTWARE_RESET = 0b1; // software reset
		};
		/* Bits AUTOZERO: */
		/*
		 * Autozero enable.
		 * AUTOZERO, when set to ‘1’, the actual pressure output value is copied in REF_P_H (0Ah),
		 * REF_P_L (09h) and REF_P_XL (08h). When this bit is enabled, the register content of REF_P is
		 * subtracted from the pressure output value.
		 */
		struct AUTOZERO
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000010; // [1]
			static const uint8_t NORMAL_MODE = 0b0; // 
			static const uint8_t AUTOZERO_ENABLED = 0b1; // 
		};
		/* Bits ONE_SHOT: */
		/*
		 * One-shot enable.
		 * The ONE_SHOT bit is used to start a new conversion when the ODR[2,0] bits in CTRL_REG1 (20h)
		 * are set to ‘000’. Writing a ‘1’ in ONE_SHOT triggers a single measurement of pressure and temperature.
		 * Once the measurement is done, the ONE_SHOT bit will self-clear, the new data are available in
		 * the output registers, and the STATUS_REG bits are updated.
		 */
		struct ONE_SHOT
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000001; // [0]
			static const uint8_t IDLE_MODE = 0b0; // idle mode
			static const uint8_t NEW_DATASET = 0b1; // a new dataset is acquired
		};
	};
	
	/* Set register CTRL_REG2 */
	void setCTRL_REG2(uint8_t value)
	{
		write(CTRL_REG2::__address, value, 8);
	}
	
	/* Get register CTRL_REG2 */
	uint8_t getCTRL_REG2()
	{
		return read8(CTRL_REG2::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                          REG CTRL_REG3                                           *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG CTRL_REG3:
	 * 8.8
	 * Control register 3 - INT_DRDY pin control register
	 * The device features one set of programmable interrupt sources (INT) that can be configured to
	 * trigger different pressure events. Figure 19 shows the block diagram of the interrupt generation
	 * block and output pressure data.
	 * The device may also be configured to generate, through the INT_DRDY pin, a Data Ready
	 * signal (DRDY) which indicates when a new measured pressure data is available, thus simplifying
	 * data synchronization in digital systems or optimizing system power consumption.
	 */
	struct CTRL_REG3
	{
		static const uint16_t __address = 34;
		
		/* Bits INT_H_L: */
		/* Interrupt active-high/low.  */
		struct INT_H_L
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b10000000; // [7]
			static const uint8_t ACTIVE_HIGH = 0b0; // active high
			static const uint8_t ACTIVE_LOW = 0b1; // active low
		};
		/* Bits PP_OD: */
		/* Push-pull/open drain selection on interrupt pads.  */
		struct PP_OD
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b01000000; // [6]
			static const uint8_t PUSH_PULL = 0b0; // push-pull
			static const uint8_t OPEN_DRAIN = 0b1; // open drain
		};
		/* Bits reserved_0: */
		struct reserved_0
		{
			static const uint8_t mask = 0b00111100; // [2,3,4,5]
		};
		/* Bits INT_S: */
		/* Data signal on INT_DRDY pin control bits.  */
		struct INT_S
		{
			static const uint8_t dflt = 0b00; // 2'b0
			static const uint8_t mask = 0b00000011; // [0,1]
			static const uint8_t DATA_SIGNAL = 0b00; // Data signal (see CTRL_REG4 (23h))
			static const uint8_t PRESSURE_HIGH = 0b01; // Pressure high (P_high)
			static const uint8_t PRESSURE_LOW = 0b10; // Pressure low (P_low)
			static const uint8_t PRESSURE_LOW_OR_HIGH = 0b11; // Pressure low OR high
		};
	};
	
	/* Set register CTRL_REG3 */
	void setCTRL_REG3(uint8_t value)
	{
		write(CTRL_REG3::__address, value, 8);
	}
	
	/* Get register CTRL_REG3 */
	uint8_t getCTRL_REG3()
	{
		return read8(CTRL_REG3::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                          REG CTRL_REG4                                           *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG CTRL_REG4:
	 * Interrupt configuration
	 */
	struct CTRL_REG4
	{
		static const uint16_t __address = 35;
		
		/* Bits reserved_0: */
		struct reserved_0
		{
			static const uint8_t dflt = 0b0000; // 4'b0
			static const uint8_t mask = 0b11110000; // [4,5,6,7]
		};
		/* Bits F_EMPTY: */
		/* FIFO empty flag on INT_DRDY pin.  */
		struct F_EMPTY
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00001000; // [3]
			static const uint8_t DISABLE = 0b0; // 
			static const uint8_t ENABLE = 0b1; // 
		};
		/* Bits F_FTH: */
		/*
		 * FIFO threshold (watermark) status on INT_DRDY pin to indicate that FIFO is filled
		 * up to the threshold level.
		 */
		struct F_FTH
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000100; // [2]
			static const uint8_t DISABLE = 0b0; // 
			static const uint8_t ENABLE = 0b1; // 
		};
		/* Bits F_OVR: */
		/*
		 * FIFO overrun interrupt on INT_DRDY pin to indicate that FIFO is full in FIFO mode or
		 * that an overrun occurred in Stream mode.
		 */
		struct F_OVR
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000010; // [1]
			static const uint8_t DISABLE = 0b0; // 
			static const uint8_t ENABLE = 0b1; // 
		};
		/* Bits DRDY: */
		/* Data-ready signal on INT_DRDY pin.  */
		struct DRDY
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000001; // [0]
			static const uint8_t DISABLE = 0b0; // 
			static const uint8_t ENABLE = 0b1; // 
		};
	};
	
	/* Set register CTRL_REG4 */
	void setCTRL_REG4(uint8_t value)
	{
		write(CTRL_REG4::__address, value, 8);
	}
	
	/* Get register CTRL_REG4 */
	uint8_t getCTRL_REG4()
	{
		return read8(CTRL_REG4::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                        REG INTERRUPT_CFG                                         *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG INTERRUPT_CFG:
	 * 8.1
	 * Interrupt configuration
	 */
	struct INTERRUPT_CFG
	{
		static const uint16_t __address = 36;
		
		/* Bits reserved_0: */
		struct reserved_0
		{
			static const uint8_t mask = 0b11111000; // [3,4,5,6,7]
		};
		/* Bits LIR: */
		/* Latch interrupt request to the INT_SOURCE (25h) register.  */
		struct LIR
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000100; // [2]
			static const uint8_t NOT_LATCHED = 0b0; // interrupt request not latched
			static const uint8_t LATHED = 0b1; // interrupt request latched
		};
		/* Bits PLE: */
		/* Enable interrupt generation on differential pressure low event.  */
		struct PLE
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000010; // [1]
			static const uint8_t DISABLE = 0b0; // disable interrupt request 
			static const uint8_t ENABLE = 0b1; // enable interrupt request on measured differential pressure value lower than preset threshold
		};
		/* Bits PHE: */
		/* Enable interrupt generation on differential pressure high event.  */
		struct PHE
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b00000001; // [0]
			static const uint8_t DISABLE = 0b0; // disable interrupt request
			static const uint8_t ENABLE = 0b1; // enable interrupt request on measured differential pressure value higher than preset threshold
		};
	};
	
	/* Set register INTERRUPT_CFG */
	void setINTERRUPT_CFG(uint8_t value)
	{
		write(INTERRUPT_CFG::__address, value, 8);
	}
	
	/* Get register INTERRUPT_CFG */
	uint8_t getINTERRUPT_CFG()
	{
		return read8(INTERRUPT_CFG::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                          REG INT_SOURCE                                           *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG INT_SOURCE:
	 * 8.15
	 * Interrupt source
	 * INT_SOURCE register is cleared by reading it.
	 */
	struct INT_SOURCE
	{
		static const uint16_t __address = 37;
		
		/* Bits reserved_0: */
		struct reserved_0
		{
			static const uint8_t dflt = 0b00000; // 5'd0
			static const uint8_t mask = 0b11111000; // [3,4,5,6,7]
		};
		/* Bits IA: */
		/* Interrupt active.  */
		struct IA
		{
			static const uint8_t mask = 0b00000100; // [2]
			static const uint8_t NO_INTERRUPT = 0b0; // no interrupt has been generated;
			static const uint8_t EVENT = 0b1; // one or more interrupt events have been generated
		};
		/* Bits PL: */
		/* Differential pressure Low.  */
		struct PL
		{
			static const uint8_t mask = 0b00000010; // [1]
			static const uint8_t NO_INTERRUPT = 0b0; // no interrupt has been generated;
			static const uint8_t EVENT = 0b1; // Low differential pressure event has occurred
		};
		/* Bits PH: */
		/* Differential pressure High.  */
		struct PH
		{
			static const uint8_t mask = 0b00000001; // [0]
			static const uint8_t NO_INTERRUPT = 0b0; // no interrupt has been generated;
			static const uint8_t EVENT = 0b1; // High differential pressure event has occurred
		};
	};
	
	/* Set register INT_SOURCE */
	void setINT_SOURCE(uint8_t value)
	{
		write(INT_SOURCE::__address, value, 8);
	}
	
	/* Get register INT_SOURCE */
	uint8_t getINT_SOURCE()
	{
		return read8(INT_SOURCE::__address, 8);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                          REG STATUS_REG                                           *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG STATUS_REG:
	 * 8.12
	 * Status register.
	 * This register is updated every ODR cycle, regardless of the BDU value in CTRL_REG1 (20h).
	 */
	struct STATUS_REG
	{
		static const uint16_t __address = 39;
		
		/* Bits reserved_0: */
		struct reserved_0
		{
			static const uint8_t mask = 0b11000000; // [6,7]
		};
		/* Bits T_OR: */
		/* Temperature data overrun.  */
		struct T_OR
		{
			static const uint8_t mask = 0b00100000; // [5]
			static const uint8_t NO_OVERRUN = 0b0; // no overrun has occurred;
			static const uint8_t OVERWRITTEN = 0b1; // a new data for temperature has overwritten the previous one
		};
		/* Bits P_OR: */
		/* Pressure data overrun.  */
		struct P_OR
		{
			static const uint8_t mask = 0b00010000; // [4]
			static const uint8_t NO_OVERRUN = 0b0; // no overrun has occurred;
			static const uint8_t OVERWRITTEN = 0b1; // new data for pressure has overwritten the previous one
		};
		/* Bits unused_1: */
		struct unused_1
		{
			static const uint8_t mask = 0b00001100; // [2,3]
		};
		/* Bits T_DA: */
		/* Temperature data available.  */
		struct T_DA
		{
			static const uint8_t mask = 0b00000010; // [1]
			static const uint8_t NO_NEW_DATA = 0b0; // new data for temperature is not yet available
			static const uint8_t NEW_DATA = 0b1; // new data for temperature is available
		};
		/* Bits P_DA: */
		/* Pressure data available.  */
		struct P_DA
		{
			static const uint8_t mask = 0b00000001; // [0]
			static const uint8_t NO_NEW_DATA = 0b0; // new data for pressure is not yet available;
			static const uint8_t NEW_DATA = 0b1; // new data for pressure is available
		};
	};
	
	/* Set register STATUS_REG */
	void setSTATUS_REG(uint8_t value)
	{
		write(STATUS_REG::__address, value, 8);
	}
	
	/* Get register STATUS_REG */
	uint8_t getSTATUS_REG()
	{
		return read8(STATUS_REG::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                          REG PRESS_OUT                                           *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG PRESS_OUT:
	 * 8.13-15
	 * The pressure output value is a 24-bit data that contains the measured pressure.
	 * It is composed of PRESS_OUT_H (2Ah), PRESS_OUT_L (29h) and PRESS_OUT_XL (28h).
	 * The value is expressed as 2’s complement.
	 */
	struct PRESS_OUT
	{
		static const uint16_t __address = 40;
		
		/* Bits PRESS_OUT: */
		struct PRESS_OUT_
		{
			/* Mode:r */
			static const uint32_t mask = 0b111111111111111111111111; // [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23]
		};
	};
	
	/* Set register PRESS_OUT */
	void setPRESS_OUT(uint32_t value)
	{
		write(PRESS_OUT::__address, value, 24);
	}
	
	/* Get register PRESS_OUT */
	uint32_t getPRESS_OUT()
	{
		return read32(PRESS_OUT::__address, 24);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                           REG TEMP_OUT                                            *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG TEMP_OUT:
	 * 8.16-17
	 * Temperature output value
	 * The temperature output value is a 16-bit data that contains the measured temperature.
	 * It is composed of TEMP_OUT_H (2Ch) and TEMP_OUT_L (2Bh). The value is expressed as 2’s complement.
	 */
	struct TEMP_OUT
	{
		static const uint16_t __address = 43;
		
		/* Bits TEMP_OUT: */
		struct TEMP_OUT_
		{
			/* Mode:r */
			static const uint16_t mask = 0b1111111111111111; // [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
		};
	};
	
	/* Set register TEMP_OUT */
	void setTEMP_OUT(uint16_t value)
	{
		write(TEMP_OUT::__address, value, 16);
	}
	
	/* Get register TEMP_OUT */
	uint16_t getTEMP_OUT()
	{
		return read16(TEMP_OUT::__address, 16);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                          REG FIFO_CTRL                                           *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG FIFO_CTRL:
	 * 8.18
	 * FIFO control register
	 * FIFO Mean mode: The FIFO can be used for implementing a HW moving average on the pressure measurements.
	 * The number of samples of the moving average can be 2, 4, 8, 16 or 32 samples by selecting the FIFO Mean
	 * mode sample size as per Table 23. Different configurations are not allowed.
	 */
	struct FIFO_CTRL
	{
		static const uint16_t __address = 46;
		
		/* Bits F_MODE: */
		/* FIFO mode selection.  */
		struct F_MODE
		{
			static const uint8_t dflt = 0b000; // 3'b0
			static const uint8_t mask = 0b11100000; // [5,6,7]
			static const uint8_t BYPASS = 0b00; // Bypass mode
			static const uint8_t FIFO = 0b01; // FIFO mode
			static const uint8_t STREAM = 0b10; // Stream mode
			static const uint8_t STREAM_TO_FIFO = 0b11; // Stream-to-FIFO mode
			static const uint8_t BYPASS_TO_STREAM = 0b100; // Bypass-to-Stream mode
			static const uint8_t reserved_0 = 0b101; // Not available
			static const uint8_t FIFO_MEAN_MODE = 0b110; // Dynamic-Stream mode
			static const uint8_t BYPASS_TO_FIFO = 0b111; // Bypass-to-FIFO mode
		};
		/* Bits WTM: */
		/*
		 * FIFO threshold (watermark) level selection.
		 * Please note that when using the FIFO Mean mode it is not possible to access the FIFO content.
		 */
		struct WTM
		{
			static const uint8_t mask = 0b00011111; // [0,1,2,3,4]
			static const uint8_t SAMPLE_2 = 0b001; // 2-sample moving average
			static const uint8_t SAMPLE_4 = 0b011; // 4-sample moving average
			static const uint8_t SAMPLE_8 = 0b111; // 8-sample moving average
			static const uint8_t SAMPLE_16 = 0b1111; // 16-sample moving average
			static const uint8_t SAMPLE_32 = 0b11111; // 32-sample moving average
		};
	};
	
	/* Set register FIFO_CTRL */
	void setFIFO_CTRL(uint8_t value)
	{
		write(FIFO_CTRL::__address, value, 8);
	}
	
	/* Get register FIFO_CTRL */
	uint8_t getFIFO_CTRL()
	{
		return read8(FIFO_CTRL::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                         REG FIFO_STATUS                                          *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG FIFO_STATUS:
	 * 8.19
	 * FIFO status
	 */
	struct FIFO_STATUS
	{
		static const uint16_t __address = 47;
		
		/* Bits FTH_FIFO: */
		/* FIFO threshold status.  */
		struct FTH_FIFO
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b10000000; // [7]
			static const uint8_t FIFO_BELOW = 0b0; // FIFO filling is lower than FTH level,
			static const uint8_t FIFO_AT_OR_ABOVE = 0b1; // FIFO filling is equal or higher than FTH level
		};
		/* Bits OVR: */
		/* FIFO overrun status.  */
		struct OVR
		{
			static const uint8_t dflt = 0b0; // 1'b0
			static const uint8_t mask = 0b01000000; // [6]
			static const uint8_t FIFO_NOT_FULL = 0b0; // FIFO is not completely full;
			static const uint8_t OVERWRITTEN = 0b1; // FIFO is full and at least one sample in the FIFO has been overwritten
		};
		/* Bits EMPTY_FIFO: */
		/* Empty FIFO bit status.  */
		struct EMPTY_FIFO
		{
			static const uint8_t dflt = 0b1; // 1'b1
			static const uint8_t mask = 0b00100000; // [5]
			static const uint8_t FIFO_NOT_EMPTY = 0b0; // FIFO not empty
			static const uint8_t FIFO_EMPTY = 0b1; // FIFO is empty
		};
		/* Bits FSS: */
		/*
		 * FIFO stored data level.
		 * 5'b00000: FIFO empty @ EMPTY_FIFO "1" or 1st sample stored in FIFO@EMPTY_FIFO "0";
		 * 5'b11111: FIFO is full and has 32 unread samples
		 */
		struct FSS
		{
			static const uint8_t dflt = 0b00000; // 5'b0
			static const uint8_t mask = 0b00011111; // [0,1,2,3,4]
			static const uint8_t FIFO_EMPTY = 0b000; // 
		};
	};
	
	/* Set register FIFO_STATUS */
	void setFIFO_STATUS(uint8_t value)
	{
		write(FIFO_STATUS::__address, value, 8);
	}
	
	/* Get register FIFO_STATUS */
	uint8_t getFIFO_STATUS()
	{
		return read8(FIFO_STATUS::__address, 8);
	}
	
	
	/****************************************************************************************************\
	 *                                                                                                  *
	 *                                            REG THS_P                                             *
	 *                                                                                                  *
	\****************************************************************************************************/
	
	/*
	 * REG THS_P:
	 * 8.20-21
	 * Threshold value for pressure interrupt generation.
	 * This register contains the threshold value for pressure interrupt generation.
	 * The threshold value for pressure interrupt generation is a 16-bit register composed
	 * of THS_P_H (31h) and THS_P_L (30h). The value is expressed as unsigned number:
	 * Interrupt threshold(hPA) = (THS_P)/16.
	 */
	struct THS_P
	{
		static const uint16_t __address = 48;
		
		/* Bits THS: */
		/* Refer to Section 10.2: "THS_P_L (0Ch)"  */
		struct THS
		{
			static const uint16_t mask = 0b1111111111111111; // [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
		};
	};
	
	/* Set register THS_P */
	void setTHS_P(uint16_t value)
	{
		write(THS_P::__address, value, 16);
	}
	
	/* Get register THS_P */
	uint16_t getTHS_P()
	{
		return read16(THS_P::__address, 16);
	}
	
	
	/*****************************************************************************************************\
	 *                                                                                                   *
	 *                                             REG RPDS                                              *
	 *                                                                                                   *
	\*****************************************************************************************************/
	
	/*
	 * REG RPDS:
	 * 8.22-23
	 * Pressure offset
	 */
	struct RPDS
	{
		static const uint16_t __address = 57;
		
		/* Bits RPDS: */
		struct RPDS_
		{
			/* Mode:rw */
			static const uint16_t mask = 0b1111111111111111; // [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
		};
	};
	
	/* Set register RPDS */
	void setRPDS(uint16_t value)
	{
		write(RPDS::__address, value, 16);
	}
	
	/* Get register RPDS */
	uint16_t getRPDS()
	{
		return read16(RPDS::__address, 16);
	}
	
};
