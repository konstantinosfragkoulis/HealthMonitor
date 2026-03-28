/*
 * st1vafe6ax.h
 *
 *  Created on: Mar 28, 2026
 *      Author: konstantinos
 */

#ifndef INC_ST1VAFE6AX_H_
#define INC_ST1VAFE6AX_H_

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * SPI protocol
 * ========================================================================= */

/** Bit 7 of the first SPI byte selects read (1) vs write (0). */
#define ST1VAFE6AX_READ_BIT             (1U << 7)

/** Default SPI timeout for blocking transfers (milliseconds). */
#define ST1VAFE6AX_SPI_TIMEOUT_MS       100U

/* =========================================================================
 * Register addresses  (datasheet Table 21)
 * ========================================================================= */

#define ST1VAFE6AX_REG_INT1_CTRL        0x0DU   /**< INT1 pin control              */
#define ST1VAFE6AX_REG_INT2_CTRL        0x0EU   /**< INT2 pin control              */
#define ST1VAFE6AX_REG_WHO_AM_I         0x0FU   /**< Device identification (R)     */
#define ST1VAFE6AX_REG_CTRL1            0x10U   /**< Accelerometer control         */
#define ST1VAFE6AX_REG_CTRL2            0x11U   /**< Gyroscope control             */
#define ST1VAFE6AX_REG_CTRL3            0x12U   /**< Boot, BDU, IF_INC, SW_RESET   */
#define ST1VAFE6AX_REG_CTRL4            0x13U   /**< DRDY mode, interrupt routing  */
#define ST1VAFE6AX_REG_CTRL5            0x14U   /**< Bus activity selection        */
#define ST1VAFE6AX_REG_CTRL6            0x15U   /**< Gyro LPF + full-scale         */
#define ST1VAFE6AX_REG_CTRL7            0x16U   /**< AH_BIO control                */
#define ST1VAFE6AX_REG_CTRL8            0x17U   /**< Accel LPF + full-scale        */
#define ST1VAFE6AX_REG_CTRL9            0x18U   /**< Accel ultra-low-power         */
#define ST1VAFE6AX_REG_CTRL10           0x19U   /**< Timestamp, EMB_FUNC enable    */

/** First output register for linear acceleration (Z-axis low byte).
 *  Registers 0x28..0x2D hold Z_L, Z_H, Y_L, Y_H, X_L, X_H respectively. */
#define ST1VAFE6AX_REG_OUTZ_L_A         0x28U

/* =========================================================================
 * Bit-field definitions
 * ========================================================================= */

/* --- WHO_AM_I (0x0F) --- */
#define ST1VAFE6AX_WHO_AM_I_VALUE       0x71U   /**< Expected device ID            */

/* --- INT1_CTRL (0x0D) --- */
#define ST1VAFE6AX_INT1_DRDY_XL         (1U << 0)  /**< Accel data-ready on INT1   */

/* --- CTRL1 (0x10) — ODR_XL[3:0] in bits [3:0] --- */
#define ST1VAFE6AX_ODR_XL_POWER_DOWN    0x00U
#define ST1VAFE6AX_ODR_XL_1_875HZ       0x01U
#define ST1VAFE6AX_ODR_XL_7_5HZ         0x02U
#define ST1VAFE6AX_ODR_XL_15HZ          0x03U
#define ST1VAFE6AX_ODR_XL_30HZ          0x04U
#define ST1VAFE6AX_ODR_XL_60HZ          0x05U
#define ST1VAFE6AX_ODR_XL_120HZ         0x06U
#define ST1VAFE6AX_ODR_XL_240HZ         0x07U
#define ST1VAFE6AX_ODR_XL_480HZ         0x08U
#define ST1VAFE6AX_ODR_XL_960HZ         0x09U
#define ST1VAFE6AX_ODR_XL_1920HZ        0x0AU
#define ST1VAFE6AX_ODR_XL_3840HZ        0x0BU
#define ST1VAFE6AX_ODR_XL_7680HZ        0x0CU

/* --- CTRL4 (0x13) --- */
#define ST1VAFE6AX_CTRL4_DRDY_PULSED    (1U << 1)  /**< 75 us data-ready pulses    */

#ifdef __cplusplus
}
#endif

#endif /* INC_ST1VAFE6AX_H_ */
