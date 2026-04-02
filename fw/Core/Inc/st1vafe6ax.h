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

/* Bit 7 of the first SPI byte selects read (1) or write (0) */
#define ST1VAFE6AX_READ_BIT             (1U << 7)

#define ST1VAFE6AX_SPI_TIMEOUT_MS       100U

#define ST1VAFE6AX_REG_INT1_CTRL        0x0DU
#define ST1VAFE6AX_REG_INT2_CTRL        0x0EU
#define ST1VAFE6AX_REG_WHO_AM_I         0x0FU
#define ST1VAFE6AX_REG_CTRL1            0x10U
#define ST1VAFE6AX_REG_CTRL2            0x11U
#define ST1VAFE6AX_REG_CTRL3            0x12U
#define ST1VAFE6AX_REG_CTRL4            0x13U
#define ST1VAFE6AX_REG_CTRL5            0x14U
#define ST1VAFE6AX_REG_CTRL6            0x15U
#define ST1VAFE6AX_REG_CTRL7            0x16U
#define ST1VAFE6AX_REG_CTRL8            0x17U
#define ST1VAFE6AX_REG_CTRL9            0x18U
#define ST1VAFE6AX_REG_CTRL10           0x19U

/*
 * First output register for gyroscope (X-axis low byte).
 * Registers 0x22..0x27 hold X_L, X_H, Y_L, Y_H, Z_L, Z_H.
 * Immediately followed by accelerometer at 0x28..0x2D.
 */
#define ST1VAFE6AX_REG_OUTX_L_G         0x22U

/*
 * First output register for linear acceleration (Z-axis low byte).
 * Registers 0x28..0x2D hold Z_L, Z_H, Y_L, Y_H, X_L, X_H.
 */
#define ST1VAFE6AX_REG_OUTZ_L_A         0x28U

/* WHO_AM_I */
#define ST1VAFE6AX_WHO_AM_I_VALUE       0x71U

/* INT1_CTRL */
#define ST1VAFE6AX_INT1_DRDY_XL         (1U << 0)  /* Accel DRDY-ready on INT1 */
#define ST1VAFE6AX_INT1_DRDY_G          (1U << 1)  /* Gyro DRDY on INT1 */

/* CTRL1: ODR_XL[3:0] in bits [3:0], OP_MODE_XL[2:0] in bits [6:4] */
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

/* CTRL2: ODR_G[3:0] in bits [3:0], OP_MODE_G[2:0] in bits [6:4] */
#define ST1VAFE6AX_ODR_G_POWER_DOWN     0x00U
#define ST1VAFE6AX_ODR_G_7_5HZ          0x02U
#define ST1VAFE6AX_ODR_G_15HZ           0x03U
#define ST1VAFE6AX_ODR_G_30HZ           0x04U
#define ST1VAFE6AX_ODR_G_60HZ           0x05U
#define ST1VAFE6AX_ODR_G_120HZ          0x06U
#define ST1VAFE6AX_ODR_G_240HZ          0x07U
#define ST1VAFE6AX_ODR_G_480HZ          0x08U
#define ST1VAFE6AX_ODR_G_960HZ          0x09U
#define ST1VAFE6AX_ODR_G_1920HZ         0x0AU
#define ST1VAFE6AX_ODR_G_3840HZ         0x0BU
#define ST1VAFE6AX_ODR_G_7680HZ         0x0CU

/* CTRL6: FS_G[3:0] in bits [3:0], LPF1_G_BW[2:0] in bits [6:4] */
#define ST1VAFE6AX_FS_G_125DPS          0x00U 
#define ST1VAFE6AX_FS_G_250DPS          0x01U
#define ST1VAFE6AX_FS_G_500DPS          0x02U
#define ST1VAFE6AX_FS_G_1000DPS         0x03U
#define ST1VAFE6AX_FS_G_2000DPS         0x04U
#define ST1VAFE6AX_FS_G_4000DPS         0x0CU

/* CTRL4 */
#define ST1VAFE6AX_CTRL4_DRDY_PULSED    (1U << 1)  /* 75 us data-ready pulses */

#ifdef __cplusplus
}
#endif

#endif /* INC_ST1VAFE6AX_H_ */
