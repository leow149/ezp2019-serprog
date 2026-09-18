/*
 * serprog.h - Serial Flasher Protocol (serprog) command / constant definitions
 *
 * Sourced from ieiao/ch554_sdcc (serprog branch), examples/serprog/serprog.h
 * Protocol specification: https://www.flashrom.org/Serprog
 */

#ifndef __SERPROG_H__
#define __SERPROG_H__

/* ---- Wire-level byte values ---------------------------------------------- */
#define S_ACK               0x06    /* command accepted */
#define S_NAK               0x15    /* command rejected */

/* ---- Serprog commands ---------------------------------------------------- */
#define S_CMD_NOP           0x00    /* no-operation (used for sync) */
#define S_CMD_Q_IFACE       0x01    /* query interface version */
#define S_CMD_Q_CMDMAP      0x02    /* query supported command bitmap */
#define S_CMD_Q_PGMNAME     0x03    /* query programmer name (16 bytes, NUL-padded) */
#define S_CMD_Q_SERBUF      0x04    /* query serial buffer size (2 bytes, LE) */
#define S_CMD_Q_BUSTYPE     0x05    /* query supported bus types */
#define S_CMD_Q_CHIPSIZE    0x06    /* query chip size (not used here) */
#define S_CMD_Q_OPBUF       0x07    /* query operation buffer size (not used) */
#define S_CMD_Q_WRNMAXLEN   0x08    /* query max write length (<ACK><len:3>) */
#define S_CMD_R_BYTE        0x09    /* read single byte */
#define S_CMD_R_NBYTES      0x0A    /* read N bytes */
#define S_CMD_O_INIT        0x0B    /* initialize operation buffer */
#define S_CMD_O_WRITEB      0x0C    /* write byte to operation buffer */
#define S_CMD_O_WRITEN      0x0D    /* write N bytes to operation buffer */
#define S_CMD_O_DELAY       0x0E    /* delay (microseconds) */
#define S_CMD_O_EXEC        0x0F    /* execute operation buffer */
#define S_CMD_SYNCNOP       0x10    /* sync: host sends, expects NAK then ACK */
#define S_CMD_Q_RDNMAXLEN   0x11    /* query max read length  (<ACK><len:3>) */
#define S_CMD_S_BUSTYPE     0x12    /* set active bus type */
#define S_CMD_O_SPIOP       0x13    /* SPI operation: write then read */
#define S_CMD_S_SPI_FREQ    0x14    /* set SPI frequency */
#define S_CMD_S_PIN_STATE   0x15    /* set programmer pin state (not used) */

/* Custom extension (not part of the standard serprog set):
 * S_CMD_S_VCC - select socket supply rail. Args: <level:1>
 *   0x00 = 3.3V (default), 0x01 = 1.8V, 0x02 = 5V. Responds <ACK> or <NAK>.
 * The actual GPIO mapping is board specific - see VCC_* defines in main.c. */
#define S_CMD_S_VCC         0x1A
#define S_VCC_3V3           0x00
#define S_VCC_1V8           0x01
#define S_VCC_5V            0x02

/* ---- Bus type flags ------------------------------------------------------ */
#define S_BUSTYPE_PARALLEL  (1 << 0)
#define S_BUSTYPE_LPC       (1 << 1)
#define S_BUSTYPE_FWH       (1 << 2)
#define S_BUSTYPE_SPI       (1 << 3)

/* ---- Supported bus types for this firmware ------------------------------- */
#define SUPPORTED_BUS       S_BUSTYPE_SPI

/* ---- Capability bitmap (one bit per command 0..31) ----------------------- */
/* We support: NOP(0), Q_IFACE(1), Q_CMDMAP(2), Q_PGMNAME(3), Q_SERBUF(4),
 *             Q_BUSTYPE(5), Q_WRNMAXLEN(8), SYNCNOP(10h), Q_RDNMAXLEN(11h),
 *             S_BUSTYPE(12h), O_SPIOP(13h), S_SPI_FREQ(14h), S_VCC(1Ah).
 * The Q_CMDMAP reply is a single 32-bit LE bitmap (bytes 0..3, bits 0..31). */

#endif /* __SERPROG_H__ */
