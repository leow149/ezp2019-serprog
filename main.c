/*****************************************************************************
 * main.c - serprog USB-CDC firmware for EZP2019+ / EZP2020 (CH552x)
 *
 * Derived from ieiao/ch554_sdcc (serprog branch), examples/serprog/main.c
 * Original: Copyright (C) ieiao, 2017  https://github.com/ieiao/ch554_sdcc
 * Upstream adaptation (EZP2020): iCE-HACK3R/EZP2020_CH552x-FW
 *
 * Hardware changes vs original ieiao firmware:
 *   CS_PIN : 0 -> 4  (chip-select on P1.4 instead of P1.0)
 *   PGMNAME: "ezp2019-serprog"
 *
 * Firmware hardening (this fork):
 *   - self-healing USB: EP2 IN/OUT state machine cannot wedge anymore when
 *     the host aborts a transfer (killed flashrom). Timeouts + endpoint
 *     re-arm make the device behave like a normal serial port.
 *   - SPI clock is capped (MAX_SPI_FREQ) so flashrom's 4 MHz negotiation
 *     cannot push the socket wiring into an unstable state.
 *   - USB device descriptor cleaned up: bcdUSB 2.00, and Vendor/Product/
 *     Serial strings identify the EZP2019+.
 *
 * Usage after flashing:
 *   Linux  : flashrom -p serprog:dev=/dev/ttyACM0:4000000,spispeed=1000000
 *   Windows: flashrom -p serprog:dev=COM3:4000000,spispeed=1000000
 *****************************************************************************/
#include <stdint.h>
#include <string.h>

#include <ch554.h>
#include <ch554_usb.h>
#include <spi.h>
#include <debug.h>
#include "serprog.h"

/* ---- Pin assignments ------------------------------------------------------ */
/* Green activity LED is on P1.1 on this EZP2019+ (traced by the user; the
 * stock firmware also toggles P1.1). Note: P3.4 in the stock image is only
 * the USB-SOF "connected" indicator, NOT the activity LED. */
#define LED_PIN  1
SBIT(LED, 0x90, LED_PIN);   /* P1.1 - green activity LED */

/* LED polarity. Confirmed by the power-on LED calibrator on the EZP2019+:
 * the green LED (P1.1) is ACTIVE-HIGH (lit with the pin high). Set
 * LED_ACTIVE_LOW=1 for boards that sink the LED from 3.3 V. */
#define LED_ACTIVE_LOW 0
#if LED_ACTIVE_LOW
#define LED_ON()  do { LED = 0; } while (0)
#define LED_OFF() do { LED = 1; } while (0)
#else
#define LED_ON()  do { LED = 1; } while (0)
#define LED_OFF() do { LED = 0; } while (0)
#endif

#define CS_PIN   4           /* EZP2020: CS on P1.4 */
SBIT(CS,  0x90, CS_PIN);    /* P1.4 - SPI flash chip-select */

/* ---- Serprog identity ----------------------------------------------------- */
#define PGMNAME "ezp2019-serprog"
#define CMD_MAP (\
    (1L << S_CMD_NOP)        | \
    (1L << S_CMD_Q_IFACE)    | \
    (1L << S_CMD_Q_CMDMAP)   | \
    (1L << S_CMD_Q_PGMNAME)  | \
    (1L << S_CMD_Q_SERBUF)   | \
    (1L << S_CMD_Q_BUSTYPE)  | \
    (1L << S_CMD_SYNCNOP)    | \
    (1L << S_CMD_S_BUSTYPE)  | \
    (1L << S_CMD_O_SPIOP)    | \
    (1L << S_CMD_S_SPI_FREQ) | \
    (1L << S_CMD_Q_WRNMAXLEN)| \
    (1L << S_CMD_Q_RDNMAXLEN)| \
    (1L << S_CMD_S_VCC)      \
)
#define BUS_SPI (1 << 3)

/* ---- Robustness limits ---------------------------------------------------- */
#define CMD_IDLE_TIMEOUT_MS  2000u  /* idle wait for the 1st command byte */
#define CMD_ABORT_TIMEOUT_MS  500u  /* mid-command wait before abort+reset */
#define MAX_SPI_FREQ        4000000u /* cap S_CMD_S_SPI_FREQ (12 MHz is silicon max but marginal on socket wiring) */
/* The fast path busy-polls long enough to cover a whole normal USB bulk
 * turnaround (~0.5 ms => ~200k iterations at ~3us/iter) so we never enter
 * the millisecond fallback during streaming. The fallback only guards the
 * "host is gone" abort case and is fine-grained (100 us) to avoid burning a
 * full ms per packet if we DO reach it. */
#define BUSY_POLL_CAP       2000000u
#define BUSY_FALLBACK_US    100u

/* ---- USB CDC request codes ------------------------------------------------ */
#define SET_LINE_CODING         0x20
#define GET_LINE_CODING         0x21
#define SET_CONTROL_LINE_STATE  0x22

/* ---- Endpoint buffers (XRAM) ---------------------------------------------- */
/* EP0: 64-byte control buffer at 0x0000 */
__xdata __at (0x0000) uint8_t Ep0Buffer[DEFAULT_ENDP0_SIZE];
/* EP1: 64-byte CDC notification buffer at 0x0040 */
__xdata __at (0x0040) uint8_t Ep1Buffer[DEFAULT_ENDP1_SIZE];
/* EP2: 128-byte single-buffer DMA region (2 x 64B) at 0x0080:
 *   [0..63]  = OUT,  [64..127] = IN */
__xdata __at (0x0080) uint8_t Ep2Buffer[2*MAX_PACKET_SIZE];

/* Software pipeline buffer: the SPI engine clocks the next packet into here
 * while the previous one is still being delivered over USB (see O_SPIOP). */
__xdata uint8_t PreBuf[64];

/* ---- Global state --------------------------------------------------------- */
uint16_t SetupLen;
uint8_t  SetupReq, UsbConfig;
const uint8_t *pDescr;

#define UsbSetupBuf  ((PUSB_SETUP_REQ)Ep0Buffer)

/* ---- USB Descriptors ------------------------------------------------------ */
/* Device descriptor */
__code uint8_t DevDesc[] = {
    0x12, 0x01,             /* bLength=18, bDescriptorType=Device */
    0x00, 0x02,             /* bcdUSB=2.00 (full-speed USB 2.0) */
    0x02, 0x00, 0x00,       /* bDeviceClass=CDC, SubClass=0, Protocol=0 */
    DEFAULT_ENDP0_SIZE,     /* bMaxPacketSize0=64 */
    0x86, 0x1a,             /* idVendor=0x1A86 (WCH) */
    0x22, 0x57,             /* idProduct=0x5722 (CH552 CDC) */
    0x00, 0x01,             /* bcdDevice=1.00 */
    0x01, 0x02, 0x03,       /* iManufacturer, iProduct, iSerialNumber */
    0x01                    /* bNumConfigurations=1 */
};

/* Configuration descriptor: CDC ACM with two interfaces */
__code uint8_t CfgDesc[] = {
    /* Configuration */
    0x09, 0x02, 0x43, 0x00, 0x02, 0x01, 0x00, 0xa0, 0x32,
    /* Interface 0 - CDC control */
    0x09, 0x04, 0x00, 0x00, 0x01, 0x02, 0x02, 0x01, 0x00,
    /* CDC header functional descriptor */
    0x05, 0x24, 0x00, 0x10, 0x01,
    /* CDC call management descriptor */
    0x05, 0x24, 0x01, 0x00, 0x00,
    /* CDC ACM functional descriptor */
    0x04, 0x24, 0x02, 0x02,
    /* CDC union functional descriptor */
    0x05, 0x24, 0x06, 0x00, 0x01,
    /* EP1 IN - interrupt (CDC notification) */
    0x07, 0x05, 0x81, 0x03, 0x08, 0x00, 0xFF,
    /* Interface 1 - CDC data */
    0x09, 0x04, 0x01, 0x00, 0x02, 0x0a, 0x00, 0x00, 0x00,
    /* EP2 OUT - bulk */
    0x07, 0x05, 0x02, 0x02, 0x40, 0x00, 0x00,
    /* EP2 IN  - bulk */
    0x07, 0x05, 0x82, 0x02, 0x40, 0x00, 0x00,
};

/* String descriptors */
__code uint8_t LangDes[]  = { 0x04, 0x03, 0x09, 0x04 };  /* English */
__code uint8_t SerDes[]   = {  /* Serial: "20260918" (build date) */
    0x12, 0x03,
    0x32,0x00, 0x30,0x00, 0x32,0x00, 0x36,0x00,
    0x30,0x00, 0x39,0x00, 0x31,0x00, 0x38,0x00,
};
__code uint8_t Prod_Des[] = {  /* Product: "EZP2019+ serprog" (UTF-16LE) */
    0x22, 0x03,
    0x45,0x00, 0x5A,0x00, 0x50,0x00, 0x32,0x00,
    0x30,0x00, 0x31,0x00, 0x39,0x00, 0x2B,0x00,
    0x20,0x00, 0x73,0x00, 0x65,0x00, 0x72,0x00,
    0x70,0x00, 0x72,0x00, 0x6F,0x00, 0x67,0x00,
};
__code uint8_t Manuf_Des[] = {  /* Manufacturer: "WCH" */
    0x08, 0x03,
    0x57,0x00, 0x43,0x00, 0x48,0x00,
};

/* CDC line-coding default: 57600 8N1 */
__xdata uint8_t LineCoding[7] = { 0x00, 0xe1, 0x00, 0x00, 0x00, 0x00, 0x08 };

/* ---- USB state flags (written from ISR, read from main loop) -------------- */
volatile __idata uint8_t USBByteCount  = 0;  /* bytes available in EP2 OUT */
volatile __idata uint8_t UpPoint2_Busy = 0;  /* EP2 IN transmit busy */


/* ---- USB device init ------------------------------------------------------ */
void USBDeviceCfg(void)
{
    USB_CTRL = 0x00;
    USB_CTRL &= ~bUC_HOST_MODE;
    USB_CTRL |= bUC_DEV_PU_EN | bUC_INT_BUSY | bUC_DMA_EN;
    USB_DEV_AD = 0x00;
    USB_CTRL &= ~bUC_LOW_SPEED;
    UDEV_CTRL &= ~bUD_LOW_SPEED;
    UDEV_CTRL = bUD_PD_DIS;   /* disable D+/D- pull-down */
    UDEV_CTRL |= bUD_PORT_EN; /* enable USB physical port */
}

void USBDeviceIntCfg(void)
{
    USB_INT_EN |= bUIE_SUSPEND;
    USB_INT_EN |= bUIE_TRANSFER;
    USB_INT_EN |= bUIE_BUS_RST;
    USB_INT_FG |= 0x1F;       /* clear all interrupt flags */
    IE_USB = 1;
    EA = 1;
}

void USBDeviceEndPointCfg(void)
{
    UEP1_DMA   = (uint16_t)Ep1Buffer;
    UEP2_DMA   = (uint16_t)Ep2Buffer;
    UEP2_3_MOD = 0xCC;  /* EP2+EP3: single-buffer RX+TX */
    UEP2_CTRL  = bUEP_AUTO_TOG | UEP_T_RES_NAK | UEP_R_RES_ACK;
    UEP1_CTRL  = bUEP_AUTO_TOG | UEP_T_RES_NAK;
    UEP0_DMA   = (uint16_t)Ep0Buffer;
    UEP4_1_MOD = 0x40;  /* EP1 TX buffer enabled */
    UEP0_CTRL  = UEP_R_RES_ACK | UEP_T_RES_NAK;
}

/* ---- USB ISR -------------------------------------------------------------- */
void DeviceInterrupt(void) __interrupt (INT_NO_USB)
{
    uint16_t len;
    if (UIF_TRANSFER)
    {
        switch (USB_INT_ST & (MASK_UIS_TOKEN | MASK_UIS_ENDP))
        {
        case UIS_TOKEN_IN | 1:
            UEP1_T_LEN = 0;
            UEP1_CTRL  = UEP1_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_NAK;
            break;

        case UIS_TOKEN_IN | 2:
            UEP2_T_LEN  = 0;
            UEP2_CTRL   = UEP2_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_NAK;
            UpPoint2_Busy = 0;
            break;

        case UIS_TOKEN_OUT | 2:
            if (U_TOG_OK)
            {
                USBByteCount = USB_RX_LEN;
                UEP2_CTRL = UEP2_CTRL & ~MASK_UEP_R_RES | UEP_R_RES_NAK;
            }
            break;

        case UIS_TOKEN_SETUP | 0:
            len = USB_RX_LEN;
            if (len == (sizeof(USB_SETUP_REQ)))
            {
                SetupLen = ((uint16_t)UsbSetupBuf->wLengthH << 8) |
                           UsbSetupBuf->wLengthL;
                len = 0;
                SetupReq = UsbSetupBuf->bRequest;
                if ((UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK) !=
                    USB_REQ_TYP_STANDARD)
                {
                    /* Non-standard (class) request */
                    switch (SetupReq)
                    {
                    case GET_LINE_CODING:
                        pDescr = LineCoding;
                        len = sizeof(LineCoding);
                        len = SetupLen >= DEFAULT_ENDP0_SIZE ?
                              DEFAULT_ENDP0_SIZE : SetupLen;
                        memcpy(Ep0Buffer, pDescr, len);
                        SetupLen -= len;
                        pDescr   += len;
                        break;
                    case SET_CONTROL_LINE_STATE:
                        break;
                    case SET_LINE_CODING:
                        break;
                    default:
                        len = 0xFF;
                        break;
                    }
                }
                else
                {
                    /* Standard request */
                    switch (SetupReq)
                    {
                    case USB_GET_DESCRIPTOR:
                        switch (UsbSetupBuf->wValueH)
                        {
                        case 1:
                            pDescr = DevDesc;
                            len    = sizeof(DevDesc);
                            break;
                        case 2:
                            pDescr = CfgDesc;
                            len    = sizeof(CfgDesc);
                            break;
                        case 3:
                            if      (UsbSetupBuf->wValueL == 0) { pDescr = LangDes;  len = sizeof(LangDes);  }
                            else if (UsbSetupBuf->wValueL == 1) { pDescr = Manuf_Des; len = sizeof(Manuf_Des);}
                            else if (UsbSetupBuf->wValueL == 2) { pDescr = Prod_Des; len = sizeof(Prod_Des); }
                            else                                { pDescr = SerDes;   len = sizeof(SerDes);   }
                            break;
                        default:
                            len = 0xFF;
                            break;
                        }
                        if (SetupLen > len) SetupLen = len;
                        len = SetupLen >= DEFAULT_ENDP0_SIZE ?
                              DEFAULT_ENDP0_SIZE : SetupLen;
                        memcpy(Ep0Buffer, pDescr, len);
                        SetupLen -= len;
                        pDescr   += len;
                        break;

                    case USB_SET_ADDRESS:
                        SetupLen = UsbSetupBuf->wValueL;
                        break;

                    case USB_GET_CONFIGURATION:
                        Ep0Buffer[0] = UsbConfig;
                        if (SetupLen >= 1) len = 1;
                        break;

                    case USB_SET_CONFIGURATION:
                        UsbConfig = UsbSetupBuf->wValueL;
                        break;

                    case USB_GET_INTERFACE:
                        break;

                    case USB_CLEAR_FEATURE:
                        if ((UsbSetupBuf->bRequestType & 0x1F) ==
                            USB_REQ_RECIP_DEVICE)
                        {
                            if (((uint16_t)UsbSetupBuf->wValueH << 8 |
                                 UsbSetupBuf->wValueL) == 0x01)
                            {
                                if (CfgDesc[7] & 0x20) { /* wakeup */ }
                                else                     { len = 0xFF; }
                            }
                            else { len = 0xFF; }
                        }
                        else if ((UsbSetupBuf->bRequestType &
                                  USB_REQ_RECIP_MASK) == USB_REQ_RECIP_ENDP)
                        {
                            switch (UsbSetupBuf->wIndexL)
                            {
                            case 0x83: UEP3_CTRL = UEP3_CTRL & ~(bUEP_T_TOG|MASK_UEP_T_RES) | UEP_T_RES_NAK; break;
                            case 0x03: UEP3_CTRL = UEP3_CTRL & ~(bUEP_R_TOG|MASK_UEP_R_RES) | UEP_R_RES_ACK; break;
                            case 0x82: UEP2_CTRL = UEP2_CTRL & ~(bUEP_T_TOG|MASK_UEP_T_RES) | UEP_T_RES_NAK; break;
                            case 0x02: UEP2_CTRL = UEP2_CTRL & ~(bUEP_R_TOG|MASK_UEP_R_RES) | UEP_R_RES_ACK; break;
                            case 0x81: UEP1_CTRL = UEP1_CTRL & ~(bUEP_T_TOG|MASK_UEP_T_RES) | UEP_T_RES_NAK; break;
                            case 0x01: UEP1_CTRL = UEP1_CTRL & ~(bUEP_R_TOG|MASK_UEP_R_RES) | UEP_R_RES_ACK; break;
                            default:   len = 0xFF; break;
                            }
                        }
                        else { len = 0xFF; }
                        break;

                    case USB_SET_FEATURE:
                        if ((UsbSetupBuf->bRequestType & 0x1F) ==
                            USB_REQ_RECIP_DEVICE)
                        {
                            if (((uint16_t)UsbSetupBuf->wValueH << 8 |
                                 UsbSetupBuf->wValueL) == 0x01)
                            {
                                if (CfgDesc[7] & 0x20)
                                {
                                    while (XBUS_AUX & bUART0_TX) { ; }
                                    SAFE_MOD = 0x55;
                                    SAFE_MOD = 0xAA;
                                    WAKE_CTRL = bWAK_BY_USB | bWAK_RXD0_LO | bWAK_RXD1_LO;
                                    PCON |= PD;
                                    SAFE_MOD = 0x55;
                                    SAFE_MOD = 0xAA;
                                    WAKE_CTRL = 0x00;
                                }
                                else { len = 0xFF; }
                            }
                            else { len = 0xFF; }
                        }
                        else if ((UsbSetupBuf->bRequestType & 0x1F) ==
                                 USB_REQ_RECIP_ENDP)
                        {
                            if (((uint16_t)UsbSetupBuf->wValueH << 8 |
                                 UsbSetupBuf->wValueL) == 0x00)
                            {
                                switch ((uint16_t)UsbSetupBuf->wIndexH << 8 |
                                        UsbSetupBuf->wIndexL)
                                {
                                case 0x83: UEP3_CTRL = UEP3_CTRL & ~bUEP_T_TOG | UEP_T_RES_STALL; break;
                                case 0x03: UEP3_CTRL = UEP3_CTRL & ~bUEP_R_TOG | UEP_R_RES_STALL; break;
                                case 0x82: UEP2_CTRL = UEP2_CTRL & ~bUEP_T_TOG | UEP_T_RES_STALL; break;
                                case 0x02: UEP2_CTRL = UEP2_CTRL & ~bUEP_R_TOG | UEP_R_RES_STALL; break;
                                case 0x81: UEP1_CTRL = UEP1_CTRL & ~bUEP_T_TOG | UEP_T_RES_STALL; break;
                                case 0x01: UEP1_CTRL = UEP1_CTRL & ~bUEP_R_TOG | UEP_R_RES_STALL; break;
                                default:   len = 0xFF;                                              break;
                                }
                            }
                            else { len = 0xFF; }
                        }
                        else { len = 0xFF; }
                        break;

                    case USB_GET_STATUS:
                        Ep0Buffer[0] = 0x00;
                        Ep0Buffer[1] = 0x00;
                        len = SetupLen >= 2 ? 2 : SetupLen;
                        break;

                    default:
                        len = 0xFF;
                        break;
                    }
                }
            }
            else { len = 0xFF; }

            if (len == 0xFF)
            {
                SetupReq = 0xFF;
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG |
                            UEP_R_RES_STALL | UEP_T_RES_STALL;
            }
            else if (len <= DEFAULT_ENDP0_SIZE)
            {
                UEP0_T_LEN = len;
                UEP0_CTRL  = bUEP_R_TOG | bUEP_T_TOG |
                             UEP_R_RES_ACK | UEP_T_RES_ACK;
            }
            else
            {
                UEP0_T_LEN = 0;
                UEP0_CTRL  = bUEP_R_TOG | bUEP_T_TOG |
                             UEP_R_RES_ACK | UEP_T_RES_ACK;
            }
            break;

        case UIS_TOKEN_IN | 0:
            switch (SetupReq)
            {
            case USB_GET_DESCRIPTOR:
                len = SetupLen >= DEFAULT_ENDP0_SIZE ?
                      DEFAULT_ENDP0_SIZE : SetupLen;
                memcpy(Ep0Buffer, pDescr, len);
                SetupLen -= len;
                pDescr   += len;
                UEP0_T_LEN = len;
                UEP0_CTRL ^= bUEP_T_TOG;
                break;
            case USB_SET_ADDRESS:
                USB_DEV_AD = USB_DEV_AD & bUDA_GP_BIT | SetupLen;
                UEP0_CTRL  = UEP_R_RES_ACK | UEP_T_RES_NAK;
                break;
            default:
                UEP0_T_LEN = 0;
                UEP0_CTRL  = UEP_R_RES_ACK | UEP_T_RES_NAK;
                break;
            }
            break;

        case UIS_TOKEN_OUT | 0:
            if (SetupReq == SET_LINE_CODING)
            {
                if (U_TOG_OK)
                {
                    memcpy(LineCoding, UsbSetupBuf, USB_RX_LEN);
                    UEP0_T_LEN = 0;
                    UEP0_CTRL |= UEP_R_RES_ACK | UEP_T_RES_ACK;
                }
            }
            else
            {
                UEP0_T_LEN = 0;
                UEP0_CTRL |= UEP_R_RES_ACK | UEP_T_RES_NAK;
            }
            break;

        default:
            break;
        }
        UIF_TRANSFER = 0;
    }

    if (UIF_BUS_RST)
    {
        UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
        UEP1_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK;
        UEP2_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK | UEP_R_RES_ACK;
        USB_DEV_AD    = 0x00;
        UIF_SUSPEND   = 0;
        UIF_TRANSFER  = 0;
        UIF_BUS_RST   = 0;
        USBByteCount  = 0;
        UsbConfig     = 0;
        UpPoint2_Busy = 0;
    }

    if (UIF_SUSPEND)
    {
        UIF_SUSPEND = 0;
        if (USB_MIS_ST & bUMS_SUSPEND)
        {
            while (XBUS_AUX & bUART0_TX) { ; }
            SAFE_MOD = 0x55;
            SAFE_MOD = 0xAA;
            WAKE_CTRL = bWAK_BY_USB | bWAK_RXD0_LO | bWAK_RXD1_LO;
            PCON |= PD;
            SAFE_MOD = 0x55;
            SAFE_MOD = 0xAA;
            WAKE_CTRL = 0x00;
        }
    }
    else
    {
        USB_INT_FG = 0xFF;
    }
}

/* ---- Serprog I/O ---------------------------------------------------------- */
__idata uint32_t slen;
__idata uint32_t rlen;
__idata int8_t   usb_send_length = 0;
__idata uint8_t  recv_index = 0;

void wdt_feed(void);   /* forward decl (defined below, used by recv path) */

static uint8_t reset_needed;   /* mid-command abort => full soft reset */
static uint8_t cmd_active;     /* set once a command byte has been received */

void usb_send(uint8_t length)
{
    UEP2_T_LEN = length;
    UEP2_CTRL  = UEP2_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_ACK;
    UpPoint2_Busy = 1;
}

/*
 * Read one byte from the EP2 OUT stream. Returns -1 on host-inactivity
 * timeout (CMD_IDLE_TIMEOUT_MS when idle, CMD_ABORT_TIMEOUT_MS mid-command),
 * so a dead host can never wedge the firmware.
 */
static int16_t recv_byte(void)
{
    uint32_t n = 0;
    uint16_t t = 0;
    uint16_t limit = cmd_active ? CMD_ABORT_TIMEOUT_MS : CMD_IDLE_TIMEOUT_MS;
    while (USBByteCount == 0)
    {
        if (n < BUSY_POLL_CAP) { n++; continue; }
        mDelayuS(BUSY_FALLBACK_US);          /* fine-grained fallback */
        wdt_feed();
        t += BUSY_FALLBACK_US / 100u;        /* 100us => 0.1ms steps */
        if (t >= (uint16_t)(limit * 10u)) return -1;
    }
    int16_t b = Ep2Buffer[recv_index];
    recv_index++;
    if (recv_index == USBByteCount)
    {
        recv_index   = 0;
        USBByteCount = 0;
        UEP2_CTRL = UEP2_CTRL & ~MASK_UEP_R_RES | UEP_R_RES_ACK;
    }
    return b;
}

static uint8_t wait_tx_done(void)
{
    uint32_t n = 0;
    uint16_t t = 0;
    while (UpPoint2_Busy)
    {
        if (n < BUSY_POLL_CAP) { n++; continue; }
        mDelayuS(BUSY_FALLBACK_US);
        wdt_feed();
        t += BUSY_FALLBACK_US / 100u;
        if (t >= (uint16_t)(CMD_ABORT_TIMEOUT_MS * 10u)) return 1;
    }
    return 0;
}

/*
 * Bring the EP2 state machine back to a clean idle state without breaking
 * the DATA0/DATA1 toggle tracking (AUTO_TOG and the toggle bits are kept).
 * If the interruption happened in the middle of a command and the host is
 * gone, we do a full software reset instead: the device re-enumerates and
 * the tty is created fresh, which also flushes any stale bytes the host had
 * buffered from the aborted transfer.
 */
void soft_reset(void)
{
    /* Deterministic self-reset: enable the watchdog and never feed it.
     * At 24 MHz the max window is ~0.7 s (WDOG_COUNT 0x00), so the chip
     * resets itself shortly. The watchdog was otherwise disabled, so this
     * does not affect normal operation. */
    SAFE_MOD = 0x55;
    SAFE_MOD = 0xAA;
    GLOBAL_CFG |= bWDOG_EN;
    SAFE_MOD = 0x00;
    WDOG_COUNT = 0x00;
    while (1);   /* never reaches here - watchdog resets us */
}

void cmd_abort(void)
{
    if (reset_needed)
    {
        reset_needed = 0;
        soft_reset();
    }
    UpPoint2_Busy = 0;
    USBByteCount  = 0;
    recv_index    = 0;
    UEP2_T_LEN    = 0;
    UEP2_CTRL     = (UEP2_CTRL & ~(MASK_UEP_T_RES | MASK_UEP_R_RES)) |
                    UEP_T_RES_NAK | UEP_R_RES_ACK;
    CS = 1;   /* release the socket chip-select if we aborted mid-O_SPIOP */
    LED_OFF();
}

/* ---- Watchdog -------------------------------------------------------------- */
/* The CH55x independent watchdog resets the chip if it is not fed in time.
 * At 24 MHz the max window is only ~0.7 s, which makes it easy to trip during
 * long sustained SPI/USB streaming. It is therefore DISABLED by default
 * (self-healing endpoint timeouts already cover host-side aborts). To enable,
 * set EZN_WATCHDOG to 1 - it is then fed on every byte/packet of the data
 * paths, and will only fire on a genuine hang (e.g. stuck SPI busy-wait). */
#define EZN_WATCHDOG 0

#define WDT_PERIOD 0x00

void wdt_init(void)
{
    SAFE_MOD = 0x55;
    SAFE_MOD = 0xAA;
    GLOBAL_CFG |= bWDOG_EN;
    SAFE_MOD = 0x00;
    WDOG_COUNT = WDT_PERIOD;
}

void wdt_feed(void)
{
    WDOG_COUNT = WDT_PERIOD;
}

#if !EZN_WATCHDOG
#undef wdt_init
#undef wdt_feed
#define wdt_init()
#define wdt_feed()
#endif

/* ---- LED status blinker ---------------------------------------------------- */
void led_blink(uint8_t times, uint16_t half_ms)
{
    uint8_t i;
    for (i = 0; i < times; i++)
    {
        LED_ON(); mDelaymS(half_ms);
        LED_OFF(); mDelaymS(half_ms);
    }
}

/* ---- Socket supply (VCC) selection ----------------------------------------- */
/* Stock-firmware analysis (the EZP2019+ OEM image):
 *   - the only GPIOs the OEM configures as outputs are P1.4..P1.7 (SPI bus) plus
 *     the LED (P1.1); there is NO voltage-select GPIO on this CH552G board.
 *   - "auto select device power voltage" is implemented in hardware, not by
 *     firmware, so there is nothing to drive.
 * Therefore S_CMD_S_VCC is kept as a protocol-level no-op that ACKs the
 * requested level (vcc_current) so a host can still ask; the rail itself is
 * handled by the board. If a future board adds a GPIO-controlled mux, define
 * VCC_ENABLE (+ port/pin macros below) to activate the driver.
 */
#define VCC_3V3 S_VCC_3V3
#define VCC_1V8 S_VCC_1V8
#define VCC_5V  S_VCC_5V
static uint8_t vcc_current = VCC_3V3;

#ifdef VCC_ENABLE
#define VCC_SEL_PORTP VCC_SEL_PORT
#define VCC_MASK ((1 << VCC_SEL0_BIT) | (1 << VCC_SEL1_BIT))
static uint8_t vcc_init_done = 0;
#endif

void vcc_set(uint8_t lev)
{
    vcc_current = lev;
#ifdef VCC_ENABLE
    if (lev > VCC_5V) lev = VCC_3V3;
    if (!vcc_init_done)
    {
        VCC_PORT_DIR_PU |= VCC_MASK;   /* outputs */
        VCC_PORT_MOD_OC &= ~VCC_MASK;  /* push-pull */
        vcc_init_done = 1;
    }
    switch (lev)
    {
    case VCC_1V8: VCC_SEL_PORTP = (VCC_SEL_PORTP & ~VCC_MASK) | (1 << VCC_SEL0_BIT); break; /* 01 */
    case VCC_5V:  VCC_SEL_PORTP = (VCC_SEL_PORTP & ~VCC_MASK) | (1 << VCC_SEL1_BIT); break; /* 10 */
    default:      VCC_SEL_PORTP &= ~VCC_MASK;                                              break; /* 00 = 3.3V */
    }
#endif
}

/* ---- Command handler ------------------------------------------------------ */
void handle_command(void)
{
    uint32_t i = 0;
    uint32_t remaining = 0;
    uint8_t  j = 0, k = 0;
    int16_t  rb, c;

#define READ_B(dst)  do { rb = recv_byte(); if (rb < 0) { reset_needed = 1; goto cmd_timeout; } (dst) = (uint8_t)rb; } while (0)
#define WAIT_TX()    do { if (wait_tx_done()) { reset_needed = 1; goto cmd_timeout; } } while (0)

    usb_send_length = 0;   /* reset for each command; O_SPIOP manages its own sends */
    cmd_active = 0;
    WAIT_TX();             /* let any previous IN packet drain before reuse */

    c = recv_byte();
    if (c < 0) goto cmd_timeout;   /* idle: no host data within timeout */
    cmd_active = 1;   /* now any timeout mid-command = abort + self-reset */

    /* Activity indicator: on only while a command is being processed, so the
     * LED is OFF when the host is merely connected but idle. */
    LED_ON();

    switch (c)
    {
    case S_CMD_NOP:
        Ep2Buffer[64 + 0] = S_ACK;
        usb_send_length = 1;
        break;

    case S_CMD_Q_WRNMAXLEN:
        Ep2Buffer[64 + 0] = S_ACK;
        Ep2Buffer[64 + 1] = 0xFF;
        Ep2Buffer[64 + 2] = 0xFF;
        Ep2Buffer[64 + 3] = 0xFF;
        usb_send_length = 4;
        break;

    case S_CMD_Q_RDNMAXLEN:
        Ep2Buffer[64 + 0] = S_ACK;
        Ep2Buffer[64 + 1] = 0xFF;
        Ep2Buffer[64 + 2] = 0xFF;
        Ep2Buffer[64 + 3] = 0xFF;
        usb_send_length = 4;
        break;

    case S_CMD_S_VCC:
        {
            uint8_t lv;
            READ_B(lv);
            if (lv <= VCC_5V)
            {
                vcc_set(lv);
                Ep2Buffer[64 + 0] = S_ACK;
            }
            else
            {
                Ep2Buffer[64 + 0] = S_NAK;
            }
            usb_send_length = 1;
        }
        break;

    case S_CMD_Q_IFACE:
        Ep2Buffer[64 + 0] = S_ACK;
        Ep2Buffer[64 + 1] = 0x01;
        Ep2Buffer[64 + 2] = 0x00;
        usb_send_length = 3;
        break;

    case S_CMD_Q_CMDMAP:
        /* bytes: NOP..Q_BUSTYPE(0x3F) | WRNMAXLEN | SYNCNOP+S_BUSTYPE+
         * O_SPIOP+S_SPI_FREQ+RDNMAXLEN | S_VCC */
        Ep2Buffer[64 + 0] = S_ACK;
        Ep2Buffer[64 + 1] = 0x3F;
        Ep2Buffer[64 + 2] = 0x01;
        Ep2Buffer[64 + 3] = 0x1F;
        Ep2Buffer[64 + 4] = 0x04;
        for (i = 5; i < 33; i++) Ep2Buffer[64 + i] = 0;
        usb_send_length = 33;
        break;

    case S_CMD_Q_PGMNAME:
        Ep2Buffer[64 + 0] = S_ACK;
        while (PGMNAME[i]) { Ep2Buffer[64 + i + 1] = PGMNAME[i]; i++; }
        for (; i < 16; i++) Ep2Buffer[64 + i + 1] = 0;
        usb_send_length = 17;
        break;

    case S_CMD_Q_SERBUF:
        Ep2Buffer[64 + 0] = S_ACK;
        Ep2Buffer[64 + 1] = 0xFF;
        Ep2Buffer[64 + 2] = 0xFF;
        usb_send_length = 3;
        break;

    case S_CMD_Q_BUSTYPE:
        Ep2Buffer[64 + 0] = S_ACK;
        Ep2Buffer[64 + 1] = BUS_SPI;
        usb_send_length = 2;
        break;

    case S_CMD_SYNCNOP:
        Ep2Buffer[64 + 0] = S_NAK;
        Ep2Buffer[64 + 1] = S_ACK;
        usb_send_length = 2;
        break;

    case S_CMD_S_BUSTYPE:
        {
            uint8_t bt;
            READ_B(bt);
            Ep2Buffer[64 + 0] = ((bt | BUS_SPI) == BUS_SPI) ? S_ACK : S_NAK;
            usb_send_length = 1;
        }
        break;

    case S_CMD_S_SPI_FREQ:
        {
            uint32_t freq, ck_se, actual;
            uint8_t  b0, b1, b2, b3;
            READ_B(b0);
            READ_B(b1);
            READ_B(b2);
            READ_B(b3);
            freq = (uint32_t)b0 | ((uint32_t)b1 << 8) |
                   ((uint32_t)b2 << 16) | ((uint32_t)b3 << 24);
            if (freq == 0) {
                Ep2Buffer[64 + 0] = S_NAK;
                usb_send_length = 1;
                break;
            }
            /* Cap the negotiated SPI clock: fly-lead/socket wiring cannot
             * reliably carry the 4 MHz flashrom asks for by default. */
            if (freq > MAX_SPI_FREQ) freq = MAX_SPI_FREQ;
            ck_se = (uint32_t)FREQ_SYS / (2UL * freq);
            if (ck_se < 1)   ck_se = 1;
            if (ck_se > 255) ck_se = 255;
            SPI0_CK_SE = (uint8_t)ck_se;
            actual = (uint32_t)FREQ_SYS / (2UL * (uint32_t)SPI0_CK_SE);
            Ep2Buffer[64 + 0] = S_ACK;
            Ep2Buffer[64 + 1] = (uint8_t)(actual);
            Ep2Buffer[64 + 2] = (uint8_t)(actual >>  8);
            Ep2Buffer[64 + 3] = (uint8_t)(actual >> 16);
            Ep2Buffer[64 + 4] = (uint8_t)(actual >> 24);
            usb_send_length = 5;
        }
        break;

    case S_CMD_O_SPIOP:
        /* Read slen and rlen (3 bytes each, little-endian) */
        {
            uint8_t b0, b1, b2, b3, b4, b5;
            READ_B(b0); READ_B(b1); READ_B(b2);
            slen = (uint32_t)b0 | ((uint32_t)b1 << 8) | ((uint32_t)b2 << 16);
            READ_B(b3); READ_B(b4); READ_B(b5);
            rlen = (uint32_t)b3 | ((uint32_t)b4 << 8) | ((uint32_t)b5 << 16);
        }

        CS = 0;
        mDelayuS(2);   /* CS setup time: let chip see CS low before first clock */

        /* Write phase */
        if (slen > 0)
        {
            for (i = 0; i < slen; i++)
            {
                uint8_t wb;
                wdt_feed();   /* long writes must keep the watchdog alive */
                READ_B(wb);
                CH554SPIMasterWrite(wb);
            }
        }

        /* Read phase:
         * For rlen == 0: just send ACK (1 byte).
         * For 1 <= rlen <= 63: send ACK + all data in one 64-byte packet.
         * For rlen >= 64: ACK as first byte of the first chunk, then 64-byte
         * chunks. The EP2 OUT endpoint stays armed the whole time (protocol
         * is half-duplex) so an aborted host never leaves the mcu stuck. */
        if (rlen == 0)
        {
            /* Write-only operation: just ACK. */
            Ep2Buffer[64 + 0] = S_ACK;
            usb_send(1);
            WAIT_TX();
        }
        else if (rlen < 64)
        {
            /* Small read: collect all data first, then send ACK+data together. */
            k = (uint8_t)rlen;
            Ep2Buffer[64] = S_ACK;
            for (j = 65; j < (uint8_t)(k + 65); j++)
            {
                SPI0_DATA = 0xFF;
                while (S0_FREE == 0);
                Ep2Buffer[j] = SPI0_DATA;
            }
            WAIT_TX();
            usb_send((uint8_t)(k + 1));
            WAIT_TX();
        }
        else
        {
            /* Large read (rlen >= 64), pipelined:
             * First packet: ACK + first 63 bytes of SPI data (64 bytes total).
             * While that packet is being delivered over USB, the SPI engine
             * already clocks the NEXT 64 bytes into PreBuf - so the SPI time
             * overlaps the USB transfer instead of adding to it.
             * Subsequent packets: 64 bytes each, then a final partial packet.
             * The watchdog is fed every packet (a multi-MB read can otherwise
             * outlive the idle-path feed window). */
            wdt_feed();
            Ep2Buffer[64] = S_ACK;
            for (j = 65; j < 128; j++)
            {
                SPI0_DATA = 0xFF;
                while (S0_FREE == 0);
                Ep2Buffer[j] = SPI0_DATA;

            }
            usb_send(64);   /* Busy=1; DMA now owns Ep2Buffer[64..127] */

            /* Remaining SPI bytes after first 63. */
            remaining = rlen - 63;

            /* Full 64-byte chunks, SPI clocked into PreBuf in parallel. */
            for (i = 0; i < (remaining / 64); i++)
            {
                for (j = 0; j < 64; j++)   /* overlap: clock while USB busy */
                {
                    SPI0_DATA = 0xFF;
                    while (S0_FREE == 0);
                    PreBuf[j] = SPI0_DATA;

                }
                WAIT_TX();  /* previous USB packet done, buffer free again */
                wdt_feed();
                for (j = 0; j < 64; j++) Ep2Buffer[64 + j] = PreBuf[j];
                usb_send(64);
            }

            /* Final partial chunk. */
            k = (uint8_t)(remaining % 64);
            if (k > 0)
            {
                for (j = 0; j < k; j++)
                {
                    SPI0_DATA = 0xFF;
                    while (S0_FREE == 0);
                    PreBuf[j] = SPI0_DATA;

                }
                WAIT_TX();
                wdt_feed();
                for (j = 0; j < k; j++) Ep2Buffer[64 + j] = PreBuf[j];
                usb_send(k);
            }

            WAIT_TX();
        }

        CS = 1;
        break;

    default:
        Ep2Buffer[64 + 0] = S_NAK;
        usb_send_length = 1;
        break;
    }

    if (usb_send_length > 0)
        usb_send(usb_send_length);
    return;

cmd_timeout:
    cmd_abort();
    return;
}

/* ---- main ----------------------------------------------------------------- */
void main(void)
{
    uint8_t blinks = 0, half_ms = 0;

    CfgFsys();          /* unlock SAFE_MOD and set 24 MHz (include/debug.c) */
    mDelaymS(5);

    /* LED (P1.1): push-pull output (green activity LED) */
    P1_DIR_PU |= (1 << LED_PIN);
    P1_MOD_OC &= ~(1 << LED_PIN);

    /* SPI pins (P1): MOSI/SCK/MISO/CS + LED as push-pull outputs;
     * MISO(P1.6) open-drain with pull-up (correct CH55x master input mode) */
    P1_MOD_OC = 0x00 | (1 << 6);
    P1_DIR_PU = 0;
    P1_DIR_PU |= (1 << CS_PIN) | (1 << 5) | (1 << 6) | (1 << 7) | (1 << LED_PIN);

    /* SPI0: master, mode 0, MSB first; default ~1.5 MHz (host can tune) */
    SPI0_SETUP = 0x00;
    SPI0_CTRL  = 0x60;   /* bS0_MOSI_OE | bS0_SCK_OE */
    SPI0_CK_SE = 0x08;
    CS = 1;

    /* Power-on self-test: JEDEC ID -> 3 fast blinks (chip) / 2 slow (none) */
    {
        uint8_t id0, id1, id2, bi;
        CS = 0;
        CH554SPIMasterWrite(0x9F);
        id0 = CH554SPIMasterRead();
        id1 = CH554SPIMasterRead();
        id2 = CH554SPIMasterRead();
        CS = 1;
        if (id0 == 0xFF && id1 == 0xFF && id2 == 0xFF) { blinks = 2; half_ms = 120; }
        else                                           { blinks = 3; half_ms =  40; }
        for (bi = 0; bi < blinks; bi++) { LED_ON(); mDelaymS(half_ms); LED_OFF(); mDelaymS(half_ms); }
    }

    USBDeviceCfg();
    USBDeviceEndPointCfg();
    USBDeviceIntCfg();
    UEP0_T_LEN = 0; UEP1_T_LEN = 0; UEP2_T_LEN = 0;

    /* blink while waiting for USB enumeration */
    while (!UsbConfig) { LED = !LED; mDelaymS(50); }
    LED_OFF();

    led_blink(1, 60);   /* acknowledge blink once the host configured us */

    vcc_set(VCC_3V3);   /* no-op on this board (VCC is hardware-managed) */

    wdt_init();         /* runtime watchdog off unless EZN_WATCHDOG=1 */

    while (1)
    {
        wdt_feed();
        handle_command();
        LED_OFF();
    }
}
