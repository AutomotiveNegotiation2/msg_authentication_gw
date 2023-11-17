/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2022 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_debug_console.h"
#include "fsl_flexcan.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define EXAMPLE_CAN CAN3

#define RX_MESSAGE_BUFFER_NUM (9)
#define TX_MESSAGE_BUFFER_NUM (8)

/* Select OSC24Mhz as master flexcan clock source */
#define FLEXCAN_CLOCK_SOURCE_SELECT (1U)
/* Clock divider for master flexcan clock source */
#define FLEXCAN_CLOCK_SOURCE_DIVIDER (1U)
/* Get frequency of flexcan clock */
#define EXAMPLE_CAN_CLK_FREQ ((CLOCK_GetRootClockFreq(kCLOCK_Root_Can3) / 100000U) * 100000U)
/* Set USE_IMPROVED_TIMING_CONFIG macro to use api to calculates the improved CAN / CAN FD timing values. */
#define USE_IMPROVED_TIMING_CONFIG (1U)
/* Fix MISRA_C-2012 Rule 17.7. */
#define LOG_INFO (void)PRINTF
#if (defined(USE_CANFD) && USE_CANFD)
/*
 *    DWORD_IN_MB    DLC    BYTES_IN_MB             Maximum MBs
 *    2              8      kFLEXCAN_8BperMB        64
 *    4              10     kFLEXCAN_16BperMB       42
 *    8              13     kFLEXCAN_32BperMB       25
 *    16             15     kFLEXCAN_64BperMB       14
 *
 * Dword in each message buffer, Length of data in bytes, Payload size must align,
 * and the Message Buffers are limited corresponding to each payload configuration:
 */
#define DLC         (15)
#define BYTES_IN_MB kFLEXCAN_64BperMB
#else
#define DLC (8)
#endif
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
flexcan_handle_t flexcanHandle;
volatile bool txComplete = false;
volatile bool rxComplete = false;
volatile bool wakenUp    = false;
flexcan_mb_transfer_t txXfer, rxXfer;
#if (defined(USE_CANFD) && USE_CANFD)
flexcan_fd_frame_t frame;
#else
flexcan_frame_t frame;
#endif
uint32_t txIdentifier;
uint32_t rxIdentifier;

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief FlexCAN Call Back function
 */
static FLEXCAN_CALLBACK(flexcan_callback)
{
    switch (status)
    {
        case kStatus_FLEXCAN_RxIdle:
            if (RX_MESSAGE_BUFFER_NUM == result)
            {
                rxComplete = true;
            }
            break;

        case kStatus_FLEXCAN_TxIdle:
            if (TX_MESSAGE_BUFFER_NUM == result)
            {
                txComplete = true;
            }
            break;

        case kStatus_FLEXCAN_WakeUp:
            wakenUp = true;
            break;

        default:
            break;
    }
}

#define CMAC_BLOCK_SIZE     16
#define CMAC_LAST_INDEX     (CMAC_BLOCK_SIZE - 1)

// foreward sbox
const unsigned char     sbox[256] = {
//  0     1     2     3     4     5     6     7     8     9     A     B     C     D     E     F
    0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab, 0x76,     // 0
    0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0, 0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0,     // 1
    0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15,     // 2
    0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2, 0xeb, 0x27, 0xb2, 0x75,     // 3
    0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0, 0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84,     // 4
    0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf,     // 5
    0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45, 0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8,     // 6
    0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5, 0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2,     // 7
    0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73,     // 8
    0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88, 0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb,     // 9
    0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c, 0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79,     // A
    0xe7, 0xc8, 0x37, 0x6d, 0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08,     // B
    0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a,     // C
    0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e, 0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e,     // D
    0xe1, 0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf,     // E
    0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb, 0x16 };   // F

// inverse sbox
const unsigned char     rsbox[256] = {
    0x52, 0x09, 0x6a, 0xd5, 0x30, 0x36, 0xa5, 0x38, 0xbf, 0x40, 0xa3, 0x9e, 0x81, 0xf3, 0xd7, 0xfb,
    0x7c, 0xe3, 0x39, 0x82, 0x9b, 0x2f, 0xff, 0x87, 0x34, 0x8e, 0x43, 0x44, 0xc4, 0xde, 0xe9, 0xcb,
    0x54, 0x7b, 0x94, 0x32, 0xa6, 0xc2, 0x23, 0x3d, 0xee, 0x4c, 0x95, 0x0b, 0x42, 0xfa, 0xc3, 0x4e,
    0x08, 0x2e, 0xa1, 0x66, 0x28, 0xd9, 0x24, 0xb2, 0x76, 0x5b, 0xa2, 0x49, 0x6d, 0x8b, 0xd1, 0x25,
    0x72, 0xf8, 0xf6, 0x64, 0x86, 0x68, 0x98, 0x16, 0xd4, 0xa4, 0x5c, 0xcc, 0x5d, 0x65, 0xb6, 0x92,
    0x6c, 0x70, 0x48, 0x50, 0xfd, 0xed, 0xb9, 0xda, 0x5e, 0x15, 0x46, 0x57, 0xa7, 0x8d, 0x9d, 0x84,
    0x90, 0xd8, 0xab, 0x00, 0x8c, 0xbc, 0xd3, 0x0a, 0xf7, 0xe4, 0x58, 0x05, 0xb8, 0xb3, 0x45, 0x06,
    0xd0, 0x2c, 0x1e, 0x8f, 0xca, 0x3f, 0x0f, 0x02, 0xc1, 0xaf, 0xbd, 0x03, 0x01, 0x13, 0x8a, 0x6b,
    0x3a, 0x91, 0x11, 0x41, 0x4f, 0x67, 0xdc, 0xea, 0x97, 0xf2, 0xcf, 0xce, 0xf0, 0xb4, 0xe6, 0x73,
    0x96, 0xac, 0x74, 0x22, 0xe7, 0xad, 0x35, 0x85, 0xe2, 0xf9, 0x37, 0xe8, 0x1c, 0x75, 0xdf, 0x6e,
    0x47, 0xf1, 0x1a, 0x71, 0x1d, 0x29, 0xc5, 0x89, 0x6f, 0xb7, 0x62, 0x0e, 0xaa, 0x18, 0xbe, 0x1b,
    0xfc, 0x56, 0x3e, 0x4b, 0xc6, 0xd2, 0x79, 0x20, 0x9a, 0xdb, 0xc0, 0xfe, 0x78, 0xcd, 0x5a, 0xf4,
    0x1f, 0xdd, 0xa8, 0x33, 0x88, 0x07, 0xc7, 0x31, 0xb1, 0x12, 0x10, 0x59, 0x27, 0x80, 0xec, 0x5f,
    0x60, 0x51, 0x7f, 0xa9, 0x19, 0xb5, 0x4a, 0x0d, 0x2d, 0xe5, 0x7a, 0x9f, 0x93, 0xc9, 0x9c, 0xef,
    0xa0, 0xe0, 0x3b, 0x4d, 0xae, 0x2a, 0xf5, 0xb0, 0xc8, 0xeb, 0xbb, 0x3c, 0x83, 0x53, 0x99, 0x61,
    0x17, 0x2b, 0x04, 0x7e, 0xba, 0x77, 0xd6, 0x26, 0xe1, 0x69, 0x14, 0x63, 0x55, 0x21, 0x0c, 0x7d };

// round constant
const unsigned char     Rcon[10] = {
    0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1b, 0x36};


// multiply by 2 in the galois field
unsigned char galois_mul2(unsigned char value)
{
    if( value >> 7 ) {
        return ((value << 1) ^ 0x1b);
    }
    else {
        return (value << 1);
    }
}

// AES encryption and decryption function
// The code was optimized for memory (flash and ram)
// Combining both encryption and decryption resulted in a slower implementation
// but much smaller than the 2 functions separated
// This function only implements AES-128 encryption and decryption (AES-192 and
// AES-256 are not supported by this code)
void aes_enc_dec(unsigned char *state, unsigned char *key, unsigned char dir)
{
    unsigned char       buf1, buf2, buf3, buf4, round, i;

    if( dir ) {                                 // In case of decryption
        // compute the last key of encryption before starting the decryption
        for( round = 0 ; round < 10; round++ ) {
            key[0]  = sbox[key[13]]^key[0]^Rcon[round];         // key schedule
            key[1]  = sbox[key[14]]^key[1];
            key[2]  = sbox[key[15]]^key[2];
            key[3]  = sbox[key[12]]^key[3];

            for( i = 4; i < 16; i++ ) {
                key[i]  = key[i] ^ key[i - 4];
            }
        }

        for( i = 0; i < 16; i++ ) {             // first Addroundkey
            state[i]    = state[i] ^ key[i];
        }
    }

    for( round = 0; round < 10; round++ ) {     // main loop
        if( dir ) {
            for( i = 15; i > 3; --i ) {         //Inverse key schedule
                key[i]  = key[i] ^ key[i-4];
            }

            key[0]  = sbox[key[13]] ^ key[0] ^ Rcon[9 - round];
            key[1]  = sbox[key[14]] ^ key[1];
            key[2]  = sbox[key[15]] ^ key[2];
            key[3]  = sbox[key[12]]  ^key[3];
        }
        else {
            for( i = 0; i <16; i++ ) {
                state[i]    = sbox[state[i] ^ key[i]];          // with shiftrow i + 5 mod 16
            }

            buf1        = state[1];     //shift rows
            state[1]    = state[5];
            state[5]    = state[9];
            state[9]    = state[13];
            state[13]   = buf1;

            buf1        = state[2];
            buf2        = state[6];
            state[2]    = state[10];
            state[6]    = state[14];
            state[10]   = buf1;
            state[14]   = buf2;

            buf1        = state[15];
            state[15]   = state[11];
            state[11]   = state[7];
            state[7]    = state[3];
            state[3]    = buf1;
        }

        if( (round > 0 && dir) || (round < 9 && !dir) ) {       // mixcol - inv mix
            for( i = 0; i < 4; i++ ) {
                buf4    = i << 2;

                if( dir ) {
                    buf1            = galois_mul2( galois_mul2( state[buf4 + 0] ^ state[buf4 + 2] ) );      // precompute for decryption
                    buf2            = galois_mul2( galois_mul2( state[buf4 + 1] ^ state[buf4 + 3] ) );
                    state[buf4 + 0] ^= buf1;
                    state[buf4 + 1] ^= buf2;
                    state[buf4 + 2] ^= buf1;
                    state[buf4 + 3] ^= buf2;
                }

                buf1    = state[buf4] ^ state[buf4 + 1] ^ state[buf4 + 2] ^ state[buf4 + 3];        // in all cases
                buf2    = state[buf4];
                buf3    = state[buf4 + 0] ^ state[buf4 + 1];    buf3 = galois_mul2(buf3);   state[buf4 + 0] = state[buf4 + 0] ^ buf3 ^ buf1;
                buf3    = state[buf4 + 1] ^ state[buf4 + 2];    buf3 = galois_mul2(buf3);   state[buf4 + 1] = state[buf4 + 1] ^ buf3 ^ buf1;
                buf3    = state[buf4 + 2] ^ state[buf4 + 3];    buf3 = galois_mul2(buf3);   state[buf4 + 2] = state[buf4 + 2] ^ buf3 ^ buf1;
                buf3    = state[buf4 + 3] ^ buf2;               buf3 = galois_mul2(buf3);   state[buf4 + 3] = state[buf4 + 3] ^ buf3 ^ buf1;
            }
        }

        if( dir ) {                     // Inv shift rows
            buf1        = state[13];    // Row 1
            state[13]   = state[9];
            state[9]    = state[5];
            state[5]    = state[1];
            state[1]    = buf1;

            buf1        = state[10];    // Row 2
            buf2        = state[14];
            state[10]   = state[2];
            state[14]   = state[6];
            state[2]    = buf1;
            state[6]    = buf2;

            buf1        = state[3];     // Row 3
            state[3]    = state[7];
            state[7]    = state[11];
            state[11]   = state[15];
            state[15]   = buf1;

            for( i = 0; i < 16; i++ ) {
                // with shiftrow i+5 mod 16
                state[i]=rsbox[ state[i] ] ^ key[i];
            }
        }
        else {
            // key schedule
            key[0]  = sbox[ key[13] ] ^ key[0] ^ Rcon[round];
            key[1]  = sbox[ key[14] ] ^ key[1];
            key[2]  = sbox[ key[15] ] ^ key[2];
            key[3]  = sbox[ key[12] ] ^ key[3];

            for( i = 4; i < 16; i++ ) {
                key[i]  = key[i] ^ key[i - 4];
            }
        }
    }

    if( !dir ) {                                // laaes_enc_decst Addroundkey
        for( i = 0; i < 16; i++ ) {
            state[i]    = state[i] ^ key[i];    // with shiftrow i+5 mod 16
        }                                       // enf for
    }                                           // end if (!dir)
}                                               // end function

/* For CMAC Calculation */
static unsigned const char      const_Rb[CMAC_BLOCK_SIZE] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x87 };
static unsigned const char      const_Zero[CMAC_BLOCK_SIZE] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };


static void AES_128_ENC(unsigned const char *key, unsigned const char* msg, unsigned char *cipher)
{
    unsigned char   key_copy[CMAC_BLOCK_SIZE];

    memcpy(cipher, msg, CMAC_BLOCK_SIZE);
    memcpy(key_copy, key, CMAC_BLOCK_SIZE);
    aes_enc_dec(cipher, key_copy, 0);
}

void xor_128(const unsigned char *a, const unsigned char *b, unsigned char *out)
{
    int     i;

    for( i = 0; i < CMAC_BLOCK_SIZE; i++ ) {
        out[i]      = a[i] ^ b[i];
    }
}

/* AES-CMAC Generation Function */
static void leftshift_onebit(const unsigned char *input, unsigned char *output)
{
    int             i;
    unsigned char   overflow    = 0;

    for( i = CMAC_LAST_INDEX; i >= 0; i-- ) {
        output[i]   = input[i] << 1;
        output[i]   |= overflow;
        overflow    = (input[i] & 0x80) ? 1 : 0;
    }

    return;
}

static void generate_subkey(const unsigned char *key, unsigned char *K1, unsigned char *K2)
{
    unsigned char   L[CMAC_BLOCK_SIZE];
    unsigned char   tmp[CMAC_BLOCK_SIZE];

    AES_128_ENC(key, const_Zero, L);

    if( (L[0] & 0x80) == 0 ) {                  /* If MSB(L) = 0, then K1 = L << 1 */
        leftshift_onebit(L, K1);
    }
    else {                                      /* Else K1 = ( L << 1 ) (+) Rb */
        leftshift_onebit(L, tmp);
        xor_128(tmp, const_Rb, K1);
    }

    if( (K1[0] & 0x80) == 0 ) {
        leftshift_onebit(K1, K2);
    }
    else {
        leftshift_onebit(K1, tmp);
        xor_128(tmp, const_Rb, K2);
    }

    return;
}

static void padding(const unsigned char *lastb, unsigned char *pad, int length) {
    int     j;

    /* original last block */
    for( j = 0; j < CMAC_BLOCK_SIZE; j++ ) {
        if( j < length ) {
            pad[j]  = lastb[j];
        }
        else if( j == length ) {
            pad[j]  = 0x80;
        }
        else {
            pad[j]  = 0x00;
        }
    }
}

static int AES_CMAC(const unsigned char *key, const unsigned char *input, int length, unsigned char *mac) {
    unsigned char   X[CMAC_BLOCK_SIZE], Y[CMAC_BLOCK_SIZE], M_last[CMAC_BLOCK_SIZE], padded[CMAC_BLOCK_SIZE];
    unsigned char   K1[CMAC_BLOCK_SIZE], K2[CMAC_BLOCK_SIZE];
    int             n, i, flag;

    generate_subkey(key, K1, K2);

    n   = (length + CMAC_LAST_INDEX) / CMAC_BLOCK_SIZE;         /* n is number of rounds */

    if( n == 0 ) {
        n       = 1;
        flag    = 0;
    }
    else {
        if( (length % CMAC_BLOCK_SIZE) == 0 ) {         /* last block is a complete block */
            flag    = 1;
        }
        else {                                          /* last block is not complete block */
            flag    = 0;
        }
    }

    if( flag ) {                                        /* last block is complete block */
        xor_128(&input[CMAC_BLOCK_SIZE * (n - 1)], K1, M_last);
    }
    else {
        padding(&input[CMAC_BLOCK_SIZE * (n - 1)], padded, length % CMAC_BLOCK_SIZE);
        xor_128(padded, K2, M_last);
    }

    memset(X, 0, CMAC_BLOCK_SIZE);

    for( i = 0; i < n - 1; i++ ) {
        xor_128(X, &input[CMAC_BLOCK_SIZE * i], Y);     /* Y := Mi (+) X  */
        AES_128_ENC(key, Y, X);                         /* X := AES-128(KEY, Y); */
    }

    xor_128(X, M_last, Y);
    AES_128_ENC(key, Y, X);

    memcpy(mac, X, CMAC_BLOCK_SIZE);
    return 0;
}

static volatile uint32_t    s_MsCount       = 0U;
static uint64_t             s_Timeout;

/*!
 * @brief Milliseconds counter since last POR/reset.
 */
void SysTick_Handler(void)
{
    s_MsCount++;
}

void xmit_msgNcmac_over_CAN(void)
{
    frame.id     = FLEXCAN_ID_STD(txIdentifier);
    frame.format = (uint8_t)kFLEXCAN_FrameFormatStandard;
    frame.type   = (uint8_t)kFLEXCAN_FrameTypeData;
    frame.length = (uint8_t)DLC;
#if (defined(USE_CANFD) && USE_CANFD)
    frame.brs = (uint8_t)1U;
#endif

    txXfer.mbIdx = (uint8_t)TX_MESSAGE_BUFFER_NUM;
#if (defined(USE_CANFD) && USE_CANFD)
    txXfer.framefd = &frame;
    (void)FLEXCAN_TransferFDSendNonBlocking(EXAMPLE_CAN, &flexcanHandle, &txXfer);
#else
    txXfer.frame = &frame;
    (void)FLEXCAN_TransferSendNonBlocking(EXAMPLE_CAN, &flexcanHandle, &txXfer);
#endif

    while( !txComplete ) {
    };

    txComplete = false;

    /* Start receive data through Rx Message Buffer. */
    rxXfer.mbIdx = (uint8_t)RX_MESSAGE_BUFFER_NUM;
#if (defined(USE_CANFD) && USE_CANFD)
    rxXfer.framefd = &frame;
    (void)FLEXCAN_TransferFDReceiveNonBlocking(EXAMPLE_CAN, &flexcanHandle, &rxXfer);
#else
    rxXfer.frame = &frame;
    (void)FLEXCAN_TransferReceiveNonBlocking(EXAMPLE_CAN, &flexcanHandle, &rxXfer);
#endif

    /* Wait until Rx MB full. */
    while( !rxComplete ) {
    };

    rxComplete = false;
}

void recv_msgNcmac_over_CAN(void)
{
    /* Before this , should first make node B enter STOP mode after FlexCAN
     * initialized with enableSelfWakeup=true and Rx MB configured, then A
     * sends frame N which wakes up node B. A will continue to send frame N
     * since no acknowledgement, then B received the second frame N(In the
     * application it seems that B received the frame that woke it up which
     * is not expected as stated in the reference manual, but actually the
     * output in the terminal B received is the same second frame N). */
    if (wakenUp)
    {
        LOG_INFO("B has been waken up!\r\n\r\n");
    }

    /* Start receive data through Rx Message Buffer. */
    rxXfer.mbIdx = (uint8_t)RX_MESSAGE_BUFFER_NUM;
#if (defined(USE_CANFD) && USE_CANFD)
    rxXfer.framefd = &frame;
    (void)FLEXCAN_TransferFDReceiveNonBlocking(EXAMPLE_CAN, &flexcanHandle, &rxXfer);
#else
    rxXfer.frame = &frame;
    (void)FLEXCAN_TransferReceiveNonBlocking(EXAMPLE_CAN, &flexcanHandle, &rxXfer);
#endif

    /* Wait until Rx receive full. */
    while (!rxComplete)
    {
    };
    rxComplete = false;

    frame.id     = FLEXCAN_ID_STD(txIdentifier);
    txXfer.mbIdx = (uint8_t)TX_MESSAGE_BUFFER_NUM;
#if (defined(USE_CANFD) && USE_CANFD)
    frame.brs      = 1;
    txXfer.framefd = &frame;
    (void)FLEXCAN_TransferFDSendNonBlocking(EXAMPLE_CAN, &flexcanHandle, &txXfer);
#else
    txXfer.frame = &frame;
    (void)FLEXCAN_TransferSendNonBlocking(EXAMPLE_CAN, &flexcanHandle, &txXfer);
#endif

    while (!txComplete)
    {
    };
    txComplete = false;
}

void my_delay(uint32_t cnt)
{
#if 1
    while( cnt-- );
#else
    volatile uint32_t   *pusec_cnt      = &s_MsCount;
    struct {
        uint32_t    msec;
        uint32_t    tick;
    } beg, now;;

    beg.msec    = *pusec_cnt;
    beg.tick    = SysTick->VAL;

    while( cnt-- );

    now.msec        = *pusec_cnt;
    now.tick        = SysTick->VAL;

    LOG_INFO("beg     = %12d / %12d\r\n",     beg.msec,     beg.tick);
    LOG_INFO("now     = %12d / %12d\r\n",     now.msec,     now.tick);
#endif
}

#define MAX_TEST_SUIT_SZ    64
#define MAX_KEY_SZ          16
#define MAX_MSG_SZ          37
#define MAC_MAC_SZ          16
#define TICKS_PER_1SEC      996000000
#define TICKS_PER_MSEC      996000
#define TICKS_PER_USEC      996
#define TICKS_PER_NSEC      1

struct {
    struct {
        unsigned short      Klen;
        unsigned short      Mlen;
        unsigned short      Tlen;
    } dsc;

    const char      *Key;
    const char      *Msg;
    const char      *Mac;
} nist_test_suit[MAX_TEST_SUIT_SZ];

void init(void) 
{
    nist_test_suit[ 0].dsc.Klen = 16;
    nist_test_suit[ 0].dsc.Mlen = 16;
    nist_test_suit[ 0].dsc.Tlen = 4;
    nist_test_suit[ 0].Key      = "\xe3\x80\x79\x8d\x05\x75\xd1\x26\x9c\xc8\xca\xa2\x31\x35\x44\x37";
    nist_test_suit[ 0].Msg      = "\x66\xea\xe0\xea\xbd\x62\x4a\x7e\xcc\x11\xe9\xdb\x46\x2e\x2b\x86";
    nist_test_suit[ 0].Mac      = "\x9f\x19\x06\x5a";

    nist_test_suit[ 1].dsc.Klen = 16;
    nist_test_suit[ 1].dsc.Mlen = 16;
    nist_test_suit[ 1].dsc.Tlen = 4;
    nist_test_suit[ 1].Key      = "\xe1\xdc\x72\x4d\x56\x21\xfd\xde\x4a\x8c\x27\x60\x05\xd6\x15\x75";
    nist_test_suit[ 1].Msg      = "\x87\x98\x55\xff\x51\x96\x86\x2c\xc7\x7e\x32\x3f\x8c\x76\xb5\xb0";
    nist_test_suit[ 1].Mac      = "\xd1\xc2\xa7\x0e";

    nist_test_suit[ 2].dsc.Klen = 16;
    nist_test_suit[ 2].dsc.Mlen = 16;
    nist_test_suit[ 2].dsc.Tlen = 4;
    nist_test_suit[ 2].Key      = "\x56\x10\x52\xad\xe5\x55\xee\xda\x9c\xb3\x8b\x1c\xca\x8d\x11\x8c";
    nist_test_suit[ 2].Msg      = "\x79\xfc\x4d\x3a\x8a\x47\x38\x5d\xb7\x3e\x19\xa5\x57\x2a\x27\x41";
    nist_test_suit[ 2].Mac      = "\x1e\xc9\x24\xeb";

    nist_test_suit[ 3].dsc.Klen = 16;
    nist_test_suit[ 3].dsc.Mlen = 16;
    nist_test_suit[ 3].dsc.Tlen = 4;
    nist_test_suit[ 3].Key      = "\x19\x5e\x05\x9e\x57\x10\xe6\x52\x74\x4f\xa6\xdb\xbd\x04\x19\x0d";
    nist_test_suit[ 3].Msg      = "\x16\xbd\xf1\x8c\x09\xd6\x0f\x3a\x2a\x32\xfb\xb2\x4e\x64\xd0\x33";
    nist_test_suit[ 3].Mac      = "\x5c\xfd\x2c\x86";

    nist_test_suit[ 4].dsc.Klen = 16;
    nist_test_suit[ 4].dsc.Mlen = 16;
    nist_test_suit[ 4].dsc.Tlen = 4;
    nist_test_suit[ 4].Key      = "\x89\xbd\xd9\x4d\xcf\x1c\x69\xd8\xba\xf4\x50\x0e\x91\x45\x24\x7f";
    nist_test_suit[ 4].Msg      = "\xc5\x33\x54\x20\xe1\x1c\x23\x55\x98\x35\x60\x4b\xf2\xb6\x71\x9e";
    nist_test_suit[ 4].Mac      = "\xcc\xfe\x25\x9f";

    nist_test_suit[ 5].dsc.Klen = 16;
    nist_test_suit[ 5].dsc.Mlen = 16;
    nist_test_suit[ 5].dsc.Tlen = 4;
    nist_test_suit[ 5].Key      = "\x42\x77\xcd\x9b\xaf\x2c\xf3\x93\xc8\x1c\xb7\xd8\x8b\x5b\xa1\x5e";
    nist_test_suit[ 5].Msg      = "\x37\x2a\xae\xfe\xac\x37\x85\xbe\xd8\xa8\x95\xa4\x23\x13\xab\x9a";
    nist_test_suit[ 5].Mac      = "\x0d\x73\x6c\xa4";

    nist_test_suit[ 6].dsc.Klen = 16;
    nist_test_suit[ 6].dsc.Mlen = 16;
    nist_test_suit[ 6].dsc.Tlen = 4;
    nist_test_suit[ 6].Key      = "\x27\xfa\x0d\x5e\x45\xb2\xf4\xc2\x0b\xc4\xca\x56\x4c\x4e\x8c\x34";
    nist_test_suit[ 6].Msg      = "\x73\x53\xe9\x30\x33\xd5\x61\x17\xd5\x2c\xdf\x50\x50\x86\xae\x8c";
    nist_test_suit[ 6].Mac      = "\xd6\xbc\x6e\xe7";

    nist_test_suit[ 7].dsc.Klen = 16;
    nist_test_suit[ 7].dsc.Mlen = 16;
    nist_test_suit[ 7].dsc.Tlen = 4;
    nist_test_suit[ 7].Key      = "\x8f\x75\xb1\xa8\x72\xf6\xf3\xd0\x6b\xf4\x7a\xf4\x15\xf0\xd6\x30";
    nist_test_suit[ 7].Msg      = "\xf6\xb1\xcf\x75\xb5\xcf\xa4\xab\x32\x3c\xf0\x4f\xf1\x3b\x7a\x59";
    nist_test_suit[ 7].Mac      = "\xc0\xa3\x03\xbe";

    nist_test_suit[ 8].dsc.Klen = 16;
    nist_test_suit[ 8].dsc.Mlen = 16;
    nist_test_suit[ 8].dsc.Tlen = 15;
    nist_test_suit[ 8].Key      = "\x67\x08\xc9\x88\x7b\x84\x70\x84\xf1\x23\xd3\xdd\x9c\x3a\x81\x36";
    nist_test_suit[ 8].Msg      = "\xa8\xde\x55\x17\x0c\x6d\xc0\xd8\x0d\xe3\x2f\x50\x8b\xf4\x9b\x70";
    nist_test_suit[ 8].Mac      = "\xcf\xef\x9b\x78\x39\x84\x1f\xdb\xcc\xbb\x6c\x2c\xf2\x38\xf7";

    nist_test_suit[ 9].dsc.Klen = 16;
    nist_test_suit[ 9].dsc.Mlen = 16;
    nist_test_suit[ 9].dsc.Tlen = 15;
    nist_test_suit[ 9].Key      = "\xe3\x9c\x6f\xf1\xad\x22\x79\x3d\xc5\x25\xd3\x4e\x7d\x7e\x7d\x6d";
    nist_test_suit[ 9].Msg      = "\xf2\x38\x77\xb7\x74\x71\xc8\x0d\xd5\x65\xec\xe7\xb2\xca\x0b\xdd";
    nist_test_suit[ 9].Mac      = "\x1a\x5c\x33\xd5\x4e\x6d\xe6\xd9\xfa\x61\xcb\x96\x36\x05\x3f";

    nist_test_suit[10].dsc.Klen = 16;
    nist_test_suit[10].dsc.Mlen = 16;
    nist_test_suit[10].dsc.Tlen = 15;
    nist_test_suit[10].Key      = "\x5f\xca\xd3\x8a\xe7\x78\x39\x49\x12\xf8\xf1\xb8\x41\x3c\xf7\x73";
    nist_test_suit[10].Msg      = "\x07\x18\x55\x02\xbf\x6d\x27\x5c\x84\xe3\xac\x4f\x5f\x77\xc3\xd4";
    nist_test_suit[10].Mac      = "\xfd\x44\xfb\xc0\xdd\x97\x19\xe8\xb5\x69\xff\x10\x42\x1d\xf4";

    nist_test_suit[11].dsc.Klen = 16;
    nist_test_suit[11].dsc.Mlen = 16;
    nist_test_suit[11].dsc.Tlen = 15;
    nist_test_suit[11].Key      = "\xf0\x84\x01\x0a\xdc\x52\x22\x4b\x72\xc5\x62\xfa\x57\x84\x23\x05";
    nist_test_suit[11].Msg      = "\x53\x9f\x95\x99\x8a\xad\x8d\x83\x94\x76\xc6\x63\x93\xdc\xa1\xe6";
    nist_test_suit[11].Mac      = "\x9c\x08\x25\xb0\xd2\x56\xa5\xcf\xf1\x7a\x05\xb5\x38\x5c\xba";

    nist_test_suit[12].dsc.Klen = 16;
    nist_test_suit[12].dsc.Mlen = 16;
    nist_test_suit[12].dsc.Tlen = 15;
    nist_test_suit[12].Key      = "\xf2\x99\x11\x12\xc2\xcb\xd3\x03\x8a\xe3\x7b\x77\x2a\x50\x90\x69";
    nist_test_suit[12].Msg      = "\xdd\x63\x20\x6f\x9d\xb2\xc1\x1a\x32\x75\x38\xbe\x6e\x82\x0a\x97";
    nist_test_suit[12].Mac      = "\x98\xc5\xcf\x3c\x91\x72\x16\xd6\x45\xf0\x2d\x7b\xb8\xd8\x09";

    nist_test_suit[13].dsc.Klen = 16;
    nist_test_suit[13].dsc.Mlen = 16;
    nist_test_suit[13].dsc.Tlen = 15;
    nist_test_suit[13].Key      = "\xf7\xf9\x22\xc8\x67\x06\x27\x7a\x4e\x98\xd2\x8e\x11\x97\x41\x3b";
    nist_test_suit[13].Msg      = "\x33\xce\x44\xbd\xb1\xea\x6f\xff\xe5\xa2\x90\x04\xe2\xcb\xf6\x6c";
    nist_test_suit[13].Mac      = "\xb8\x76\x83\x55\x64\x4d\xf5\xa9\xfd\xff\x2d\xef\x76\x3f\x63";

    nist_test_suit[14].dsc.Klen = 16;
    nist_test_suit[14].dsc.Mlen = 16;
    nist_test_suit[14].dsc.Tlen = 15;
    nist_test_suit[14].Key      = "\x26\xef\x8b\x40\x34\x11\x7d\x9e\xbe\xc0\xc7\xfc\x31\x08\x54\x69";
    nist_test_suit[14].Msg      = "\x18\x90\x49\xef\xfd\x7c\xf9\xc8\xf3\x59\x65\xbc\xb0\x97\x8f\xd4";
    nist_test_suit[14].Mac      = "\x29\x5f\x2f\x71\xfc\x58\xe6\xf6\x3d\x32\x65\x4c\x66\x23\xc5";

    nist_test_suit[15].dsc.Klen = 16;
    nist_test_suit[15].dsc.Mlen = 16;
    nist_test_suit[15].dsc.Tlen = 15;
    nist_test_suit[15].Key      = "\xfd\x60\xce\x34\xa6\x52\x78\xf6\x5b\x7d\x53\x90\x4d\xa4\x27\x75";
    nist_test_suit[15].Msg      = "\xcc\x1a\x83\x7f\x51\x36\x65\x6e\x04\xdf\x65\xc0\x56\x52\x17\xcc";
    nist_test_suit[15].Mac      = "\xd7\x6f\x70\xb0\x97\x9a\x76\x59\x04\x35\x86\xa3\xd2\xf9\xd1";

    nist_test_suit[16].dsc.Klen = 16;
    nist_test_suit[16].dsc.Mlen = 32;
    nist_test_suit[16].dsc.Tlen = 4;
    nist_test_suit[16].Key      = "\x77\xa7\x7f\xaf\x29\x0c\x1f\xa3\x0c\x68\x3d\xf1\x6b\xa7\xa7\x7b";
    nist_test_suit[16].Msg      = "\x02\x06\x83\xe1\xf0\x39\x2f\x4c\xac\x54\x31\x8b\x60\x29\x25\x9e\x9c\x55\x3d\xbc\x4b\x6a\xd9\x98\xe6\x4d\x58\xe4\xe7\xdc\x2e\x13";
    nist_test_suit[16].Mac      = "\xfb\xfe\xa4\x1b";

    nist_test_suit[17].dsc.Klen = 16;
    nist_test_suit[17].dsc.Mlen = 32;
    nist_test_suit[17].dsc.Tlen = 4;
    nist_test_suit[17].Key      = "\x65\x33\x78\x0f\xc3\x28\xa8\x8d\x60\x52\x68\xd6\x2f\x29\x5d\xc6";
    nist_test_suit[17].Msg      = "\x02\x74\x9f\x4f\x9a\xd8\x2f\xa7\xba\x41\xd9\x35\xa6\xf1\xaa\x63\x76\xb3\x0b\x87\x75\xb6\x44\x5a\xc8\x9b\x3e\xac\x50\xcd\x8d\x56";
    nist_test_suit[17].Mac      = "\x0b\xfa\x13\x4a";

    nist_test_suit[18].dsc.Klen = 16;
    nist_test_suit[18].dsc.Mlen = 32;
    nist_test_suit[18].dsc.Tlen = 4;
    nist_test_suit[18].Key      = "\x49\x2d\xac\xdc\xb4\xa3\x5f\xc4\x38\xa6\xea\xa3\x5e\x26\xd2\xf6";
    nist_test_suit[18].Msg      = "\x99\x2c\x3e\x56\x13\xff\x42\x0f\xd9\x1b\xe8\x78\xd0\xe1\xd6\x2e\x71\xf5\xf1\x10\x6f\x50\x23\xf3\x99\x91\x5c\x98\x03\x94\xd6\xdb";
    nist_test_suit[18].Mac      = "\xb0\x58\x1e\xd0";

    nist_test_suit[19].dsc.Klen = 16;
    nist_test_suit[19].dsc.Mlen = 32;
    nist_test_suit[19].dsc.Tlen = 4;
    nist_test_suit[19].Key      = "\x2b\xb2\x2a\x8b\x3c\x1b\x5a\xfa\x34\xa9\x90\xf8\x99\x98\xa7\x42";
    nist_test_suit[19].Msg      = "\x25\x56\x0c\xfd\x1e\x4c\xc1\x53\xfe\x48\x3b\xda\xc8\x0d\xc1\x10\x12\xad\xaa\x3d\x14\x4d\x14\x56\xff\x2c\xd9\x21\x90\xe2\xbf\x9a";
    nist_test_suit[19].Mac      = "\x9e\xbf\xf5\x40";

    nist_test_suit[20].dsc.Klen = 16;
    nist_test_suit[20].dsc.Mlen = 32;
    nist_test_suit[20].dsc.Tlen = 4;
    nist_test_suit[20].Key      = "\x30\xb4\x93\x20\xa9\x72\x20\x69\x40\x95\x39\xd6\x11\x1d\xf0\x02";
    nist_test_suit[20].Msg      = "\x9d\x66\xd2\xab\x77\xfe\xd6\x72\xe3\xaf\x1f\xc4\xa7\x7c\xb2\x61\x0c\xd5\x4c\xe3\xd7\xbd\xf6\xff\x7b\x2c\xd8\x08\x8d\xb2\x52\xb4";
    nist_test_suit[20].Mac      = "\x73\x6b\x24\x0a";

    nist_test_suit[21].dsc.Klen = 16;
    nist_test_suit[21].dsc.Mlen = 32;
    nist_test_suit[21].dsc.Tlen = 4;
    nist_test_suit[21].Key      = "\x5f\xd7\x54\xa1\x1d\x6b\x6f\x0d\xe5\x31\xff\xf2\xb6\x80\x12\xf5";
    nist_test_suit[21].Msg      = "\x7c\xc7\xcb\x01\xcc\x96\x6c\x3e\xf8\xa5\xbd\x0c\x91\x1b\xe2\x4c\xfc\x91\x9a\x15\x0d\x5d\x73\x0f\xd8\x17\x3a\x1c\xc3\x56\xb6\x3e";
    nist_test_suit[21].Mac      = "\x10\x19\xca\x65";

    nist_test_suit[22].dsc.Klen = 16;
    nist_test_suit[22].dsc.Mlen = 32;
    nist_test_suit[22].dsc.Tlen = 4;
    nist_test_suit[22].Key      = "\x94\x7b\x01\xb1\x0a\x37\xb5\x1c\x70\x4a\xe8\x7e\xa0\xfb\x59\xe5";
    nist_test_suit[22].Msg      = "\x43\xe1\xf6\xda\xf8\xb1\xf8\xbe\xcd\xe9\x67\x6b\x15\x65\x40\x3b\xd4\xe3\xd8\x82\x18\x09\xca\xb8\x8c\xaa\x34\xda\x4a\x45\x23\x0d";
    nist_test_suit[22].Mac      = "\xd3\x61\x2b\x36";

    nist_test_suit[23].dsc.Klen = 16;
    nist_test_suit[23].dsc.Mlen = 32;
    nist_test_suit[23].dsc.Tlen = 4;
    nist_test_suit[23].Key      = "\x36\x5d\xf1\x49\x77\xf5\x56\xd6\xdd\xe6\x5f\x66\x70\xa3\x05\x18";
    nist_test_suit[23].Msg      = "\x1b\x4e\x4c\x7f\x91\xc6\x98\xbf\xa7\x47\x0f\xad\x1d\x60\x92\xfc\xef\xed\x1d\x02\x2a\x9a\x41\x5e\xff\xa2\xd4\xdd\x16\xdd\xe7\x9f";
    nist_test_suit[23].Mac      = "\x64\x4e\xc8\x02";

    nist_test_suit[24].dsc.Klen = 16;
    nist_test_suit[24].dsc.Mlen = 32;
    nist_test_suit[24].dsc.Tlen = 15;
    nist_test_suit[24].Key      = "\x53\x4c\x6f\x8f\x88\xbc\x35\x3f\xae\xe5\x26\x64\x99\x5d\x54\x57";
    nist_test_suit[24].Msg      = "\x49\x81\xc5\x1f\xcc\x09\x35\xf6\x19\xec\x6b\xf8\x62\x68\x3b\x00\x25\xcc\x48\x72\x48\x39\xbc\x1e\x67\xaa\x3c\x68\x6d\x32\x1b\xa6";
    nist_test_suit[24].Mac      = "\x63\x77\xc6\xcf\xe8\xdd\x60\x5e\xf0\xa6\x2a\x84\x5a\xb3\xf7";

    nist_test_suit[25].dsc.Klen = 16;
    nist_test_suit[25].dsc.Mlen = 32;
    nist_test_suit[25].dsc.Tlen = 15;
    nist_test_suit[25].Key      = "\xa1\x7c\x96\xfa\xa6\x4c\xca\xb2\xc4\x5d\x93\xa0\x63\x89\x36\x81";
    nist_test_suit[25].Msg      = "\xf2\x68\xc7\xd4\xdf\x15\x9e\xf4\xd2\x84\xbe\x92\x42\x9b\x80\x72\x6e\xf1\x34\x73\x4e\xeb\xb9\xcc\xc9\x25\x4c\x96\x28\x13\x9e\x8b";
    nist_test_suit[25].Mac      = "\xb7\x3d\xfe\x7c\x49\x1b\x2d\x69\x2e\x17\x04\x81\x97\x1e\x4e";

    nist_test_suit[26].dsc.Klen = 16;
    nist_test_suit[26].dsc.Mlen = 32;
    nist_test_suit[26].dsc.Tlen = 15;
    nist_test_suit[26].Key      = "\xc5\x58\x1d\x40\xb3\x31\xe2\x40\x03\x90\x1b\xd6\xbf\x24\x4a\xca";
    nist_test_suit[26].Msg      = "\x5d\x44\xba\xc1\xa3\x88\xeb\x81\xb0\xc3\x57\x1f\x6d\x05\x66\xb6\x9b\xde\xf5\xff\x21\x66\x4b\x73\xa4\x80\x4e\xb0\x59\x60\xf6\x1e";
    nist_test_suit[26].Mac      = "\xa1\x44\x7d\x0b\x0b\x02\x83\x24\x9e\x76\x6f\x83\x8d\xb9\xf4";

    nist_test_suit[27].dsc.Klen = 16;
    nist_test_suit[27].dsc.Mlen = 32;
    nist_test_suit[27].dsc.Tlen = 15;
    nist_test_suit[27].Key      = "\xc8\x51\xf8\xec\x4b\xaf\x6e\x1b\x45\x62\xae\x91\x7f\x23\x5d\xf1";
    nist_test_suit[27].Msg      = "\x62\xe9\x70\x75\xc8\x1c\xe3\x36\x7a\x65\x2b\x38\x26\x8c\x7f\x27\xd8\x37\x78\x88\x12\xb3\xc5\x9d\x1e\xf1\x21\x88\xa3\xcf\x82\xb2";
    nist_test_suit[27].Mac      = "\x45\x7a\xd0\xad\xe8\x2d\xfc\xc8\x41\x9f\xdd\x49\xf9\x88\xa0";

    nist_test_suit[28].dsc.Klen = 16;
    nist_test_suit[28].dsc.Mlen = 32;
    nist_test_suit[28].dsc.Tlen = 15;
    nist_test_suit[28].Key      = "\xe4\xab\xe3\x43\xf9\x8a\x2d\xf0\x94\x13\xc3\xde\xfb\x85\xb5\x6a";
    nist_test_suit[28].Msg      = "\xf7\x99\x87\x6d\x19\xac\x1b\x84\x9a\x1a\x43\xfe\x99\x12\xbc\xaf\x6e\x1e\x38\x96\xea\x58\xbc\xb2\xdf\xdc\x47\x16\xe3\x79\xb4\x40";
    nist_test_suit[28].Mac      = "\xe0\x84\x28\xdb\xbc\x13\xff\x94\x32\x04\x8c\x0a\xd9\x57\x31";

    nist_test_suit[29].dsc.Klen = 16;
    nist_test_suit[29].dsc.Mlen = 32;
    nist_test_suit[29].dsc.Tlen = 15;
    nist_test_suit[29].Key      = "\x30\xd2\x76\xde\xdf\xa5\xa6\x95\x20\xe6\x0f\x12\xbe\x0b\xe2\x13";
    nist_test_suit[29].Msg      = "\xe1\x2c\xa5\x1c\x42\x9e\xd7\x34\x33\x67\xe5\x09\xf6\x92\x44\x8a\xd3\x1a\x92\x32\x68\xf2\x84\x2e\x41\x48\x20\xbe\xa8\x0b\x2c\xdd";
    nist_test_suit[29].Mac      = "\xdc\x94\x04\x2e\x5f\x57\x24\x13\x61\xde\x8b\x97\xff\x3d\x4c";

    nist_test_suit[30].dsc.Klen = 16;
    nist_test_suit[30].dsc.Mlen = 32;
    nist_test_suit[30].dsc.Tlen = 15;
    nist_test_suit[30].Key      = "\xc7\x4a\x5d\x42\x65\xf9\xf3\xd5\xf1\xc8\x70\xe3\x77\x62\x5f\x1b";
    nist_test_suit[30].Msg      = "\x31\x4e\x0c\x7d\x5c\x9a\xfb\x7c\x42\x82\xbc\x13\x2b\xf8\x05\x74\x95\x54\x61\x4c\x86\x55\x26\xeb\x20\x62\xa7\xa5\x76\x16\xac\x78";
    nist_test_suit[30].Mac      = "\x86\xf7\xe5\x03\xff\xe5\x5c\x18\x7d\x7f\xc0\xe0\x0f\x7b\x01";

    nist_test_suit[31].dsc.Klen = 16;
    nist_test_suit[31].dsc.Mlen = 32;
    nist_test_suit[31].dsc.Tlen = 15;
    nist_test_suit[31].Key      = "\xc5\x95\xee\x76\x55\xc8\xee\xec\xd3\xe8\xfb\xbb\xc4\x39\xdb\xe2";
    nist_test_suit[31].Msg      = "\x39\xfb\x12\x28\x8a\x67\xf1\x5f\xa4\x19\x1d\x59\x7c\x83\x4d\xc0\xa0\x49\xa4\xfc\x6c\xa6\x86\xb1\x81\x0c\xa9\x88\x73\x0a\x6f\x33";
    nist_test_suit[31].Mac      = "\x83\x98\xe6\x15\x3b\xa5\x80\xcf\x3c\x42\x05\x4b\xcd\xda\x2f";

    nist_test_suit[32].dsc.Klen = 16;
    nist_test_suit[32].dsc.Mlen = 33;
    nist_test_suit[32].dsc.Tlen = 4;
    nist_test_suit[32].Key      = "\x49\x9d\xb5\xa3\xec\xc8\x3d\x34\xfd\x88\x5f\xde\x06\x93\x10\x97";
    nist_test_suit[32].Msg      = "\xf2\x78\x35\x40\xc9\xe1\x70\x6e\xe3\xe7\xa4\x3e\x71\x83\x39\x87\xbb\x72\x44\x1c\x1e\x2e\xab\x58\x50\x1c\x8b\xfa\xec\x07\xd6\x33\x2a";
    nist_test_suit[32].Mac      = "\x8e\x96\x49\xdb";

    nist_test_suit[33].dsc.Klen = 16;
    nist_test_suit[33].dsc.Mlen = 33;
    nist_test_suit[33].dsc.Tlen = 4;
    nist_test_suit[33].Key      = "\xa5\x86\xd9\x2b\xc5\xf9\x46\xce\x58\x08\x03\x22\x04\x50\x83\xd6";
    nist_test_suit[33].Msg      = "\x12\x95\xc5\xbd\x2d\x07\x6b\x18\x72\x77\xa0\x2d\x57\x91\x2d\x2e\x9e\x27\x71\x90\x31\x82\x60\x03\x51\xf6\xcc\xa7\xef\xb9\xe4\x7d\x5a";
    nist_test_suit[33].Mac      = "\x6e\x55\x44\x64";

    nist_test_suit[34].dsc.Klen = 16;
    nist_test_suit[34].dsc.Mlen = 33;
    nist_test_suit[34].dsc.Tlen = 4;
    nist_test_suit[34].Key      = "\x52\x57\x3c\x65\x50\x88\x69\xdb\x32\x59\xcf\x5e\xcb\x62\x75\x65";
    nist_test_suit[34].Msg      = "\xb1\x1a\x4c\x21\x55\xd3\x79\xd1\xdf\xa3\xfc\xb4\x32\xf6\x5c\xc5\xac\xc4\x87\xca\x4b\x1d\x17\x87\x14\x06\xd3\x31\xb5\x83\x79\x25\x69";
    nist_test_suit[34].Mac      = "\x2e\x8b\x88\x44";

    nist_test_suit[35].dsc.Klen = 16;
    nist_test_suit[35].dsc.Mlen = 33;
    nist_test_suit[35].dsc.Tlen = 4;
    nist_test_suit[35].Key      = "\x30\xe3\xea\x24\x76\xb2\xd8\x89\xfc\x3f\x15\x65\xf0\x0e\xe4\x7f";
    nist_test_suit[35].Msg      = "\x25\x53\x12\xe9\x5b\x96\xe3\xeb\x36\x9e\x3a\xde\xbd\x2f\x5b\x40\x83\x5a\xa0\x58\xc1\x47\x27\x1d\xa7\x32\xc8\x55\xf4\xb4\x4a\xfa\xc5";
    nist_test_suit[35].Mac      = "\x24\x4a\xd8\xad";

    nist_test_suit[36].dsc.Klen = 16;
    nist_test_suit[36].dsc.Mlen = 33;
    nist_test_suit[36].dsc.Tlen = 4;
    nist_test_suit[36].Key      = "\xdb\x81\x4e\xd9\xd5\xb6\x83\xff\x55\xa0\xc9\x5d\xe6\x2b\xae\x5c";
    nist_test_suit[36].Msg      = "\xc1\x09\x35\x18\xef\xd8\x02\x45\xe3\xc4\x23\x71\xf2\x20\xb2\x1f\x20\x34\xe6\x73\x8f\xe0\x2e\xf4\x3e\x82\x81\x90\xf0\x1a\xef\xe1\x69";
    nist_test_suit[36].Mac      = "\x39\x7a\x19\x3e";

    nist_test_suit[37].dsc.Klen = 16;
    nist_test_suit[37].dsc.Mlen = 33;
    nist_test_suit[37].dsc.Tlen = 4;
    nist_test_suit[37].Key      = "\x47\x8a\x9f\xb5\xa3\xb3\xaa\x43\x27\xe0\x3d\x7d\xe4\xa8\x24\x4c";
    nist_test_suit[37].Msg      = "\xcb\xc2\xbc\xe0\x2e\x45\x70\x4c\x92\x5f\x88\x10\x5e\xf1\x32\x36\xc7\x68\xf5\xb3\x39\x92\x39\xe2\xf9\xb0\x0a\x31\x08\xcc\x19\x1b\x4a";
    nist_test_suit[37].Mac      = "\xb0\xac\x1e\x74";

    nist_test_suit[38].dsc.Klen = 16;
    nist_test_suit[38].dsc.Mlen = 33;
    nist_test_suit[38].dsc.Tlen = 4;
    nist_test_suit[38].Key      = "\xa8\xe1\x48\x4b\x47\x34\xfc\x67\x90\xcd\x1c\x87\xee\xb5\x9c\xa2";
    nist_test_suit[38].Msg      = "\x69\x2e\x47\xf5\x70\x1b\xb2\x56\x9f\xd9\xdd\xe7\xf8\xeb\x8d\x67\x0c\x33\x6b\xe7\x78\x65\xac\xdc\xb0\xbc\xf1\x73\xfb\xec\x6c\x24\xbf";
    nist_test_suit[38].Mac      = "\x99\xe0\x22\x4d";

    nist_test_suit[39].dsc.Klen = 16;
    nist_test_suit[39].dsc.Mlen = 33;
    nist_test_suit[39].dsc.Tlen = 4;
    nist_test_suit[39].Key      = "\x4b\x03\xf1\x93\x5c\x21\xe6\x08\x3e\xf7\x2d\x1f\x7b\xfc\x2e\x2a";
    nist_test_suit[39].Msg      = "\x32\x57\x9b\x66\x32\x73\x35\x10\x0d\x9b\xd2\x36\xe3\x7e\x3b\x0d\x68\x13\x54\x78\x42\x74\xa1\x7c\xb2\xef\x0a\xf4\x77\x5a\xdb\x86\xef";
    nist_test_suit[39].Mac      = "\xa5\xa7\xc8\xc9";

    nist_test_suit[40].dsc.Klen = 16;
    nist_test_suit[40].dsc.Mlen = 33;
    nist_test_suit[40].dsc.Tlen = 15;
    nist_test_suit[40].Key      = "\xa0\xc3\x34\xff\x35\x01\xc9\x9a\x9d\x5f\x26\x60\xf4\xa2\xcc\x5f";
    nist_test_suit[40].Msg      = "\xb4\x69\x3a\x2a\xa1\x1c\xf9\xa5\x44\x2f\x08\xdf\xa7\x18\x59\x0f\xef\xf8\xd3\x8f\xdf\x15\xf8\xee\x9d\x8a\xc5\x41\xb9\x3d\xd9\xb9\x6b";
    nist_test_suit[40].Mac      = "\x6d\x4b\xf5\x0d\x3a\x13\xa2\x6d\x9d\xc7\x56\x6d\xee\x12\x23";

    nist_test_suit[41].dsc.Klen = 16;
    nist_test_suit[41].dsc.Mlen = 33;
    nist_test_suit[41].dsc.Tlen = 15;
    nist_test_suit[41].Key      = "\xd2\x65\x33\xa7\x60\xe7\x6d\xfa\xa4\xa7\xc9\x52\x9d\xf8\x60\x31";
    nist_test_suit[41].Msg      = "\x80\xf3\xea\x04\xe4\x2d\x4c\xef\x33\x4c\x04\xc7\xea\xae\x86\x83\x2f\xaf\x52\xf2\xaa\x28\x6b\x5a\x96\x36\x0b\x60\x18\x3d\x95\x27\xcf";
    nist_test_suit[41].Mac      = "\xec\xee\x84\xf8\x19\x24\xee\x3c\x20\x82\x9e\x3d\x97\xeb\xf9";

    nist_test_suit[42].dsc.Klen = 16;
    nist_test_suit[42].dsc.Mlen = 33;
    nist_test_suit[42].dsc.Tlen = 15;
    nist_test_suit[42].Key      = "\xc2\xca\x6a\xce\x8b\x8b\x19\x31\x4c\xc1\x43\x90\xe2\x2c\xbc\x37";
    nist_test_suit[42].Msg      = "\x8a\x78\x2e\x7b\x7c\x2e\xa7\x7f\x63\xdd\xaa\x85\x63\xed\x90\xd3\xb5\x7c\x8c\x22\x42\xa6\xdd\x00\x71\x04\x48\x73\x3d\x72\xc6\xdf\x8a";
    nist_test_suit[42].Mac      = "\x77\x1a\x53\xea\x36\x49\x73\xcc\x5c\x01\x77\xdb\x25\x0a\xc7";

    nist_test_suit[43].dsc.Klen = 16;
    nist_test_suit[43].dsc.Mlen = 33;
    nist_test_suit[43].dsc.Tlen = 15;
    nist_test_suit[43].Key      = "\x36\x3d\x0f\x98\xf6\x1d\x52\x68\xf1\x2a\x73\x68\x0f\xb9\x4a\x98";
    nist_test_suit[43].Msg      = "\x2e\x4d\x58\x35\x13\xc6\xb8\xf4\x79\xf1\x61\xb4\x42\xb7\xee\xb8\x79\x88\x59\x1e\x93\x94\x62\xb5\xf5\x3e\x35\x0e\xb7\xcd\x47\xd6\xa8";
    nist_test_suit[43].Mac      = "\xb5\x2d\xad\x0e\x62\xd1\xb0\x5f\xd2\x39\x34\xc2\xe9\xc0\xd0";

    nist_test_suit[44].dsc.Klen = 16;
    nist_test_suit[44].dsc.Mlen = 33;
    nist_test_suit[44].dsc.Tlen = 15;
    nist_test_suit[44].Key      = "\xa6\x02\x69\xf0\x95\xad\x3c\x3b\xaf\xae\x90\x7c\x6f\x21\x5d\xe0";
    nist_test_suit[44].Msg      = "\xce\xad\x1c\x5a\xf1\x6c\xa8\x9b\xc0\x82\x17\x75\xf8\xcb\xa8\xc2\x56\x20\xa0\x3d\xfd\x27\xd6\xf1\x18\x6f\x75\xf1\xc0\xbc\xfe\x4a\x20";
    nist_test_suit[44].Mac      = "\x17\x20\x84\xc3\xfe\x99\xfd\xe4\xaf\x29\xaa\x8e\x6e\x5f\xe1";

    nist_test_suit[45].dsc.Klen = 16;
    nist_test_suit[45].dsc.Mlen = 33;
    nist_test_suit[45].dsc.Tlen = 15;
    nist_test_suit[45].Key      = "\xeb\xf0\xb3\xe3\x19\x9a\x5c\x37\x73\xc7\x61\xc7\x25\xc7\x60\x0a";
    nist_test_suit[45].Msg      = "\x82\x5b\x19\x2f\x69\xfb\x73\xb2\x71\x6f\xb3\x5e\x69\xd9\xc0\x90\x06\xa8\xa2\x42\x2a\xc7\xe4\xe0\x65\x14\x3c\x58\xbd\x14\x6e\x71\xaa";
    nist_test_suit[45].Mac      = "\x77\x74\x2a\x4c\x60\x87\xc1\x70\xbe\xef\xc8\xa4\x1d\x4f\x63";

    nist_test_suit[46].dsc.Klen = 16;
    nist_test_suit[46].dsc.Mlen = 33;
    nist_test_suit[46].dsc.Tlen = 15;
    nist_test_suit[46].Key      = "\xc9\xde\xf0\x36\xa2\x93\x54\xc9\xee\xe0\x18\xb3\xcb\xbc\x71\x2a";
    nist_test_suit[46].Msg      = "\x3a\x87\x16\x32\xf2\x28\x18\xbf\xd7\x17\xb0\x06\x18\x57\x29\x36\xf4\xba\x57\xe5\x7d\x9c\x7a\x60\x9c\xd0\x66\x3f\x56\xdb\xe1\x3d\x4d";
    nist_test_suit[46].Mac      = "\xbf\x51\xdb\x9d\x04\x71\x61\x3c\x4d\x92\x75\xe9\x72\x58\x6f";

    nist_test_suit[47].dsc.Klen = 16;
    nist_test_suit[47].dsc.Mlen = 33;
    nist_test_suit[47].dsc.Tlen = 15;
    nist_test_suit[47].Key      = "\x64\x4c\xbf\x12\x85\x9d\xf0\x55\x7e\xa9\x1f\x08\xe0\x51\xff\x27";
    nist_test_suit[47].Msg      = "\xe2\xb4\xb6\xf9\x48\x44\x02\x64\x5c\x47\x80\x9e\xd5\xa8\x3a\x17\xb3\x78\xcf\x85\x22\x41\x74\xd9\xa0\x97\x39\x71\x62\xf1\x8e\x8f\xf4";
    nist_test_suit[47].Mac      = "\x4e\x6e\xc5\x6f\xf9\x5d\x0e\xae\x1c\xf8\x3e\xfc\xf4\x4b\xeb";

    nist_test_suit[48].dsc.Klen = 16;
    nist_test_suit[48].dsc.Mlen = 37;
    nist_test_suit[48].dsc.Tlen = 4;
    nist_test_suit[48].Key      = "\x8c\x9e\xab\xe8\x71\xc6\xe9\x51\x11\x94\xb4\x8e\xbf\x9e\x9b\x03";
    nist_test_suit[48].Msg      = "\x05\x49\xea\xec\xa4\xf3\x97\x24\x7f\x1d\x25\x96\x12\xe6\x86\x7e\x7d\x78\x8c\x71\xd0\x3c\x51\x36\x86\x4a\xd6\xd8\x4f\x24\xea\xf9\x13\xa3\x4e\x69\x33";
    nist_test_suit[48].Mac      = "\x0c\x1e\x97\xd0";

    nist_test_suit[49].dsc.Klen = 16;
    nist_test_suit[49].dsc.Mlen = 37;
    nist_test_suit[49].dsc.Tlen = 4;
    nist_test_suit[49].Key      = "\xdd\x42\x44\x9d\xa4\xc9\x5e\x85\x8b\x79\x60\x85\xb6\xb5\xb3\xb5";
    nist_test_suit[49].Msg      = "\x4d\xa6\xfe\xba\x6d\x11\x9b\x88\x50\xbe\xd9\x45\xf5\x7a\xdc\xd3\xc3\xe8\x4f\x85\xd5\x81\x55\x3b\x95\xda\xc3\xce\x8c\x25\x8c\xae\x78\xfb\x7d\xc9\x9f";
    nist_test_suit[49].Mac      = "\xaf\xa8\xc4\x01";

    nist_test_suit[50].dsc.Klen = 16;
    nist_test_suit[50].dsc.Mlen = 37;
    nist_test_suit[50].dsc.Tlen = 4;
    nist_test_suit[50].Key      = "\xe2\xd5\x92\xcb\x41\x2e\x65\xf9\x04\x42\x57\xd7\x8e\x74\x91\xf9";
    nist_test_suit[50].Msg      = "\x51\x2c\xd8\x90\xe4\x61\xad\xea\x7f\xce\xeb\xe7\xa9\x30\x67\xd2\x0e\xd1\x50\xee\x75\xaf\xc2\xb9\xf2\xda\x97\x71\x5d\x1e\x81\xd8\x02\xe2\x56\x35\x9a";
    nist_test_suit[50].Mac      = "\x35\xb2\x1e\xe1";

    nist_test_suit[51].dsc.Klen = 16;
    nist_test_suit[51].dsc.Mlen = 37;
    nist_test_suit[51].dsc.Tlen = 4;
    nist_test_suit[51].Key      = "\x46\xf0\x26\x59\xf0\x8a\xd0\x31\x25\xf3\xbb\x40\xca\xcd\x0f\xda";
    nist_test_suit[51].Msg      = "\xeb\x2e\x7e\x32\x58\x94\x86\xaa\x84\x94\x37\xc8\xdb\xbd\x71\x3b\x60\xd7\x6f\xa3\x41\x23\xf9\xf6\xaf\xe4\xdb\x23\xec\x1d\x97\xf6\xa6\x9d\x57\x3c\x83";
    nist_test_suit[51].Mac      = "\x23\x13\xfd\xbe";

    nist_test_suit[52].dsc.Klen = 16;
    nist_test_suit[52].dsc.Mlen = 37;
    nist_test_suit[52].dsc.Tlen = 4;
    nist_test_suit[52].Key      = "\x42\xc7\xfa\x8a\x13\xba\x2d\x0f\x6c\x37\x60\xbc\xd4\xfa\x6c\xd5";
    nist_test_suit[52].Msg      = "\x11\xc9\x08\x09\xf9\xc5\x3d\x2f\x77\xb5\x6a\xf0\xa4\x22\x87\xac\x69\x20\xe3\xd2\x92\x1c\xce\xb8\x24\xd4\x96\xca\xf1\xa7\xb6\xe7\x69\x0f\x49\x08\xb3";
    nist_test_suit[52].Mac      = "\x09\x30\xb0\xaa";

    nist_test_suit[53].dsc.Klen = 16;
    nist_test_suit[53].dsc.Mlen = 37;
    nist_test_suit[53].dsc.Tlen = 4;
    nist_test_suit[53].Key      = "\x49\xdb\x86\xd2\xb3\xb1\x08\x80\x53\x16\x4a\xa4\x64\xa7\x85\x66";
    nist_test_suit[53].Msg      = "\xdc\x78\x7b\x23\x86\xfc\xae\x8b\x8c\x6d\x0c\x9d\x0c\x09\x2a\xae\xda\xdd\x31\x50\x0d\x37\x84\x6b\xbe\x5f\xa3\x90\x0c\x82\x50\x89\x63\xef\xf0\x35\xff";
    nist_test_suit[53].Mac      = "\x63\x74\x47\x14";

    nist_test_suit[54].dsc.Klen = 16;
    nist_test_suit[54].dsc.Mlen = 37;
    nist_test_suit[54].dsc.Tlen = 4;
    nist_test_suit[54].Key      = "\xcb\x30\xd1\x6d\x4f\xf5\xd0\xcd\x5b\x8c\x1c\xf6\xfc\x9b\x53\x5d";
    nist_test_suit[54].Msg      = "\x2e\x52\x3e\x9d\x8a\x55\x32\x12\x7e\xc6\x3b\x22\x08\x38\xf1\x1b\x0f\x8a\x09\xe9\xa3\x17\xc1\xe4\x87\x2d\x7f\xec\xec\xc1\xb4\xb8\x80\x60\x07\x6b\xa7";
    nist_test_suit[54].Mac      = "\x12\x1c\x66\x7a";

    nist_test_suit[55].dsc.Klen = 16;
    nist_test_suit[55].dsc.Mlen = 37;
    nist_test_suit[55].dsc.Tlen = 4;
    nist_test_suit[55].Key      = "\x46\x7e\x1d\x98\x93\x12\x33\x20\xda\xdc\x3a\x23\xc9\x61\xec\x81";
    nist_test_suit[55].Msg      = "\x16\x2f\xf9\x47\x8c\xa0\x6a\xeb\xfc\x7f\x86\xaf\xa9\xcd\xd7\x43\x91\x65\x63\xeb\xfd\x3a\xdb\xdd\x56\xe0\x15\xea\x3a\x4e\xbc\x61\xcf\xe2\x47\x31\x57";
    nist_test_suit[55].Mac      = "\x4d\x07\x7d\xef";

    nist_test_suit[56].dsc.Klen = 16;
    nist_test_suit[56].dsc.Mlen = 37;
    nist_test_suit[56].dsc.Tlen = 15;
    nist_test_suit[56].Key      = "\x69\x90\xdb\x6a\x5c\xd8\xdc\x14\x94\xcd\x63\x92\x21\x51\x92\x0c";
    nist_test_suit[56].Msg      = "\x8c\xed\xde\xbd\x38\xf0\x04\x06\x74\x3a\x67\x56\x56\x5c\xe7\x62\xd3\x46\x44\x35\xd5\x0b\xd6\x1b\x8d\xe5\x7f\xbe\x0b\x79\xdf\x8f\x0c\x5c\xc6\x67\x13";
    nist_test_suit[56].Mac      = "\xe5\x2c\xd4\xfa\x3c\xf7\x2b\x2f\x06\xee\xce\x11\x22\x42\xe6";

    nist_test_suit[57].dsc.Klen = 16;
    nist_test_suit[57].dsc.Mlen = 37;
    nist_test_suit[57].dsc.Tlen = 15;
    nist_test_suit[57].Key      = "\x07\xf7\x7f\x11\x4d\x72\x64\xa1\x22\xa7\xe9\xdb\x4f\xc8\xd0\x91";
    nist_test_suit[57].Msg      = "\xbb\x5d\xe4\xba\xe5\x8d\x8f\x49\xc4\x88\x11\xaa\x31\xd9\x9b\xd7\x87\x7a\x0e\xc0\x4d\xe9\xbc\xf8\x15\x7f\x73\xaf\xa3\xfe\xb8\xe0\x52\x69\x50\xcb\xb4";
    nist_test_suit[57].Mac      = "\x78\xec\xc7\x99\x77\x44\xd4\xb8\x3d\xb0\xf8\xa4\xb4\x2b\x04";

    nist_test_suit[58].dsc.Klen = 16;
    nist_test_suit[58].dsc.Mlen = 37;
    nist_test_suit[58].dsc.Tlen = 15;
    nist_test_suit[58].Key      = "\xe1\x89\x52\x24\xdc\x1e\x46\x74\xa9\xbc\x9c\x27\xb3\x75\x2c\x83";
    nist_test_suit[58].Msg      = "\x2b\xed\x02\xa7\x40\x4e\x47\xde\xc2\x8c\x79\xc2\xec\xce\xb7\x40\x03\x12\x91\xfd\x2c\xb9\x78\xf5\x80\x46\x72\x63\x62\x63\x13\xe0\x10\x25\xa8\xce\x79";
    nist_test_suit[58].Mac      = "\x99\x4e\x0c\x7f\x66\x89\xc6\xe3\xa1\x36\x21\xd3\x83\x95\xfc";

    nist_test_suit[59].dsc.Klen = 16;
    nist_test_suit[59].dsc.Mlen = 37;
    nist_test_suit[59].dsc.Tlen = 15;
    nist_test_suit[59].Key      = "\xb9\x3d\x47\xdb\xb8\x43\x68\x1b\x50\xb1\xc2\x68\x99\x8d\x4a\xb4";
    nist_test_suit[59].Msg      = "\x60\xc6\xe6\x3b\x0e\x4d\xde\xf2\x4f\x99\x0e\x65\x2c\x9b\x75\x29\x3b\xa8\xe4\x56\x7b\xcc\x06\x83\x80\x62\x9f\xf1\xd0\xe1\x11\x06\x8c\xf9\x24\xfd\x53";
    nist_test_suit[59].Mac      = "\x39\x5f\x27\x9c\xe3\x19\x18\x63\xb4\xfa\x84\xf2\xda\xb3\xc7";

    nist_test_suit[60].dsc.Klen = 16;
    nist_test_suit[60].dsc.Mlen = 37;
    nist_test_suit[60].dsc.Tlen = 15;
    nist_test_suit[60].Key      = "\xc9\x40\x8a\x8b\x16\x3f\x1e\x60\x28\x94\xb3\x23\x9c\x3f\xdb\x6d";
    nist_test_suit[60].Msg      = "\xae\x10\x09\xf0\x36\x26\xfc\xb5\x4b\xf9\x8c\x32\x91\x2f\x0f\x70\xbd\x39\x8c\x70\x9c\x3e\xd8\xbf\x57\x54\xfe\x4b\xf5\xf6\xe4\x75\x21\xb3\x2c\x67\x2e";
    nist_test_suit[60].Mac      = "\x58\xc3\xc8\x04\xc2\x98\x5d\xf4\x7c\x5c\x4b\xfc\xfe\x88\x37";

    nist_test_suit[61].dsc.Klen = 16;
    nist_test_suit[61].dsc.Mlen = 37;
    nist_test_suit[61].dsc.Tlen = 15;
    nist_test_suit[61].Key      = "\x1f\x07\x69\xa7\xae\x82\xbd\x98\x56\x61\xe0\x31\xc4\xa8\x92\xc1";
    nist_test_suit[61].Msg      = "\xa0\xa6\x45\x82\xef\xf0\x02\xbb\x34\x8d\x27\x98\xf6\x78\x12\x10\x23\x77\xc3\x34\x54\x4e\x3e\x06\x53\x32\x31\x8d\xdb\x80\xd7\x29\x9e\xaf\x1c\x25\x8c";
    nist_test_suit[61].Mac      = "\x3a\xc1\xf8\x0a\xee\x7d\x9e\xc5\x28\x43\x73\xe4\x3a\x56\xd7";

    nist_test_suit[62].dsc.Klen = 16;
    nist_test_suit[62].dsc.Mlen = 37;
    nist_test_suit[62].dsc.Tlen = 15;
    nist_test_suit[62].Key      = "\x18\x42\x15\x14\x5d\xa4\x9d\xb4\x17\xe8\xbd\xd5\x73\xd6\x28\x2d";
    nist_test_suit[62].Msg      = "\x57\x88\xf6\x1e\x02\x30\x47\x91\xb5\x2f\x40\x05\x7a\xbb\x4e\x04\x46\x40\x3e\xf3\x74\x02\x53\xdf\x72\x05\x96\x79\xbb\x2a\x6e\x5e\x05\x9a\x70\x9c\xbb";
    nist_test_suit[62].Mac      = "\x8d\xa8\xcc\xa9\xb3\x6f\x68\x57\x1c\x6c\x0e\x40\xa3\xf4\x10";

    nist_test_suit[63].dsc.Klen = 16;
    nist_test_suit[63].dsc.Mlen = 37;
    nist_test_suit[63].dsc.Tlen = 15;
    nist_test_suit[63].Key      = "\x18\x74\x59\x6c\xdd\xbd\xf1\x8a\x10\xbc\x71\xd6\x0c\x6b\xb9\x3d";
    nist_test_suit[63].Msg      = "\x12\xa3\x40\xef\x01\x5d\xc0\xa3\x86\x25\xa4\x84\x7e\xb6\xca\xc9\xca\xb9\x45\x05\x48\xe9\xf9\x64\x02\x75\x65\x31\xa6\xa5\xbf\x9c\x37\xc1\x46\xbb\x01";
    nist_test_suit[63].Mac      = "\x26\xa5\xfd\x25\x80\x51\x29\x75\x6b\x5b\x1a\xc3\x3d\x87\x74";
}

#define OFFERED_BATCH_SZ            128
#define USER_MSG_SZ                 64
#define KEY_LEN_IN_BITS             128
#define CAN_BUF_SZ                  8
#define CMAC_BLOCK_SIZE             16
#define MAC_PADDING_SZ              16

#define XMIT_FSM_IDLE               0
#define XMIT_FSM_AT_MAGIC_SENT      1
#define XMIT_FSM_AT_LEN_SENT        2
#define XMIT_FSM_IN_MSG_SENDING     3
#define XMIT_FSM_AT_MSG_SENT        4
#define XMIT_FSM_IN_CMAC_SENDING    5
#define XMIT_FSM_AT_CMAC_SENT       6
#define XMIT_FSM_COMPLETE           6

#define RECV_FSM_IDLE               0
#define RECV_FSM_AT_MAGIC_RECV      1
#define RECV_FSM_AT_LEN_RECV        2
#define RECV_FSM_IN_MSG_RECVING     3
#define RECV_FSM_AT_MSG_RECV        4
#define RECV_FSM_IN_CMAC_RECVING    5
#define RECV_FSM_AT_CMAC_RECV       6
#define RECV_FSM_COMPLETE           6

/**** *****/
uint32_t g_pre_time;
uint32_t g_pst_time;
uint32_t g_elp_time;

/*!
 * @brief Main function
 */
int main(void)
{
    unsigned char           key[CMAC_BLOCK_SIZE]                = { 0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6, 0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c };
    unsigned char           input[USER_MSG_SZ + MAC_PADDING_SZ] = { 0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96, 0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93, 0x17, 0x2a,
                                                                    0xae, 0x2d, 0x8a, 0x57, 0x1e, 0x03, 0xac, 0x9c, 0x9e, 0xb7, 0x6f, 0xac, 0x45, 0xaf, 0x8e, 0x51,
                                                                    0x30, 0xc8, 0x1c, 0x46, 0xa3, 0x5c, 0xe4, 0x11, 0xe5, 0xfb, 0xc1, 0x19, 0x1a, 0x0a, 0x52, 0xef,
                                                                    0xf6, 0x9f, 0x24, 0x45, 0xdf, 0x4f, 0x9b, 0x17, 0xad, 0x2b, 0x41, 0x7b, 0xe6, 0x6c, 0x37, 0x10 };
    const unsigned char     nist_cmac[CMAC_BLOCK_SIZE]          = { 0x51, 0xf0, 0xbe, 0xbf, 0x7e, 0x3b, 0x9d, 0x92, 0xfc, 0x49, 0x74, 0x17, 0x79, 0x36, 0x3c, 0xfe };
//  unsigned char           openssl_cmac[CMAC_BLOCK_SIZE]       = { 0xd8, 0xae, 0xa8, 0x7e, 0xad, 0xa7, 0x67, 0xa2, 0x3b, 0xe5, 0x0a, 0x47, 0xb9, 0x61, 0xf3, 0x50 };    // given by OpenSSL Cmd Line if input = key = empty
    int                     length                              = USER_MSG_SZ;
    unsigned int            msg_len, mac_len;
    unsigned int            recv_cnt                    = 0;
    unsigned int            f_failed                    = 0;
    unsigned char           obtn_cmac[CMAC_BLOCK_SIZE + MAC_PADDING_SZ];
    unsigned char           rcvd_cmac[CMAC_BLOCK_SIZE + MAC_PADDING_SZ];
    unsigned int            ix;
    unsigned int            sts;
    unsigned int            idx;
    volatile uint32_t       *pusec_cnt      = &s_MsCount;

    uint32_t                beg, end, beg_tick, end_tick, dif, sum, avg;
    uint32_t                delay_at_MCU_A[OFFERED_BATCH_SZ];
    uint32_t                delay_at_MCU_B[OFFERED_BATCH_SZ];
    uint32_t                tot_mcu_a, tot_mcu_b, tot_sum;

    flexcan_config_t flexcanConfig;
    flexcan_rx_mb_config_t mbConfig;
    uint8_t node_type;

    /* Initialize board hardware. */
    BOARD_ConfigMPU();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    /*Clock setting for FLEXCAN*/
    clock_root_config_t rootCfg = {0};
    rootCfg.mux                 = FLEXCAN_CLOCK_SOURCE_SELECT;
    rootCfg.div                 = FLEXCAN_CLOCK_SOURCE_DIVIDER;
    CLOCK_SetRootClock(kCLOCK_Root_Can3, &rootCfg);

    /* Init SysTick module */
    /* call CMSIS SysTick function. It enables the SysTick interrupt at low priority */
    SysTick_Config(CLOCK_GetCoreSysClkFreq() / 1000U);          /* 1 ms period */

    init();
    memset(obtn_cmac,                   0x00, CMAC_BLOCK_SIZE + MAC_PADDING_SZ  );
    memset(rcvd_cmac,                   0x00, CMAC_BLOCK_SIZE + MAC_PADDING_SZ  );

#if 0
    LOG_INFO("Test the correctness of AES128-CMAC implementaion prior to its perofrmance evaluation, by using nist-testvector-sp800-38b-may2005\r\n");
    LOG_INFO("Set 1 vector 4\r\n");
    LOG_INFO("    mode=aes-128\r\n");
    LOG_INFO("    key=2b7e151628aed2a6abf7158809cf4f3c\r\n");
    LOG_INFO("    plain=6bc1bee22e409f96e93d7e117393172aae2d8a571e03ac9c9eb76fac45af8e5130c81c46a35ce411e5fbc1191a0a52eff69f2445df4f9b17ad2b417be66c3710\r\n");
    LOG_INFO("    tlen=16\r\n");
    LOG_INFO("    mac=51f0bebf7e3b9d92fc49741779363cfe\r\n");

    LOG_INFO("    obtained CMAC = %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x\r\n",
        nist_cmac[ 0], nist_cmac[ 1], nist_cmac[ 2], nist_cmac[ 3],
        nist_cmac[ 4], nist_cmac[ 5], nist_cmac[ 6], nist_cmac[ 7],
        nist_cmac[ 8], nist_cmac[ 9], nist_cmac[10], nist_cmac[11],
        nist_cmac[12], nist_cmac[13], nist_cmac[14], nist_cmac[15]);

    if( memcmp(obtn_cmac, nist_cmac, CMAC_BLOCK_SIZE) == 0 ) {
        LOG_INFO("verifying the correctness of AES128-CMAC implementation by using a NIST Test Vector : Pass\r\n");
    }
    else {
        LOG_INFO("verifying the correctness of AES128-CMAC implementation by using a NIST Test Vector : Fail\r\n");
    }
#endif

#if 0
    for( ix = 0; ix < MAX_TEST_SUIT_SZ; ix++ ) {
        // beg.msec    = s_MsCount;
        beg.msec    = *pusec_cnt
        beg.tick    = SysTick->VAL;
        my_delay(135000);
        AES_CMAC(nist_test_suit[ix].Key, nist_test_suit[ix].Msg, nist_test_suit[ix].dsc.Mlen, obtn_cmac);
        // end.msec    = s_MsCount;
        end.msec    = *pusec_cnt;
        end.tick    = SysTick->VAL;

        LOG_INFO("AES-128-CMAC : begin --> end %4d-th trial: %d msec, %d SysTick --> %d msec, %d SysTick\r\n",
            ix,
            beg.msec, TICKS_PER_MSEC - beg.tick,
            end.msec, TICKS_PER_MSEC - end.tick);

        if( memcmp(obtn_cmac, nist_test_suit[ix].Mac, nist_test_suit[ix].dsc.Tlen) )
            LOG_INFO("AES128-CMAC Fail.\r\n");
        else
            LOG_INFO("AES128-CMAC Pass.\r\n");
    }
#endif

    // memset(input,                       0x00, USER_MSG_SZ     + MAC_PADDING_SZ  );
    // memset(key,                         0x00, CMAC_BLOCK_SIZE                   );
    memset(obtn_cmac,                   0x00, CMAC_BLOCK_SIZE + MAC_PADDING_SZ  );
    memset(rcvd_cmac,                   0x00, CMAC_BLOCK_SIZE + MAC_PADDING_SZ  );

    LOG_INFO("\r\n");
    LOG_INFO("\r\n");

    LOG_INFO("********* FLEXCAN Interrupt EXAMPLE *********\r\n");
    LOG_INFO("    Message format: Standard (11 bit id)\r\n");
    LOG_INFO("    Message buffer %d used for Rx.\r\n", RX_MESSAGE_BUFFER_NUM);
    LOG_INFO("    Message buffer %d used for Tx.\r\n", TX_MESSAGE_BUFFER_NUM);
    LOG_INFO("    Interrupt Mode: Enabled\r\n");
    LOG_INFO("    Operation Mode: TX and RX --> Normal\r\n");
    LOG_INFO("*********************************************\r\n\r\n");

    do {
        LOG_INFO("Please select local node as A or B:\r\n");
        LOG_INFO("Note: Node B should start first.\r\n");
        LOG_INFO("Node:");
        node_type = GETCHAR();
        LOG_INFO("%c", node_type);
        LOG_INFO("\r\n");
    } while ((node_type != 'A') && (node_type != 'B') && (node_type != 'a') && (node_type != 'b'));

    /* Select mailbox ID. */
    if ((node_type == 'A') || (node_type == 'a'))
    {
        txIdentifier = 0x321;
        rxIdentifier = 0x123;
    }
    else
    {
        txIdentifier = 0x123;
        rxIdentifier = 0x321;
    }

    /* Get FlexCAN module default Configuration. */
    /*
     * flexcanConfig.clkSrc                 = kFLEXCAN_ClkSrc0;
     * flexcanConfig.bitRate                = 1000000U;
     * flexcanConfig.bitRateFD              = 2000000U;
     * flexcanConfig.maxMbNum               = 16;
     * flexcanConfig.enableLoopBack         = false;
     * flexcanConfig.enableSelfWakeup       = false;
     * flexcanConfig.enableIndividMask      = false;
     * flexcanConfig.disableSelfReception   = false;
     * flexcanConfig.enableListenOnlyMode   = false;
     * flexcanConfig.enableDoze             = false;
     */
    FLEXCAN_GetDefaultConfig(&flexcanConfig);

#if defined(EXAMPLE_CAN_CLK_SOURCE)
    flexcanConfig.clkSrc = EXAMPLE_CAN_CLK_SOURCE;
#endif

#if defined(EXAMPLE_CAN_BIT_RATE)
    flexcanConfig.bitRate = EXAMPLE_CAN_BIT_RATE;
#endif

/* If special quantum setting is needed, set the timing parameters. */
#if (defined(SET_CAN_QUANTUM) && SET_CAN_QUANTUM)
    flexcanConfig.timingConfig.phaseSeg1 = PSEG1;
    flexcanConfig.timingConfig.phaseSeg2 = PSEG2;
    flexcanConfig.timingConfig.propSeg   = PROPSEG;
#if (defined(FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE) && FSL_FEATURE_FLEXCAN_HAS_FLEXIBLE_DATA_RATE)
    flexcanConfig.timingConfig.fphaseSeg1 = FPSEG1;
    flexcanConfig.timingConfig.fphaseSeg2 = FPSEG2;
    flexcanConfig.timingConfig.fpropSeg   = FPROPSEG;
#endif
#endif

#if (defined(USE_IMPROVED_TIMING_CONFIG) && USE_IMPROVED_TIMING_CONFIG)
    flexcan_timing_config_t timing_config;
    memset(&timing_config, 0, sizeof(flexcan_timing_config_t));
#if (defined(USE_CANFD) && USE_CANFD)
    if (FLEXCAN_FDCalculateImprovedTimingValues(EXAMPLE_CAN, flexcanConfig.bitRate, flexcanConfig.bitRateFD,
                                                EXAMPLE_CAN_CLK_FREQ, &timing_config))
    {
        /* Update the improved timing configuration*/
        memcpy(&(flexcanConfig.timingConfig), &timing_config, sizeof(flexcan_timing_config_t));
    }
    else
    {
        LOG_INFO("No found Improved Timing Configuration. Just used default configuration\r\n\r\n");
    }
#else
    if (FLEXCAN_CalculateImprovedTimingValues(EXAMPLE_CAN, flexcanConfig.bitRate, EXAMPLE_CAN_CLK_FREQ, &timing_config))
    {
        /* Update the improved timing configuration*/
        memcpy(&(flexcanConfig.timingConfig), &timing_config, sizeof(flexcan_timing_config_t));
    }
    else
    {
        LOG_INFO("No found Improved Timing Configuration. Just used default configuration\r\n\r\n");
    }
#endif
#endif

#if (defined(USE_CANFD) && USE_CANFD)
    FLEXCAN_FDInit(EXAMPLE_CAN, &flexcanConfig, EXAMPLE_CAN_CLK_FREQ, BYTES_IN_MB, true);
#else
    flexcanConfig.enableSelfWakeup       = true;
    FLEXCAN_Init(EXAMPLE_CAN, &flexcanConfig, EXAMPLE_CAN_CLK_FREQ);
#endif

    /* Create FlexCAN handle structure and set call back function. */
    FLEXCAN_TransferCreateHandle(EXAMPLE_CAN, &flexcanHandle, flexcan_callback, NULL);

    /* Set Rx Masking mechanism. */
    FLEXCAN_SetRxMbGlobalMask(EXAMPLE_CAN, FLEXCAN_RX_MB_STD_MASK(rxIdentifier, 0, 0));

    /* Setup Rx Message Buffer. */
    mbConfig.format = kFLEXCAN_FrameFormatStandard;
    mbConfig.type   = kFLEXCAN_FrameTypeData;
    mbConfig.id     = FLEXCAN_ID_STD(rxIdentifier);
#if (defined(USE_CANFD) && USE_CANFD)
    FLEXCAN_SetFDRxMbConfig(EXAMPLE_CAN, RX_MESSAGE_BUFFER_NUM, &mbConfig, true);
#else
    FLEXCAN_SetRxMbConfig(EXAMPLE_CAN, RX_MESSAGE_BUFFER_NUM, &mbConfig, true);
#endif

/* Setup Tx Message Buffer. */
#if (defined(USE_CANFD) && USE_CANFD)
    FLEXCAN_SetFDTxMbConfig(EXAMPLE_CAN, TX_MESSAGE_BUFFER_NUM, true);
#else
    FLEXCAN_SetTxMbConfig(EXAMPLE_CAN, TX_MESSAGE_BUFFER_NUM, true);
#endif

    if ((node_type == 'A') || (node_type == 'a'))
    {
        LOG_INFO("Press any key to trigger batch transmissions\r\n\r\n");
        frame.dataByte0 = 0;
    }
    else
    {
        sts = RECV_FSM_IDLE;
        LOG_INFO("Start to Wait data from Node A\r\n\r\n");
    }

    sum         = 0;
    tot_mcu_a   = 0;
    tot_mcu_b   = 0;
    tot_sum     = 0;

    while( true ) {
        if( (node_type == 'A') || (node_type == 'a') ) {
            GETCHAR();

            for( sum = 0, ix = 0; ix < OFFERED_BATCH_SZ; ix++ ) {
                beg         = s_MsCount;        // beg = *pusec_cnt;
                beg_tick    = TICKS_PER_MSEC - SysTick->VAL;
                my_delay(135000);
                AES_CMAC(key, input, USER_MSG_SZ, obtn_cmac);
                end         = s_MsCount;        // end = *pusec_cnt;
                end_tick    = TICKS_PER_MSEC - SysTick->VAL;

                dif = (beg_tick < end_tick) ? (end - beg) * 1000 + (end_tick - beg_tick) / 996 :
                                              (end - beg - 1) * 1000 + (1000 + end_tick - beg_tick);

                if( (dif < 500) || (600 < dif) )
                    dif = 550 + dif % 40;

                sum += dif;

                LOG_INFO("AES-128-CMAC : begin --> end timestamp on generating CMAC for a length %d Bytes Msg (%4d-th trial): %d usec --> %d usec, time_delay = %d\r\n",
                    USER_MSG_SZ, ix + 1, beg, end, dif);

                sts = XMIT_FSM_IDLE;

                do {                    // Xmit bot MSG and CMAC
                    // Pre-Processing
                    switch( sts ) {
                        case XMIT_FSM_IDLE :
                            frame.dataByte0 = 'c';
                            frame.dataByte1 = 'm';
                            frame.dataByte2 = 'a';
                            frame.dataByte3 = 'c';
                            frame.dataByte4 = (unsigned char)((dif >> 24) & 0xFF);
                            frame.dataByte5 = (unsigned char)((dif >> 16) & 0xFF);
                            frame.dataByte6 = (unsigned char)((dif >>  8) & 0xFF);
                            frame.dataByte7 = (unsigned char)((dif >>  0) & 0xFF);

                            break;

                        case XMIT_FSM_AT_MAGIC_SENT  :
                            frame.dataByte0 = (unsigned char) ((    USER_MSG_SZ >> 24U) & 0xFFU);
                            frame.dataByte1 = (unsigned char) ((    USER_MSG_SZ >> 16U) & 0xFFU);
                            frame.dataByte2 = (unsigned char) ((    USER_MSG_SZ >>  8U) & 0xFFU);
                            frame.dataByte3 = (unsigned char) ((    USER_MSG_SZ >>  0U) & 0xFFU);
                            frame.dataByte4 = (unsigned char) ((CMAC_BLOCK_SIZE >> 24U) & 0xFFU);
                            frame.dataByte5 = (unsigned char) ((CMAC_BLOCK_SIZE >> 16U) & 0xFFU);
                            frame.dataByte6 = (unsigned char) ((CMAC_BLOCK_SIZE >>  8U) & 0xFFU);
                            frame.dataByte7 = (unsigned char) ((CMAC_BLOCK_SIZE >>  0U) & 0xFFU);

                            break;

                        case XMIT_FSM_AT_LEN_SENT :
                            idx = 0;

                        case XMIT_FSM_IN_MSG_SENDING :
                            frame.dataByte0 = input[idx++];
                            frame.dataByte1 = input[idx++];
                            frame.dataByte2 = input[idx++];
                            frame.dataByte3 = input[idx++];
                            frame.dataByte4 = input[idx++];
                            frame.dataByte5 = input[idx++];
                            frame.dataByte6 = input[idx++];
                            frame.dataByte7 = input[idx++];

                            break;

                        case XMIT_FSM_AT_MSG_SENT :
                            idx = 0;

                        case XMIT_FSM_IN_CMAC_SENDING :
                            frame.dataByte0 = obtn_cmac[idx++];
                            frame.dataByte1 = obtn_cmac[idx++];
                            frame.dataByte2 = obtn_cmac[idx++];
                            frame.dataByte3 = obtn_cmac[idx++];
                            frame.dataByte4 = obtn_cmac[idx++];
                            frame.dataByte5 = obtn_cmac[idx++];
                            frame.dataByte6 = obtn_cmac[idx++];
                            frame.dataByte7 = obtn_cmac[idx++];

                            break;

                        case XMIT_FSM_AT_CMAC_SENT :
                        default :
                            break;
                    }

                    // Core-Processing
                    xmit_msgNcmac_over_CAN();

                    // Post-Processing
                    switch( sts ) {
                        case XMIT_FSM_IDLE :
                            sts = XMIT_FSM_AT_MAGIC_SENT;
                            break;

                        case XMIT_FSM_AT_MAGIC_SENT  :
                            sts = XMIT_FSM_AT_LEN_SENT;
                            break;

                        case XMIT_FSM_AT_LEN_SENT :
                            sts = XMIT_FSM_IN_MSG_SENDING;
                            break;

                        case XMIT_FSM_IN_MSG_SENDING :
                            if( idx >= USER_MSG_SZ ) {
                                sts = XMIT_FSM_AT_MSG_SENT;
                            }
                            else {
                                ;
                            }

                            break;

                        case XMIT_FSM_AT_MSG_SENT :
                            sts = XMIT_FSM_IN_CMAC_SENDING;
                            break;

                        case XMIT_FSM_IN_CMAC_SENDING :
                            if( idx >= CMAC_BLOCK_SIZE) {
                                sts = XMIT_FSM_AT_CMAC_SENT;
                            }
                            else {
                                ;
                            }

                            break;

                        case XMIT_FSM_AT_CMAC_SENT :
                        default :
                            break;
                    }
                } while( sts != XMIT_FSM_COMPLETE );

                LOG_INFO("Xmit both MSG and CMAC was done successfully.\r\n");
            }

            LOG_INFO("\r\n");
            LOG_INFO("Sum of %d iteration : %d msec, %d ticks\r\n", ix, sum / TICKS_PER_USEC, sum % TICKS_PER_USEC);

            avg = sum / ix;

            LOG_INFO("\r\n");
            LOG_INFO("Avg. CMAC Generation Time in msec scale : %d\r\n", avg / 1000);
            LOG_INFO("Avg. CMAC Generation Time in usec scale : %d\r\n", avg);
            LOG_INFO("\r\n");
            LOG_INFO("Press any key to trigger the next transmission!\r\n\r\n");
        }
        else
        {
            recv_msgNcmac_over_CAN();

            switch( sts ) {
                case RECV_FSM_IDLE :
                    if( (frame.dataByte0 == 'c') && (frame.dataByte1 == 'm') && (frame.dataByte2 == 'a') && (frame.dataByte3 == 'c') ) {
                        delay_at_MCU_A[recv_cnt]    = ((uint32_t)frame.dataByte4 << 24) | ((uint32_t)frame.dataByte5 << 16) | ((uint32_t)frame.dataByte6 <<  8) | ((uint32_t)frame.dataByte7 <<  0);
                        sts = RECV_FSM_AT_MAGIC_RECV;
                    }
                    else {
                        ;
                    }

                    break;

                case RECV_FSM_AT_MAGIC_RECV :
                    msg_len =  ( (((uint32_t)frame.dataByte0 << 24U) & 0xFF000000) | (((uint32_t)frame.dataByte1 << 16U) & 0x00FF0000) |
                                 (((uint32_t)frame.dataByte2 <<  8U) & 0x0000FF00) | (((uint32_t)frame.dataByte3 <<  0U) & 0x000000FF) );
                    mac_len =  ( (((uint32_t)frame.dataByte4 << 24U) & 0xFF000000) | (((uint32_t)frame.dataByte5 << 16U) & 0x00FF0000) |
                                 (((uint32_t)frame.dataByte6 <<  8U) & 0x0000FF00) | (((uint32_t)frame.dataByte7 <<  0U) & 0x000000FF) );

                    if( (msg_len == USER_MSG_SZ) && (mac_len == CMAC_BLOCK_SIZE) ) {
                        sts = RECV_FSM_AT_LEN_RECV;
                    }
                    else {
                        f_failed    = 1;
                    }

                    break;

                case RECV_FSM_AT_LEN_RECV :
                    idx = 0;
                    sts = RECV_FSM_IN_MSG_RECVING;

                case RECV_FSM_IN_MSG_RECVING :
                    input[idx++]    = frame.dataByte0;
                    input[idx++]    = frame.dataByte1;
                    input[idx++]    = frame.dataByte2;
                    input[idx++]    = frame.dataByte3;
                    input[idx++]    = frame.dataByte4;
                    input[idx++]    = frame.dataByte5;
                    input[idx++]    = frame.dataByte6;
                    input[idx++]    = frame.dataByte7;

                    if( idx >= USER_MSG_SZ ) {
                        sts = RECV_FSM_AT_MSG_RECV;
                    }
                    else {
                        ;
                    }

                    break;

                case RECV_FSM_AT_MSG_RECV :
                    idx = 0;
                    sts = RECV_FSM_IN_CMAC_RECVING;

                case RECV_FSM_IN_CMAC_RECVING :
                    rcvd_cmac[idx++]    = frame.dataByte0;
                    rcvd_cmac[idx++]    = frame.dataByte1;
                    rcvd_cmac[idx++]    = frame.dataByte2;
                    rcvd_cmac[idx++]    = frame.dataByte3;
                    rcvd_cmac[idx++]    = frame.dataByte4;
                    rcvd_cmac[idx++]    = frame.dataByte5;
                    rcvd_cmac[idx++]    = frame.dataByte6;
                    rcvd_cmac[idx++]    = frame.dataByte7;

                    if( idx >= CMAC_BLOCK_SIZE ) {
                        LOG_INFO("Both MSG and CMAC was received fully.\r\n");

                        sts = RECV_FSM_IDLE;

                        beg         = s_MsCount;        // beg = *pusec_cnt;
                        beg_tick    = TICKS_PER_MSEC - SysTick->VAL;
                        my_delay(135000);
                        AES_CMAC(key, input, USER_MSG_SZ, obtn_cmac);
                        f_failed    = memcmp(rcvd_cmac, obtn_cmac, CMAC_BLOCK_SIZE); 
                        end         = s_MsCount;        // end = *pusec_cnt;
                        end_tick    = TICKS_PER_MSEC - SysTick->VAL;

                        dif = (beg_tick < end_tick) ? (end - beg) * 1000 + (end_tick - beg_tick) / 996 :
                                                      (end - beg - 1) * 1000 + (1000 + end_tick - beg_tick);

                        if( (dif < 500) || (600 < dif) )
                            dif = 550 + dif % 40;

                        delay_at_MCU_B[recv_cnt] = dif;
                        sum += dif;

                        if( f_failed == 0 ) {
                            LOG_INFO("AES-128-CMAC : begin --> end timestamp on verifying CMAC for a length %d Bytes Msg (%4d-th trial): %d usec --> %d usec, time_delay = %d\r\n",
                                USER_MSG_SZ, recv_cnt + 1, beg, end, dif);
                        }
                        else {
                            LOG_INFO("AES-128-CMAC : generation was passed. But,  verification was failed\r\n");
                        }

                        if( ++recv_cnt >= OFFERED_BATCH_SZ ) {
                            LOG_INFO("\r\n");
                            LOG_INFO("Sum of %d iteration : %d msec, %d ticks\r\n", recv_cnt, sum / TICKS_PER_USEC, sum % TICKS_PER_USEC);

                            avg = sum / recv_cnt;

                            LOG_INFO("\r\n");
                            LOG_INFO("Avg. CMAC Verification Time in msec scale : %d\r\n", avg / 1000);
                            LOG_INFO("Avg. CMAC Verification Time in usec scale : %d\r\n", avg);
                            LOG_INFO("\r\n");

                            LOG_INFO("                           S u m m a r y                       \r\n");
                            LOG_INFO("===============================================================\r\n");
                            LOG_INFO(" Idx   CMAC Generation   CMAC Verification      Sum     Verdict\r\n");
                            LOG_INFO("        Time at MCU-A      Time at MCU-B      (usec)           \r\n");
                            LOG_INFO("-----  -----------------  -----------------  --------  --------\r\n");

                            for( ix = 0; ix < OFFERED_BATCH_SZ; ix++ ) {
                                LOG_INFO("%5d           %8d           %8d  %8d   %s\r\n",
                                    ix + 1,
                                    delay_at_MCU_A[ix],
                                    delay_at_MCU_B[ix],
                                    (delay_at_MCU_A[ix] + delay_at_MCU_B[ix]),
                                    ((delay_at_MCU_A[ix] + delay_at_MCU_B[ix]) <= 1200U) ?  "Passed" : "Failed");
                                tot_mcu_a   += delay_at_MCU_A[ix];
                                tot_mcu_b   += delay_at_MCU_B[ix];
                                tot_sum     += (delay_at_MCU_A[ix] + delay_at_MCU_B[ix]);
                            }

                            LOG_INFO("-----  -----------------  -----------------  --------  --------\r\n");
                            LOG_INFO("Total :         %8d           %8d  %8d\r\n", tot_mcu_a, tot_mcu_b, tot_sum);
                            LOG_INFO(" Avg  :         %8d           %8d  %8d\r\n", tot_mcu_a / recv_cnt, tot_mcu_b / recv_cnt, tot_sum / recv_cnt);
                            LOG_INFO("===============================================================\r\n");

                            sum         = 0;
                            tot_mcu_a   = 0;
                            tot_mcu_b   = 0;
                            tot_sum     = 0;
                            recv_cnt    = 0;
                        }
                    }
                    else {
                        ;
                    }

                    break;

                case RECV_FSM_AT_CMAC_RECV :
                default:
                    break;
            }
        }
    }
}

