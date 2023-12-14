/******************************************************************************
 * Copyright (C) 2023 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 ******************************************************************************/

/**
 * @file    main.c
 * @brief   Main for KWS20
 * @details
 *
 *
 */

/* **** Includes **** */
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "mxc_sys.h"
#include "fcr_regs.h"
#include "icc.h"
#include "mxc_device.h"
#include "mxc_delay.h"
#include "nvic_table.h"
#include "i2s_regs.h"
#include "board.h"
#include "mxc.h"
#include "i2s.h"
#include "tmr.h"
#include "dma.h"
#include "led.h"



#include "cnn.h"

#include "sd.h"

/*********************zzzzzzzzz*********************ImgCapture相关**************************************************/

#include <stdlib.h>

#include "camera.h"
#include "example_config.h"

#ifndef CNN_MEMUTILS_H  // 文件名大写加下划线的形式
#define CNN_MEMUTILS_H

// 这里是头文件的内容
#include "cnn_memutils.h"

#endif  // CNN_MEMUTILS_H


#define SD

/**************************zzzzzzzzzzzzzz**************************************************************************/

#ifdef BOARD_FTHR_REVA
#include "tft_ili9341.h"
#endif


#include <math.h>

#define VERSION "3.2.3 (5/05/23)" // trained with background noise and more unknown keywords
/* **** Definitions **** */
#define CLOCK_SOURCE 0 // 0: IPO,  1: ISO, 2: IBRO
#define SLEEP_MODE 0 // 0: no sleep,  1: sleep,   2:deepsleep(LPM)
#define WUT_ENABLE // enables WUT timer
#define WUT_USEC 380 // continuous WUT duration close to I2S polling time in usec
//#define ENERGY            // if enabled, turn off LED2, toggle LED1 for 10sec for energy measurements on Power monitor (System Power)





/* Enable/Disable Features */
#define ENABLE_PRINT_ENVELOPE // enables printing average waveform envelope for samples
//#define ENABLE_CLASSIFICATION_DISPLAY  // enables printing classification result
#define ENABLE_SILENCE_DETECTION // Starts collecting only after avg > THRESHOLD_HIGH, otherwise starts from first sample
#undef EIGHT_BIT_SAMPLES // samples from Mic or Test vectors are eight bit, otherwise 16-bit
#define ENABLE_MIC_PROCESSING // enables capturing Mic, otherwise a header file Test vector is used as sample data

#ifndef ENABLE_MIC_PROCESSING
#include "kws_five.h"
#else
#undef ENABLE_PRINT_ENVELOPE // print is slow with live mic data
#endif

#define CON_BAUD 1 * 115200

/*-----------------------------*/
/* keep following unchanged */
#define SAMPLE_SIZE 16384 // size of input vector for CNN, keep it multiple of 128
#define CHUNK \
    128 // number of data points to read at a time and average for threshold, keep multiple of 128
#define TRANSPOSE_WIDTH 128 // width of 2d data model to be used for transpose
#define NUM_OUTPUTS CNN_NUM_OUTPUTS // number of classes
#define I2S_RX_BUFFER_SIZE 64 // I2S buffer size
#define TFT_BUFF_SIZE 50 // TFT buffer size
/*-----------------------------*/

/* Adjustable Parameters */
#ifdef ENABLE_MIC_PROCESSING
#define SAMPLE_SCALE_FACTOR \
    4 // multiplies 16-bit samples by this scale factor before converting to 8-bit
#define THRESHOLD_HIGH 350 // voice detection threshold to find beginning of a keyword
#define THRESHOLD_LOW 100 // voice detection threshold to find end of a keyword
#define SILENCE_COUNTER_THRESHOLD \
    20 // [>20] number of back to back CHUNK periods with avg < THRESHOLD_LOW to declare the end of a word
#define PREAMBLE_SIZE 30 * CHUNK // how many samples before beginning of a keyword to include
#define INFERENCE_THRESHOLD 91 // min probability (0-100) to accept an inference
#else
#define SAMPLE_SCALE_FACTOR \
    1 // multiplies 16-bit samples by this scale factor before converting to 8-bit
#define THRESHOLD_HIGH 130 // voice detection threshold to find beginning of a keyword
#define THRESHOLD_LOW 70 // voice detection threshold to find end of a keyword
#define SILENCE_COUNTER_THRESHOLD \
    20 // [>20] number of back to back CHUNK periods with avg < THRESHOLD_LOW to declare the end of a word
#define PREAMBLE_SIZE 30 * CHUNK // how many samples before beginning of a keyword to include
#define INFERENCE_THRESHOLD 49 // min probability (0-100) to accept an inference
#endif


/* DEBUG Print */
#ifdef ENERGY
#define PR_DEBUG(fmt, args...) \
    if (0)                     \
    printf(fmt, ##args)
#define PR_INFO(fmt, args...) \
    if (1)                    \
    printf(fmt, ##args)
#else
#define PR_DEBUG(fmt, args...) \
    if (1)                     \
    printf(fmt, ##args)
#define PR_INFO(fmt, args...) \
    if (1)                    \
    printf(fmt, ##args)
#endif

/* **** Globals **** */


/**********zzzzzzzzzzzzzz********************************ImgCapture相关**************************************************/
char g_serial_buffer[256];
int g_buffer_index = 0;
static mxc_uart_regs_t *Con_Uart = MXC_UART_GET_UART(CONSOLE_UART);

int my_sd_cheak; 
char filename[100];
//根目录下的PHOTO
const char *myDir = "/PHOTO";
//文件前缀
const char *prefix = "photo_";
//文件编号后缀
int myFlag = 0;

/***************zzzzzzzzzzzzzzzzz***************************************************************************************/

volatile uint32_t cnn_time; // Stopwatch
volatile uint32_t fileCount = 0;

int8_t micBuff[SAMPLE_SIZE];
int micBufIndex = 0;



int utteranceIndex = 0;
uint16_t utteranceAvg = 0;
int zeroPad = 0;

static int32_t ml_data[NUM_OUTPUTS];
static q15_t ml_softmax[NUM_OUTPUTS];
uint8_t pAI85Buffer[SAMPLE_SIZE];

int16_t Max, Min;
uint16_t thresholdHigh = THRESHOLD_HIGH;
uint16_t thresholdLow = THRESHOLD_LOW;

volatile uint8_t i2s_flag = 0;
int32_t i2s_rx_buffer[I2S_RX_BUFFER_SIZE];

/* **** Constants **** */
typedef enum _mic_processing_state {
    STOP = 0, /* No processing  */
    SILENCE = 1, /* Threshold not detected yet  */
    KEYWORD = 2 /* Threshold has been detected, gathering keyword samples */
} mic_processing_state;


 /****************zzzzzzzzzzzzz**************************ImgCapture相关**************************************************/
 // This describes a complete image for a standard blocking capture
typedef struct {
    uint8_t *raw; // Pointer to raw img data in SRAM.
    uint32_t imglen; // Length of img data (in bytes)
    uint32_t w; // Width of the image (in pixels)
    uint32_t h; // Height of the image (in pixels)
    uint8_t *pixel_format; // Pixel format string
} img_data_t;

// This describes a complete image for a streaming capture saved
// to CNN data SRAM
typedef struct {
    uint32_t *raw; // Pointer to raw img data in CNN data SRAM.
    uint32_t imglen; // Length of img data (in bytes)
    uint32_t w; // Width of the image (in pixels)
    uint32_t h; // Height of the image (in pixels)
    uint8_t *pixel_format; // Pixel format string
} cnn_img_data_t;

typedef enum { BAYER_FUNCTION_PASSTHROUGH = 0, BAYER_FUNCTION_BILINEAR } bayer_function_t;

// This contains global application settings
typedef struct {
    int dma_channel;
    dmamode_t dma_mode;
    unsigned int imgres_w;
    unsigned int imgres_h;
    pixformat_t pixel_format;
    bayer_function_t bayer_function;
} app_settings_t;

app_settings_t g_app_settings;

    /***************zzzzzzzzzzzzzzzzzz*************************************************************************************/
/* Set of detected words */
const char keywords[NUM_OUTPUTS][10] = { "kaca","Unknown" };

void i2s_isr(void)
{
    i2s_flag = 1;
    /* Clear I2S interrupt flag */
    MXC_I2S_ClearFlags(MXC_F_I2S_INTFL_RX_THD_CH0);
}


/* **** Functions Prototypes **** */


void fail(void);
uint8_t cnn_load_data(uint8_t *pIn);
uint8_t MicReadChunk(uint16_t *avg);
uint8_t AddTranspose(uint8_t *pIn, uint8_t *pOut, uint16_t inSize, uint16_t outSize,
                     uint16_t width);
uint8_t check_inference(q15_t *ml_soft, int32_t *ml_data, int16_t *out_class, double *out_prob);
void I2SInit();
void HPF_init(void);
int16_t HPF(int16_t input);


/*******************zzzzzzz***********************ImgCapture相关**************************************************/

void clear_serial_buffer(void);
img_data_t capture_img(uint32_t w, uint32_t h, pixformat_t pixel_format, dmamode_t dma_mode,
                       int dma_channel);
void transmit_capture_uart(img_data_t img_data);
cnn_img_data_t stream_img(uint32_t w, uint32_t h, pixformat_t pixel_format, int dma_channel);
void transmit_stream_uart(cnn_img_data_t img_data);
void save_stream_sd(cnn_img_data_t img_data, char *file);





void clear_serial_buffer(void)
{
    memset(g_serial_buffer, '\0', 256);
    g_buffer_index = 0;
}



/**
* @brief Capture an image using a standard blocking PCIF capture.
* @param[in] w Set the width of the image (in pixels)
* @param[in] h Set the height of the image (in pixels)
* @param[in] pixel_format Set the pixel format.  See 'pixformat_t' in 'camera.h'
* @param[in] dma_mode Set the dma mode format.  Should be either "USE_DMA" or "NO_DMA" for standard captures.
* @param[in] dma_channel DMA channel to use if DMA mode is "USE_DMA".  Must be acquired by the application first.
* @return "img_data_t" struct describing the captured image.  If the "raw" struct member is NULL, the image capture
* failed.
* @details 
    An SRAM buffer is allocated to hold the entire image, limiting the max resolution to the available memory on the device.
    This function contains the full setup sequence for capturing an image.  
    In practice, step 1 can be broken out into a separate initialization 
    sequence if you're not reconfiguring the camera settings on the fly.
****************************************************************************/
img_data_t capture_img(uint32_t w, uint32_t h, pixformat_t pixel_format, dmamode_t dma_mode,
                       int dma_channel)
{
    img_data_t img_data;

    // 1. Configure the camera with the 'camera_setup' function.
    // Image dimensions should fall within the limitations
    // of the camera hardware and MCU SRAM limits.  In this simple capture mode the
    // camera.h drivers will allocate an SRAM buffer whose size is equal to
    // width * height * bytes_per_pixel.  See camera.c for implementation details.
    printf("Configuring camera\n");
    fifomode_t fifo_mode = (pixel_format == PIXFORMAT_RGB888) ? FIFO_THREE_BYTE : FIFO_FOUR_BYTE;
    int ret = camera_setup(w, // width
                           h, // height
                           pixel_format, // pixel format
                           fifo_mode, // FIFO mode (four bytes is suitable for most cases)
                           dma_mode, // DMA (enabling DMA will drastically decrease capture time)
                           dma_channel); // Allocate the DMA channel retrieved in initialization

    // Error check the setup function.
    if (ret != STATUS_OK) {
        printf("Failed to configure camera!  Error %i\n", ret);
        img_data.raw = NULL;
        return img_data;
    }

    // 2. Start the camera capture with the 'camera_start_capture_image()' function.
    // This will 'launch' the image capture, but is non-blocking.  The CameraIF peripheral's
    // hardware handles the capture in the background.
    printf("Capturing image\n");
    MXC_TMR_SW_Start(MXC_TMR0);
    camera_start_capture_image();

    // 3. Wait until the image is fully received.
    while (!camera_is_image_rcv()) {}
    int elapsed_us = MXC_TMR_SW_Stop(MXC_TMR0);
    printf("Done! (Took %i us)\n", elapsed_us);

    // 4. Retrieve details of the captured image from the camera driver with the
    // 'camera_get_image' function.  We don't need to copy any image data here, we'll
    // just retrieve a pointer to the camera driver's internal SRAM buffer.
    img_data.pixel_format = camera_get_pixel_format(); // Retrieve the pixel format of the image
    camera_get_image(&img_data.raw, &img_data.imglen, &img_data.w,
                     &img_data.h); // Retrieve info using driver function.
    printf("Captured %ux%u %s image to buffer location 0x%x (%i bytes)\n", img_data.w, img_data.h,
           img_data.pixel_format, img_data.raw, img_data.imglen);

    // 5. At this point, "img_data.raw" is pointing to the fully captured
    // image data, and all the info needed to decode it has been collected.
    return img_data;
}

/**
* @brief Transmit an image that has been captured with 'capture_img' over UART.
* @param[in] img_data The cnn_img_data_t struct describing the stored image.  This
* struct will be provided from a call to 'stream_img'.
* @details
    This function will send a header first, then the raw image data.  The header is
    of the following format:
    *IMG* [PIXEL FORMAT] [LENGTH (in bytes)] [WIDTH (in pixels)] [HEIGHT (in pixels)]

    When the console receives a header of this format, it enters into a "receive raw bytes"
    mode that listens for LENGTH bytes.  
    
    This is necessary because the raw image data has a non-zero probability to contain any 
    sequence of ASCII characters, including '\n', '\r\n', etc.
    So the console cannot rely on the same 'readline()' polling as it does for standard
    commands.
****************************************************************************/
void transmit_capture_uart(img_data_t img_data)
{
    if (img_data.raw != NULL) {
        // Send the image data over the serial port...
        MXC_TMR_SW_Start(MXC_TMR0);

        // First, tell the host that we're about to send the image.
        clear_serial_buffer();
        snprintf(g_serial_buffer, 256,
                 "*IMG* %s %i %i %i", // Format img info into a string
                 img_data.pixel_format, img_data.imglen, img_data.w, img_data.h);
        send_msg(g_serial_buffer);

        // The console should now be expecting to receive 'imglen' bytes.

        // Since standard image captures are buffered into SRAM, sending them
        // over the serial port is straightforward...
        clear_serial_buffer();
        MXC_UART_WriteBytes(Con_Uart, img_data.raw, img_data.imglen);

        int elapsed = MXC_TMR_SW_Stop(MXC_TMR0);
        printf("Done! (serial transmission took %i us)\n", elapsed);
    }
}

/**
* @brief Capture an image using DMA streaming mode.  This method only requires two
* rows worth of SRAM, allowing for high image resolutions.
* @param[in] w Set the width of the image (in pixels)
* @param[in] h Set the height of the image (in pixels)
* @param[in] pixel_format Set the pixel format.  See 'pixformat_t' in 'camera.h'
* @param[in] dma_channel DMA channel to use if DMA mode is "USE_DMA".  Must be acquired by the application first.
* @return "img_data_t" struct describing the captured image.  If the "raw" struct member is NULL, the image capture
* failed.
* @details 
    This method has a much lower memory footprint, but dramatically increased 
    timing requirements for higher resolutions.  Image data must be streamed to
    a destination with a high enough "sink" datarate to deal with the massive "source" 
    datarate coming from the PCIF.

    For the sake of this example, image data is streamed directly into the CNN
    accelerator's data SRAM.  **This is not the correct way to load data for inferences**
    The CNN accelerator is the only device with enough memory to store high resolution
    images, so this example essentially turns it into a giant SRAM buffer.
    This method allows for collecting high resolution images at full speed to send
    to any arbitrary lower bandwidth output destination.
****************************************************************************/
cnn_img_data_t stream_img(uint32_t w, uint32_t h, pixformat_t pixel_format, int dma_channel)
{
    // Enable CNN accelerator memory.
    // CNN clock: APB (50 MHz - PCLK) div 1
    cnn_enable((0 << MXC_F_GCR_PCLKDIV_CNNCLKSEL_POS), MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV1);
    cnn_init();

    cnn_img_data_t img_data;

    // Resolution check.  This method only supports resolutions that are multiples of 32.
    // Additionally, resolutions beyond 352x352 may result in image artifacts.
    if ((w * h) % 32 != 0) {
        img_data.raw = NULL;
        printf("Failed to stream!  Image resolutions must be multiples of 32.\n");
        return img_data;
    }

    // 1. Configure the camera.  This is the same as the standard blocking capture, except
    // the DMA mode is set to "STREAMING_DMA".
    printf("Configuring camera\n");
    fifomode_t fifo_mode = (pixel_format == PIXFORMAT_RGB888) ? FIFO_THREE_BYTE : FIFO_FOUR_BYTE;
    int ret = camera_setup(w, // width
                           h, // height
                           pixel_format, // pixel format
                           fifo_mode, // FIFO mode
                           STREAMING_DMA, // Set streaming mode
                           dma_channel); // Allocate the DMA channel retrieved in initialization

    // Error check the setup function.
    if (ret != STATUS_OK) {
        printf("Failed to configure camera!  Error %i\n", ret);
        img_data.raw = NULL;
        return img_data;
    }

    // 2. Retrieve image format and info.
    img_data.pixel_format = camera_get_pixel_format(); // Retrieve the pixel format of the image
    camera_get_image(NULL, &img_data.imglen, &img_data.w,
                     &img_data.h); // Retrieve info using driver function.
    img_data.raw = (uint32_t *)
        CNN_QUAD0_DSRAM_START; // Manually save the destination address at the first quadrant of CNN data SRAM

    printf("Starting streaming capture...\n");
    MXC_TMR_SW_Start(MXC_TMR0);

    // 3. Start streaming
    camera_start_capture_image();

    uint8_t *data = NULL;
    int buffer_size = camera_get_stream_buffer_size();
    uint32_t *cnn_addr = img_data.raw; // Hard-coded to Quadrant 0 starting address

    // 4. Process the incoming stream data.
    while (!camera_is_image_rcv()) {
        if ((data = get_camera_stream_buffer()) !=
            NULL) { // The stream buffer will return 'NULL' until an image row is received.
            // 5. Unload buffer
            cnn_addr = write_bytes_to_cnn_sram(data, buffer_size, cnn_addr);
            // 6. Release buffer in time for next row
            release_camera_stream_buffer();
        }
    }

    int elapsed_us = MXC_TMR_SW_Stop(MXC_TMR0);
    printf("Done! (Took %i us)\n", elapsed_us);

    // 7. Check for any overflow
    stream_stat_t *stat = get_camera_stream_statistic();
    printf("DMA transfer count = %d\n", stat->dma_transfer_count);
    printf("OVERFLOW = %d\n", stat->overflow_count);

    return img_data;
}

/**
* @brief Transmit an image that has been captured with 'stream_img' over UART.
* @param[in] img_data The cnn_img_data_t struct describing the stored image.  This
* struct will be provided from a call to 'stream_img'.
* @details
    This function will send a header first, then the raw image data.  The header is
    of the following format:
    *IMG* [PIXEL FORMAT] [LENGTH (in bytes)] [WIDTH (in pixels)] [HEIGHT (in pixels)]

    When the console receives a header of this format, it enters into a "receive raw bytes"
    mode that listens for LENGTH bytes.  
    
    This is necessary because the raw image data has a non-zero probability to contain any 
    sequence of ASCII characters, including '\n', '\r\n', etc.
    So the console cannot rely on the same 'readline()' polling as it does for standard
    commands.
****************************************************************************/
void transmit_stream_uart(cnn_img_data_t img_data)
{
    if (img_data.raw !=
        NULL) { // If img_data.raw is NULL, then there was an error collecting the image.
        printf("Transmitting image data over UART...\n");
        MXC_TMR_SW_Start(MXC_TMR0);

        // Tell the host console we're about to send an image.
        clear_serial_buffer();
        snprintf(g_serial_buffer, 256,
                 "*IMG* %s %i %i %i", // Format img info into a string
                 img_data.pixel_format, img_data.imglen, img_data.w, img_data.h);
        send_msg(g_serial_buffer);

        // Console should be ready to receive raw bytes now.

        clear_serial_buffer();
        int transfer_len = 256;
        uint32_t *cnn_addr = img_data.raw;

        // Transfer the bytes out of CNN memory and into the serial buffer, then write.
        // Since the CNN data SRAM is non-contiguous, some pointer manipulation at the
        // quadrant boundaries is required.
        for (int i = 0; i < img_data.imglen; i += transfer_len) {
            cnn_addr = read_bytes_from_cnn_sram((uint8_t *)g_serial_buffer, transfer_len, cnn_addr);
            MXC_UART_WriteBytes(Con_Uart, (uint8_t *)g_serial_buffer, transfer_len);
        }

        int elapsed = MXC_TMR_SW_Stop(MXC_TMR0);
        printf("Done! (serial transmission took %i us)\n", elapsed);
    }
}

#ifdef SD

/**
* @brief Save an image that has been captured with 'stream_img' to an SD card.
* @param[in] img_data The cnn_img_data_t struct describing the stored image.  This
* struct will be provided from a call to 'stream_img'.
* @param[in] file Filename to save to.
* @details
    The same image info header used for UART streaming will also be written to the
    first line of the raw image file.  Then, the raw image data will be written.
    This allows the raw image data to be decoded later.  
    
    It also allows you to quickly verify that the image has been saved to the SD card 
    with the "cat" command, which prints out the contents of a file.  The 
    Python console is always checking for the *IMG* ... header format.  So "cat"
    will print out the image header followed by the raw image data, just like
    'transmit_stream_uart'.  As a result, this can be used to read an image file off
    the SD card and the console will save it as a .png on the host PC.
****************************************************************************/
void save_stream_sd(cnn_img_data_t img_data, char *file)
{
    if (img_data.raw != NULL) { // Image data will be NULL if something went wrong during streaming
        // If file is NULL, find the next available file to save to.
        if (file == NULL) {
            int i = 0;

            for (;;) {
                // We'll use the global sd_filename buffer for this and
                // try to find /raw/imgN while incrementing N.
                memset(sd_filename, '\0', sizeof(sd_filename));
                snprintf(sd_filename, sizeof(sd_filename), "/raw/img%u", i++);
                sd_err = f_stat(sd_filename, &sd_fno);
                if (sd_err == FR_NO_FILE) {
                    file = sd_filename; // Point 'file' to the available path string
                    break;
                } else if (sd_err != FR_OK) {
                    printf("Error while searching for next available file: %s\n",
                           FR_ERRORS[sd_err]);
                    break;
                }
            }
        }

        sd_err = f_open(&sd_file, (const TCHAR *)file, FA_WRITE | FA_CREATE_NEW);

        if (sd_err != FR_OK) {
            printf("Error opening file: %s\n", FR_ERRORS[sd_err]);
        } else {
            printf("Saving image to %s\n", file);

            MXC_TMR_SW_Start(MXC_TMR0);

            // Write image info as the first line of the file.
            clear_serial_buffer();
            snprintf(g_serial_buffer, 256,
                     "*IMG* %s %i %i %i\n", // Format img info into a new-line terminated string
                     img_data.pixel_format, img_data.imglen, img_data.w, img_data.h);

            unsigned int wrote = 0;
            sd_err = f_write(&sd_file, g_serial_buffer, strlen(g_serial_buffer), &wrote);
            if (sd_err != FR_OK || wrote != strlen(g_serial_buffer)) {
                printf("Failed to write header to file: %s\n", FR_ERRORS[sd_err]);
            }
            clear_serial_buffer();

            // Similar to streaming over UART, a secondary buffer is needed to
            // save the raw data to the SD card since the CNN data SRAM is non-contiguous.
            // Raw image data is written row by row.
            uint32_t *cnn_addr = img_data.raw;
            uint8_t *buffer = (uint8_t *)malloc(img_data.w);
            for (int i = 0; i < img_data.imglen; i += img_data.w) {
                cnn_addr = read_bytes_from_cnn_sram(buffer, img_data.w, cnn_addr);
                sd_err = f_write(&sd_file, buffer, img_data.w, &wrote);

                if (sd_err != FR_OK || wrote != img_data.w) {
                    printf("Failed to image data to file: %s\n", FR_ERRORS[sd_err]);
                }

                // Print progress %
                if (i % (img_data.w * 32) == 0) {
                    printf("%.1f%%\n", ((float)i / img_data.imglen) * 100.0f);
                }
            }
            free(buffer);
            int elapsed = MXC_TMR_SW_Stop(MXC_TMR0);
            printf("Finished (took %ius)\n", elapsed);
        }
        f_close(&sd_file);
    }
}
#endif



/********************zzzzzzzzzzzzzzzzzzzzzzzzz*************************************************************************************/



int32_t tot_usec = -100000;
#ifdef WUT_ENABLE
void WUT_IRQHandler()
{
    i2s_flag = 1;
    MXC_WUT_IntClear();

    tot_usec += WUT_USEC;
    //LED_On(LED2);
    //LED_Off(LED2);
}
#endif

int console_UART_init(uint32_t baud)
{
    mxc_uart_regs_t *ConsoleUart = MXC_UART_GET_UART(CONSOLE_UART);
    int err;
    NVIC_ClearPendingIRQ(MXC_UART_GET_IRQ(CONSOLE_UART));
    NVIC_DisableIRQ(MXC_UART_GET_IRQ(CONSOLE_UART));
    NVIC_SetPriority(MXC_UART_GET_IRQ(CONSOLE_UART), 1);
    NVIC_EnableIRQ(MXC_UART_GET_IRQ(CONSOLE_UART));

    if ((err = MXC_UART_Init(ConsoleUart, baud, MXC_UART_IBRO_CLK)) != E_NO_ERROR) {
        return err;
    }

    return 0;
}

void testtttcnnINIT()
{
    /* Enable peripheral, enable CNN interrupt, turn on CNN clock */
    /* CNN clock: 50 MHz div 1 */
    cnn_enable(MXC_S_GCR_PCLKDIV_CNNCLKSEL_PCLK, MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV1);

    /* Configure P2.5, turn on the CNN Boost */
    cnn_boost_enable(MXC_GPIO2, MXC_GPIO_PIN_5);


#ifndef FIRST

    PR_INFO("\n\nANALOG DEVICES \nKeyword Spotting Demo\nVer. %s \n", VERSION);
    PR_INFO("\n***** Init *****\n");

#endif

    memset(pAI85Buffer, 0x0, sizeof(pAI85Buffer));

#ifndef FIRST
    // PR_DEBUG("pChunkBuff: %d\n", sizeof(pChunkBuff));testtttcnnINIT
    PR_DEBUG("pAI85Buffer: %d\n", sizeof(pAI85Buffer));
#endif

    /* Bring state machine into consistent state */
    cnn_init();
    /* Load kernels */
    cnn_load_weights();
    /* Configure state machine */
    cnn_configure();

#define FIRST
}




/* **************************************************************************** */

int main(void)
{

 /****************zzzzzzzzzzzzz**************************ImgCapture相关**************************************************/
        

    // Initialization...
    int ret = 0;
    int slaveAddress;
    int id;
    //初始化 照片规格
    g_app_settings.dma_mode = USE_DMA;
    g_app_settings.imgres_w = 240;
    g_app_settings.imgres_h = 240;
    g_app_settings.pixel_format = PIXFORMAT_RGB888; // This default may change during initialization


    /* Enable cache */
    // MXC_ICC_Enable(MXC_ICC0);

    /* Set system clock to 100 MHz */
    // MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IPO);
    // SystemCoreClockUpdate();



#ifdef SD
    sd_mount();
#endif

    // Initialize DMA and acquire a channel for the camera interface to use
    printf("Initializing DMA\n");
    MXC_DMA_Init();
    g_app_settings.dma_channel = MXC_DMA_AcquireChannel();

    // Initialize the camera driver.
    printf("Initializing camera\n");
    camera_init(CAMERA_FREQ);

    slaveAddress = camera_get_slave_address();
    printf("Camera I2C slave address: %02x\n", slaveAddress);

    ret = camera_get_manufacture_id(&id);
    if (ret != STATUS_OK) {
        printf("Error returned from reading camera id. Error %d\n", ret);
        return -1;
    }
    printf("Camera ID detected: %04x\n", id);



    // *********************************END CLEANME*************************************

    printf("Ready!\n");
    

    

    /***************zzzzzzzzzzzzzzzzzz*************************************************************************************/

    uint32_t sampleCounter = 0;
    mxc_tmr_unit_t units;

    uint8_t pChunkBuff[CHUNK];

    uint16_t avg = 0;
    uint16_t ai85Counter = 0;
    uint16_t wordCounter = 0;

    uint16_t avgSilenceCounter = 0;

    mic_processing_state procState = STOP;

#if defined(BOARD_FTHR_REVA)
    // Wait for PMIC 1.8V to become available, about 180ms after power up.
    MXC_Delay(200000);
#endif
    //启动缓存
    /* Enable cache */
    MXC_ICC_Enable(MXC_ICC0);

    switch (CLOCK_SOURCE) {
    case 0:
        printf("CLOCK_SOURCE----->0\n");
        MXC_SYS_ClockSourceEnable(MXC_SYS_CLOCK_IPO);
        MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IPO);
        MXC_GCR->pm &= ~MXC_F_GCR_PM_IPO_PD; // enable IPO during sleep
        break;

    case 1:
        MXC_SYS_ClockSourceEnable(MXC_SYS_CLOCK_ISO);
        MXC_SYS_Clock_Select(MXC_SYS_CLOCK_ISO);
        MXC_GCR->pm &= ~MXC_F_GCR_PM_ISO_PD; // enable ISO during sleep
        break;

    case 2:
        MXC_SYS_ClockSourceEnable(MXC_SYS_CLOCK_IBRO);
        MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IBRO);
        MXC_GCR->pm &= ~MXC_F_GCR_PM_IBRO_PD; // enable IBRO during sleep
        break;

    default:
        printf("UNKNOWN CLOCK SOURCE \n");

        while (1) {}
    }


    SystemCoreClockUpdate();

#ifdef ENABLE_MIC_PROCESSING
Microphone_Power(POWER_ON);
PR_INFO("ENABLE_MIC_PROCESSING----->0\n");

#endif

    // Initialize UART
    console_UART_init(CON_BAUD);

    // /* Enable peripheral, enable CNN interrupt, turn on CNN clock */
    // /* CNN clock: 50 MHz div 1 */
    // cnn_enable(MXC_S_GCR_PCLKDIV_CNNCLKSEL_PCLK, MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV1);

    // /* Configure P2.5, turn on the CNN Boost */
    // cnn_boost_enable(MXC_GPIO2, MXC_GPIO_PIN_5);

    // PR_INFO("\n\nANALOG DEVICES \nKeyword Spotting Demo\nVer. %s \n", VERSION);
    // PR_INFO("\n***** Init *****\n");
    // memset(pAI85Buffer, 0x0, sizeof(pAI85Buffer));

    // PR_DEBUG("pChunkBuff: %d\n", sizeof(pChunkBuff));
    // PR_DEBUG("pAI85Buffer: %d\n", sizeof(pAI85Buffer));


    // /* Bring state machine into consistent state */
    // cnn_init();
    // /* Load kernels */
    // cnn_load_weights();
    // /* Configure state machine */
    // cnn_configure();

    testtttcnnINIT();   

#ifdef WUT_ENABLE
PR_INFO("WUT_ENABLE----->0\n");
    // Get ticks based off of microseconds
    mxc_wut_cfg_t cfg;
    uint32_t ticks;

    MXC_WUT_GetTicks(WUT_USEC, MXC_WUT_UNIT_MICROSEC, &ticks);
    // config structure for one shot timer to trigger in a number of ticks
    cfg.mode = MXC_WUT_MODE_CONTINUOUS;
    cfg.cmp_cnt = ticks;
    // Init WUT
    MXC_WUT_Init(MXC_WUT_PRES_1);
    //Config WUT
    MXC_WUT_Config(&cfg);

    MXC_LP_EnableWUTAlarmWakeup();
    NVIC_EnableIRQ(WUT_IRQn);
#endif

    /* Disable CNN clock  */
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_CNN);

    /* switch to silence state*/
    procState = SILENCE;

#ifdef ENABLE_MIC_PROCESSING
    /* initialize I2S interface to Mic */
    I2SInit();
#endif


MXC_Delay(SEC(2)); // wait for debugger to connect

    PR_INFO("\n*** READY ***\n");
#ifdef WUT_ENABLE
    MXC_WUT_Enable(); // Start WUT
#endif

    /* Read samples */
    while (1) {


        /* Read from Mic driver to get CHUNK worth of samples, otherwise next sample*/
        if (MicReadChunk(&avg) == 0) {
            //直接执行下一个循环 不执行后面的代码了
            continue;
        }

        sampleCounter += CHUNK;

        /* wait for at least PREAMBLE_SIZE samples before detecting the utterance */
        if (sampleCounter < PREAMBLE_SIZE)
            continue;

//防止立即采集
#ifdef ENABLE_SILENCE_DETECTION // disable to start collecting data immediately.

        /* if we have not detected voice, check the average*/
        if (procState == SILENCE) {
            /* compute average, proceed if greater than threshold */
            if (avg >= thresholdHigh) {
                /* switch to keyword data collection*/
                procState = KEYWORD;

                /* record the average and index of the begining of the word */
                utteranceAvg = avg;
                utteranceIndex = micBufIndex;

                ai85Counter += PREAMBLE_SIZE;
                continue;
            }
        }
        /* if it is in data collection, add samples to buffer*/
        else if (procState == KEYWORD)
#endif //#ifdef ENABLE_SILENCE_DETECTION
        {
            uint8_t ret = 0;

            /* increment number of stored samples */
            ai85Counter += CHUNK;

            /* if there is silence after at least 1/3 of samples passed, increment number of times back to back silence to find end of keyword */
            if ((avg < thresholdLow) && (ai85Counter >= SAMPLE_SIZE / 3)) {
                avgSilenceCounter++;
            } else {
                avgSilenceCounter = 0;
            }

            /* if this is the last sample and there are not enough samples to
             * feed to CNN, or if it is long silence after keyword,  append with zero (for reading file)
             */

            if (avgSilenceCounter > SILENCE_COUNTER_THRESHOLD)
            {
                memset(pChunkBuff, 0, CHUNK);

                zeroPad = SAMPLE_SIZE - ai85Counter;
                ai85Counter = SAMPLE_SIZE;
            }

            /* if enough samples are collected, start CNN */
            if (ai85Counter >= SAMPLE_SIZE) {
                int16_t out_class = -1;
                double probability = 0;

                /* end of the utterance */
                int endIndex =
                    (utteranceIndex + SAMPLE_SIZE - PREAMBLE_SIZE - zeroPad) % SAMPLE_SIZE;

                PR_DEBUG("Word starts from index %d to %d, padded with %d zeros, avg:%d > %d \n",
                         utteranceIndex, endIndex, zeroPad, utteranceAvg, thresholdHigh);

                // zero padding
                memset(pChunkBuff, 0, CHUNK);


                /***
                 * 这段代码中包含了对函数 `AddTranspose` 的多次调用，用于处理音频数据的转置，并将转置后的数据存储到目标缓冲区 `pAI85Buffer` 中。
                 * 整体来看，这段代码的主要目的是对音频数据进行处理，以适应某种特定的格式或要求。
                 * 具体来说，代码的逻辑分为以下部分：
                 * 1. 首先进行了对音频数据的预处理，以确保输入给 `AddTranspose` 函数的数据满足特定条件，并调用 `AddTranspose` 来实际进行数据转置操作。
                 * 2. 根据数据的相对位置和长度，分别处理了首部和尾部的情况，确保音频数据的完整性。
                 * 3. 针对剩余部分的音频数据进行了转置和存储。
                 * 4. 最后，根据特定条件，通过循环调用 `AddTranspose` 函数进行零填充的处理。
                 * 根据代码逻辑，整个过程涉及了对输入音频数据的分段处理和转置，以确保转置后的数据按照特定要求存储到目标缓冲区中。
                 ***/
                /* PREAMBLE copy  */
                if (utteranceIndex - PREAMBLE_SIZE >= 0) {
                    if (AddTranspose((uint8_t *)&micBuff[utteranceIndex - PREAMBLE_SIZE],
                                     pAI85Buffer, PREAMBLE_SIZE, SAMPLE_SIZE, TRANSPOSE_WIDTH)) {
                        PR_DEBUG("ERROR: Transpose ended early \n");
                    }
                } else {
                    /* copy oldest samples to the beginning*/
                    if (AddTranspose(
                            (uint8_t *)&micBuff[SAMPLE_SIZE - PREAMBLE_SIZE + utteranceIndex],
                            pAI85Buffer, PREAMBLE_SIZE - utteranceIndex, SAMPLE_SIZE,
                            TRANSPOSE_WIDTH)) {
                        PR_DEBUG("ERROR: Transpose ended early \n");
                    }

                    /* copy latest samples afterwards */
                    if (AddTranspose((uint8_t *)&micBuff[0], pAI85Buffer, utteranceIndex,
                                     SAMPLE_SIZE, TRANSPOSE_WIDTH)) {
                        PR_DEBUG("ERROR: Transpose ended early \n");
                    }
                }

                /* Utterance copy */
                if (utteranceIndex < endIndex) {
                    /* copy from utternace to the end */
                    if (AddTranspose((uint8_t *)&micBuff[utteranceIndex], pAI85Buffer,
                                     endIndex - utteranceIndex, SAMPLE_SIZE, TRANSPOSE_WIDTH)) {
                        PR_DEBUG("ERROR: Transpose ended early \n");
                    }
                    // copy zero padding
                    while (!ret) {
                        ret = AddTranspose(pChunkBuff, pAI85Buffer, CHUNK, SAMPLE_SIZE,
                                           TRANSPOSE_WIDTH);
                    }
                } else {
                    /* copy from utternace to the end*/
                    if (AddTranspose((uint8_t *)&micBuff[utteranceIndex], pAI85Buffer,
                                     SAMPLE_SIZE - utteranceIndex, SAMPLE_SIZE, TRANSPOSE_WIDTH)) {
                        PR_DEBUG("ERROR: Transpose ended early \n");
                    }

                    /* copy from begining*/
                    if (AddTranspose((uint8_t *)&micBuff[0], pAI85Buffer, endIndex, SAMPLE_SIZE,
                                     TRANSPOSE_WIDTH)) {
                        PR_DEBUG("ERROR: Transpose ended early \n");
                    }
                    // copy zero padding
                    while (!ret) {
                        ret = AddTranspose(pChunkBuff, pAI85Buffer, CHUNK, SAMPLE_SIZE,
                                           TRANSPOSE_WIDTH);
                    }
                }

                /* reset counters */
                ai85Counter = 0;
                avgSilenceCounter = 0;

                /* new word */
                wordCounter++;

                /* change state to silence */
                procState = SILENCE;

                /* sanity check, last transpose should have returned 1, as enough samples should have already been added */
                if (ret != 1) {
                    PR_DEBUG("ERROR: Transpose incomplete!\n");
                    fail();
                }

                //----------------------------------  : invoke AI85 CNN
                PR_DEBUG("%.6d: Starts CNN: %d\n", sampleCounter, wordCounter);
                /* enable CNN clock */
                MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_CNN);

                /* load to CNN */
                if (!cnn_load_data(pAI85Buffer)) {
                    PR_DEBUG("ERROR: Loading data to CNN! \n");
                    fail();
                }

                /* Start CNN */
                if (!cnn_start()) {
                    PR_DEBUG("ERROR: Starting CNN! \n");
                    fail();
                }

#if SLEEP_MODE == 0

                /* Wait for CNN  to complete */
                while (cnn_time == 0) {
                    __WFI();
                }


#endif // #if SLEEP_MODE==0

                /* Read CNN result */
                cnn_unload((uint32_t *)ml_data);
                /* Stop CNN */
                cnn_stop();

                // testtttcnnINIT();

                /* Disable CNN clock to save power */
                MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_CNN);
                /* Get time */
                MXC_TMR_GetTime(MXC_TMR0, cnn_time, (void *)&cnn_time, &units);
                PR_DEBUG("%.6d: Completes CNN: %d\n", sampleCounter, wordCounter);

                switch (units) {
                case TMR_UNIT_NANOSEC:
                    cnn_time /= 1000;
                    break;

                case TMR_UNIT_MILLISEC:
                    cnn_time *= 1000;
                    break;

                case TMR_UNIT_SEC:
                    cnn_time *= 1000000;
                    break;

                default:
                    break;
                }

                PR_DEBUG("CNN Time: %d us\n", cnn_time);

                /* run softmax */
                softmax_q17p14_q15((const q31_t *)ml_data, NUM_OUTPUTS, ml_softmax);

                /* find detected class with max probability */
                ret = check_inference(ml_softmax, ml_data, &out_class, &probability);

                PR_DEBUG("----------------------------------------- \n");
                /* Treat low confidence detections as unknown*/
                if (!ret || out_class == NUM_OUTPUTS - 1) {
                    PR_DEBUG("Detected word: %s", "Unknown");
                    LED_Off(LED_BLUE);
                } else {
                    if(!strcmp(keywords[out_class],"kaca"))  
                    {
                        LED_Toggle(LED_BLUE);
                        
                            my_sd_cheak = sd_cheak(myDir);
                            if(my_sd_cheak == 3) sd_mkdir(myDir);
                            

                            myFlag = sd_lsLast(myDir,prefix);
                            // printf("ssssssssss---------->%d",myFlag);

                            myFlag++;

                            //拼接文件名
                            sprintf(filename, "%s/%s%d", myDir,prefix,myFlag);

                            // Stream and save an image to the SD card

                            cnn_img_data_t img_data = stream_img(g_app_settings.imgres_w, g_app_settings.imgres_h,
                                                                g_app_settings.pixel_format,
                                                                g_app_settings.dma_channel);

                            // cnn_img_data_t img_data = stream_img(320, 240,
                            //                                     PIXFORMAT_RGB888,
                            //                                     MXC_DMA_AcquireChannel());


                            save_stream_sd(img_data, filename);

                            cnn_disable();

                            testtttcnnINIT();
                    }
                    PR_DEBUG("Detected word: %s (%0.1f%%)", keywords[out_class], probability);
                }
                PR_DEBUG("\n----------------------------------------- \n");

                Max = 0;
                Min = 0;
                //------------------------------------------------------------


                /* clear the buffer */
                memset(micBuff, 0, SAMPLE_SIZE);
                micBufIndex = 0;
                //sampleCounter = 0;  // comment to start immediately after the last utterance
            }
        }

 /****************zzzzzzzzzzzzz**************************ImgCapture相关**************************************************/

        // #ifdef SD
        // // SD card is enabled.
        // if (PB_Get(0)) { // Enable image capture by pushing pushbutton 0
        //     cnn_img_data_t img_data = stream_img(g_app_settings.imgres_w, g_app_settings.imgres_h,
        //                                          g_app_settings.pixel_format,
        //                                          g_app_settings.dma_channel);
        //     save_stream_sd(img_data, NULL);
        // }
        // #endif

 /***************zzzzzzzzzzzzzzzzzz*************************************************************************************/

    }

    /* Turn off LED2 (Red) */
    LED_Off(LED2);
    PR_DEBUG("Total Samples:%d, Total Words: %d \n", sampleCounter, wordCounter);



    while (1) {}
}

/* **************************************************************************** */

/***
 * 这段代码是一个 `I2SInit` 函数，看起来是用于初始化 I2S（Inter-IC Sound）接口，并开始对来自麦克风的音频数据进行处理。
 * 让我来解释一下这段代码的主要功能：
 * - 该函数首先初始化了一个 `mxc_i2s_req_t` 类型的变量 `req` 和一个 `int32_t` 类型的变量 `err`。
 * - 通过调用 `HPF_init` 函数，来初始化高通滤波器。
 * - 使用 `memset` 函数将 `i2s_rx_buffer` 初始化为全零，`i2s_rx_buffer` 似乎是用来存放接收到的音频数据。
 * - 配置了 I2S 接口的参数，包括数据位大小、采样大小、数据对齐方式、时钟极性、通道模式等。
 * - 设置了 I2S 接口的时钟分频、接收数据的缓冲区等参数。
 * 随后，代码进行了以下操作：
 * - 调用 `MXC_I2S_Init` 函数来初始化 I2S 接口，并将结果保存在 `err` 变量中。
 * - 如果初始化出现错误，通过调用 `PR_DEBUG` 打印错误信息，并进入一个无限循环。
 * - 如果没有出现错误，设置了 I2S 接收方的 FIFO 阈值，并根据条件设置了中断服务函数，并使能了相关的中断。
 * - 最后，开启了 I2S 接收并使能全局中断。
 * 整体来看，这段代码是在配置并初始化 I2S 接口以用于从麦克风接收音频数据，并设置相应的中断处理函数和中断使能。这样一来，系统就可以开始接收来自麦克风的音频数据，并进行进一步的处理。
***/
#ifdef ENABLE_MIC_PROCESSING
void I2SInit()
{
    mxc_i2s_req_t req;
    int32_t err;

    PR_INFO("\n*** I2S & Mic Init ***\n");
    /* Initialize High Pass Filter */
    HPF_init();
    /* Initialize I2S RX buffer */
    memset(i2s_rx_buffer, 0, sizeof(i2s_rx_buffer));
    /* Configure I2S interface parameters */
    req.wordSize = MXC_I2S_DATASIZE_WORD;
    req.sampleSize = MXC_I2S_SAMPLESIZE_THIRTYTWO;
    req.justify = MXC_I2S_MSB_JUSTIFY;
    req.wsPolarity = MXC_I2S_POL_NORMAL;
    req.channelMode = MXC_I2S_INTERNAL_SCK_WS_0;
    /* Get only left channel data from on-board microphone. Right channel samples are zeros */
    req.stereoMode = MXC_I2S_MONO_LEFT_CH;
    req.bitOrder = MXC_I2S_MSB_FIRST;
    /* I2S clock = PT freq / (2*(req.clkdiv + 1)) */
    /* I2S sample rate = I2S clock/64 = 16kHz */
    req.clkdiv = 5;
    req.rawData = NULL;
    req.txData = NULL;
    req.rxData = i2s_rx_buffer;
    req.length = I2S_RX_BUFFER_SIZE;

    if ((err = MXC_I2S_Init(&req)) != E_NO_ERROR) {
        PR_DEBUG("\nError in I2S_Init: %d\n", err);

        while (1) {}
    }

    /* Set I2S RX FIFO threshold to generate interrupt */
    MXC_I2S_SetRXThreshold(4);

#ifndef WUT_ENABLE
    MXC_NVIC_SetVector(I2S_IRQn, i2s_isr);
    NVIC_EnableIRQ(I2S_IRQn);
    /* Enable RX FIFO Threshold Interrupt */
    MXC_I2S_EnableInt(MXC_F_I2S_INTEN_RX_THD_CH0);
#endif

    MXC_I2S_RXEnable();
    __enable_irq();
}
#endif

/* **************************************************************************** */
/***
 * 这段代码展示了一个名为 `check_inference` 的函数，它用于检查机器学习推理结果并确定推理结果的置信度。
 * 具体来说，这个函数接受了三个输入参数：`ml_soft`（类型为 `q15_t*`，表示经过软最大化处理的机器学习输出）、
 *                                    `ml_data`（类型为 `int32_t*`，表示未经处理的机器学习输出）、
 *          以及 `out_class` 和 `out_prob`（分别为 `int16_t*` 和 `double*` 类型的指针，用于存储最终的分类和置信度结果）。
 * 下面是这个函数的主要逻辑：
 * 1、首先，函数初始化了一些变量，包括 `temp` 数组用于存储未处理的机器学习输出、`max` 用于存储软最大化后的最大值、
 *                                  `max_ml` 用于存储未处理的最大值、`max_index` 用于存储最大值的索引。
 * 2、接着通过循环遍历机器学习输出数据，找到前 5 个最大的值，并计算这些最大值对应的类别和置信度。
 * 3、将找到的最大值和对应的类别和置信度保存到 `out_class` 和 `out_prob` 中，并在找到最大值后立即结束循环。
 * 4、最后，函数根据置信度与预定义阈值 `INFERENCE_THRESHOLD` 进行比较，并返回相应的结果。
 * 总的来说，这个函数的作用是根据机器学习输出，找到最大的几个值并计算对应的类别和置信度，并判断置信度是否高于预定义的阈值。函数中使用了循环和数学计算来实现这一目的。
**/
uint8_t check_inference(q15_t *ml_soft, int32_t *ml_data, int16_t *out_class, double *out_prob)
{

    int32_t temp[NUM_OUTPUTS];
    q15_t max = 0; // soft_max output is 0->32767
    int32_t max_ml = 1 << 31; // ml before going to soft_max
    int16_t max_index = -1;

    memcpy(temp, ml_data, sizeof(int32_t) * NUM_OUTPUTS);

    /* find the top 5 classes */
    for (int top = 0; top < 5; top++) {
        /* find the class with highest */
        for (int i = 0; i < NUM_OUTPUTS; i++) {
            if ((int32_t)temp[i] > max_ml) {
                max_ml = (int32_t)temp[i];
                max = ml_soft[i];
                max_index = i;
            }
        }

        /* print top 1 separately */
        if (top == 0) {
            *out_class = max_index;
            *out_prob = 100.0 * max / 32768.0;
            break;
        }

    }

    PR_DEBUG("Min: %d,   Max: %d \n", Min, Max);

    /* check if probability is low */
    if (*out_prob > INFERENCE_THRESHOLD) {
        return 1;
    } else {
        return 0;
    }
}

/* **************************************************************************** */
void fail(void)
{
    PR_DEBUG("\n*** FAIL ***\n\n");

    while (1) {}
}
/* **************************************************************************** */
/***
 * 这段代码展示了一个名为 `cnn_load_data` 的函数，它的作用是将输入的数据 `pIn` 按照特定的规则拷贝到内存中的不同位置。
 * 具体来说，这个函数将 `pIn` 中的数据按照一定的顺序分别拷贝到不同的内存地址中，每次拷贝 1024 字节（1KB）的数据。
 * 拷贝的目标内存地址是预定义的一组连续内存空间。
 * 下面是这个函数的主要逻辑：
 * 1、函数首先初始化了一个用于索引 `pIn` 的指针 `index`，并设置一个初始值为 0。
 * 2、接着通过多个循环，将 `pIn` 中的数据拷贝到预定义的一组内存地址中，每次拷贝 1024 字节的数据。
 * 3、每次循环中都会更新 `index` 指针，以便指向 `pIn` 中的下一个数据块。
 * 4、最后函数返回了一个值 `CNN_OK`，用于表示函数执行成功。
 * 总的来说，这个函数的作用是将输入的数据按照一定规则拷贝到预定义的一组连续内存空间中，以便后续对这些数据的使用。函数中使用了循环和内存拷贝操作来实现这一目的。
**/
uint8_t cnn_load_data(uint8_t *pIn)
{
    uint32_t mem;
    uint16_t index = 0;

    /* data should already be formatted correctly */
    /* pIn is 16KB, each 1KB belongs to a memory group */
    for (mem = 0x50400000; mem <= 0x50418000; mem += 0x8000) {
        memcpy((uint8_t *)mem, &pIn[index], 1024);
        //PR_DEBUG("%.10X \n",(uint8_t *)mem);
        index += 1024;
    }

    for (mem = 0x50800000; mem <= 0x50818000; mem += 0x8000) {
        memcpy((uint8_t *)mem, &pIn[index], 1024);
        index += 1024;
    }

    for (mem = 0x50C00000; mem <= 0x50C18000; mem += 0x8000) {
        memcpy((uint8_t *)mem, &pIn[index], 1024);
        index += 1024;
    }

    for (mem = 0x51000000; mem <= 0x51018000; mem += 0x8000) {
        memcpy((uint8_t *)mem, &pIn[index], 1024);
        index += 1024;
    }

    return CNN_OK;
}
/* **************************************************************************** */

/***
 * 这段代码实现了一个将输入数据进行转置并重新排列，以适应特定输出数据格式的函数 `AddTranspose`。
 * 函数的主要作用是根据特定的规则将输入数据按照一定的顺序存储到输出数组中。以下是代码的主要流程：
 * 1. 使用 `row`、`col` 和 `total` 这几个静态变量来跟踪转置的过程状态。
 * 2. 通过循环对输入数据进行遍历处理，根据特定规则计算出输出数组中的索引位置。
 * 3. 将输入数据按照计算得到的索引位置存储到输出数组中。
 * 4. 根据条件判断，当处理的数据量达到指定的输出数据尺寸时，进行结尾的检查并返回1；否则返回0。
 * 这个函数的主要目的是将输入数据重新排列并存储到输出数组中，以适应特定的数据格式要求。
 * 在代码中，也包含了一些对于 `row`、`col` 和 `total` 变量的状态管理，以及一些用于计算输出数组索引位置的逻辑。
。**/
uint8_t AddTranspose(uint8_t *pIn, uint8_t *pOut, uint16_t inSize, uint16_t outSize, uint16_t width)
{
    /* Data order in Ai85 memory (transpose is included):
    input(series of 8 bit samples): (0,0) ...  (0,127)  (1,0) ... (1,127) ...... (127,0)...(127,127)    16384 samples
    output (32bit word): 16K samples in a buffer. Later, each 1K goes to a separate CNN memory group
    0x0000:
        (0,3)(0,2)(0,1)(0,0)
        (0,67)(0,66)(0,65)(0,64)
        (1,3)(1,2)(1,1)(1,0)
        (1,67)(1,66)(1,65)(1,64)
        ....
        (127,67)(127,66)(127,65)(127,64)
    0x0400:
        (0,7)(0,6)(0,5)(0,4)
        (0,71)(0,70)(0,69)(0,68)
        ....
        (127,71)(127,70)(127,69)(127,68)
    ...
    0x3C00:
        (0,63)(0,62)(0,61)(0,60)
        (0,127)(0,126)(0,125)(0,124)
        ....
        (127,127)(127,126)(127,125)(127,124)
    */

    static uint16_t row = 0, col = 0, total = 0;
    uint16_t secondHalf = 0, wordRow = 0, byteInWord = 0, group = 0, index = 0;

    for (int i = 0; i < inSize; i++) {
        /* is it above 63? */
        if (col >= (width >> 1)) {
            secondHalf = 1; // odd word rows
        } else {
            secondHalf = 0; // even word rows
        }

        /* which group (0-15) it should be */
        group = (col % (width >> 1)) / 4;

        /* which word row (0-255) within the group */
        wordRow = secondHalf + (row << 1);

        /* which byte (0-3) in the word */
        byteInWord = col % 4;

        /* find output index */
        index = 1024 * group + 4 * wordRow + byteInWord;

        /* place sample in correct output location */
        pOut[index] = pIn[i];

        total++;

        /* increment row and col index */
        col++;

        if (col >= width) {
            col = 0;
            row++;
        }
    }

    if (total >= outSize) {
        /* sanity check */
        if (row != width) {
            PR_DEBUG("ERROR: Rearranging!\n");
        }

        total = 0;
        row = 0;
        col = 0;
        return 1;
    } else {
        return 0;
    }
}
/* **************************************************************************** */

/***
 * 这段代码实现了一个从测试向量文件中读取音频样本并返回一个样本的函数 `MicReader`。以下是函数的主要步骤：
 * 1. `micSampleCount` 用于跟踪已经读取的样本数量，它是一个静态变量，保证了在不同函数调用之间保持状态。
 * 2. 从测试向量文件中读取一个样本，并存储到 `temp` 变量中。
 * 3. 将读取到的样本值赋给 `sample`，并将其转换为 `int32_t` 类型。
 * 4. 返回值为 1，表示成功读取了一个样本。
 * 根据代码来看，函数并没有通过文件操作等方式来读取测试向量，很可能是测试向量已经预先存储在 `voiceVector` 数组中。
 * 因此，这段代码看起来更像是在模拟读取真实的音频数据，以便进行算法的测试和调试。
***/
// #ifndef ENABLE_MIC_PROCESSING
// int8_t MicReader(int32_t *sample)
// {
//     static uint32_t micSampleCount = 0;
//     int16_t temp;

//     /* reads from Test Vector file and return one sample */
//     temp = voiceVector[(micSampleCount++) % KWS20_TEST_VECTOR_SIZE];
//     *sample = temp;
//     return (1);
// }
// #endif
/* **************************************************************************** */

/***
 * 这段代码中的 `MicReadChunk` 函数是用于从麦克风或 I2S 接口中读取音频样本并进行处理的。以下是函数的详细流程：
 * 1. `i2s_flag` 用于标识音频样本是否准备就绪，如果未准备好，则将 `*avg` 设置为0并返回0；
 * 2. 重置 `i2s_flag` 标志位，并读取 I2S 接收 FIFO 中的样本数量；
 * 3. 当 FIFO 非空且采集到的样本数量未达到阈值 `CHUNK` 时，进入循环；
 * 4. 从 I2S FIFO 中读取音频样本，并将其向右移动14位，得到实际的音频值；
 * 5. 对音频信号进行高通滤波处理，并丢弃因麦克风充电电容效应而导致的前10000个样本；
 * 6. 对采集到的音频样本进行绝对值处理，并累加到 `sum` 中；
 * 7. 将处理后的音频样本转换成8位无符号数，并记录最大值和最小值；
 * 8. 如果采集到的样本数量未达到阈值 `CHUNK`，则将 `*avg` 设置为0并返回0；
 * 9. 若采集到了足够的样本，则计算平均值，将其存储到 `*avg` 中，并将 `chunkCount` 和 `sum` 复位后返回1。
 * 这个函数主要用于在每次调用时从音频输入中读取一定数量的样本，并计算其平均值。
 * 根据代码实现来看，函数中的一些变量和宏可能是在其他文件中定义的，比如 `i2s_flag` 和 `SAMPLE_SCALE_FACTOR`。函数中的一些具体处理步骤可能需要根据实际情况来理解。
***/
uint8_t MicReadChunk(uint16_t *avg)
{
    static uint16_t chunkCount = 0;
    static uint16_t sum = 0;
    int32_t sample = 0;
    int16_t temp = 0;
    uint32_t rx_size = 0;

    static uint32_t index = 0;
    /* sample not ready */
    if (!i2s_flag) {
        *avg = 0;
        return 0;
    }

    /* Clear flag */
    i2s_flag = 0;
    /* Read number of samples in I2S RX FIFO */
    rx_size = MXC_I2S->dmach0 >> MXC_F_I2S_DMACH0_RX_LVL_POS;
    //  PR_DEBUG("%d ", rx_size);


    /* read until fifo is empty or enough samples are collected */
    while ((rx_size--) && (chunkCount < CHUNK)) {


        /* Read microphone sample from I2S FIFO */
        sample = (int32_t)MXC_I2S->fifoch0;

        /* The actual value is 18 MSB of 32-bit word */

        temp = sample >> 14;


        /* Remove DC from microphone signal */
        sample = HPF((int16_t)temp); // filter needs about 1K sample to converge

        /* Discard first 10k samples due to microphone charging cap effect */
        if (index++ < 10000) {
            continue;
        }



        /* absolute for averaging */
        if (sample >= 0) {
            sum += sample;
        } else {
            sum -= sample;
        }

        /* Convert to 8 bit unsigned */
        micBuff[micBufIndex] = (sample)*SAMPLE_SCALE_FACTOR / 256;
        chunkCount++;

        temp = (int8_t)micBuff[micBufIndex];
        /* record max and min */
        if (temp > Max) {
            Max = temp;
        }

        if (temp < Min) {
            Min = temp;
        }

        micBufIndex = (micBufIndex + 1) % SAMPLE_SIZE;
    }

    /* if not enough samples, return 0 */
    if (chunkCount < CHUNK) {
        *avg = 0;
        return 0;
    }

    /* enough samples are collected, calculate average and return 1 */
    *avg = ((uint16_t)(sum / CHUNK));

    chunkCount = 0;
    sum = 0;
    return 1;
}

static int16_t x_0, x_1, Coeff;
static int32_t y_0, y_1;

/***************
 * 这段代码实现了一个一阶 IIR 高通滤波器，用于实现高通滤波功能。
 * 在HPF_init函数中，初始化了滤波器的一些参数，包括系数(Coeff)和初始状态变量(x_0, y_0, y_1, x_1)。
 * 在HPF函数中实现了对输入信号的高通滤波操作，并返回滤波后的输出。
 * 具体来说，这个滤波器的差分方程为：y(n) = x(n) - x(n-1) + A * y(n-1)，其中 A = 0.995*2^15。
 * 在每次输入一个新的样本时，会根据该方程进行滤波计算，最后的结果被限制在-32768到32767之间（即进行了饱和处理）。
 * 另外，需要注意的是，这里的Coeff的值为32604，这对应于系数0.995乘以2^15。这个值会影响滤波器的特性，也就是滤波器的截止频率和响应特性。
 * 如果需要不同的截止频率，可以调整Coeff的值。
 * *********/

/************************************************************************************/
void HPF_init(void)
{
    Coeff = 32604; //0.995
    x_0 = 0;
    y_0 = 0;
    y_1 = y_0;
    x_1 = x_0;
}

/************************************************************************************/
int16_t HPF(int16_t input)
{
    int16_t Acc, output;
    int32_t tmp;

    /* a 1st order IIR high pass filter (100 Hz cutoff frequency)  */
    /* y(n)=x(n)-x(n-1)+A*y(n-1) and A =.995*2^15 */

    x_0 = input;

    tmp = (Coeff * y_1);
    Acc = (int16_t)((tmp + (1 << 14)) >> 15);
    y_0 = x_0 - x_1 + Acc;

    /* Clipping */
    if (y_0 > 32767) {
        y_0 = 32767;
    }

    if (y_0 < -32768) {
        y_0 = -32768;
    }

    /* Update filter state */
    y_1 = y_0;
    x_1 = x_0;

    output = (int16_t)y_0;

    return (output);
}

