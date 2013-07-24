//MPU9150 I2C library test code for ARM STM32F103xx Microcontrollers 
// 25/06/2013 by Harinadha Reddy Chintalapalli <harinath.ec@gmail.com>
// Changelog:
//     2013-07-06 - Configuration of MPU9150 to populate Compass data into EXT_SENS_DATA_xx 
//                  registers is working. Function "setup_compass(void) in inv_mpu.c updated.
//                  Added Compass data packets streaming code.
//     2013-06-25 - Initial release. Thanks to Invensense for releasing MPU 
//                  driver test code for msp430.
/* ============================================================================================
MPU9150 device I2C library code for ARM STM32F103xx is placed under the MIT license
Copyright (c) 2013 Harinadha Reddy Chintalapalli

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
================================================================================================
*/
//--------Includes ----------------------
#include "hw_config.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"
#include "stm32_CPAL_mpu9150.h"
#include "mpu9150_interrupts.h"
#include "utils.h"

#include <string.h>
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
//---------------------------------------
extern u32 count_out;
extern uint8_t USB_Rx_Buffer;

/* Data requested by client. */
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
#define PRINT_COMPASS   (0x08)

#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)

#define MOTION          (0)
#define NO_MOTION       (1)

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ  (100)

struct rx_s {
    unsigned char header[3];
    unsigned char cmd;
};
struct hal_s {
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned short report;
    unsigned short dmp_features;
    unsigned char motion_int_mode;
    struct rx_s rx;
};
static struct hal_s hal = {0};

char rx_new = 0;
/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from their
 * driver(s).
 * TODO: The following matrices refer to the configuration on an internal test
 * board at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};

enum packet_type_e {
    PACKET_TYPE_ACCEL,
    PACKET_TYPE_GYRO,
    PACKET_TYPE_QUAT,
    PACKET_TYPE_TAP,
    PACKET_TYPE_ANDROID_ORIENT,
    PACKET_TYPE_PEDO,
    PACKET_TYPE_MISC,
    PACKET_TYPE_COMPASS,
};

/* Send data to the Python client application.
 * Data is formatted as follows:
 * packet[0]    = $
 * packet[1]    = packet type (see packet_type_e)
 * packet[2+]   = data
 */
void send_packet(char packet_type, void *data)
{
#define MAX_BUF_LENGTH  (18)
    char buf[MAX_BUF_LENGTH], length;

    memset(buf, 0, MAX_BUF_LENGTH);
    buf[0] = '$';
    buf[1] = packet_type;

    if (packet_type == PACKET_TYPE_ACCEL || packet_type == PACKET_TYPE_GYRO || packet_type == PACKET_TYPE_COMPASS) {
        short *sdata = (short*)data;
        buf[2] = (char)(sdata[0] >> 8);
        buf[3] = (char)sdata[0];
        buf[4] = (char)(sdata[1] >> 8);
        buf[5] = (char)sdata[1];
        buf[6] = (char)(sdata[2] >> 8);
        buf[7] = (char)sdata[2];
        length = 8;
    } else if (packet_type == PACKET_TYPE_QUAT) {
        long *ldata = (long*)data;
        buf[2] = (char)(ldata[0] >> 24);
        buf[3] = (char)(ldata[0] >> 16);
        buf[4] = (char)(ldata[0] >> 8);
        buf[5] = (char)ldata[0];
        buf[6] = (char)(ldata[1] >> 24);
        buf[7] = (char)(ldata[1] >> 16);
        buf[8] = (char)(ldata[1] >> 8);
        buf[9] = (char)ldata[1];
        buf[10] = (char)(ldata[2] >> 24);
        buf[11] = (char)(ldata[2] >> 16);
        buf[12] = (char)(ldata[2] >> 8);
        buf[13] = (char)ldata[2];
        buf[14] = (char)(ldata[3] >> 24);
        buf[15] = (char)(ldata[3] >> 16);
        buf[16] = (char)(ldata[3] >> 8);
        buf[17] = (char)ldata[3];
        length = 18;
    } else if (packet_type == PACKET_TYPE_TAP) {
        buf[2] = ((char*)data)[0];
        buf[3] = ((char*)data)[1];
        length = 4;
    } else if (packet_type == PACKET_TYPE_ANDROID_ORIENT) {
        buf[2] = ((char*)data)[0];
        length = 3;
    } else if (packet_type == PACKET_TYPE_PEDO) {
        long *ldata = (long*)data;
        buf[2] = (char)(ldata[0] >> 24);
        buf[3] = (char)(ldata[0] >> 16);
        buf[4] = (char)(ldata[0] >> 8);
        buf[5] = (char)ldata[0];
        buf[6] = (char)(ldata[1] >> 24);
        buf[7] = (char)(ldata[1] >> 16);
        buf[8] = (char)(ldata[1] >> 8);
        buf[9] = (char)ldata[1];
        length = 10;
    } else if (packet_type == PACKET_TYPE_MISC) {
        buf[2] = ((char*)data)[0];
        buf[3] = ((char*)data)[1];
        buf[4] = ((char*)data)[2];
        buf[5] = ((char*)data)[3];
        length = 6;
    }
    Virtual_Com_Write_Buffer(buf, length);  
}

/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
static inline unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static inline unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}
/* Handle sensor on/off combinations. */
static void setup_gyro(void)
{
    unsigned char mask = 0;
    if (hal.sensors & ACCEL_ON)
        mask |= INV_XYZ_ACCEL;
    if (hal.sensors & GYRO_ON)
        mask |= INV_XYZ_GYRO;
    /* If you need a power transition, this function should be called with a
     * mask of the sensors still enabled. The driver turns off any sensors
     * excluded from this mask.
     */
    mpu_set_sensors(mask);
    if (!hal.dmp_on)
        mpu_configure_fifo(mask);
}

static void tap_cb(unsigned char direction, unsigned char count)
{
    char data[2];
    data[0] = (char)direction;
    data[1] = (char)count;
    send_packet(PACKET_TYPE_TAP, data);
}

static void android_orient_cb(unsigned char orientation)
{
    send_packet(PACKET_TYPE_ANDROID_ORIENT, &orientation);
}
static inline void stm32_reset(void)
{
    //PMMCTL0 |= PMMSWPOR;
}
static inline void run_self_test(void)
{
    int result;
    char test_packet[4] = {0};
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
    if (result == 0x7) {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
    }

    /* Report results. */
    test_packet[0] = 't';
    test_packet[1] = result;
    send_packet(PACKET_TYPE_MISC, test_packet);
}
static void handle_input(void)
{
    char c;
    const unsigned char header[3] = "inv";
    unsigned long pedo_packet[2];

    /* Check the commands in the data received from USB*/
    rx_new = 0;
   /* Check USB_Rx_Buffer for extracting commands.
    * It processes one character at a time. After constructing 
    * a full command consisting of 4 char's, it proceeds to execute
    * that command.
    */
    c = USB_Rx_Buffer;			
    if(count_out !=0)
      rx_new = count_out;
		
    if (hal.rx.header[0] == header[0]) {
        if (hal.rx.header[1] == header[1]) {
            if (hal.rx.header[2] == header[2]) {
                memset(&hal.rx.header, 0, sizeof(hal.rx.header));
                hal.rx.cmd = c;
            } else if (c == header[2])
                hal.rx.header[2] = c;
            else
                memset(&hal.rx.header, 0, sizeof(hal.rx.header));
        } else if (c == header[1])
            hal.rx.header[1] = c;
        else
            memset(&hal.rx.header, 0, sizeof(hal.rx.header));
    } else if (c == header[0])
        hal.rx.header[0] = header[0];
    if (!hal.rx.cmd)
        return;
		/********************************************************************/
		
    switch (hal.rx.cmd) {
    /* These commands turn the hardware sensors on/off. */
    case '8':
        if (!hal.dmp_on) {
            /* Accel and gyro need to be on for the DMP features to work
             * properly.
             */
            hal.sensors ^= ACCEL_ON;
            setup_gyro();
        }
        break;
    case '9':
        if (!hal.dmp_on) {
            hal.sensors ^= GYRO_ON;
            setup_gyro();
        }
        break;
    /* The commands start/stop sending data to the client. */
    case 'a':
        hal.report ^= PRINT_ACCEL;
        break;
    case 'g':
        hal.report ^= PRINT_GYRO;
        break;
    case 'q':
        hal.report ^= PRINT_QUAT;
        break;
    case 'c':
        hal.report ^= PRINT_COMPASS;
        break;
    /* The hardware self test can be run without any interaction with the
     * MPL since it's completely localized in the gyro driver. Logging is
     * assumed to be enabled; otherwise, a couple LEDs could probably be used
     * here to display the test results.
     */
    case 't':
        run_self_test();
        break;
    /* Depending on your application, sensor data may be needed at a faster or
     * slower rate. These commands can speed up or slow down the rate at which
     * the sensor data is pushed to the MPL.
     *
     * In this example, the compass rate is never changed.
     */
    case '1':
        if (hal.dmp_on)
            dmp_set_fifo_rate(10);
        else
            mpu_set_sample_rate(10);
        break;
    case '2':
        if (hal.dmp_on)
            dmp_set_fifo_rate(20);
        else
            mpu_set_sample_rate(20);
        break;
    case '3':
        if (hal.dmp_on)
            dmp_set_fifo_rate(40);
        else
            mpu_set_sample_rate(40);
        break;
    case '4':
        if (hal.dmp_on)
            dmp_set_fifo_rate(50);
        else
            mpu_set_sample_rate(50);
        break;
    case '5':
        if (hal.dmp_on)
            dmp_set_fifo_rate(100);
        else
            mpu_set_sample_rate(100);
        break;
    case '6':
        if (hal.dmp_on)
            dmp_set_fifo_rate(200);
        else
            mpu_set_sample_rate(200);
        break;
	case ',':
        /* Set hardware to interrupt on gesture event only. This feature is
         * useful for keeping the MCU asleep until the DMP detects as a tap or
         * orientation event.
         */
        dmp_set_interrupt_mode(DMP_INT_GESTURE);
        break;
    case '.':
        /* Set hardware to interrupt periodically. */
        dmp_set_interrupt_mode(DMP_INT_CONTINUOUS);
        break;
    case '7':
        /* Reset pedometer. */
        dmp_set_pedometer_step_count(0);
        dmp_set_pedometer_walk_time(0);
        break;
    case 'f':
        /* Toggle DMP. */
        if (hal.dmp_on) {
            unsigned short dmp_rate;
            hal.dmp_on = 0;
            mpu_set_dmp_state(0);
            /* Restore FIFO settings. */
            mpu_configure_fifo(INV_XYZ_ACCEL | INV_XYZ_GYRO);
            /* When the DMP is used, the hardware sampling rate is fixed at
             * 200Hz, and the DMP is configured to downsample the FIFO output
             * using the function dmp_set_fifo_rate. However, when the DMP is
             * turned off, the sampling rate remains at 200Hz. This could be
             * handled in inv_mpu.c, but it would need to know that
             * inv_mpu_dmp_motion_driver.c exists. To avoid this, we'll just
             * put the extra logic in the application layer.
             */
            dmp_get_fifo_rate(&dmp_rate);
            mpu_set_sample_rate(dmp_rate);
        } else {
            unsigned short sample_rate;
            hal.dmp_on = 1;
            /* Both gyro and accel must be on. */
            hal.sensors |= ACCEL_ON | GYRO_ON;
            mpu_set_sensors(INV_XYZ_ACCEL | INV_XYZ_GYRO);
            mpu_configure_fifo(INV_XYZ_ACCEL | INV_XYZ_GYRO);
            /* Preserve current FIFO rate. */
            mpu_get_sample_rate(&sample_rate);
            dmp_set_fifo_rate(sample_rate);
            mpu_set_dmp_state(1);
        }
        break;
    case 'm':
        /* Test the motion interrupt hardware feature. */
        hal.motion_int_mode = 1;
        break;
    case 'p':
        /* Read current pedometer count. */
        dmp_get_pedometer_step_count(pedo_packet);
        dmp_get_pedometer_walk_time(pedo_packet + 1);
        send_packet(PACKET_TYPE_PEDO, pedo_packet);
        break;
    case 'x':
        stm32_reset();
        break;
    case 'v':
        /* Toggle LP quaternion.
         * The DMP features can be enabled/disabled at runtime. Use this same
         * approach for other features.
         */
        hal.dmp_features ^= DMP_FEATURE_6X_LP_QUAT;
        dmp_enable_feature(hal.dmp_features);
        break;
    default:
        break;
    }
    hal.rx.cmd = 0;
		
}
/* Every time new gyro data is available, this function is called in an
 * ISR context. In this example, it sets a flag protecting the FIFO read
 * function.
 */
void EXTI9_5_IRQHandler(void) 
{  
   if(EXTI_GetITStatus(EXTI_Line9))
   {             
     if(MPU9150_Get_INTpin_State())
     {
       hal.new_gyro = 1;
     }
     EXTI_ClearITPendingBit(EXTI_Line9);        
   }   
}
/* Sample application code for testing MPU9150 with STM32F103xx family microcontrollers.
 * It initializes Clock system, USB, I2C & MPU9150. Listens to the data received on USB,
 * constructs commands & executes them.
 * You can obtain Accelerometer, Gyro, 6-Axes DMP quaternion, Pedometer count, TAP, 
 * Android_Orientation & other data.
 */
void Stm32MPU9150noCompass_test(void)
{
    int result;
    unsigned char accel_fsr;
    unsigned short gyro_rate, gyro_fsr;
    //unsigned long timestamp;
    //struct int_param_s int_param;
    /**************************** Setup STM32 hardware **********************/
    Set_System();
    Set_USBClock();
    USB_Interrupts_Config();
    USB_Init();
    /************ 1. Initialize CPAL for I2C communication with MPU9150 *****/
    Delay(993400); // USB configuration takes some time ( useful in case USB not present)
    //while(bDeviceState != CONFIGURED); // If USB present, Wait until USB is configured
    MPU9150_Config();
    if(MPU9150_GetStatus() == SUCCESS)
    {
      Virtual_Com_Write_Buffer("MPU9150-Status is fine", 22);  
    }
    /******** 2. Configure INT pin of STM32 for MPU9150 interrupts *********/
    MPU9150_Interrupt_Init(MPU9150_INT_MODE_EXTI);
    /* If you're not using an MPU9150 AND you're not using DMP features, this
     * function will place all slaves on the primary bus.
     * mpu_set_bypass(1);
     */
    /* Get/set hardware configuration. Start gyro. */
    /* Wake up all sensors. */
    result = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    /* Push both gyro and accel data into the FIFO. */
    result = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    result = mpu_set_sample_rate(DEFAULT_MPU_HZ);
    result = mpu_set_gyro_fsr(2000);
    /* Read back configuration in case it was set improperly. */
    result = mpu_get_sample_rate(&gyro_rate);
    result = mpu_get_gyro_fsr(&gyro_fsr);
    result = mpu_get_accel_fsr(&accel_fsr);

    /* Initialize HAL state variables. */
    memset(&hal, 0, sizeof(hal));
    hal.sensors = ACCEL_ON | GYRO_ON;
    hal.report = PRINT_QUAT;

    /* To initialize the DMP:
     * 1. Call dmp_load_motion_driver_firmware(). This pushes the DMP image in
     *    inv_mpu_dmp_motion_driver.h into the MPU memory.
     * 2. Push the gyro and accel orientation matrix to the DMP.
     * 3. Register gesture callbacks. Don't worry, these callbacks won't be
     *    executed unless the corresponding feature is enabled.
     * 4. Call dmp_enable_feature(mask) to enable different features.
     * 5. Call dmp_set_fifo_rate(freq) to select a DMP output rate.
     * 6. Call any feature-specific control functions.
     *
     * To enable the DMP, just call mpu_set_dmp_state(1). This function can
     * be called repeatedly to enable and disable the DMP at runtime.
     *
     * The following is a short summary of the features supported in the DMP
     * image provided in inv_mpu_dmp_motion_driver.c:
     * DMP_FEATURE_LP_QUAT: Generate a gyro-only quaternion on the DMP at
     * 200Hz. Integrating the gyro data at higher rates reduces numerical
     * errors (compared to integration on the MCU at a lower sampling rate).
     * DMP_FEATURE_6X_LP_QUAT: Generate a gyro/accel quaternion on the DMP at
     * 200Hz. Cannot be used in combination with DMP_FEATURE_LP_QUAT.
     * DMP_FEATURE_TAP: Detect taps along the X, Y, and Z axes.
     * DMP_FEATURE_ANDROID_ORIENT: Google's screen rotation algorithm. Triggers
     * an event at the four orientations where the screen should rotate.
     * DMP_FEATURE_GYRO_CAL: Calibrates the gyro data after eight seconds of
     * no motion.
     * DMP_FEATURE_SEND_RAW_ACCEL: Add raw accelerometer data to the FIFO.
     * DMP_FEATURE_SEND_RAW_GYRO: Add raw gyro data to the FIFO.
     * DMP_FEATURE_SEND_CAL_GYRO: Add calibrated gyro data to the FIFO. Cannot
     * be used in combination with DMP_FEATURE_SEND_RAW_GYRO.
     */
    result = dmp_load_motion_driver_firmware();
    result = dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
    result = dmp_register_tap_cb(tap_cb);
    result = dmp_register_android_orient_cb(android_orient_cb);
    hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
        DMP_FEATURE_GYRO_CAL;
    result = dmp_enable_feature(hal.dmp_features);
    result = dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    result = mpu_set_dmp_state(1);
    hal.dmp_on = 1;
		/************** 3. Enable the Interrupt now **************************/
    MPU9150_Interrupt_Cmd(ENABLE);
		
    while (1) {
        unsigned long sensor_timestamp;
        if (count_out != 0)
            /* A byte has been received via USB. See handle_input for a list of
             * valid commands.
             */
        {   
            handle_input();				   		
            count_out = 0; // received data has been processed
				}

        if (hal.motion_int_mode) {
            /* Enable motion interrupt. */
            mpu_lp_motion_interrupt(500, 1, 5);
            hal.new_gyro = 0;
            /* Wait for the MPU interrupt. */
            while (!hal.new_gyro){
						//Write code for Low Power Mode (LPM)
						}
            /* Restore the previous sensor configuration. */
            mpu_lp_motion_interrupt(0, 0, 0);
            hal.motion_int_mode = 0;
        }

        if (!hal.sensors || !hal.new_gyro) {
            /* Put the stm32 to sleep until a timer interrupt or data ready
             * interrupt is detected.
             */            
            //continue;
        }

        if (hal.new_gyro && hal.dmp_on) {
            short gyro[3], accel[3], sensors;
            unsigned char more;
            long quat[4];
            /* This function gets new data from the FIFO when the DMP is in
             * use. The FIFO can contain any combination of gyro, accel,
             * quaternion, and gesture data. The sensors parameter tells the
             * caller which data fields were actually populated with new data.
             * For example, if sensors == (INV_XYZ_GYRO | INV_WXYZ_QUAT), then
             * the FIFO isn't being filled with accel data.
             * The driver parses the gesture data to determine if a gesture
             * event has occurred; on an event, the application will be notified
             * via a callback (assuming that a callback function was properly
             * registered). The more parameter is non-zero if there are
             * leftover packets in the FIFO.
             */
            dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,
                &more);
            if (!more)
                hal.new_gyro = 0;
            /* Gyro and accel data are written to the FIFO by the DMP in chip
             * frame and hardware units. This behavior is convenient because it
             * keeps the gyro and accel outputs of dmp_read_fifo and
             * mpu_read_fifo consistent.
             */
            if (sensors & INV_XYZ_GYRO && hal.report & PRINT_GYRO)
                send_packet(PACKET_TYPE_GYRO, gyro);
            if (sensors & INV_XYZ_ACCEL && hal.report & PRINT_ACCEL)
                send_packet(PACKET_TYPE_ACCEL, accel);
            /* Unlike gyro and accel, quaternions are written to the FIFO in
             * the body frame, q30. The orientation is set by the scalar passed
             * to dmp_set_orientation during initialization.
             */
            if (sensors & INV_WXYZ_QUAT && hal.report & PRINT_QUAT)
                send_packet(PACKET_TYPE_QUAT, quat);
        } else if (hal.new_gyro) {
            short gyro[3], accel[3];
            unsigned char sensors, more;
            /* This function gets new data from the FIFO. The FIFO can contain
             * gyro, accel, both, or neither. The sensors parameter tells the
             * caller which data fields were actually populated with new data.
             * For example, if sensors == INV_XYZ_GYRO, then the FIFO isn't
             * being filled with accel data. The more parameter is non-zero if
             * there are leftover packets in the FIFO.
             */
            mpu_read_fifo(gyro, accel, &sensor_timestamp, &sensors, &more);
            if (!more)
                hal.new_gyro = 0;
            if (sensors & INV_XYZ_GYRO && hal.report & PRINT_GYRO)
                send_packet(PACKET_TYPE_GYRO, gyro);
            if (sensors & INV_XYZ_ACCEL && hal.report & PRINT_ACCEL)
                send_packet(PACKET_TYPE_ACCEL, accel);
        }
    }
}
/* Sample application code for testing MPU9150 with STM32F103xx family microcontrollers.
 * It initializes Clock system, USB, I2C & MPU9150. Listens to the data received on USB,
 * constructs commands & executes them.
 * You can obtain Accelerometer, Gyro, Magnetometer, 6-Axes DMP quaternion, 
 * Pedometer count, TAP, Android_Orientation & other data.
 */
void Stm32MPU9150compass_test(void)
{
    int result;
    struct int_param_s int_param;
    /**************************** Setup STM32 hardware **********************/
    Set_System();
    Set_USBClock();
    USB_Interrupts_Config();
    USB_Init();
    /************ 1. Initialize CPAL for I2C communication with MPU9150 *****/
    Delay(993400); // USB configuration takes some time ( useful in case USB not present)
    //while(bDeviceState != CONFIGURED); // If USB present, Wait until USB is configured
    MPU9150_Config();
    if(MPU9150_GetStatus() == SUCCESS)
    {
      Virtual_Com_Write_Buffer("MPU9150-Status is fine", 22);  
    }
    /******** 2. Configure INT pin of STM32 for MPU9150 interrupts *********/
    // This time it is done inside mpu_init (inv_mpu.c)
    // MPU9150_Interrupt_Init(MPU9150_INT, MPU9150_INT_MODE_EXTI);
		
    int_param.cb = NULL;
    int_param.pin = 0;
    int_param.lp_exit = 0;
    int_param.active_low = 1;
    // "int_param" structure is doing nothing here, just statisfying calling convention
    result = mpu_init(&int_param); 
    result = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);   
    result = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);   
    result = dmp_load_motion_driver_firmware();// load the DMP firmware
    result = dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
    result = dmp_register_tap_cb(tap_cb);
    result = dmp_register_android_orient_cb(android_orient_cb);
    hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
        DMP_FEATURE_GYRO_CAL;
    result = dmp_enable_feature(hal.dmp_features);
    result = dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    result = mpu_set_dmp_state(1);
		/* Initialize HAL state variables. */
    memset(&hal, 0, sizeof(hal));
    hal.sensors = ACCEL_ON | GYRO_ON;
    hal.report = PRINT_QUAT;
    result = mpu_set_sample_rate(DEFAULT_MPU_HZ);
    mpu_set_compass_sample_rate(100);       // set the compass update rate to match
    /************** 3. Enable the Interrupt now **************************/
    MPU9150_Interrupt_Cmd(ENABLE);
    hal.dmp_on = 1;
		
    while (1) 
    {
        unsigned long sensor_timestamp;
        if (count_out != 0)
        {    /* A byte has been received via USB. See handle_input for a list of
             * valid commands.
             */  
          handle_input();
          count_out = 0;
        }
        //msp430_get_clock_ms(&timestamp);

        if (hal.motion_int_mode) {
            /* Enable motion interrupt. */
            mpu_lp_motion_interrupt(500, 1, 5);
            hal.new_gyro = 0;
            /* Wait for the MPU interrupt. */
            while (!hal.new_gyro){
                //Write code for Low Power Mode (LPM)
            }
            /* Restore the previous sensor configuration. */
            mpu_lp_motion_interrupt(0, 0, 0);
            hal.motion_int_mode = 0;
        }

        if (!hal.sensors || !hal.new_gyro) {
            /* Put the stm32 to sleep until a timer interrupt or data ready
             * interrupt is detected.
             */
            //continue;
        }

        if (hal.new_gyro && hal.dmp_on) {
            short gyro[3], accel[3], sensors;
            short compass[3];
            unsigned char more;
            long quat[4];
            /* This function gets new data from the FIFO when the DMP is in
             * use. The FIFO can contain any combination of gyro, accel,
             * quaternion, and gesture data. The sensors parameter tells the
             * caller which data fields were actually populated with new data.
             * For example, if sensors == (INV_XYZ_GYRO | INV_WXYZ_QUAT), then
             * the FIFO isn't being filled with accel data.
             * The driver parses the gesture data to determine if a gesture
             * event has occurred; on an event, the application will be notified
             * via a callback (assuming that a callback function was properly
             * registered). The more parameter is non-zero if there are
             * leftover packets in the FIFO.
             */
            dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,
                &more);
            //if ( hal.report & PRINT_COMPASS)
            mpu_get_compass_reg(compass,&sensor_timestamp);
            if (!more)
                hal.new_gyro = 0;
            /* Gyro and accel data are written to the FIFO by the DMP in chip
             * frame and hardware units. This behavior is convenient because it
             * keeps the gyro and accel outputs of dmp_read_fifo and
             * mpu_read_fifo consistent.
             */
            if (sensors & INV_XYZ_GYRO && hal.report & PRINT_GYRO)
                send_packet(PACKET_TYPE_GYRO, gyro);
            if (sensors & INV_XYZ_ACCEL && hal.report & PRINT_ACCEL)
                send_packet(PACKET_TYPE_ACCEL, accel);
            if ( hal.report & PRINT_COMPASS)
            		send_packet(PACKET_TYPE_COMPASS, compass);
            /* Unlike gyro and accel, quaternions are written to the FIFO in
             * the body frame, q30. The orientation is set by the scalar passed
             * to dmp_set_orientation during initialization.
             */
            if (sensors & INV_WXYZ_QUAT && hal.report & PRINT_QUAT)
                send_packet(PACKET_TYPE_QUAT, quat);
        } else if (hal.new_gyro) {
            short gyro[3], accel[3], compass[3];
            unsigned char sensors, more;
            /* This function gets new data from the FIFO. The FIFO can contain
             * gyro, accel, both, or neither. The sensors parameter tells the
             * caller which data fields were actually populated with new data.
             * For example, if sensors == INV_XYZ_GYRO, then the FIFO isn't
             * being filled with accel data. The more parameter is non-zero if
             * there are leftover packets in the FIFO.
             */
            mpu_read_fifo(gyro, accel, &sensor_timestamp, &sensors, &more);
            if ( hal.report & PRINT_COMPASS)
               mpu_get_compass_reg(compass,&sensor_timestamp); // get the compass data
            if (!more)
                hal.new_gyro = 0;
            if (sensors & INV_XYZ_GYRO && hal.report & PRINT_GYRO)
                send_packet(PACKET_TYPE_GYRO, gyro);
            if (sensors & INV_XYZ_ACCEL && hal.report & PRINT_ACCEL)
                send_packet(PACKET_TYPE_ACCEL, accel);
            if (hal.report & PRINT_COMPASS)
                send_packet(PACKET_TYPE_COMPASS, compass);						
        }
    }
}
/* Simple application code for testing Virtual COM port on STM32F103xx family microcontrollers.
 * It initializes Clock system, USB & virtual COM port. Listens to the data received on USB &
 * sends the same data back ( Software loopback).
 */
void SimpleUSBvirtualCOMtest()
{
   Set_System();
   Set_USBClock(); 
   USB_Interrupts_Config();
   USB_Init();
   while(bDeviceState != CONFIGURED); // wait until USB is configured	
   while (1)
   { 
    if(count_out)
    {
      Virtual_Com_Write_Buffer(&USB_Rx_Buffer, count_out);  // Just send whatever received on USB
      count_out = 0;
    }
   }
}
/*******************************************************************************
* Function Name  : main.
* Description    : Main routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int main()
{
  //SimpleUSBvirtualCOMtest();
  //Stm32MPU9150noCompass_test();
  Stm32MPU9150compass_test();
  return 0;
}

#ifdef USE_FULL_ASSERT
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif