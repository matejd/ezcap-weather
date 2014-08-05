// TWINS weather station's wireless temp/humidity sensor transmission decoding.
// Uses librtlsdr and is based on rtl_433 (https://github.com/merbanan/rtl_433).
// Compiled on 64-bit Win8, Visual Studio 2012 with prebuilt version of librtlsdr (http://sdr.osmocom.org/trac/wiki/rtl-sdr).
// Zadig (http://rtlsdr.org/softwarewindows) used to install libusb-win32 driver.
// Tested with Ezcap USB 2.0 DVB-T/DAB/FM dongle.
// GPL

#include "rtl-sdr.h"
#include <iostream>
#include <cassert>
#include <vector>

const int DEV_INDEX        = 0; // Device index
const int DECIMATION_LEVEL = 0;

// OOK (on-off keying), pulse distance coding
const int LEVEL_LIMIT      = 10000;
const int SHORT_LIMIT      = 3500/4;
const int LONG_LIMIT       = 7000/4;
const int RESET_LIMIT      = 15000/4;

const int PPM_ERROR        = 0;
const uint32_t SAMPLE_RATE = 250000;
const int FREQUENCY        = 433920000; // 433.92MHz

const int BUFF_NUM         = 32;
const int BUFF_LEN         = 16 * 16384;
const int MAX_BUF_LEN      = 256 * 16384;

// Low-pass filter
const int FILTER_ORDER     = 1;
const int F_SCALE          = 15;
const int S_CONST          = (1 << F_SCALE);

// Pulse distance modulation: the distance between pulses decodes into bits
void pwm_d_decode(int16_t* buf, uint32_t len)
{
    static bool sample_counting = false;
    static bool pulse_counting  = false;
    static bool pulse_distance  = false;
    static int sample_counter   = 0;
    static std::vector<std::vector<bool>> bits; // Rows of bits
    if (bits.size() == 0)
        bits.push_back(std::vector<bool>());

    for (uint32_t i = 0; i < len; i++)
    {
        // A finite-state-machine-like bits decoding: counting samples between
        // successive pulses (which have value over LEVEL_LIMIT).
        // Pulse distance over LONG_LIMIT means new row. Pulse distance
        // over RESET_LIMIT signals end of transmission. Same row
        // is sent several times.
        if (buf[i] > LEVEL_LIMIT)
        {
            pulse_counting = true;
            sample_counting = true;
        }
        if (pulse_counting && (buf[i] < LEVEL_LIMIT))
        {
            pulse_distance = true;
            sample_counter = 0;
            pulse_counting = false;
        }
        if (sample_counting)
            sample_counter++;
        if (pulse_distance && (buf[i] > LEVEL_LIMIT))
        {
            if (sample_counter < LONG_LIMIT)
            {
                bits[bits.size()-1].push_back(sample_counter > SHORT_LIMIT); // 0 or 1
            }
            else
            {
                bits.push_back(std::vector<bool>()); // New row
                pulse_counting = false;
                sample_counter = 0;
            }
            pulse_distance = false;
        }
        if (sample_counter > RESET_LIMIT)
        {
            sample_counting = false;
            sample_counter = 0;
            pulse_distance = false;

            // 12-bit signed temperature (scaled by 10), taken from second row, TODO: check rows are the same
            // or pick the most common one!
            int temp = bits[1][2*8+ 0]*(-1)  +
                       bits[1][2*8+ 1]*1024  +
                       bits[1][2*8+ 2]*512   +
                       bits[1][2*8+ 3]*256   +
                       bits[1][2*8+ 4]*128   +
                       bits[1][2*8+ 5]*64    +
                       bits[1][2*8+ 6]*32    +
                       bits[1][2*8+ 7]*16    +
                       bits[1][2*8+ 8]*8     +
                       bits[1][2*8+ 9]*4     +
                       bits[1][2*8+10]*2     +
                       bits[1][2*8+11]*1;

            // 8-bit unsigned humidity
            uint8_t humi = bits[1][3*8+ 4]*128  +
                           bits[1][3*8+ 5]*64   +
                           bits[1][3*8+ 6]*32   +
                           bits[1][3*8+ 7]*16   +
                           bits[1][3*8+ 8]*8    +
                           bits[1][3*8+ 9]*4    +
                           bits[1][3*8+10]*2    +
                           bits[1][3*8+11]*1;

            std::cout << "temp: " << temp / 10 << "." << abs(temp % 10) << std::endl;
            std::cout << "humi: " << (unsigned int)humi                 << std::endl;

            bits.clear();
        }
    }
}

void envelope_detect(unsigned char* buf, uint32_t len, int decimate)
{
    static uint16_t scaled_squares[256] = {0};
    if (scaled_squares[0] == 0)
    {
        // Precompute scaled squares
        for (int i = 0; i < 256; i++)
            scaled_squares[i] = (128-i) * (128-i);
    }

    uint16_t* sample_buffer = reinterpret_cast<uint16_t*>(buf);
    unsigned int op = 0;
    unsigned int stride = (1 << decimate);

    for (unsigned int i = 0; i < len/2; i += stride)
    {
        sample_buffer[op++] = scaled_squares[buf[2*i]] + scaled_squares[buf[2*i+1]];
    }
}

void low_pass_filter(const uint16_t* x_buf, int16_t* y_buf, uint32_t len)
{
    static int a[FILTER_ORDER+1] = {(int)(1.00000*S_CONST),  (int)(0.96907*S_CONST)};
    static int b[FILTER_ORDER+1] = {(int)(0.015466*S_CONST), (int)(0.015466*S_CONST)};
    static uint16_t lp_xmem[FILTER_ORDER] = {0};

    y_buf[0] = ((a[1] * y_buf[ -1]>>1) +
                (b[0] * x_buf[  0]>>1)  +
                (b[1] * lp_xmem[0]>>1))
                >> (F_SCALE-1);
    for (unsigned int i = 1; i < len; i++)
    {
        y_buf[i] = ((a[1] * y_buf[i-1]>>1) +
                    (b[0] * x_buf[i  ]>>1) +
                    (b[1] * x_buf[i-1]>>1))
                    >> (F_SCALE-1);
    }

    memcpy(lp_xmem,               &x_buf[len-1-FILTER_ORDER], FILTER_ORDER*sizeof(int16_t));
    memcpy(&y_buf[-FILTER_ORDER], &y_buf[len-1-FILTER_ORDER], FILTER_ORDER*sizeof(int16_t));
}

void rtlsdr_callback(unsigned char* buf, uint32_t len, void* ctx)
{
    uint16_t* sbuf = reinterpret_cast<uint16_t*>(buf);
    int16_t* fbuf  = reinterpret_cast<int16_t*>(ctx);

    envelope_detect(buf,  len,  DECIMATION_LEVEL);
    low_pass_filter(sbuf, fbuf, len >> (DECIMATION_LEVEL+1));
    pwm_d_decode   (fbuf, len/2);
}

int main(int argc, char** args)
{
    using namespace std;

    int device_count = rtlsdr_get_device_count();
    if (device_count == 0)
    {
        cout << "No SDR devices found!" << endl;
        return 1;
    }

    cout << "Found " << device_count << " device(s):" << endl;
    for (int i = 0; i < device_count; i++)
    {
        char vendor[256], product[256], serial[256];
        rtlsdr_get_device_usb_strings(i, vendor, product, serial);
        cout << i << ": " << vendor << " " << product << " " << serial << endl;
    }

    cout << "Using device: " << rtlsdr_get_device_name(DEV_INDEX) << endl;

    rtlsdr_dev_t* dev = nullptr;
    int result = -1;
    result = rtlsdr_open                 (&dev, DEV_INDEX);   assert(result >= 0);
    result = rtlsdr_set_sample_rate      (dev,  SAMPLE_RATE); assert(result >= 0);
    result = rtlsdr_set_tuner_gain_mode  (dev,  0);           assert(result >= 0);
    result = rtlsdr_reset_buffer         (dev);               assert(result >= 0);
    result = rtlsdr_set_center_freq      (dev,  FREQUENCY);   assert(result >= 0);
    //result = rtlsdr_set_freq_correction  (dev,  PPM_ERROR);   assert(result >= 0);

    int fbuf_size = MAX_BUF_LEN+FILTER_ORDER;
    int16_t* fbuf = new int16_t[fbuf_size];
    memset(fbuf, 0, fbuf_size * sizeof(int16_t));
    int16_t* ptr = fbuf + FILTER_ORDER;

    result = rtlsdr_read_async(dev, rtlsdr_callback, ptr, BUFF_NUM, BUFF_LEN);
    assert(result >= 0);


    // TODO: rtlsdr_cancel_async does not seem to stop the callbacks!


    rtlsdr_close(dev);
    delete fbuf;
    return 0;
}