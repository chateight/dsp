/* See https://m0agx.eu/practical-fft-on-microcontrollers-using-cmsis-dsp.html */
//
// CPU reads ADC data version
//
#include <math.h>
#include <stdio.h>
#include "arm_math.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/sync.h"

// For ADC input
#include "hardware/adc.h"
#include "hardware/dma.h" // not in use
#include "hardware/irq.h" // not in use

// use multi core
#include "pico/multicore.h"
#include "hardware/pio.h"

// activate PWM
#include "hardware/pwm.h"
// on board LED
#include "pico/cyw43_arch.h"
// LCD display control library
#include "lcd_st7789_library.h"

void core1_main();

#define FFT_SIZE 256 * 2
#define FRAME_RATE 10
#define RAW_SAMPLES 2560 * 2
#define DOWNSAMPLED 256 * 2
#define DECIMATE_N 10
#define ADC_CLKDIV 96.0f // 50Ksps : not applicable

// Channel 0 is GPIO26 for ADC sampling
#define CAPTURE_CHANNEL 0

int dma_chan; // not in use

uint32_t start_adc_time;
uint32_t start_preprocess_time;
uint32_t start_fft_time;
uint32_t end_fft_time;
uint32_t end_display_time;

q15_t fft_output[FFT_SIZE * 2]; // 出力（複素数 interleaved）
q15_t mag_squared[FFT_SIZE];    // パワースペクトル（Q13形式）
q15_t hann_window[FFT_SIZE];
arm_rfft_instance_q15 fft_instance;

uint16_t capture_buf[RAW_SAMPLES];
q15_t filtered_downsampled[DOWNSAMPLED];

// FFT結果（dB変換後の値 : dual buffer for display control）
// Core0が更新する。Core1は読み取りのみ
int16_t ffft_result_tmp[FFT_SIZE / 2];
volatile int16_t fft_result[2][FFT_SIZE / 2];
volatile int non_active_index = 0;
volatile int next = 0;

void __not_in_flash_func(adc_capture)(uint16_t *buf, size_t count)
{
    adc_fifo_setup(true, false, 0, false, false);
    adc_run(true);
    for (size_t i = 0; i < count; i = i + 1)
        buf[i] = adc_fifo_get_blocking();
    adc_run(false);
    adc_fifo_drain();
}

static inline q15_t lowpass_filter_q15(q15_t input, q15_t prev, q15_t alpha)
{
    int32_t one_minus_alpha = 32768 - alpha; // Q15で (1 - α)
    int32_t filtered = ((int32_t)input * alpha + (int32_t)prev * one_minus_alpha) >> 15;
    return (q15_t)__SSAT(filtered, 16);
}

void filter_and_downsample()
{
    q15_t prev = 0;     // IIRの初期値
    q15_t alpha = 8192; // cut off freq. 23KHz

    int down_idx = 0;

    for (int i = 0; i < RAW_SAMPLES; i++)
    {
        // ADC raw は 12bit（0～4095）想定 → 中心化＆スケーリング
        int32_t centered = (int32_t)capture_buf[i] - 2048;
        q15_t sample = (q15_t)__SSAT(centered << 3, 16); // ≒ Q15スケーリング　Clipping would not happen in this case

        // IIR フィルタ適用
        prev = lowpass_filter_q15(sample, prev, alpha);

        // N点ごとに出力へ保存
        if ((i % DECIMATE_N) == 0 && down_idx < DOWNSAMPLED)
        {
            filtered_downsampled[down_idx++] = prev;
        }
    }
}

// FFT & Power calc
void perform_fft_and_power_spectrum(arm_rfft_instance_q15 *instance, q15_t *input, q15_t *output, q15_t *power_spectrum)
{
    arm_rfft_q15(instance, input, output);
    arm_cmplx_mag_squared_q15(output, power_spectrum, FFT_SIZE);
}

void adc_initialize()
{
    adc_gpio_init(26 + CAPTURE_CHANNEL);
    adc_init();
    adc_select_input(CAPTURE_CHANNEL);
    adc_fifo_setup(
        true,  // enable
        false, // DREQ無効 (DMA使わないので)
        1,     // FIFOに1件あればフラグ立つ
        false, // ERR無視
        false  // 12bitデータをそのまま
    );
    // adc_set_clkdiv(ADC_CLKDIV);
}

// to apply Hann window & call FFT/Power calc
void fft_exec()
{
    // q15_t input[FFT_SIZE];
    q15_t windowed_input[FFT_SIZE];

    float hann_correction = 1.0f / 0.5f; // ハニング窓で約0.5倍になる補正

    for (int n = 0; n < FFT_SIZE; n++)
    {
        int32_t val = filtered_downsampled[n] * hann_window[n];
        windowed_input[n] = (q15_t)(val >> 15);
    }

    start_fft_time = time_us_32();

    perform_fft_and_power_spectrum(&fft_instance, windowed_input, fft_output, mag_squared);

    float q13_to_float = 1.0f / 8192.0f; // Q13 → float

    for (uint32_t j = 0; j < FFT_SIZE / 2; j++)
    {
        float mag_q13 = (float)mag_squared[j] * q13_to_float;
        float mag_corr = mag_q13 * hann_correction; // Hanning補正（約2倍）

        float voltage_rms = sqrtf(mag_corr);
        ffft_result_tmp[j] = (int)20.0f * log10f(voltage_rms + 1e-6f);
    }

    end_fft_time = time_us_32();
}

#define OUTPUT_PIN 2

void setup_pwm()
{
    gpio_set_function(OUTPUT_PIN, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(OUTPUT_PIN);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 1.0f); // PWM clock = 125 MHz / 1 = 125 MHz

    // 周期 = 25,000クロック → 2.5kHz（= 125M / 50.0k）
    pwm_config_set_wrap(&config, 63999);
    pwm_init(slice_num, &config, true);

    // デューティー比 = 50%
    pwm_set_gpio_level(OUTPUT_PIN, 32000);
}

int main()
{
    stdio_init_all();

    cyw43_arch_init(); // debug purpose
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    sleep_ms(1000);
    multicore_launch_core1(core1_main);
    sleep_ms(1000);

    setup_pwm();

    // display buffer initialize
    memset((void *)fft_result, 0, sizeof(fft_result));

    // to prepare Hanning window coefficient
    for (int n = 0; n < FFT_SIZE; n++)
    {
        float hann = 0.5f * (1.0f - cosf(2.0f * M_PI * n / (FFT_SIZE - 1)));
        hann_window[n] = (q15_t)(hann * 32767.0f);
    }
    // initialise FFT instance
    arm_status status = arm_rfft_init_q15(&fft_instance, FFT_SIZE, 0, 1);

    adc_initialize();

    int disp_index = 0;

    while (1)
    {
        adc_run(true);

        start_adc_time = time_us_32();

        adc_capture(capture_buf, RAW_SAMPLES);

        start_preprocess_time = time_us_32();

        adc_run(false);
        filter_and_downsample();

        //  LCD refresh is a sampling mode
        if ((disp_index % FRAME_RATE) == 0)
        {
            fft_exec();
            disp_index = 0;

            // notify that the display data is available
            uint32_t message = 9999;
            multicore_fifo_push_blocking(message);
        }
        else
        {
            disp_index++;
        }
    }
    // wait forever(doesn't reach here)
    __wfi();

    //__BKPT(1);
}

// ----------------------------------------------------------------------------
// core1 will be used for LCD display control
#include "pico/stdlib.h"
#include "lcd_st7789_library.h"
#include "hardware/spi.h"

#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240
#define DB_MIN -120
#define DB_MAX 0
#define COLOR_BG create_color(0, 0, 0)
#define COLOR_FG create_color(255, 255, 255)
#define COLOR_LINE create_color(0, 0, 255)

int offset = 64;
int char_offset = 20;
// y座標をdB値（-120～0）から画面の高さ（0～239）へマッピング（上が0）
int db_to_y(int db_value)
{
    if (db_value < DB_MIN)
        db_value = DB_MIN;
    if (db_value > DB_MAX)
        db_value = DB_MAX;
    return (int)((DB_MAX - db_value) * (SCREEN_HEIGHT - 1) / (DB_MAX - DB_MIN));
}

// FFT棒グラフの描画（差分のみ更新）
void draw_fft_graph()
{
    for (int x = 0; x < FFT_SIZE / 2; x++)
    {
        int y_new = db_to_y(fft_result[1 - non_active_index][x]);
        int y_old = db_to_y(fft_result[non_active_index][x]);

        if (y_new != y_old)
        {
            //  消す（古い棒） → 黒
            if (y_old < SCREEN_HEIGHT)
                lcd_draw_line(x + offset, y_old, x + offset, SCREEN_HEIGHT - 1, COLOR_BG);

            // 描く（新しい棒） → 白
            if (y_new < SCREEN_HEIGHT)
                lcd_draw_line(x + offset, y_new, x + offset, SCREEN_HEIGHT - 1, COLOR_FG);
        }
    }
}

void core1_main()
{
    stdio_init_all();

    sleep_ms(500);
    // Initialize the LCD
    lcd_init();
    // to draw the display format
    lcd_fill_color(COLOR_BG);
    // print level guide
    lcd_draw_text(char_offset + 10, 0, "0db", COLOR_FG, COLOR_BG, 1);
    lcd_draw_text(char_offset, 40, "-20db", COLOR_FG, COLOR_BG, 1);
    lcd_draw_text(char_offset, 80, "-40db", COLOR_FG, COLOR_BG, 1);
    lcd_draw_text(char_offset, 120, "-60db", COLOR_FG, COLOR_BG, 1);
    lcd_draw_text(char_offset, 160, "-80db", COLOR_FG, COLOR_BG, 1);
    lcd_draw_text(char_offset - 5, 200, "-100db", COLOR_FG, COLOR_BG, 1);
    lcd_draw_text(0, 230, "<0~25KHz>", COLOR_FG, COLOR_BG, 1);

    lcd_draw_line(offset - 1, 0, offset - 1, SCREEN_HEIGHT - 1, COLOR_FG);
    lcd_draw_line(offset - 1, SCREEN_HEIGHT - 1, SCREEN_WIDTH, SCREEN_HEIGHT - 1, COLOR_FG);

    while (1)
    {
        uint32_t data = multicore_fifo_pop_blocking();

        next = 1 - non_active_index;
        // to move fft data to the display buffer
        for (int i = 0; i < FFT_SIZE / 2; i++)
        {
            fft_result[next][i] = ffft_result_tmp[i];
        }

        draw_fft_graph();

        // change the dual buffer active one
        non_active_index = next;

        end_display_time = time_us_32();

        // to draw db reference lines
        for (int i = 0; i < 5; i++)
        {
            lcd_draw_line(offset - 1, (SCREEN_HEIGHT / 6) * (i + 1), SCREEN_WIDTH, (SCREEN_HEIGHT / 6) * (i + 1), COLOR_LINE);
        }
    }
}
