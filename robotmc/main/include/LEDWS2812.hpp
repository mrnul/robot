#include <vector>

#include "driver/rmt_tx.h"
#include "MutexGuard.hpp"

using std::vector;

#define WS2812_LED_STRIP_RESOLUTION_HZ 10000000

const rmt_symbol_word_t ws2812_zero = {
    .duration0 = calculate_duration(0.4, WS2812_LED_STRIP_RESOLUTION_HZ),
    .level0 = 1,
    .duration1 = calculate_duration(0.85, WS2812_LED_STRIP_RESOLUTION_HZ),
    .level1 = 0};
const rmt_symbol_word_t ws2812_one = {
    .duration0 = calculate_duration(0.8, WS2812_LED_STRIP_RESOLUTION_HZ),
    .level0 = 1,
    .duration1 = calculate_duration(0.45, WS2812_LED_STRIP_RESOLUTION_HZ),
    .level1 = 0};

class LEDWS2812
{
private:
    const int num_of_leds;
    vector<uint8_t> led_strip_payload;
    rmt_transmit_config_t tx_config;
    rmt_encoder_handle_t simple_encoder;
    rmt_channel_handle_t led_chan;
    SemaphoreHandle_t mutex;

private:
    void clean()
    {
        MutexGuard lock(mutex, "LEDWS2812:~LEDWS2812");
        ESP_ERROR_CHECK(rmt_disable(led_chan));
        ESP_ERROR_CHECK(rmt_del_encoder(simple_encoder));
        ESP_ERROR_CHECK(rmt_del_channel(led_chan));
        led_strip_payload.clear();

        led_chan = nullptr;
        simple_encoder = nullptr;
        led_chan = nullptr;
    }

public:
    LEDWS2812(gpio_num_t gpio_num = GPIO_NUM_8, const int led_count = 1)
        : num_of_leds(led_count),
          led_strip_payload(vector<uint8_t>(num_of_leds * 3)),
          tx_config(),
          simple_encoder(nullptr),
          led_chan(nullptr),
          mutex(xSemaphoreCreateMutex())
    {
        rmt_tx_channel_config_t tx_chan_config = {
            .gpio_num = gpio_num,
            .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
            .resolution_hz = WS2812_LED_STRIP_RESOLUTION_HZ,
            .mem_block_symbols = 64, // increase the block size can make the LED less flickering
            .trans_queue_depth = 4,  // set the number of transactions that can be pending in the background
            .intr_priority = 0,
            .flags = {}};

        ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));

        const rmt_bytes_encoder_config_t bytes_encoder_config = {
            .bit0 = ws2812_zero,
            .bit1 = ws2812_one,
            .flags = {.msb_first = 1}};

        tx_config.loop_count = 0; // no transfer loop
        ESP_ERROR_CHECK(rmt_new_bytes_encoder(&bytes_encoder_config, &simple_encoder));
        ESP_ERROR_CHECK(rmt_enable(led_chan));
    }

    void set(uint8_t r, uint8_t g, uint8_t b)
    {
        for (int led = 0; led < num_of_leds; led++)
        {
            led_strip_payload[led * 3 + 0] = r;
            led_strip_payload[led * 3 + 1] = g;
            led_strip_payload[led * 3 + 2] = b;
        }

        {
            MutexGuard lock(mutex, "LEDWS2812:set");
            ESP_ERROR_CHECK(rmt_transmit(led_chan, simple_encoder, (const void *)led_strip_payload.data(), sizeof(led_strip_payload), &tx_config));
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, -1));
        }
    }

    ~LEDWS2812()
    {
        clean();
        vSemaphoreDelete(mutex);
        mutex = nullptr;
    }
};
