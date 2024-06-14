static const char *const RFTAG = "SecplusRF";

#include <inttypes.h>
#include <string.h>
#include "gdo_rf.h"

#define __STDC_FORMAT_MACROS 1
#define RMT_TX_CHANNEL RMT_CHANNEL_0
#define RMT_TX_GPIO 16
#define RMT_RX_CHANNEL RMT_CHANNEL_1
#define RMT_RX_GPIO 14
#define RMT_CLK_DIV 80
#define RMT_TICK_10_US (80000000 / RMT_CLK_DIV / 100000) // 10 us per tick
#define BAUD_RATE 4000
#define PREAMBLE 0x0000F
#define PREAMBLE_BITS 20
#define BIT_DURATION_US (1000000 / BAUD_RATE)        // 250 microseconds
#define SLOW_IDLE_DURATION_US (10000000 / BAUD_RATE) // 25 ms
#define ITEM_DURATION(d) ((d & 0x7fff) * 10 / RMT_TICK_10_US)

static RingbufHandle_t rb = NULL;

void rmt_rx_init() {
  rmt_config_t config = RMT_DEFAULT_CONFIG_RX(RMT_RX_GPIO, RMT_RX_CHANNEL);
  config.clk_div = 80;
  config.mem_block_num = 1;
  ESP_ERROR_CHECK(rmt_config(&config));
  ESP_ERROR_CHECK(rmt_driver_install(config.channel, 1000, 0));
  ESP_ERROR_CHECK(rmt_set_rx_idle_thresh(config.channel, 100));
  ESP_ERROR_CHECK(rmt_get_ringbuf_handle(RMT_RX_CHANNEL, &rb));
}

void decode_manchester(rmt_item32_t *items, size_t num_items) {
  for (size_t i = 0; i < num_items; i++) {
    uint32_t duration0 = ITEM_DURATION(items[i].duration0);
    uint32_t duration1 = ITEM_DURATION(items[i].duration1);
    bool bit = (duration0 > duration1) ? 1 : 0;
    ESP_LOGI(TAG, "Bit: %d", bit);
  }
}

void rmt_rx_task(void *arg) {
    while (true) {
        // Wait for ring buffer data
        size_t rx_size = 0;
        rmt_item32_t* items = (rmt_item32_t*) xRingbufferReceive(rb, &rx_size, portMAX_DELAY);
        if (items) {
            // Decode the Manchester signal
            decode_manchester(items, rx_size / sizeof(rmt_item32_t));

            // Free the items
            vRingbufferReturnItem(rb, (void*) items);
        }
    }
}

void rmt_tx_init() {
  rmt_config_t config = RMT_DEFAULT_CONFIG_TX(RMT_TX_GPIO, RMT_TX_CHANNEL);
  config.clk_div = 80; // This gives us 1 MHz clock (80 MHz / 80 = 1 MHz)

  // Initialize the RMT driver
  ESP_ERROR_CHECK(rmt_config(&config));
  ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));
}

rmt_item32_t encode_manchester(bool bit) {
  rmt_item32_t item;
  item.duration0 = BIT_DURATION_US / 2; // 0.125 ms
  item.level0 = bit ? 0 : 1;
  item.duration1 = BIT_DURATION_US / 2; // 0.125 ms
  item.level1 = bit ? 1 : 0;
  return item;
}

rmt_item32_t encode_wait() {
  rmt_item32_t item;
  item.duration0 = BIT_DURATION_US * 2; // 0.5 ms
  item.level0 = 0;
  item.duration1 = BIT_DURATION_US * 2; // 0.5 ms
  return item;
}

void load_tx_buffer(uint8_t *buffer, size_t buffer_size, uint8_t *packet,
                    uint8_t packet_id, uint8_t dataBits) {
  size_t totalBits = 0;
  uint32_t preamble = PREAMBLE;

  // Shift packet_id onto the LSB of the preamble
  preamble = (preamble << 2) | (packet_id & 0x03);

  // Load the preamble + packet_id into the buffer
  for (int i = 0; i < (PREAMBLE_BITS + 2); i++) {
    if (totalBits / 8 >= buffer_size)
      return; // Prevent overflow
    int bit = (preamble >> ((PREAMBLE_BITS + 1) - i)) & 1;
    buffer[totalBits / 8] |= bit << (7 - (totalBits % 8));
    totalBits++;
  }

  // Load the data into the buffer
  for (int i = 0; i < dataBits + 1; i++) {
    if (totalBits / 8 >= buffer_size)
      return; // Prevent overflow
    int byteIndex = i / 8;
    int bitIndex = 7 - (i % 8);
    int bit = (packet[byteIndex] >> bitIndex) & 1;
    buffer[totalBits / 8] |= bit << (7 - (totalBits % 8));
    totalBits++;
  }
}

esp_err_t rf_send(uint32_t rolling, uint64_t fixed, uint32_t data) {
  esp_err_t err = ESP_OK;
  uint8_t data_size = 40;
  uint8_t frame_type = 0; // 1 = 64 bits using data, 0 = 40 bits e.g. no data
  if (data > 0) {
    data_size = 64;
    frame_type = 1;
  };

  // Prepare packets
  static uint8_t packet1[8];
  static uint8_t packet2[8];

  // Clear the packets
  memset(packet1, 0, sizeof(packet1));
  memset(packet2, 0, sizeof(packet2));

  // Encode the rolling, fixed, and optional data into a code using the secplus
  // library
  if (encode_v2(rolling, fixed, data, frame_type, packet1, packet2) != 0) {
    return ESP_FAIL;
  }

  ESP_LOG_BUFFER_HEX(RFTAG, packet1, sizeof(packet1));
  ESP_LOG_BUFFER_HEX(RFTAG, packet2, sizeof(packet2));

  // Total RMT items size for both packets and idle duration
  uint8_t total_items_size = (2 * (PREAMBLE_BITS + 2 + data_size)) + 25;
  uint8_t packet_length = PREAMBLE_BITS + 2 + data_size;

  // Allocate memory for the total items
  rmt_item32_t *total_items =
      (rmt_item32_t *)malloc(total_items_size * sizeof(rmt_item32_t));
  if (total_items == NULL) {
    ESP_LOGE(TAG, "Failed to allocate memory for total RMT items");
    return ESP_FAIL;
  }

  // Declare and initialize buffer
  uint8_t buffer[32]; // Buffer size in bytes
  memset(buffer, 0, sizeof(buffer));

  uint8_t index = 0;

  // Load packet1 into buffer and encode into RMT items
  load_tx_buffer(buffer, sizeof(buffer), packet1, 0,
                 packet_length); // packet_id 0
  for (uint8_t i = 0; i < packet_length; ++i) {
    total_items[index++] =
        encode_manchester((buffer[i / 8] >> (7 - (i % 8))) & 0x01);
  }

  // Add idle period
  for (uint8_t i = 0; i < 25; ++i) {
    total_items[index++] = encode_wait();
  }

  // Clear the buffer before loading the second packet
  memset(buffer, 0, sizeof(buffer));

  // Load packet2 into buffer and encode into RMT items
  load_tx_buffer(buffer, sizeof(buffer), packet2, 1,
                 packet_length); // packet_id 1
  for (uint8_t i = 0; i < packet_length; ++i) {
    total_items[index++] =
        encode_manchester((buffer[i / 8] >> (7 - (i % 8))) & 0x01);
  }

  // Send the complete item set
  ESP_ERROR_CHECK(rmt_write_items(RMT_TX_CHANNEL, total_items, index,
                                  true)); // Blocking write

  // Free the allocated memory
  free(total_items);
  return err;
}