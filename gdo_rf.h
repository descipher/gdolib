#include "gdo.h"
#include "esp_log.h"
#include "gdo_priv.h"
#include "secplus.h"
#include <esp_timer.h>



void rmt_rx_init();
void rmt_rx_task(void *arg);
void rmt_tx_init();
void decode_manchester(rmt_item32_t *items, size_t num_items);
rmt_item32_t encode_manchester(bool bit);
esp_err_t rf_send(uint32_t rolling, uint64_t fixed, uint32_t data);