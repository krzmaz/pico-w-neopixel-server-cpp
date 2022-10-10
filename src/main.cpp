#include <cstdlib>
#include <string.h>
#include <stdlib.h>

#include "lwip/apps/httpd.h"
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "lwipopts.h"

#include "ws2812.pio.h"

#define WS2812_PIN 2
#define IS_RGBW false
#define NUM_PIXELS 8

#define DATA_BUFSIZE 2048

static inline void put_pixel(uint32_t pixel_grb) {
    pio_sm_put_blocking(pio0, 0, pixel_grb << 8u);
}

static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return
            ((uint32_t) (r) << 8) |
            ((uint32_t) (g) << 16) |
            (uint32_t) (b);
}

// static void *current_connection;
// static void *valid_connection;
// static bool valid_data = true;
// static char last_user[DATA_BUFSIZE];

err_t
httpd_post_begin(void *connection, const char *uri, const char *http_request,
                 u16_t http_request_len, int content_len, char *response_uri,
                 u16_t response_uri_len, u8_t *post_auto_wnd)
{
  LWIP_UNUSED_ARG(connection);
  LWIP_UNUSED_ARG(http_request);
  LWIP_UNUSED_ARG(http_request_len);
  if (!memcmp(uri, "/pixels", 8)) {
    // change this to 0 and handle in httpd_post_data_recved if we will be too slow
    *post_auto_wnd = 1;
    return ERR_OK;
  }
  return ERR_VAL;
}

err_t
httpd_post_receive_data(void *connection, struct pbuf *p)
{
  err_t ret;

  LWIP_ASSERT("NULL pbuf", p != NULL);

  if (true) {
    u16_t token_data = pbuf_memfind(p, "data=", 5, 0);
    if ((token_data != 0xFFFF)) {
      u16_t value_data = token_data + 5;
      u16_t len_data = 0;
      /* find data len */
      len_data = p->tot_len - value_data;
      // TODO: modulo 3 the len (each pixel needs 3 bytes)
      if ((len_data > 0) && (len_data < DATA_BUFSIZE)) {
        /* provide contiguous storage if p is a chained pbuf */
        char buf_data[DATA_BUFSIZE];
        char *data = (char *)pbuf_get_contiguous(p, buf_data, sizeof(buf_data), len_data, value_data);
        if (data) {
          data[len_data] = 0;
          for (int i=0; i<len_data; i+=3) {
            put_pixel(urgb_u32(data[i], data[i+1], data[i+2]));
          }
        }
      }
    }
    /* not returning ERR_OK aborts the connection, so return ERR_OK unless the
       connection is unknown */
    ret = ERR_OK;
  } else {
    ret = ERR_VAL;
  }

  /* this function must ALWAYS free the pbuf it is passed or it will leak memory */
  pbuf_free(p);

  return ret;
}

void
httpd_post_finished(void *connection, char *response_uri, u16_t response_uri_len)
{
  snprintf(response_uri, response_uri_len, "/res.json");
}

void run_server() {
    httpd_init();
    printf("Http server initialized.\n");
    // infinite loop for now
    for (;;) {}
}

int main() {
    stdio_init_all();

    if (cyw43_arch_init()) {
        printf("failed to initialise\n");
        return 1;
    }
    // turn on LED to distinguish from BOOTSEL mode
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    cyw43_arch_enable_sta_mode();
    // this seems to be the best be can do using the predefined `cyw43_pm_value` macro:
    // cyw43_wifi_pm(&cyw43_state, CYW43_PERFORMANCE_PM);
    // however it doesn't use the `CYW43_NO_POWERSAVE_MODE` value, so we do this instead:
    cyw43_wifi_pm(&cyw43_state, cyw43_pm_value(CYW43_NO_POWERSAVE_MODE, 20, 1, 1, 1));

    printf("Connecting to WiFi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("failed to connect.\n");
        return 1;
    } else {
        printf("Connected.\n");
    }

    PIO pio = pio0;
    int sm = 0;
    uint offset = pio_add_program(pio, &ws2812_program);

    ws2812_program_init(pio, sm, offset, WS2812_PIN, 800000, IS_RGBW);

    
    run_server();
}