#include "J1.h"
#include "../event/event.h"
#include "../event/subscribe.h"
#include "../protocol/protocol_sacp.h"

HardwareSerial *hmi_serial = &MSerial2;
SACP_param_t hmi_sacp_params = {0};

void hmi_loop(void *arg) {
  hmi_serial->begin(115200);
  evevt_struct_t event;
  event.write = [](unsigned char ch)->size_t{return hmi_serial->write(ch);};
  event.onwer = EVENT_SOURCE_HMI;
  event.info = &hmi_sacp_params.sacp;

  while (true) {
    int ch = hmi_serial->read();
    if (ch != -1) {
      uint8_t data = ch&0xFF;
      if (protocol_sacp.parse(&data, 1, hmi_sacp_params) == E_SUCCESS) {
        event_handler.parse(event);
      }
    } else {
      vTaskDelay(pdMS_TO_TICKS(5));
    }
  }
}

void J1_setup() {
  BaseType_t ret = xTaskCreate(hmi_loop, "hmi_loop", 1024*3,NULL, 5, NULL);
  if (ret != pdPASS) {
    SERIAL_ECHO("Failed to create hmi_loop!\n");
  }
  else {
    SERIAL_ECHO("Created hmi_loop task!\n");
  }
  subscribe_init();
  event_init();
}
