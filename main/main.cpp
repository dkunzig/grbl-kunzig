/*
An example showing the ESP32 as a
WebSocket server.

Demonstrates:
the ESP32 as a WiFi Access Point,
embedding files (html, js, etc.) for a server,
WebSockets,
LED control.

All example options are in "Example Options"

All WebSocket Server options are in:
Component config ---> WebSocket Server

Connect an LED to pin 2 (default)
Connect to the Access Point,
default name: "ESP32 Test"
password: "hello world"

go to 192.168.4.1 in a browser

Note that there are two regular server tasks.
The first gets incoming clients, then passes them
to a queue for the second task to actually handle.
I found that connections were dropped much less frequently
this way, especially when handling large requests. It
does use more RAM though.
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "lwip/api.h"

#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/ledc.h"
#include "esp_event.h"
#include "string.h"
#include "stdio.h"
#include "esp_wifi_types.h"
#include "websocket_server.h"
#include "nuts_bolts.h"
#include "system.h"
#include "main.h"
#include "esp_wifi.h"
#include "stepper.h"
//#include "stddef.h"
#include "serial.h"
#include "settings.h"
#include "stepper.h"
#include "system.h"
#include "gcode.h"
#include "serial2Socket.h"
#include "protocol.h"
int  gline=0;
void debugTask (void *pvParameters);
#define LED_PIN CONFIG_LED_PIN
#define AP_SSID CONFIG_AP_SSID
#define AP_PSSWD CONFIG_AP_PSSWD
#define EXAMPLE_ESP_WIFI_SSID "home-2.4G"
#define EXAMPLE_ESP_WIFI_PASS "6453kunz" 
extern void serial_store(char[]);
extern bool state_one;
extern bool state_two;
extern bool state_three;
extern bool state_four;
int numberOfSpacesInWebsocketQueue;
static EventGroupHandle_t s_wifi_event_group;
char dest[20];
/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
QueueHandle_t websocket_queue;
QueueHandle_t gcode_queue;
int32_t lValueToSend;
// Declare system global variable structure
system_t sys; 
int32_t sys_position[N_AXIS];      // Real-time machine (aka home) position vector in steps.
//define DEBUG_WEBSOCKETS
int32_t sys_probe_position[N_AXIS]; // Last probe position in machine coordinates and steps.
volatile uint8_t sys_probe_state;   // Probing state value.  Used to coordinate the probing cycle with stepper ISR.
volatile uint8_t sys_rt_exec_state;   // Global realtime executor bitflag variable for state management. See EXEC bitmasks.
volatile uint8_t sys_rt_exec_alarm;   // Global realtime executor bitflag variable for setting various alarms.
volatile uint8_t sys_rt_exec_motion_override; // Global realtime executor bitflag variable for motion-based overrides.
volatile uint8_t sys_rt_exec_accessory_override; // Global realtime executor bitflag variable for spindle/coolant overrides.
#ifdef DEBUG
  volatile uint8_t sys_rt_exec_debug;
#endif
static const char *TAG = "wifi station";
static int s_retry_num = 0;
static QueueHandle_t client_queue;
const static int client_queue_size = 10;
bool websocketConnected;
char line_buffer[20][20];
//char line_bufferB[20];
 
static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < 10) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}
void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));
/*
    wifi_config_t wifi_config = {
        .sta = {
               .ssid = EXAMPLE_ESP_WIFI_SSID,
               .password = EXAMPLE_ESP_WIFI_PASS,
        },
    };
*/    
    
    wifi_config_t wifi_config;
    wifi_config = {};
    //memset(wifi_config.sta.ssid,0,32);
    //memset(wifi_config.sta.password,0,64);
    memcpy(wifi_config.sta.ssid,EXAMPLE_ESP_WIFI_SSID,strlen(EXAMPLE_ESP_WIFI_SSID));
    memcpy(wifi_config.sta.password,EXAMPLE_ESP_WIFI_PASS,strlen(EXAMPLE_ESP_WIFI_PASS)); 
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 wifi_config.sta.ssid, wifi_config.sta.password);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler));
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler));
    vEventGroupDelete(s_wifi_event_group);
}




// handles websocket events
void websocket_callback(uint8_t num,WEBSOCKET_TYPE_t type,char msg[][30],uint64_t len) {
  const static char* TAG = "websocket_callback"; 
  int value;
  switch(type) {
    case WEBSOCKET_CONNECT:
      ESP_LOGI(TAG,"client %i connected!",num);
      websocketConnected = true;
      break;
    case WEBSOCKET_DISCONNECT_EXTERNAL:
      ESP_LOGI(TAG,"client %i sent a disconnect message",num);
      websocketConnected = false;
      break;
    case WEBSOCKET_DISCONNECT_INTERNAL:
      ESP_LOGI(TAG,"client %i was disconnected",num);
      websocketConnected = false;
      break;
    case WEBSOCKET_DISCONNECT_ERROR:
      ESP_LOGI(TAG,"client %i was disconnected due to an error",num);
      websocketConnected = false;
      break;
    case WEBSOCKET_TEXT:
      if(len) // if the message length was greater than zero
      {      
            //numberOfSpacesInWebsocketQueue = uxQueueSpacesAvailable(websocket_queue);
					  //if(numberOfSpacesInWebsocketQueue > 18)
					  	 //printf("number of spaces in webSocket_queue = %d\n",numberOfSpacesInWebsocketQueue);
          int i;
          for(i=0;i < 20;i++)
          {    
            if(msg[i][0] != '\0')
            {   
              printf("main.cpp websocket_callback msg = %s\n",msg[i]);
              if( xQueueSendToBack( websocket_queue, msg[i], ( TickType_t ) 500 ) != pdPASS )
                  printf("Failed to post the message on websocket Queue after 50 ticks\n");
            }
            else
            {                
                break;
            }
          }
      }
      break;
    case WEBSOCKET_BIN:
      //ESP_LOGI(TAG,"client %i sent binary message of size %i:\n%s",num,(uint32_t)len,msg);
      break;
    case WEBSOCKET_PING:
      //ESP_LOGI(TAG,"client %i pinged us with message of size %i:\n%s",num,(uint32_t)len,msg);
      break;
    case WEBSOCKET_PONG:
      //ESP_LOGI(TAG,"client %i responded to the ping",num);
      break;
  }
}

// serves any clients
static void http_serve(struct netconn *conn) {
  const static char* TAG = "http_server";
  const static char HTML_HEADER[] = "HTTP/1.1 200 OK\nContent-type: text/html\n\n";
  const static char ERROR_HEADER[] = "HTTP/1.1 404 Not Found\nContent-type: text/html\n\n";
  const static char JS_HEADER[] = "HTTP/1.1 200 OK\nContent-type: text/javascript\n\n";
  const static char CSS_HEADER[] = "HTTP/1.1 200 OK\nContent-type: text/css\n\n";
  //const static char PNG_HEADER[] = "HTTP/1.1 200 OK\nContent-type: image/png\n\n";
  const static char ICO_HEADER[] = "HTTP/1.1 200 OK\nContent-type: image/x-icon\n\n";
  //const static char PDF_HEADER[] = "HTTP/1.1 200 OK\nContent-type: application/pdf\n\n";
  //const static char EVENT_HEADER[] = "HTTP/1.1 200 OK\nContent-Type: text/event-stream\nCache-Control: no-cache\nretry: 3000\n\n";
  struct netbuf* inbuf;
  static char* buf;
  static uint16_t buflen;
  static err_t err;

  // default page
  extern const uint8_t root_html_start[] asm("_binary_root_html_start");
  extern const uint8_t root_html_end[] asm("_binary_root_html_end");
  const uint32_t root_html_len = root_html_end - root_html_start;

  // test.js
  extern const uint8_t test_js_start[] asm("_binary_test_js_start");
  extern const uint8_t test_js_end[] asm("_binary_test_js_end");
  const uint32_t test_js_len = test_js_end - test_js_start;

  // test.css
  extern const uint8_t test_css_start[] asm("_binary_test_css_start");
  extern const uint8_t test_css_end[] asm("_binary_test_css_end");
  const uint32_t test_css_len = test_css_end - test_css_start;

  // favicon.ico
  extern const uint8_t favicon_ico_start[] asm("_binary_favicon_ico_start");
  extern const uint8_t favicon_ico_end[] asm("_binary_favicon_ico_end");
  const uint32_t favicon_ico_len = favicon_ico_end - favicon_ico_start;

  // error page
  extern const uint8_t error_html_start[] asm("_binary_error_html_start");
  extern const uint8_t error_html_end[] asm("_binary_error_html_end");
  const uint32_t error_html_len = error_html_end - error_html_start;

  netconn_set_recvtimeout(conn,1000); // allow a connection timeout of 1 second
  ESP_LOGI(TAG,"reading from client...");
  err = netconn_recv(conn, &inbuf);
  ESP_LOGI(TAG,"read from client");
  if(err==ERR_OK) {
    netbuf_data(inbuf, (void**)&buf, &buflen);
    if(buf) {

      // default page
      if     (strstr(buf,"GET / ")
          && !strstr(buf,"Upgrade: websocket")) {
        ESP_LOGI(TAG,"Sending /");
        netconn_write(conn, HTML_HEADER, sizeof(HTML_HEADER)-1,NETCONN_NOCOPY);
        netconn_write(conn, root_html_start,root_html_len,NETCONN_NOCOPY);
        netconn_close(conn);
        netconn_delete(conn);
        netbuf_delete(inbuf);
      }

      // default page websocket
      else if(strstr(buf,"GET / ")
           && strstr(buf,"Upgrade: websocket")) {
        ESP_LOGI(TAG,"Requesting websocket on /");
        ws_server_add_client(conn,buf,buflen,"/",websocket_callback);
        netbuf_delete(inbuf);
      }

      else if(strstr(buf,"GET /test.js ")) {
        ESP_LOGI(TAG,"Sending /test.js");
        netconn_write(conn, JS_HEADER, sizeof(JS_HEADER)-1,NETCONN_NOCOPY);
        netconn_write(conn, test_js_start, test_js_len,NETCONN_NOCOPY);
        netconn_close(conn);
        netconn_delete(conn);
        netbuf_delete(inbuf);
      }

      else if(strstr(buf,"GET /test.css ")) {
        ESP_LOGI(TAG,"Sending /test.css");
        netconn_write(conn, CSS_HEADER, sizeof(CSS_HEADER)-1,NETCONN_NOCOPY);
        netconn_write(conn, test_css_start, test_css_len,NETCONN_NOCOPY);
        netconn_close(conn);
        netconn_delete(conn);
        netbuf_delete(inbuf);
      }

      else if(strstr(buf,"GET /favicon.ico ")) {
        ESP_LOGI(TAG,"Sending favicon.ico");
        netconn_write(conn,ICO_HEADER,sizeof(ICO_HEADER)-1,NETCONN_NOCOPY);
        netconn_write(conn,favicon_ico_start,favicon_ico_len,NETCONN_NOCOPY);
        netconn_close(conn);
        netconn_delete(conn);
        netbuf_delete(inbuf);
      }

      else if(strstr(buf,"GET /")) {
        ESP_LOGI(TAG,"Unknown request, sending error page: %s",buf);
        netconn_write(conn, ERROR_HEADER, sizeof(ERROR_HEADER)-1,NETCONN_NOCOPY);
        netconn_write(conn, error_html_start, error_html_len,NETCONN_NOCOPY);
        netconn_close(conn);
        netconn_delete(conn);
        netbuf_delete(inbuf);
      }

      else {
        ESP_LOGI(TAG,"Unknown request");
        netconn_close(conn);
        netconn_delete(conn);
        netbuf_delete(inbuf);
      }
    }
    else {
      ESP_LOGI(TAG,"Unknown request (empty?...)");
      netconn_close(conn);
      netconn_delete(conn);
      netbuf_delete(inbuf);
    }
  }
  else { // if err==ERR_OK
    ESP_LOGI(TAG,"error on read, closing connection");
    netconn_close(conn);
    netconn_delete(conn);
    netbuf_delete(inbuf);
  }
}

// handles clients when they first connect. passes to a queue
static void server_task(void* pvParameters) {
  const static char* TAG = "server_task";
  struct netconn *conn, *newconn;
  static err_t err;
  client_queue = xQueueCreate(client_queue_size,sizeof(struct netconn*));

  conn = netconn_new(NETCONN_TCP);
  netconn_bind(conn,NULL,80);
  netconn_listen(conn);
  ESP_LOGI(TAG,"server listening");
  do {
    err = netconn_accept(conn, &newconn);
    ESP_LOGI(TAG,"new client");
    if(err == ERR_OK) {
      xQueueSendToBack(client_queue,&newconn,portMAX_DELAY);
      //http_serve(newconn);
    }
  } while(err == ERR_OK);
  netconn_close(conn);
  netconn_delete(conn);
  ESP_LOGE(TAG,"task ending, rebooting board");
  esp_restart();
}

// receives clients from queue, handles them
static void server_handle_task(void* pvParameters) {
  const static char* TAG = "server_handle_task";
  struct netconn* conn;
  ESP_LOGI(TAG,"task starting");
  for(;;) {
    xQueueReceive(client_queue,&conn,portMAX_DELAY);
    if(!conn) continue;
    http_serve(conn);
  }
  vTaskDelete(NULL);
}

static void count_task(void* pvParameters) {
  const static char* TAG = "count_task";
  char out[20];
  int len;
  int clients;
  const static char* word = "%i";
  uint8_t n = 0;
  const int DELAY = 1000 / portTICK_PERIOD_MS; // 1 second

  ESP_LOGI(TAG,"starting task");
  for(;;) {
    len = sprintf(out,word,n);
    clients = ws_server_send_text_all(out,len);
    if(clients > 0) {
      //ESP_LOGI(TAG,"sent: \"%s\" to %i clients",out,clients);
    }
    n++;
    vTaskDelay(DELAY);
  }
}



void setup() {
	printf("core = %d\n",xPortGetCoreID());
	serial_init();   // Setup serial baud rate and interrupts
  settings_init(); // Load Grbl settings from EEPROM
  stepper_init();  // Configure stepper pins and interrupt timers bob
  system_ini();   // Configure pinout pins and pin-change interrupt (Renamed due to conflict with esp32 files)
  memset(sys_position,0,sizeof(sys_position)); // Clear machine position.

	//printf("in setup-4\n");
  // Check for power-up and set system alarm if homing is enabled to force homing cycle
  // by setting Grbl's alarm state. Alarm locks out all g-code commands, including the
  // startup scripts, but allows access to settings and internal commands. Only a homing
  // cycle '$H' or kill alarm locks '$X' will disable the alarm.
  // NOTE: The startup script will run after successful completion of the homing cycle, but
  // not after disabling the alarm locks. Prevents motion startup blocks from crashing into
  // things uncontrollably. Very bad.
  #ifdef HOMING_INIT_LOCK
    if (bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE)) { sys.state = STATE_ALARM; }  
  #endif

 
  //wifi_init_sta();
  ws_server_start();
  xTaskCreate(&server_task,"server_task",3000,NULL,9,NULL);
  xTaskCreate(&server_handle_task,"server_handle_task",4000,NULL,6,NULL);
  //xTaskCreate(&count_task,"count_task",6000,NULL,2,NULL);
}

void loop() {  
  
	printf("in loop\n");
  // Reset system variables.
  uint8_t prior_state = sys.state;
  memset(&sys, 0, sizeof(system_t)); // Clear system struct variable.
  sys.state = prior_state;
  sys.f_override = DEFAULT_FEED_OVERRIDE;  // Set to 100%
  sys.r_override = DEFAULT_RAPID_OVERRIDE; // Set to 100%
  sys.spindle_speed_ovr = DEFAULT_SPINDLE_SPEED_OVERRIDE; // Set to 100%
  memset(sys_probe_position,0,sizeof(sys_probe_position)); // Clear probe position.
  sys_probe_state = 0;
  sys_rt_exec_state = 0;
  sys_rt_exec_alarm = 0;
  sys_rt_exec_motion_override = 0;
  sys_rt_exec_accessory_override = 0;

  // Reset Grbl primary systems.
  //serial_reset_read_buffer(CLIENT_ALL); // Clear serial read buffer  
  gc_init(); // Set g-code parser to default state  
  spindle_init();  
  coolant_init();  
  limits_init();   
  plan_reset(); // Clear block buffer and planner variables   
  st_reset(); // Clear stepper subsystem variables   

  // Sync cleared gcode and planner positions to current system position.
  plan_sync_position();   
  gc_sync_position();     

  // put your main code here, to run repeatedly:
  report_init_message();   
  
  //start Grbl main loop. Processes program inputs and executes them.  
  protocol_init();  
  //protocol_process_gcode();
}

typedef struct {
	uint32_t steps[N_AXIS];
	uint32_t step_event_count;
	uint8_t direction_bits;
#ifdef VARIABLE_SPINDLE
	uint8_t is_pwm_rate_adjusted; // Tracks motions that require constant laser power/rate
#endif
} st_block_t;

typedef struct {
	uint16_t n_step;           // Number of step events to be executed for this segment
	uint32_t cycles_per_tick;  // Step distance traveled per ISR tick, aka step rate.
	uint8_t  st_block_index;   // Stepper block data index. Uses this information to execute this segment.
#ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
	uint8_t amass_level;    // Indicates AMASS level for the ISR to execute this segment
#else
	uint8_t prescaler;      // Without AMASS, a prescaler is required to adjust for slow timing.
#endif
#ifdef VARIABLE_SPINDLE
	uint8_t spindle_pwm;
#endif
} segment_t;

typedef struct {
	// Used by the bresenham line algorithm
	uint32_t counter_x,        // Counter variables for the bresenham line tracer
	         counter_y,
	         counter_z,
			 counter_a;
#ifdef STEP_PULSE_DELAY
	uint8_t step_bits;  // Stores out_bits output to complete the step pulse delay
#endif

	uint8_t execute_step;     // Flags step execution for each interrupt.
	uint8_t step_pulse_time;  // Step pulse reset time after step rise
	uint8_t step_outbits;         // The next stepping-bits to be output
	uint8_t dir_outbits;
	uint16_t step_count;       // Steps remaining in line segment motion
	uint8_t exec_block_index; // Tracks the current st_block index. Change indicates new block.
	st_block_t *exec_block;   // Pointer to the block data for the segment being executed
	segment_t *exec_segment;  // Pointer to the segment being executed
} stepper_t;

QueueHandle_t debug_queue;

 extern "C" void app_main() {
 //void app_main() {
  nvs_flash_init();
  websocket_queue = xQueueCreate( 20, 80  );
  gcode_queue = xQueueCreate( 20, 80  );
  debug_queue = xQueueCreate( 1000, sizeof(stepper_t) );
  static TaskHandle_t debugTaskHandle = 0;
  xTaskCreate(debugTask,    // task
													"debugTask", // name for task
													8192,   // size of task stack
													NULL,   // parameters
													1, // priority
													&debugTaskHandle); 
  esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
  wifi_init_sta();
  setup();
  loop();
}
BaseType_t *t;
void debugTask (void *pvParameters)
{
  stepper_t st;
  while(1)
  {
    xQueueReceive(debug_queue,&st,portMAX_DELAY);
    printf("st.dir_outbits = %x\n",st.dir_outbits);
    printf("st.step_count = %d\n",st.step_count );
    printf("st.steps[X_AXIS] = %d\n",st.exec_block->steps[0]);
    printf("st.steps[Y_AXIS] = %d\n",st.exec_block->steps[1]);
    printf("st.steps[Z_AXIS] = %d\n",st.exec_block->steps[2]);
    printf("st.step_outbits = %x\n",st.step_outbits);
        printf("st.exec_block->direction_bits = %d\n",st.exec_block->direction_bits);
  } 
}

