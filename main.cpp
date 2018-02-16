#define DTH22_SENSOR
#define LCD_MODULE
#define DEBUG_CLIENT_TIME

#include <time.h>
#include <iostream>
#include <cstring>
#include <vector>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "driver/touch_pad.h"
#include "esp_attr.h"
#include "esp_system.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "cJSON.h"
#include "driver/uart.h"

#include "freertos/portmacro.h"
#include "driver/periph_ctrl.h"
#include "driver/ledc.h"
#include "driver/pcnt.h"
#include "soc/gpio_sig_map.h"
//Arduino library
#include "Arduino.h"
//Memory library
#include <Preferences.h>
//SHT1x sensor
#include "SHT1x.h"
//LCD library
#include "U8g2lib.h"
//Keypad library
#include "Keypad.h"
//DHT22 library
#include "DHT.h"

#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"

#include <sys/socket.h>
#include <netdb.h>
#include "esp_ota_ops.h"
#include "esp_partition.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

//server library
#include "mongoose.h"

//Software Version
const String version = "2.9";

//Uart buffer size
#define BUF_SIZE (1024)

//define max number of input pins
#define MAX_INPUTS 8

//PCNT FILTER VALUE
#define FILTER 100

//MAX COUNTER VALUE
#define MAX_COUNTER_VALUE 9999999

//Limite of events to save offline
#define LIMIT_OFFLINE_SAVE 10000

//rele pin
#define rele 27
//bt1 pin
#define BT1 2
//bt2 pin
#define BT2 4

//Pulse counter High limit value
#define PCNT_H_LIM_VAL  30000

//Pulse counter max number
#define PCNT_MAXCOUNTER 8

const uint8_t PCNT_UNIT_NUM[MAX_INPUTS] = {0,1,2,3,4,5,6,7};

//if dht22 sensor is define
#ifdef DTH22_SENSOR
//sensor pin 21
#define DHTPIN 21     // what digital pin we're connected to
//sensor type dht22
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//initialize sensor
DHT dht(DHTPIN, DHTTYPE);
//end define
#endif

//SHT1x sensor defines
#define dataPin  21

#define clockPin 22

SHT1x sht1x(dataPin, clockPin);

//#################### OTA ####################//


/* Constants that aren't configurable in menuconfig */
#define WEB_SERVER "172.16.1.111"
//#define WEB_SERVER "192.168.1.200"
String ipSend = "172.16.1.111";
#define WEB_PORT 80

#define EXAMPLE_SERVER_IP   "172.16.1.111"
#define EXAMPLE_SERVER_PORT "80"
//#define EXAMPLE_FILENAME "/fluxodata.bin"
String file_update = "";
#define BUFFSIZE 1024
#define TEXT_BUFFSIZE 1024

//Network name
String network = "fluxotecDS01";
//Network password
String network_pass = "fluxotec01";
//Hardware version
String hardware_version = "0";

/*an ota data write buffer ready to write to the flash*/
static char ota_write_data[BUFFSIZE + 1] = { 0 };
/*an packet receive buffer*/
static char text[BUFFSIZE + 1] = { 0 };
/* an image total length*/
static int binary_file_length = 0;
/*socket id*/
static int socket_id = -1;
static char http_request[64] = {0};

int total_file_size = 0;

bool updating = false;

String version_update = "";

//############### OTA ######################//

//module ID
int id_module = 0;

//Memory namespace
const char * NamespacePreferences = "flx";
//Memory initialization
Preferences memory;

//define temperature and humidity strings
String temperature = "0.00",humidity = "0.00";

//define signal_quality variable
int signal_quality = 4; //0->Unusable || 1->Not Good || 2->Okay || 3->Very Good || 4->Amazing
                        //-90 dBm     || -80 dBm     || -70 dBm || -67 dBm      || -30 dBm

//declare input pins
const uint8_t INPUT_PINS[MAX_INPUTS] = {36,39,34,35,32,33,25,26};
//declare input types 1-counter || 0-events || 2-encoder
uint8_t INPUT_FUNCS[MAX_INPUTS] = {1,0,0,0,0,0,0,0};
//declare input states
bool INPUT_STATES[MAX_INPUTS] = {false,false,false,false,false,false,false,false};
//declare INPUT CHANGE timeout
int INPUT_CHANGE_TIME[MAX_INPUTS] = {MAX_COUNTER_VALUE,MAX_COUNTER_VALUE,MAX_COUNTER_VALUE,MAX_COUNTER_VALUE,MAX_COUNTER_VALUE,MAX_COUNTER_VALUE,MAX_COUNTER_VALUE,MAX_COUNTER_VALUE};
//declare INPUT CHANGE TIME STAMP
unsigned long INPUT_CHANGE_TIME_STAMP[MAX_INPUTS] = {0,0,0,0,0,0,0,0};

unsigned long int COUNTERS[MAX_INPUTS] = {0,0,0,0,0,0,0,0};
//unsigned long int COUNTERS_OLD[MAX_INPUTS] = {0,0,0,0,0,0,0,0};

int INPUT_OVERFLOW_GUARD[MAX_INPUTS] =  {10000,10000,10000,10000,10000,10000,10000,10000};

unsigned long int INPUT_OVERFLOW_TIMESTAMP[MAX_INPUTS] =  {0,0,0,0,0,0,0,0};

//########################### LCD VARIABLES ######################//

//lcd blink circle timeout
const int HeartLimiteTime = 500;
//lcd blink circle timestamp
unsigned long int HeartTimeStamp = 0;
//lcd blink circle states
bool heart = false;
//define lcd pins
U8G2_ST7920_128X64_F_SW_SPI u8g2(U8G2_R0, /* clock=*/ 14, /* data=*/ 12, /* CS=*/ 13, /* reset=*/ U8X8_PIN_NONE);

//declare last click string for teste menu
String last_click = "";

int menu_id[50];
char menu[50][26];
bool confirm_menu[50];
bool in_menu[50];
bool reset_count_menu[50];
bool as_sub_menu[50];

int sub_menu_id[100];
int sub_sub_menu_id[100];

char sub_menu[100][26];
bool sub_confirm_menu[100];
bool sub_in_menu[100];
bool sub_reset_menu[100];

char tmp_sub_menu[100][26];
bool tmp_sub_confirm_menu[100];
bool tmp_sub_in_menu[100];
bool tmp_sub_reset_menu[100];

int maxMenu = 0;
int maxSubMenu = 0;

int pos_menu = 0;
int pos_sub_menu = 0;

int tmp_max_sub = 0;
bool pendingResponse = false;
bool onSubMenu = false;
bool menuOn = false;

bool menuSelection = false;
bool online_data_sending = false;

bool inputMode = false;
String inputString = "_____________";
String inputString_old = "_____________";
int inputPos = 0;
bool come_input_barcode = false;
bool check_bar_code = false;

String FX1Line = "Welcome";
String FX2Line = "to";
String FX3Line = "Fluxodata";

bool on_action = false,on_pending = false;

TaskHandle_t xHandle = NULL;

unsigned long int watchTimeStamp;

bool stopedTask = false;

//########################### END LCD VARIABLES ######################//


const byte ROWS = 4; //four rows
const byte COLS = 4; //four columns
//define the cymbols on the buttons of the keypads

char hexaKeys[ROWS][COLS] = {
  {'E','S','B','C'},
  {'R','9','6','3'},
  {'0','8','5','2'},
  {'L','7','4','1'}
};

byte rowPins[ROWS] = {23, 19, 18, 5}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {17, 16, 15, 10}; //connect to the column pinouts of the keypad

//initialize an instance of class NewKeypad
Keypad customKeypad = Keypad( makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);

//########################### WIFI VARIABLES ########################//


//MAC address variable
char macStr[18] = { 0 };
//IP address variable
char MyAddr[20] = { 0 };

bool ONLINE = false,ONLINE_OLD = false;

bool readMenuConfigs = false;

static const char *TAG = "FluxoTec";

struct addrinfo hints;

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
const int CONNECTED_BIT = BIT0;

String counter_event = ""; //Counter event prefix
String io_event = "";     //Input event prefix
String mn_event = "";    //Menu event prefix

int timeoutRequest = 1000; //Time between requests in ms

int offline_saving = 10; //Time in s

float factor = 1.00000; //factor to LCD print

int offline_counter = 0,offline_counter_now = 0;//Event offline counter
int bottom_offline = 0;//Event last offline value send

int countRequest = 0;

clock_t t1, t2;

//######################### END WIFI VARIABLES #####################//

int16_t cnt[8] = {0};
int16_t cnt_old[8] = {0};

xQueueHandle pcnt_evt_queue;  /*A queue to handle pulse counter event*/

typedef struct {
    int unit;        /*pulse counter unit*/
    uint32_t status; /*pulse counter internal status*/
} pcnt_evt_t;


int bt1_change_time = 300;
unsigned long bt1_change_time_stamp = 0;
bool bt1_state = false;

int bt2_change_time = 300;
unsigned long bt2_change_time_stamp = 0;
bool bt2_state = false;

//||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||//

String network_name_su = "fluxotecDS00";
bool net_show = false;
int pos_net = 10;

//#################### FUNCTION DECLARATIONS ########################//

//###################  OTA ##############################//

static void ota_example_task(void *pvParameter);
static bool read_past_http_header(char text[], int total_len, esp_ota_handle_t update_handle);
static bool connect_to_http_server();

//################# END OTA ###########################//

//////////////////// WIFI ///////////////////////////

static esp_err_t event_handler(void *ctx, system_event_t *event);
static void initialise_wifi(void);

static void clientTask(void *pvParameters);
static void checkOffline(void *pvParameters);
bool requestHttpNoRead(String urlSend);
String requestHttp(String urlSend);

bool getConfigsModule();
bool getMenus();
bool getSubMenus();
bool getMenusOFF();
bool getSubMenusOFF();
void save_my_ip();

void sendMenu(String desc);

int countOcurenc(String s,char f);
String getValue(String data, char separator, int index);

//////////////////// END WIFI //////////////////////

////////////////WEB SERVER//////////////////////////

char *mongoose_eventToString(int ev);
char *mgStrToStr(struct mg_str mgStr);
void mongoose_event_handler(struct mg_connection *nc, int ev, void *evData);
void mongooseTask(void *data);

//////////////////////////////////////////////////////

//////////////////// TEST MODE ////////////////////////
static void readInputsTestMenu(void *pvParameters);

static void writeLCDTaskTestMode(void *pvParameters);
///////////////////END TEST MODE/////////////////////

static void touchTask(void *pvParameters);

static void DHT22Task(void *pvParameters);

/////////////////// LCD FUNCTIONS //////////////////
static void writeLCDTask(void *pvParameters);
static void watchTask(void *pvParameters);

void drawLCD();

void clearMain();
void piscaON();
void piscaOFF();

void drawIP(char *ip_DC);
void drawOffline();
void drawOfflineCounter();
void drawSignal();
void clearTop();
void drawID();
void clearBottom();
void drawBottom();
void drawLineFX(int line,char *txt,bool center);
void drawCounter();
void drawMenuLine(int pos,char* txt);
void menu_roling(int pos);
void menu_roling2(int pos);
void enterMenu();
void actionSend(char *txt);
void actionDecider(int action);
void pendingConfirmation();
void acceptSubMenu();
void acceptMenu();
void keyClick(int num,char c);

static void keypadTask(void *pvParameters);
static void keypadTask2(void *pvParameters);
//////////////// END LCD FUNCTION /////////////////

///////////////// MEMORY FUNCTIONS ///////////////

void read_offline_config();
void saveOfflineEvent(int typeOf,String desc);
void resetCounters();
void readOverflowResto();
void clearOfflineData();

/////////////// END MEMORY FUNCTIONS/////////////

////////////////// INPUT FUNCTIONS ////////////
static void pcnt_example_init(int index,int encoder);
static void IRAM_ATTR pcnt_example_intr_handler(void* arg);
void configure_encoder_type(int index);
static void pcnt_task(void *pcnt_task);
///////////////// END INPUT FUNCTIONS /////////


//##################################################################//


//#######################  OTA ####################################//


/*read buffer by byte still delim ,return read bytes counts*/
static int read_until(char *buffer, char delim, int len)
{
//  /*TODO: delim check,buffer check,further: do an buffer length limited*/
    int i = 0;
    while (buffer[i] != delim && i < len) {
        ++i;
    }
    return i + 1;
}

/* resolve a packet from http socket
 * return true if packet including \r\n\r\n that means http packet header finished,start to receive packet body
 * otherwise return false
 * */
static bool read_past_http_header(char text[], int total_len, esp_ota_handle_t update_handle)
{
    /* i means current position */
    int i = 0, i_read_len = 0;
    while (text[i] != 0 && i < total_len) {
        i_read_len = read_until(&text[i], '\n', total_len);
        // if we resolve \r\n line,we think packet header is finished
        if (i_read_len == 2) {
            int i_write_len = total_len - (i + 2);
            memset(ota_write_data, 0, BUFFSIZE);
            /*copy first http packet body to write buffer*/
            memcpy(ota_write_data, &(text[i + 2]), i_write_len);

            esp_err_t err = esp_ota_write( update_handle, (const void *)ota_write_data, i_write_len);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Error: esp_ota_write failed! err=0x%x", err);
                return false;
            } else {
                ESP_LOGI(TAG, "esp_ota_write header OK");
                binary_file_length += i_write_len;
            }
            return true;
        }
        i += i_read_len;
    }
    return false;
}

static bool connect_to_http_server()
{
    ESP_LOGI(TAG, "Server IP: %s Server Port:%s", EXAMPLE_SERVER_IP, EXAMPLE_SERVER_PORT);
    sprintf(http_request, "GET %s HTTP/1.1\r\nHost: %s:%s \r\n\r\n", file_update.c_str(), EXAMPLE_SERVER_IP, EXAMPLE_SERVER_PORT);

    int  http_connect_flag = -1;
    struct sockaddr_in sock_info;

    socket_id = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_id == -1) {
        ESP_LOGE(TAG, "Create socket failed!");
        return false;
    }

    // set connect info
    memset(&sock_info, 0, sizeof(struct sockaddr_in));
    sock_info.sin_family = AF_INET;
    sock_info.sin_addr.s_addr = inet_addr(EXAMPLE_SERVER_IP);
    sock_info.sin_port = htons(atoi(EXAMPLE_SERVER_PORT));

    // connect to http server
    http_connect_flag = connect(socket_id, (struct sockaddr *)&sock_info, sizeof(sock_info));
    if (http_connect_flag == -1) {
        ESP_LOGE(TAG, "Connect to server failed! errno=%d", errno);
        close(socket_id);
        return false;
    } else {
        ESP_LOGI(TAG, "Connected to server");
        return true;
    }
    return false;
}

static void __attribute__((noreturn)) task_fatal_error()
{
    ESP_LOGE(TAG, "Exiting task due to fatal error...");
    close(socket_id);
    (void)vTaskDelete(NULL);

    while (1) {
        ;
    }
}

static void ota_example_task(void *pvParameter)
{
    esp_err_t err;
    /* update handle : set by esp_ota_begin(), must be freed via esp_ota_end() */
    esp_ota_handle_t update_handle = 0 ;
    const esp_partition_t *update_partition = NULL;

    ESP_LOGI(TAG, "Starting OTA example...");

    const esp_partition_t *configured = esp_ota_get_boot_partition();
    const esp_partition_t *running = esp_ota_get_running_partition();

    assert(configured == running); /* fresh from reset, should be running from configured boot partition */
    ESP_LOGI(TAG, "Running partition type %d subtype %d (offset 0x%08x)",
             configured->type, configured->subtype, configured->address);

    /* Wait for the callback to set the CONNECTED_BIT in the
       event group.
    */
    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
                        false, true, portMAX_DELAY);
    ESP_LOGI(TAG, "Connect to Wifi ! Start to Connect to Server....");

    /*connect to http server*/
    if (connect_to_http_server()) {
        ESP_LOGI(TAG, "Connected to http server");
    } else {
        ESP_LOGE(TAG, "Connect to http server failed!");
        task_fatal_error();
    }

    int res = -1;
    /*send GET request to http server*/
    res = send(socket_id, http_request, strlen(http_request), 0);
    if (res == -1) {
        ESP_LOGE(TAG, "Send GET request to server failed");
        task_fatal_error();
    } else {
        ESP_LOGI(TAG, "Send GET request to server succeeded");
    }

    update_partition = esp_ota_get_next_update_partition(NULL);

    ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%x",
             update_partition->subtype, update_partition->address);
    assert(update_partition != NULL);

    err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed, error=%d", err);
        task_fatal_error();
    }
    ESP_LOGI(TAG, "esp_ota_begin succeeded");

    bool resp_body_start = false, flag = true;
    /*deal with all receive packet*/
    while (flag) {
        memset(text, 0, TEXT_BUFFSIZE);
        memset(ota_write_data, 0, BUFFSIZE);
        int buff_len = recv(socket_id, text, TEXT_BUFFSIZE, 0);
        if (buff_len < 0) { /*receive error*/
            ESP_LOGE(TAG, "Error: receive data error! errno=%d", errno);
            task_fatal_error();
        } else if (buff_len > 0 && !resp_body_start) { /*deal with response header*/
            memcpy(ota_write_data, text, buff_len);
            resp_body_start = read_past_http_header(text, buff_len, update_handle);
        } else if (buff_len > 0 && resp_body_start) { /*deal with response body*/
            memcpy(ota_write_data, text, buff_len);
            err = esp_ota_write( update_handle, (const void *)ota_write_data, buff_len);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Error: esp_ota_write failed! err=0x%x", err);
                task_fatal_error();
            }
            binary_file_length += buff_len;
            //ESP_LOGI(TAG, "Have written image length %d/%d", binary_file_length,total_file_size);
            //printf("Updating: %d%% | %d/%d\n",(binary_file_length*100/total_file_size),binary_file_length,total_file_size);
        } else if (buff_len == 0) {  /*packet over*/
            flag = false;
            ESP_LOGI(TAG, "Connection closed, all packets received");
            close(socket_id);
        } else {
            ESP_LOGE(TAG, "Unexpected recv result");
        }

        vTaskDelay(15 / portTICK_PERIOD_MS);
    }

    ESP_LOGI(TAG, "Total Write binary data length : %d", binary_file_length);

    updating = false;

    if (esp_ota_end(update_handle) != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed!");
        task_fatal_error();
    }
    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed! err=0x%x", err);
        task_fatal_error();
    }
    ESP_LOGI(TAG, "Prepare to restart system!");
    esp_restart();
    return ;
}


//###################### END OTA #################################//

//####################### SERVER ################################//

///////////////////////////////////////////SERVER/////////////////////////////////////////////

char *mongoose_eventToString(int ev) {
	static char temp[100];
	switch (ev) {
	case MG_EV_CONNECT:
		return (char*)"MG_EV_CONNECT";
	case MG_EV_ACCEPT:
		return (char*)"MG_EV_ACCEPT";
	case MG_EV_CLOSE:
		return (char*)"MG_EV_CLOSE";
	case MG_EV_SEND:
		return (char*)"MG_EV_SEND";
	case MG_EV_RECV:
		return (char*)"MG_EV_RECV";
	case MG_EV_HTTP_REQUEST:
		return (char*)"MG_EV_HTTP_REQUEST";
	case MG_EV_HTTP_REPLY:
		return (char*)"MG_EV_HTTP_REPLY";
	case MG_EV_MQTT_CONNACK:
		return (char*)"MG_EV_MQTT_CONNACK";
	case MG_EV_MQTT_CONNACK_ACCEPTED:
		return (char*)"MG_EV_MQTT_CONNACK";
	case MG_EV_MQTT_CONNECT:
		return (char*)"MG_EV_MQTT_CONNECT";
	case MG_EV_MQTT_DISCONNECT:
		return (char*)"MG_EV_MQTT_DISCONNECT";
	case MG_EV_MQTT_PINGREQ:
		return (char*)"MG_EV_MQTT_PINGREQ";
	case MG_EV_MQTT_PINGRESP:
		return (char*)"MG_EV_MQTT_PINGRESP";
	case MG_EV_MQTT_PUBACK:
		return (char*)"MG_EV_MQTT_PUBACK";
	case MG_EV_MQTT_PUBCOMP:
		return (char*)"MG_EV_MQTT_PUBCOMP";
	case MG_EV_MQTT_PUBLISH:
		return (char*)"MG_EV_MQTT_PUBLISH";
	case MG_EV_MQTT_PUBREC:
		return (char*)"MG_EV_MQTT_PUBREC";
	case MG_EV_MQTT_PUBREL:
		return (char*)"MG_EV_MQTT_PUBREL";
	case MG_EV_MQTT_SUBACK:
		return (char*)"MG_EV_MQTT_SUBACK";
	case MG_EV_MQTT_SUBSCRIBE:
		return (char*)"MG_EV_MQTT_SUBSCRIBE";
	case MG_EV_MQTT_UNSUBACK:
		return (char*)"MG_EV_MQTT_UNSUBACK";
	case MG_EV_MQTT_UNSUBSCRIBE:
		return (char*)"MG_EV_MQTT_UNSUBSCRIBE";
	case MG_EV_WEBSOCKET_HANDSHAKE_REQUEST:
		return (char*)"MG_EV_WEBSOCKET_HANDSHAKE_REQUEST";
	case MG_EV_WEBSOCKET_HANDSHAKE_DONE:
		return (char*)"MG_EV_WEBSOCKET_HANDSHAKE_DONE";
	case MG_EV_WEBSOCKET_FRAME:
		return (char*)"MG_EV_WEBSOCKET_FRAME";
	}
	sprintf(temp, "Unknown event: %d", ev);
	return temp;
} //eventToString

// Convert a Mongoose string type to a string.
char *mgStrToStr(struct mg_str mgStr) {
	char *retStr = (char *) malloc(mgStr.len + 1);
	memcpy(retStr, mgStr.p, mgStr.len);
	retStr[mgStr.len] = 0;
	return retStr;
} // mgStrToStr

// Mongoose event handler.
void mongoose_event_handler(struct mg_connection *nc, int ev, void *evData) {
	switch (ev)
	{
		case MG_EV_HTTP_REQUEST:
		{
			struct http_message *message = (struct http_message *) evData;

			char *query_string = mgStrToStr(message->query_string);

      String query = String(query_string);
      String func = query.substring(0,query.indexOf('='));
      if(func == "update")
      {
          version_update = query.substring(query.indexOf('=')+1,query.length());
          if(!updating)
          {
            file_update = "/firmware/"+version_update;
            //printf("file_update: %s\n",file_update.c_str());

            String url = "GET http://"+ipSend+"/get_file_size.php?file_name=firmware/"+version_update+" HTTP/1.0\r\n"+"Host: "+ipSend+"\r\n"+"User-Agent: esp-idf/1.0 esp32\r\n"+"\r\n";
          	String dataRecive = requestHttp(url);
            dataRecive.replace("%20"," ");
          	//printf("\n\nRecieved: %s\n\n",(char*)dataRecive.c_str());
            total_file_size = dataRecive.toInt();

            xTaskCreate(&ota_example_task, "ota_example_task", 8192, NULL, 5, NULL);
            updating = true;
          }
      }

			char *p;
			p = strtok(query_string, "&");

			while (p != NULL)
			{
			  if(p[0]=='r')
			  {
				  if(p[2]!=(char)NULL)
				  {
					  if(p[2]=='1'){
						  digitalWrite(rele,HIGH);
						  //printf("Rele: HIGH\n");
					  }
					  else if(p[2]=='0'){
						  digitalWrite(rele,LOW);
						  //printf("Rele: LOW\n");
					  }
				  }
			  }
        else if(p[0]=='m')
        {
            #ifdef LCD_MODULE
            if(getMenus())
            {
              if(getSubMenus())
              {

              }
            }
            #endif
        }
        #ifdef LCD_MODULE
			  else if(p[0]=='l')
			  {
  				char *c;
  				c = strtok(p, "=");
  				c = strtok(NULL, "=");

  				std::vector<char*> v;
          int g=0;
  				char* chars_array = strtok(c, ";");
  				while(chars_array)
  				{
            if(g==0)
            {
              FX1Line = String(chars_array);
              memory.putString("FX1",FX1Line);
              //printf("FX1 saved : %s\n",FX1Line.c_str());
            }
            if(g==1)
            {
              FX2Line = String(chars_array);
              memory.putString("FX2",FX2Line);
              //printf("FX2: %s\n",FX2Line.c_str());
            }
            if(g==2)
            {
              FX3Line = String(chars_array);
              memory.putString("FX3",FX3Line);
              //printf("FX3: %s\n",FX3Line.c_str());
            }
  					chars_array = strtok(NULL, ";");
            g++;
  				}
			  }
        #endif
			  else if(p[0]=='c')
			  {
				  if(getConfigsModule())
				  {
            #ifdef LCD_MODULE
					  if(getMenus())
					  {
						  if(getSubMenus())
						  {

						  }
					  }
            #endif
				  }
			  }
        else if(p[0]=='x')
			  {
          //printf("\n\nRESET DC\n\n");
				  esp_restart();
			  }
        else if(p[0]=='y')
			  {
				  resetCounters();
          //printf("\n\nRESET COUNTERS\n\n");
			  }

			  p = strtok (NULL, "&");

			}// End of tokens

			nc->flags |= MG_F_SEND_AND_CLOSE;
			free(query_string);
			free(p);
			break;

		}// End of casedisconnecting
	} // End of switch
} // End of mongoose_event_handler


// FreeRTOS task to start Mongoose.
void mongooseTask(void *data) {
	ESP_LOGD(TAG, "Mongoose task starting");
	struct mg_mgr mgr;
	ESP_LOGD(TAG, "Mongoose: Starting setup");
	mg_mgr_init(&mgr, NULL);
	ESP_LOGD(TAG, "Mongoose: Succesfully inited");
	struct mg_connection *c = mg_bind(&mgr, ":80", mongoose_event_handler);
	ESP_LOGD(TAG, "Mongoose Successfully bound");
	if (c == NULL) {
		ESP_LOGE(TAG, "No connection from the mg_bind()");
		vTaskDelete(NULL);
		return;
	}
	mg_set_protocol_http_websocket(c);

	while (1) {
		mg_mgr_poll(&mgr, 1000);
	}
} // mongooseTask

//##############################################################//


//##################### WIFI FUNCTIONS ########################//

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
		//ESP_LOGD(TAG, "Got an IP: " IPSTR, IP2STR(&event->event_info.got_ip.ip_info.ip));
		sprintf(MyAddr,"%d.%d.%d.%d",IP2STR(&event->event_info.got_ip.ip_info.ip));
		//printf("\n\nMy Ip is: %s\n\n",MyAddr);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently auto-reassociate. */
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        break;
    default:
        break;
    }
    return ESP_OK;
}

static void initialise_wifi(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );

    wifi_config_t wifi_config = {0};

    strcpy(reinterpret_cast<char*>(wifi_config.sta.ssid), network.c_str());
    strcpy(reinterpret_cast<char*>(wifi_config.sta.password), network_pass.c_str());

    //printf("Setting WiFi configuration SSID: %s\n", wifi_config.sta.ssid);
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );

	  uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    //printf("\n\n\n\n\n\nMy Mac is: %s\n\n\n\n\n",macStr);

    memset(&hints, 0, sizeof(hints));
    hints.ai_family   = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
}

static void checkOffline(void *pvParameters)
{
	while(1)
	{
		struct addrinfo *res;
		int s;

		int err = getaddrinfo(WEB_SERVER, "80", &hints, &res);
		if(err != 0 || res == NULL) {
				ONLINE = false;
				ONLINE_OLD = false;
				//printf("err: %d OFFLINE\n",err);
		}

		s = socket(res->ai_family, res->ai_socktype, 0);
		if(s < 0) {
			ESP_LOGE(TAG, "... Failed to allocate socket. OFFLINE");
			freeaddrinfo(res);
			ONLINE = false;
			ONLINE_OLD = false;
			vTaskDelay(1000 / portTICK_RATE_MS);
		}


		if(connect(s, res->ai_addr, res->ai_addrlen) != 0) {
			ESP_LOGE(TAG, "... socket connect failed errno=%d OFFLINE", errno);
			close(s);
			freeaddrinfo(res);
			ONLINE = false;
			ONLINE_OLD = false;
			vTaskDelay(1000 / portTICK_RATE_MS);
		}
		else
		{

			freeaddrinfo(res);
			close(s);

			if(!ONLINE_OLD)
			{
				//printf("ONLINE AGAIN!!\n");

				if(id_module!=0) clearOfflineData();

				ONLINE = true;
				ONLINE_OLD = true;
			}

		}

		vTaskDelay(5000 / portTICK_RATE_MS);
	}
}

void clearOfflineData()
{
	online_data_sending = true;

  bottom_offline = memory.getUInt("OB",0);
  //printf("Readed OB = %d\n",bottom_offline);

	for(int i = bottom_offline+1;i<offline_counter;i++)
	{
		//printf("Sending saved data %d of %d\n",i,offline_counter);

    String offString = memory.getString(String(i).c_str(),"-,-,-,-,-,-,-,-,-,-,-,-,-,-");
    //printf("LIDO OFFLINE STRING %d = %s\n",i,offString.c_str());

		std::vector<char*> v;
		char* chars_array = strtok((char*)offString.c_str(), ",");
		while(chars_array)
		{
			v.push_back(chars_array);
			chars_array = strtok(NULL, ",");
		}

		String url = "GET http://"+ipSend+"/event_fluxotec.php";
		for(size_t n = 0; n < v.size(); ++n)
		{
  			switch(n)
  			{
  				case 0:
  				url += "?id_module=";
  				url += String(v[n]);
  				break;
  				case 1:
  				url += "&type=";
  				url += String(v[n]);
  				break;
  				case 2:
  				url += "&relay=";
  				url += String(v[n]);
  				break;
  				case 3:
  				url += "&io1=";
  				url += String(v[n]);
  				break;
  				case 4:
  				url += "&io2=";
  				url += String(v[n]);
  				break;
  				case 5:
  				url += "&io3=";
  				url += String(v[n]);
  				break;
  				case 6:
  				url += "&usb1=";
  				url += String(v[n]);
  				break;
  				case 7:
  				url += "&io4=";
  				url += String(v[n]);
  				break;
  				case 8:
  				url += "&io5=";
  				url += String(v[n]);
  				break;
  				case 9:
  				url += "&io6=";
  				url += String(v[n]);
  				break;
  				case 10:
  				url += "&io7=";
  				url += String(v[n]);
  				break;
  				case 11:
  				url += "&io8=";
  				url += String(v[n]);
  				break;
  				case 12:
  				url += "&timeStamp=";
  				url += String(v[n]);
  				break;

  			}
			}
			//printf("\n");

			url += " HTTP/1.0\r\nHost: ";
      url += ipSend;
      url += "\r\nUser-Agent: esp-idf/1.0 esp32\r\n\r\n";

			//printf("REQUEST: %s\n",url.c_str());

			 requestHttpNoRead(url);
			//if(!sucess); //printf("\n\nERROR ON sending saved data\n\n");

      memory.remove(String(i).c_str());
			//printf("Erasing %d from sending saved data!\n",i);

      memory.putUInt("OB",i);
      //printf("Saved Offline Bottom: %d\n",i);

		offline_counter_now = i;

		//printf("Sucess send data %d\n",i);

		vTaskDelay(200 / portTICK_RATE_MS);
	}

	online_data_sending = false;
	offline_counter_now = 0;
	bottom_offline = 0;
	offline_counter = 0;

	//printf("\n\n\n\nAll send to server\n\n\n\n");

  memory.putUInt("OC",0);
  //printf("Saving offline counter 0\n");

  memory.putUInt("OB",0);
  //printf("Saving bottom_offline 0\n");

}

///////////////////////////////////CLIENT/////////////////////////////////////////////
static void clientTask(void *pvParameters)
{

  #ifdef DEBUG_CLIENT_TIME
	t1 = clock();
  #endif
	while(1)
	{
		if(ONLINE)
		{

			if(!readMenuConfigs)
			{
        bool sucess_n = true;
				if(getConfigsModule())
				{

          #ifdef LCD_MODULE
					if(getMenus() && getSubMenus()) sucess_n = true;
          else sucess_n = false;
          #endif

          if(sucess_n)
					{
						save_my_ip();
						ONLINE = true;
						//printf("All configs Readed!\n");
						readMenuConfigs = true;
					}
				}
        else
				{
					//printf("###MODULE CAN NOT GET ID####\n");
				}
			}


			if(id_module!=0)
			{
				String url = "GET http://"+ipSend+"/event_fluxotec.php";
				url += "?id_module=";
				url += id_module;
				url += "&type=";
				url += counter_event;
				url += "&relay=";
				url += digitalRead(rele);
        for(int i=0;i<MAX_INPUTS;i++)
        {
          url += "&io"+String(i+1)+"=";
          if(INPUT_FUNCS[i]!=0) url += COUNTERS[i];
  				else url += digitalRead(INPUT_PINS[i]);
        }
        url += "&usb1=dataUSB";
        url += "&temp=";
				url += String(temperature);
				url += "&hum=";
				url += String(humidity);
				url += "&timeStamp=";
				url += millis();
				url += " HTTP/1.0\r\nHost: ";
        url += ipSend;
        url += "\r\nUser-Agent: esp-idf/1.0 esp32\r\n\r\n";

				 requestHttpNoRead(url);
				//if(!sucess); ////printf("\n\nERROR ON CLIENT_TASK\n\n");

        #ifdef DEBUG_CLIENT_TIME

        t2 = clock();

				int TimeStamp = ((float)(t2 - t1) / CLOCKS_PER_SEC ) * 1000;

				//printf("Client Request: %d TimeStamp: %d\n",countRequest,TimeStamp);

				t1 = clock();

        #endif

				countRequest++;
			}
			else
			{
				//printf("Module do not have ID\n");
			}
		}
		else
		{
			//printf("MAC: %s to network %s to ip %s\n",macStr,network.c_str(),ipSend.c_str());
			//printf("Offline!\n");
		}

		vTaskDelay(timeoutRequest / portTICK_RATE_MS);
	}

}

bool requestHttpNoRead(String urlSend)
{
	struct addrinfo *res;
    int s;

	xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
						false, true, portMAX_DELAY);

	int err = getaddrinfo(WEB_SERVER, "80", &hints, &res);

	if(err != 0 || res == NULL) {
		ESP_LOGE(TAG, "DNS lookup failed err=%d res=%p", err, res);
		vTaskDelay(1000 / portTICK_RATE_MS);
		return false;
	}


	s = socket(res->ai_family, res->ai_socktype, 0);
	if(s < 0) {
		ESP_LOGE(TAG, "... Failed to allocate socket.");
		freeaddrinfo(res);
		vTaskDelay(1000 / portTICK_RATE_MS);
		return false;
	}


	if(connect(s, res->ai_addr, res->ai_addrlen) != 0) {
		ESP_LOGE(TAG, "... socket connect failed errno=%d", errno);
		close(s);
		freeaddrinfo(res);
		vTaskDelay(1000 / portTICK_RATE_MS);
		return false;
	}


	freeaddrinfo(res);

	if (write(s, urlSend.c_str(), strlen(urlSend.c_str())) < 0) {
		ESP_LOGE(TAG, "... socket send failed");
		close(s);
		vTaskDelay(1000 / portTICK_RATE_MS);
		return false;
	}

	close(s);

	return true;
}

String requestHttp(String urlSend)
{
	struct addrinfo *res;
  int s;
	int r;
  xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
                      false, true, portMAX_DELAY);
  int err = getaddrinfo(WEB_SERVER, "80", &hints, &res);

  if(err != 0 || res == NULL) {
      ESP_LOGE(TAG, "DNS lookup failed err=%d res=%p", err, res);
      vTaskDelay(1000 / portTICK_RATE_MS);
      return "";
  }

  s = socket(res->ai_family, res->ai_socktype, 0);
  if(s < 0) {
      ESP_LOGE(TAG, "... Failed to allocate socket.");
      freeaddrinfo(res);
      vTaskDelay(1000 / portTICK_RATE_MS);
      return "";
  }
  //ESP_LOGI(TAG, "... allocated socket\r\n");

  if(connect(s, res->ai_addr, res->ai_addrlen) != 0) {
      ESP_LOGE(TAG, "... socket connect failed errno=%d", errno);
      close(s);
      freeaddrinfo(res);
      vTaskDelay(4000 / portTICK_RATE_MS);
      return "";
  }

  //ESP_LOGI(TAG, "... connected");
  freeaddrinfo(res);

  if (write(s, urlSend.c_str(), strlen(urlSend.c_str())) < 0) {
      ESP_LOGE(TAG, "... socket send failed");
      close(s);
      vTaskDelay(4000 / portTICK_RATE_MS);
      return "";
  }
  //ESP_LOGI(TAG, "... socket send success");

 String dataRecieve = "";
 char recv_buf[64];
  do
  {
     bzero(recv_buf, sizeof(recv_buf));
     r = read(s, recv_buf, sizeof(recv_buf)-1);
     for(int i = 0; i < r; i++)
     {
         dataRecieve += recv_buf[i];

     }
  }  while(r > 0);


	close(s);

  dataRecieve = dataRecieve.substring(dataRecieve.indexOf('{')+1,dataRecieve.indexOf('}'));

	return dataRecieve;
}

bool getConfigsModule()
{

  String url = "GET http://"+ipSend+"/get_configs_fluxotec.php?mac="+macStr+" HTTP/1.0\r\n"+"Host: "+ipSend+"\r\n"+"User-Agent: esp-idf/1.0 esp32\r\n"+"\r\n";

	String dataRecive = requestHttp(url);
  dataRecive.replace("%20"," ");

	//printf("\n\nRecieved: %s\n\n",(char*)dataRecive.c_str());

	std::vector<char*> v;
  char* chars_array = strtok((char*)dataRecive.c_str(), ",");
  while(chars_array)
  {
      v.push_back(chars_array);
      chars_array = strtok(NULL, ",");
  }

	//printf("\n\n\n\nDATA SIZE %d\n\n\n\n\n",v.size());

	if(v.size()>=26)
	{
    id_module = atoi(v[0]);
    memory.putUInt("ID",id_module);
    //printf("Saved ID: %d\n",id_module);

    for(int i=0;i<MAX_INPUTS;i++)
    {
      String FName = "F"+String(i+1);
      INPUT_FUNCS[i] = atoi(v[i+1]);
      memory.putUInt(FName.c_str(),INPUT_FUNCS[i]);
      //printf("Saved %s: %d\n",FName.c_str(),INPUT_FUNCS[i]);
    }

    for(int i=0;i<MAX_INPUTS;i++)
    {
      String IName = "I"+String(i+1);
      INPUT_CHANGE_TIME[i] = atoi(v[i+9]);
      memory.putUInt(IName.c_str(),INPUT_CHANGE_TIME[i]);
      //printf("Saved %s: %d\n",IName.c_str(),INPUT_CHANGE_TIME[i]);
    }

    counter_event = String(v[17]);
    memory.putString("CE",counter_event);
    //printf("Saved CE: %s\n",counter_event.c_str());
    io_event = String(v[18]);
    memory.putString("IE",io_event);
    //printf("Saved IE: %s\n",io_event.c_str());
    mn_event = String(v[19]);
    memory.putString("MN",mn_event);
    //printf("Saved MN: %s\n",mn_event.c_str());

    timeoutRequest = atoi(v[20]);
    memory.putUInt("TR",timeoutRequest);
    //printf("Saved TR: %d\n",timeoutRequest);
    offline_saving = atoi(v[21]);
    memory.putUInt("OS",offline_saving);
    //printf("Saved OS: %d\n",offline_saving);

    sscanf(v[22],"%5f",&factor);
    memory.putFloat("FC",factor);
    //printf("Saved FC: %5f\n",factor);

    network = String(v[23]);
    memory.putString("NT",network);
    //printf("Saved NT: %s\n",network.c_str());

    network_pass = String(v[24]);
    memory.putString("NTP",network_pass);
    //printf("Saved NTP: %s\n",network_pass.c_str());

    hardware_version = String(v[25]);
    memory.putString("HV",hardware_version);
    printf("Saved hardware_version: %s\n",hardware_version.c_str());

    //Reconfigure PCNT functions
    for(int i=0;i<MAX_INPUTS;i++) configure_encoder_type(i);

    printf("End getConfigsOnline\n");
	}
	else
	{
		//printf("Error data not correct!\n");
    printf("Module not registered\n");
    FX1Line = "Module not registered";
    FX2Line = "Rebooting...";
    delay(2000);
    esp_restart();
	}

	return true;
}

bool getMenus()
{
  #ifdef LCD_MODULE
  String url = "GET http://"+ipSend+"/get_menus.php?mac="+macStr+" HTTP/1.0\r\n"+"Host: "+ipSend+"\r\n"+"User-Agent: esp-idf/1.0 esp32\r\n"+"\r\n";

	String dataRecive = requestHttp(url);
	dataRecive.replace("%20"," ");

  //printf("DATA RECIEVE MENUS: %s\n",dataRecive.c_str());

	if(dataRecive!="") maxMenu = countOcurenc(dataRecive,',');
	else maxMenu = 0;

  memory.putUInt("maxMenu",maxMenu);
  //printf("Saved maxMenu %d\n",maxMenu);

  if(maxMenu>0)
  {
  	//printf("MaxMenu: %d\n",maxMenu+1);

  	for(int i=0;i<maxMenu+1;i++)
  	{
  		String allDivide = getValue(dataRecive,',',i);

      String Mname = "M"+String(i);
      memory.putString(Mname.c_str(),allDivide);
      //printf("Saved %s = %s\n",Mname.c_str(),allDivide.c_str());

  		String menuID = allDivide.substring(0,allDivide.indexOf('['));
  		String menuName = allDivide.substring(allDivide.indexOf('[')+1,allDivide.indexOf(']'));
  		String confirms = allDivide.substring(allDivide.indexOf(']')+1,allDivide.indexOf('@'));
  		String input_data = allDivide.substring(allDivide.indexOf('@')+1,allDivide.indexOf('~'));
      String reset_count = allDivide.substring(allDivide.indexOf('~')+1,allDivide.indexOf('^'));
  		String as_sub_menus = allDivide.substring(allDivide.indexOf('^')+1,allDivide.length());

  		menu_id[i] = menuID.toInt();
  		menuName.toCharArray(menu[i], menuName.length()+1);
  		confirm_menu[i] = confirms.toInt();
  		in_menu[i] = input_data.toInt();
      reset_count_menu[i] = reset_count.toInt();
  		as_sub_menu[i] = as_sub_menus.toInt();
  		//printf("MenuID %d MenuName %s ConfirmMenu %d InputMenu %d ResetMenu %d as_sub_menu: %d\n",menu_id[i],menu[i],confirm_menu[i],in_menu[i],reset_count_menu[i],as_sub_menu[i]);
  	}
  }

  #endif
	return true;
}

bool getMenusOFF()
{
  #ifdef LCD_MODULE

	maxMenu = memory.getUInt("maxMenu",0);
	//printf("Read maxMenu = %d\n",maxMenu);

 if(maxMenu>0)
 {
  	//printf("MaxMenu OFF: %d\n",maxMenu+1);

  	int cntN = 0;

  	for(int i=0;i<maxMenu+1;i++)
  	{
  		String key = "M"+String(i);
  		String allDivide = memory.getString(key.c_str(),"1[1-FAIL]0@0~0^0");
      //printf("Read menu %d = %s\n",i,allDivide.c_str());

  		String menuID = allDivide.substring(0,allDivide.indexOf('['));
  		String menuName = allDivide.substring(allDivide.indexOf('[')+1,allDivide.indexOf(']'));
  		String confirms = allDivide.substring(allDivide.indexOf(']')+1,allDivide.indexOf('@'));
  		String input_data = allDivide.substring(allDivide.indexOf('@')+1,allDivide.indexOf('~'));
      String reset_count = allDivide.substring(allDivide.indexOf('~')+1,allDivide.indexOf('^'));
  		String as_sub_menus = allDivide.substring(allDivide.indexOf('^')+1,allDivide.length());

  		menu_id[i] = menuID.toInt();
  		menuName.toCharArray(menu[i], menuName.length()+1);
  		confirm_menu[i] = confirms.toInt();
  		in_menu[i] = input_data.toInt();
      reset_count_menu[i] = reset_count.toInt();
  		as_sub_menu[i] = as_sub_menus.toInt();
  		//printf("MenuID %d MenuName %s ConfirmMenu %d InputMenu %d ResetMenu %d as_sub_menu: %d\n",menu_id[i],menu[i],reset_count_menu[i],confirm_menu[i],in_menu[i],as_sub_menu[i]);

  		cntN++;
  	}

  }
  #endif
	return true;
}

bool getSubMenus()
{
  #ifdef LCD_MODULE
  String url = "GET http://"+ipSend+"/get_sub_menus.php?mac="+macStr+" HTTP/1.0\r\n"+"Host: "+ipSend+"\r\n"+"User-Agent: esp-idf/1.0 esp32\r\n"+"\r\n";

	String dataRecive = requestHttp(url);
	dataRecive.replace("%20"," ");

  //printf("DATA RECIEVED SUBMENUS: %s\n",dataRecive.c_str());

	if(dataRecive!="") maxSubMenu = countOcurenc(dataRecive,',');
	else maxSubMenu = 0;

  memory.putUInt("maxSubMenu",maxSubMenu);
  //printf("Saved maxSubMenu %d\n",maxSubMenu);

	 if(dataRecive!="")
	 {
	  	for(int i=0;i<maxSubMenu+1;i++)
	  	{
	  		String allDivide = getValue(dataRecive,',',i);
        String Sname = "S"+String(i);
        memory.putString(Sname.c_str(),allDivide);
        //printf("Saved %s = %s\n",Sname.c_str(),allDivide.c_str());

	  		String sub_menuID = allDivide.substring(0,allDivide.indexOf("º"));
	  		String sub_sub_menuID = allDivide.substring(allDivide.indexOf("º")+1,allDivide.indexOf("["));
	  		String sub_menuName = allDivide.substring(allDivide.indexOf("[")+1,allDivide.indexOf("]"));
	  		String sub_confirms = allDivide.substring(allDivide.indexOf("]")+1,allDivide.indexOf("@"));
	      String sub_input_data = allDivide.substring(allDivide.indexOf("@")+1,allDivide.indexOf("§"));
	  		String sub_reset = allDivide.substring(allDivide.indexOf("§")+2,allDivide.length());

	  		sub_menu_id[i] = sub_menuID.toInt();
	  		sub_sub_menu_id[i] = sub_sub_menuID.toInt();
	  		sub_menuName.toCharArray(sub_menu[i], sub_menuName.length()+1);
	  		sub_confirm_menu[i] = sub_confirms.toInt();
	  		sub_in_menu[i] = sub_input_data.toInt();
        sub_reset_menu[i] = sub_reset.toInt();

	  		//printf("SubMenuID %d  SubSubMenuID %dSubMenuName %s SubConfirmMenu %d SubInputMenu %d sub_reset_menu %d\n",sub_menu_id[i],sub_sub_menu_id[i],sub_menu[i],sub_confirm_menu[i],sub_in_menu[i],sub_reset_menu[i]);
	  	}
	  }
  #endif
	return true;
}

bool getSubMenusOFF()
{
  #ifdef LCD_MODULE

  maxSubMenu = memory.getUInt("maxSubMenu",0);
	//printf("Read maxSubMenu = %d\n",maxSubMenu);

  if(maxSubMenu>0)
  {

  	for(int i=0;i<maxSubMenu+1;i++)
  	{
      String key = "S"+String(i);
  		String allDivide = memory.getString(key.c_str(),"1º1[FAIL]0@0§0");
      //printf("Read Submenu %d = %s\n",i,allDivide.c_str());

  		String sub_menuID = allDivide.substring(0,allDivide.indexOf("º"));
  		String sub_sub_menuID = allDivide.substring(allDivide.indexOf("º")+1,allDivide.indexOf("["));
  		String sub_menuName = allDivide.substring(allDivide.indexOf("[")+1,allDivide.indexOf("]"));
  		String sub_confirms = allDivide.substring(allDivide.indexOf("]")+1,allDivide.indexOf("@"));
      String sub_input_data = allDivide.substring(allDivide.indexOf("@")+1,allDivide.indexOf("§"));
  		String sub_reset = allDivide.substring(allDivide.indexOf("§")+1,allDivide.length());

  		sub_menu_id[i] = sub_menuID.toInt();
  		sub_sub_menu_id[i] = sub_sub_menuID.toInt();
  		sub_menuName.toCharArray(sub_menu[i], sub_menuName.length()+1);
  		sub_confirm_menu[i] = sub_confirms.toInt();
  		sub_in_menu[i] = sub_input_data.toInt();
      sub_reset_menu[i] = sub_reset.toInt();

  		//printf("SubMenuID %d  SubSubMenuID %dSubMenuName %s SubConfirmMenu %d SubInputMenu %d\n",sub_menu_id[i],sub_sub_menu_id[i],sub_menu[i],sub_confirm_menu[i],sub_in_menu[i]);

  	}
  }
  #endif
	return true;
}

void save_my_ip()
{
	String url = "GET http://"+ipSend+"/save_my_ip.php";
	url += "?id_module=";
	url += id_module;
	url += "&ip=";
	url += String(MyAddr);
  url += "&version=";
	url += version;
	url += " HTTP/1.0\r\nHost: ";
  url += ipSend;
  url += "\r\nUser-Agent: esp-idf/1.0 esp32\r\n\r\n";

	 requestHttpNoRead(url.c_str());
	//if(!sucess); //printf("\n\nERROR ON save_my_ip\n\n");
}

void sendInterrupt(int pino)
{
	if(ONLINE)
	{
			String url = "GET http://"+ipSend+"/event_fluxotec.php";
			url += "?id_module=";
			url += id_module;
			url += "&type=";
			url += io_event;
			url += pino;
			url += "&relay=";
			url += digitalRead(rele);
      for(int i=0;i<MAX_INPUTS;i++)
      {
        url += "&io"+String(i+1)+"=";
        if(INPUT_FUNCS[i]!=0) url += COUNTERS[i];
        else url += digitalRead(INPUT_PINS[i]);
      }
      url += "&usb1=dataUSB";
			url += "&timeStamp=";
			url += millis();
			url += " HTTP/1.0\r\nHost: ";
      url += ipSend;
      url += "\r\nUser-Agent: esp-idf/1.0 esp32\r\n\r\n";

			 requestHttpNoRead(url);
			//if(!sucess); //printf("\n\nERROR ON sendInterrupt\n\n");

	}
	else
	{
		saveOfflineEvent(1,String(pino));
	}

}

void sendOverFlow(int pin)
{
	if(ONLINE)
	{
			String url = "GET http://"+ipSend+"/event_fluxotec.php";
			url += "?id_module=";
			url += id_module;
			url += "&type=";
			url += counter_event;
			url += "&relay=";
			url += digitalRead(rele);
      for(int i=0;i<MAX_INPUTS;i++)
      {
        url += "&io"+String(i+1)+"=";
        if(INPUT_FUNCS[i]!=0 && pin!=(i+1)) url += COUNTERS[i];
        else if(pin==1 && INPUT_FUNCS[i]!=0) url += "0";
        else url += digitalRead(INPUT_PINS[i]);
      }
      url += "&usb1=dataUSB";
			url += "&timeStamp=";
			url += millis();
			url += " HTTP/1.0\r\nHost: ";
      url += ipSend;
      url += "\r\nUser-Agent: esp-idf/1.0 esp32\r\n\r\n";

			 requestHttpNoRead(url);
			//if(!sucess); //printf("\n\nERROR ON sendMenu\n\n");
	}
	else
	{
		saveOfflineEvent(3,String(pin));
	}
}

void sendMenu(String desc)
{
	if(ONLINE)
	{
			String url = "GET http://"+ipSend+"/event_fluxotec.php";
			url += "?id_module=";
			url += id_module;
			url += "&type=";
			url += mn_event;
			url += desc;
			url += "&relay=";
			url += digitalRead(rele);
      for(int i=0;i<MAX_INPUTS;i++)
      {
        url += "&io"+String(i+1)+"=";
        if(INPUT_FUNCS[i]!=0) url += COUNTERS[i];
        else url += digitalRead(INPUT_PINS[i]);
      }
      url += "&usb1=dataUSB";
			url += "&timeStamp=";
			url += millis();
			url += " HTTP/1.0\r\nHost: ";
      url += ipSend;
      url += "\r\nUser-Agent: esp-idf/1.0 esp32\r\n\r\n";

			bool sucess = requestHttpNoRead(url);
			if(!sucess); //printf("\n\nERROR ON sendMenu\n\n");
	}
	else
	{
		saveOfflineEvent(2,desc);
	}
}

int countOcurenc(String s,char f)
{
	int count=0;

	for (int i=0; i<s.length(); i++) if(s[i]==f) count++;

	return count;
}

String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}

//##################### END WIFI FUNCTIONS ########################//

//##################### LCD BASE FUNCTIONS ########################//

#ifdef LCD_MODULE
void drawLCD()
{
  if(!on_action && !menuSelection){
    u8g2.clearBuffer();
  }

	if(ONLINE)
	{

    String map_ip = network;
    map_ip.replace("fluxotec","");
    map_ip += "/";
    String finalIp = MyAddr;
    finalIp.replace("10.42.1.","");
    String ip_rec = "";
    if(finalIp.length()<3) ip_rec+="0";
    ip_rec += finalIp;
    map_ip += ip_rec;
		drawIP((char*)map_ip.c_str());
		drawSignal();
		drawID();
	}
	else
	{
		drawOffline();
		drawID();
		drawOfflineCounter();
	}

	if(heart) piscaOFF();
	else piscaON();

	if(!menuOn && !onSubMenu && !inputMode && !online_data_sending)
	{

		drawLineFX(1,(char*)FX1Line.c_str(),true);
		drawLineFX(2,(char*)FX2Line.c_str(),true);
		drawLineFX(3,(char*)FX3Line.c_str(),true);

		u8g2.drawLine(0, 30, 127, 30);
		drawCounter();

	}
	else if(online_data_sending)
	{
		clearMain();
		drawLineFX(1,(char*)"Sending online",true);
		char dataSe[15];
		sprintf(dataSe,"%d of %d done",offline_counter_now,offline_counter);
		drawLineFX(3,(char*)dataSe,true);
	}
	else if(inputMode)
	{
		clearMain();
		drawLineFX(1,(char*)"Input Mode: ",true);
		char dt[25];
		inputString.toCharArray(dt, inputString.length()+1);
		drawLineFX(3,dt,true);
	}
	else if(onSubMenu)
	{
		if(!menuSelection){
		clearMain();
		menu_roling2(pos_sub_menu);
		}
	}
	else
	{
		if(!menuSelection){
			clearMain();
			menu_roling(pos_menu);
		}
	}

	clearBottom();
  drawBottom();

  u8g2.sendBuffer();

}

static void watchTask(void *pvParameters)
{
	while(1)
	{
		if(millis()-watchTimeStamp>600)
		{
			//printf("STOPED!!\n");
			if( xHandle != NULL && !stopedTask)
			{
				vTaskDelete( xHandle );
				stopedTask = true;
				xTaskCreatePinnedToCore(&writeLCDTask, "writeLCDTask", 2048, NULL, 1, &xHandle,0);
			}
			watchTimeStamp = millis();
		}

		if (millis() - HeartTimeStamp>HeartLimiteTime)
    {
  	  if(heart)heart = false;
  	  else heart = true;
  	  HeartTimeStamp = millis();
    }

	  vTaskDelay(100 / portTICK_RATE_MS);
	}
}


static void writeLCDTask(void *pvParameters)
{
	while(1)
	{
	  stopedTask = false;
	  drawLCD();
	  watchTimeStamp = millis();
	  vTaskDelay(250 / portTICK_RATE_MS);
	}

	vTaskDelete(NULL);
}

void drawBottom()
{
  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.drawLine(0, 56, 127, 56);
  String bottom = "";
  if(hardware_version=="0" && !updating)
  {
    bottom += "    ";
    for(int i=0;i<MAX_INPUTS;i++)
    {
      //bottom += " ";
      if(digitalRead(INPUT_PINS[i])) bottom += String(i+1);
      else bottom += "_";
      bottom += " ";
    }
    bottom += " V"+version;
  }
  else if(!updating) bottom = "TEMP "+temperature+" HUM "+humidity+"% V"+version;
  else bottom = "      UPDATING "+String(binary_file_length*100/total_file_size)+" %";
  u8g2.drawUTF8(0,64,bottom.c_str());  // write something to the internal memory
  u8g2.setFontMode(1);
  if(!updating && hardware_version!="0") u8g2.drawCircle(46, 59, 1, U8G2_DRAW_ALL);//GRAUS

  /*
  if(hardware_version=="0" && !updating)
  {
    u8g2.setFont(u8g2_font_4x6_tr);
    String vs = "V"+version;
    u8g2.drawUTF8(0,64,vs.c_str());
  }
  */
}



void clearBottom()
{
  u8g2.setDrawColor(0);
  u8g2.drawBox(0,56,130,80);
  u8g2.setDrawColor(1);
}

void clearMain()
{
	u8g2.setDrawColor(0);
	u8g2.drawBox(0,9,128,47);//x,y,width,height
}

void piscaON()
{
  u8g2.setDrawColor(1);
  u8g2.drawDisc(3, 3, 2, U8G2_DRAW_ALL);
}

void piscaOFF()
{
  u8g2.setFontMode(0);
  u8g2.setDrawColor(0);
  u8g2.drawCircle(3, 3, 2, U8G2_DRAW_ALL);
}


void drawID()
{
  char dados[10];
  if(id_module<10)sprintf(dados,"DC000%d",id_module);
  else if(id_module<100)sprintf(dados,"DC00%d",id_module);
  else if(id_module<1000)sprintf(dados,"DC0%d",id_module);
  else sprintf(dados,"DC%d",id_module);
  u8g2.drawStr(8,6,(char*)dados);
  u8g2.drawLine(0, 7, 127, 7);
}

void drawSignal()
{
   u8g2.setDrawColor(1);

   if(signal_quality>0) u8g2.drawBox(46,5,3,1);
   if(signal_quality>1) u8g2.drawBox(50,4,3,2);
   if(signal_quality>2) u8g2.drawBox(54,2,3,4);
   if(signal_quality>3) u8g2.drawBox(58,0,3,6);
}

void drawIP(char *ip_DC)
{
  u8g2.setFont(u8g2_font_5x7_tr);
  clearTop();
  u8g2.drawStr(86,6,ip_DC);
}

void clearTop()
{
  u8g2.setDrawColor(0);
  u8g2.drawBox(0,0,127,7);
  u8g2.setDrawColor(1);
}

void drawOffline()
{
  u8g2.setFont(u8g2_font_5x7_tr);
  clearTop();
  u8g2.drawStr(47,6,"OFFLINE");
}

void drawLineFX(int line,char *txt,bool center)
{
	u8g2.setFont(u8g2_font_5x7_tr);
	int sizeOfArray = strlen(txt);
	int x = 0;
	if(center) x = (int)(((25-sizeOfArray)/2)*5)+2;
	switch(line)
	{
		case 1:
		//u8g2.setDrawColor(0);
		//u8g2.drawBox(0,8,128,7);//x,y,width,height
		u8g2.setDrawColor(1);
		u8g2.drawUTF8(x,15,txt);
		break;
		case 2:
		//u8g2.setDrawColor(0);
		//u8g2.drawBox(0,15,128,7);//x,y,width,height
		u8g2.setDrawColor(1);
		u8g2.drawUTF8(x,22,txt);
		break;
		case 3:
		//u8g2.setDrawColor(0);
		//u8g2.drawBox(0,22,128,8);//x,y,width,height
		u8g2.setDrawColor(1);
		u8g2.drawUTF8(x,29,txt);
		break;
    case 4:
		//u8g2.setDrawColor(0);
		//u8g2.drawBox(0,29,128,8);//x,y,width,height
		u8g2.setDrawColor(1);
		u8g2.drawUTF8(x,36,txt);
		break;
    case 5:
		//u8g2.setDrawColor(0);
		//u8g2.drawBox(0,36,128,8);//x,y,width,height
		u8g2.setDrawColor(1);
		u8g2.drawUTF8(x,43,txt);
		break;
    case 6:
    //u8g2.setDrawColor(0);
    //u8g2.drawBox(0,43,128,8);//x,y,width,height
    u8g2.setDrawColor(1);
    u8g2.drawUTF8(x,50,txt);
    break;

		default:
		break;
	}
}

void drawCounter()
{
	char dadosCounter[10];
    float tmp_counter = (float)COUNTERS[0]*factor;
    sprintf(dadosCounter,"%d",(int)tmp_counter);

	u8g2.setFont(u8g2_font_9x15_tr);
	int sizeOfArray = strlen(dadosCounter);
	int x = 0;
	x = (int)(((14-sizeOfArray)/2)*9);

	u8g2.setDrawColor(0);
	u8g2.drawBox(0,31,128,25);//x,y,width,height
	u8g2.setDrawColor(1);
	u8g2.drawUTF8(x,49,dadosCounter);
}

void drawOfflineCounter()
{
  u8g2.setFont(u8g2_font_5x7_tr);
  char dados[10];
  if(offline_counter<10)sprintf(dados,"      %d",offline_counter);
  else if(offline_counter<100)sprintf(dados,"     %d",offline_counter);
  else if(offline_counter<1000)sprintf(dados,"    %d",offline_counter);
  else if(offline_counter<10000)sprintf(dados,"   %d",offline_counter);
  else if(offline_counter<100000)sprintf(dados,"  %d",offline_counter);
  else if(offline_counter<1000000) sprintf(dados," %d",offline_counter);
  else sprintf(dados,"%d",offline_counter);
  u8g2.drawStr(93,6,dados);
}

void drawMenuLine(int pos,char* txt)
{

	int sizeOfArray = strlen(txt);
	int x = 0;
	x = (int)(((21-sizeOfArray)/2)*6);

	switch(pos)
	{
		case 1:
		u8g2.setFont(u8g2_font_6x12_tr);
		u8g2.setDrawColor(0);
		u8g2.drawBox(0,9,128,10);//x,y,width,height
		u8g2.setDrawColor(1);
		u8g2.drawUTF8(x,17,txt);
		u8g2.drawLine(0, 19, 127, 19);
		break;
		case 2:
		u8g2.setFont(u8g2_font_7x13_tr);
		x = (int)(((19-sizeOfArray)/2)*7-4);
		u8g2.setDrawColor(0);
		u8g2.drawLine(0, 20, 127, 20);
		u8g2.setDrawColor(1);
		u8g2.drawBox(0,21,128,11);//x,y,width,height
		u8g2.setDrawColor(0);
		u8g2.drawUTF8(x,31,txt);
		u8g2.setDrawColor(0);
		u8g2.drawLine(0, 32, 127, 32);
		u8g2.setDrawColor(1);
		u8g2.drawLine(0, 33, 127, 33);
		break;
		case 3:
		u8g2.setFont(u8g2_font_6x12_tr);
		u8g2.setDrawColor(0);
		u8g2.drawBox(0,35,128,10);//x,y,width,height
		u8g2.setDrawColor(1);
		u8g2.drawUTF8(x,42,txt);
		u8g2.drawLine(0, 44, 127, 44);
		break;
		case 4:
		u8g2.setFont(u8g2_font_6x12_tr);
		u8g2.setDrawColor(0);
		u8g2.drawBox(0,46,128,10);//x,y,width,height
		u8g2.setDrawColor(1);
		u8g2.drawUTF8(x,54,txt);
		break;
		default:
		break;
	}
}

void enterMenu()
{
	menuOn = true;
	pos_menu = 0;
	clearMain();
	drawMenuLine(1,(char*)"MENUS");
	drawMenuLine(2,menu[0]);
	drawMenuLine(3,menu[1]);
	drawMenuLine(4,menu[2]);
}

void menu_roling(int pos)
{
	if(pos==maxMenu)
	{
		drawMenuLine(1,(char*)menu[pos-1]);
		drawMenuLine(2,(char*)menu[pos]);
		drawMenuLine(3,(char*)menu[0]);
		drawMenuLine(4,(char*)menu[1]);
	}
	else if(pos==(maxMenu-1))
	{
		drawMenuLine(1,(char*)menu[pos-1]);
		drawMenuLine(2,(char*)menu[pos]);
		drawMenuLine(3,(char*)menu[pos+1]);
		drawMenuLine(4,(char*)menu[0]);
	}
	else if(pos==0)
	{
		drawMenuLine(1,(char*)menu[maxMenu]);
		drawMenuLine(2,(char*)menu[pos]);
		drawMenuLine(3,(char*)menu[pos+1]);
		drawMenuLine(4,(char*)menu[pos+2]);
	}
	else
	{
		drawMenuLine(1,(char*)menu[pos-1]);
		drawMenuLine(2,(char*)menu[pos]);
		drawMenuLine(3,(char*)menu[pos+1]);
		drawMenuLine(4,(char*)menu[pos+2]);
	}
}

void menu_roling2(int pos)
{
	if(pos==tmp_max_sub)
	{
		drawMenuLine(1,(char*)tmp_sub_menu[pos-1]);
		drawMenuLine(2,(char*)tmp_sub_menu[pos]);
		if(tmp_max_sub>=2)drawMenuLine(3,(char*)tmp_sub_menu[0]);
		if(tmp_max_sub>=3)drawMenuLine(4,(char*)tmp_sub_menu[1]);
	}

	else if(pos==(tmp_max_sub-1) && tmp_max_sub>2)
	{
		drawMenuLine(1,(char*)tmp_sub_menu[pos-1]);
		drawMenuLine(2,(char*)tmp_sub_menu[pos]);
		if(tmp_max_sub>=2)drawMenuLine(3,(char*)tmp_sub_menu[pos+1]);
		if(tmp_max_sub>=3)drawMenuLine(4,(char*)tmp_sub_menu[0]);
	}
	else if(pos==0)
	{
		drawMenuLine(1,(char*)tmp_sub_menu[tmp_max_sub]);
		drawMenuLine(2,(char*)tmp_sub_menu[pos]);
		if(tmp_max_sub>=2)drawMenuLine(3,(char*)tmp_sub_menu[pos+1]);
		if(tmp_max_sub>=3)drawMenuLine(4,(char*)tmp_sub_menu[pos+2]);
	}
	else
	{
		drawMenuLine(1,(char*)tmp_sub_menu[pos-1]);
		drawMenuLine(2,(char*)tmp_sub_menu[pos]);
		if(tmp_max_sub>=2)drawMenuLine(3,(char*)tmp_sub_menu[pos+1]);
		if(tmp_max_sub>=3)drawMenuLine(4,(char*)tmp_sub_menu[pos+2]);
	}
}


void pendingConfirmation()
{
	menuSelection = true;
	clearMain();
	char dataM[25];
	if(onSubMenu){
    if(tmp_sub_in_menu[pos_sub_menu]){
      sprintf(dataM,"Confirma %s:%s?",tmp_sub_menu[pos_sub_menu],inputString.c_str());
    }
    else sprintf(dataM,"Confirma %s?",tmp_sub_menu[pos_sub_menu]);
  }
	else {
    if(in_menu[pos_menu]) sprintf(dataM,"Confirma %s:%s?",menu[pos_menu],inputString.c_str());
    else sprintf(dataM,"Confirma %s?",menu[pos_menu]);
  }
	drawLineFX(1,(char*)dataM,true);
	drawLineFX(2,(char*)"Sim - Vermelho",true);
	drawLineFX(3,(char*)"Nao - Azul",true);
	pendingResponse = true;
}

void acceptSubMenu()
{
	menuSelection = true;
	if(tmp_sub_in_menu[pos_sub_menu]){
    char dd[100];
    String tmp_str = String(tmp_sub_menu[pos_sub_menu])+":"+inputString;
    tmp_str.toCharArray(dd, tmp_str.length()+1);
    actionSend(dd);
  }
  else actionSend((char*)tmp_sub_menu[pos_sub_menu]);
	/////////SEND SUB MENU///////////
	String desc = String(pos_menu+1);
  desc += ":";
  desc += String(pos_sub_menu+1);
	desc += ":";
	desc += String(tmp_sub_menu[pos_sub_menu]);
  desc.replace(" ","%20");
  if(tmp_sub_in_menu[pos_sub_menu])
  {
    desc += ":"+inputString;
  }
  //printf("Send sub menu: desc:%s\n",desc.c_str());
	sendMenu(desc);
  //printf("ONacceptSubMenu: tmp_sub_reset_menu[pos_sub_menu]= %d\n",tmp_sub_reset_menu[pos_sub_menu]);
  if(tmp_sub_reset_menu[pos_sub_menu]) resetCounters();
	//////////////////////////////
	onSubMenu = false;
	menuOn = false;
	menuSelection = false;
	pos_sub_menu = 0;


}

void acceptMenu()
{
	menuSelection = true;
  if(in_menu[pos_menu])
  {
    char dd[100];
    String tmp_str = String(menu[pos_menu])+":"+inputString;
    tmp_str.toCharArray(dd, tmp_str.length()+1);
    actionSend(dd);
  }
	else actionSend((char*)menu[pos_menu]);
	/////////SEND MENU///////////
	if(in_menu[pos_menu]) sendMenu(String(pos_menu+1)+":"+inputString);
  else sendMenu(String(pos_menu+1));
	//////////////////////////////
	menuOn = false;
	menuSelection = false;
	pos_menu = 0;
}

void actionSend(char *txt)
{
  on_action = true;
	clearMain();
	drawLineFX(1,txt,true);
	delay(500);
	if(ONLINE) drawLineFX(2,(char*)"Enviado!",true);
	else drawLineFX(2,(char*)"Gravado!",true);
	delay(500);
	drawLineFX(3,(char*)"Obrigado",true);
	delay(1000);
  on_action = false;
}


#endif

//#################### END LCD BASE FUNCTIONS ####################//

//####################### BUTTONS FUNCTIONS #####################//

static void touchTask(void *pvParameters)
{

    while(1)
  	{
      if(check_bar_code)
      {
        printf("BARCODE: |%s|\n",inputString.c_str());

        if(inputString.indexOf("MN:")==-1)
        {
            if(come_input_barcode)
            {
              printf("come_input_barcode %d\n",tmp_sub_confirm_menu[pos_sub_menu]);

              if(onSubMenu)
              {
                if(!tmp_sub_confirm_menu[pos_sub_menu])//Nao necessita de confirmacao enviar!
                {
                  inputMode = false;
                  menuOn = true;
                  menuSelection = true;

                  char dd[100];
                  String tmp_str = String(tmp_sub_menu[pos_sub_menu])+":"+inputString;
                  tmp_str.toCharArray(dd, tmp_str.length()+1);
                  delay(50);
                  actionSend(dd);

                  sendMenu(String(pos_menu+1)+":"+String(pos_sub_menu+1)+":"+String(tmp_sub_menu[pos_sub_menu])+":"+inputString);

                  come_input_barcode = false;
                  menuOn = false;
                  menuSelection = false;
                  onSubMenu = false;
                }
                else //Necessita de confirmacao
                {
                    inputMode = false;
                    pendingConfirmation();
                    come_input_barcode = false;
                    printf("Pending confirmation\n");
                }
              }
              else//Input On Menu
              {
                if(!confirm_menu[pos_menu])//Nao necessita de confirmacao enviar!
                {
                  inputMode = false;
                  menuOn = true;
                  menuSelection = true;

                  char dd[100];
                  String tmp_str = String(menu[pos_menu])+":"+inputString;
                  tmp_str.toCharArray(dd, tmp_str.length()+1);
                  delay(50);
                  actionSend(dd);

                  sendMenu(String(pos_menu+1)+":"+inputString);

                  come_input_barcode = false;
                  menuOn = false;
                  menuSelection = false;
                  onSubMenu = false;
                }
                else
                {
                  inputMode = false;
                  pendingConfirmation();
                  come_input_barcode = false;
                  printf("Pending confirmation\n");
                }
              }

            }
            else
            {

            }

        }
        else
        {
          //Bar code para menu


          String mnId = inputString;
          mnId.replace("MN:","");

          if(mnId.indexOf(":")!=-1)
          {
            //Então tem Sub Menu
            onSubMenu = true;

            String id_subMenu = mnId.substring(mnId.indexOf(":")+1,mnId.length());
            String conf = mnId.substring(0,mnId.indexOf(":"))+":"+mnId.substring(mnId.indexOf(":")+1,mnId.length());

            pos_menu = mnId.substring(0,mnId.indexOf(":")).toInt()-1;
            pos_sub_menu = id_subMenu.toInt()-1;

            tmp_max_sub = 0;
            for(int x=0;x<maxSubMenu+1;x++)
            {
              //printf("Checkin %d sub_menu_id: %d to %d\n",x,sub_menu_id[x],menu_id[pos_menu]);
              if(sub_menu_id[x]==menu_id[pos_menu])
              {
                strcpy(tmp_sub_menu[tmp_max_sub],sub_menu[x]);
                tmp_sub_in_menu[tmp_max_sub] = sub_in_menu[x];
                tmp_sub_confirm_menu[tmp_max_sub] = sub_confirm_menu[x];
                tmp_sub_reset_menu[tmp_max_sub] = sub_reset_menu[x];

                tmp_max_sub++;
              }
            }
            tmp_max_sub--;

            if(tmp_sub_in_menu[pos_sub_menu])
            {
              inputMode = true;
              inputString = "_____________";
              come_input_barcode = true;
              //sendMenu(conf+":"+String(tmp_sub_menu[pos_sub_menu])+"AS_IN");
            }
            else
            {
              char dd[100];
              String(tmp_sub_menu[pos_sub_menu]).toCharArray(dd, String(tmp_sub_menu[pos_sub_menu]).length()+1);

              menuOn = true;
              menuSelection = true;
              delay(50);
              actionSend(dd);

              sendMenu(conf+":"+String(tmp_sub_menu[pos_sub_menu]));

              menuOn = false;
              menuSelection = false;
              onSubMenu = false;
            }

          }
          else
          {
            //Apenas Menu

            menuOn = true;
            menuSelection = true;
            delay(50);

            pos_menu = mnId.toInt()-1;
            if(in_menu[pos_menu])
            {
              inputMode = true;
              inputString = "_____________";
              come_input_barcode = true;
            }
            else
            {
              char dd[100];
              String(menu[pos_menu]).toCharArray(dd, String(menu[pos_menu]).length()+1);
              actionSend(dd);
              sendMenu(mnId);
              menuOn = false;
              menuSelection = false;
              onSubMenu = false;
            }


          }//End menu Only

        }


        check_bar_code = false;
      }

		bt1_state = digitalRead(BT1);
		bt2_state = digitalRead(BT2);

		if(bt1_state && millis()-bt1_change_time_stamp > bt1_change_time)
		{
			//printf("BT1 Click!\n");

			actionDecider(0);

			bt1_change_time_stamp = millis();
		}

		if(bt2_state && millis()-bt2_change_time_stamp > bt2_change_time)
		{
			//printf("BT2 Click!\n");

			actionDecider(1);

			bt2_change_time_stamp = millis();
		}
		////printf("BT1: %d , BT2: %d\n",bt1_state,bt2_state);

		vTaskDelay(150 / portTICK_RATE_MS);
	}
	vTaskDelete(NULL);

}

void actionDecider(int action)
{
	// 0 -> BT1 Click
	// 1 -> BT2 Click

	switch (action)
	{
		case 0: //BT1 Click

		if(inputMode)
		{
			inputMode = false;
			menuSelection = true;

			/////////SEND SUB MENU///////////
			String desc = String(pos_menu+1);
      desc += ":";
			desc += String(pos_sub_menu+1);
			desc += ":";
			desc += String(tmp_sub_menu[pos_sub_menu]);
			desc += ":";
			desc += inputString;
      desc.replace(" ","%20");
			sendMenu(desc);
      if(tmp_sub_reset_menu[pos_sub_menu]) resetCounters();
			//////////////////////////////

			//inputString[inputPos]='\0';
			String conf = mn_event+":"+inputString;
			char dd[100];
			conf.toCharArray(dd, conf.length()+1);

			actionSend(dd);

      inputString[inputPos]='\0';
			menuOn = false;
			menuSelection = false;
			onSubMenu = false;
			pos_menu = 0;
			pos_sub_menu = 0;
			inputString = "_____________";
			inputPos = 0;
		}
		else if(onSubMenu && !pendingResponse)
		{
			if(tmp_sub_in_menu[pos_sub_menu]) inputMode = true;
			else if(tmp_sub_confirm_menu[pos_sub_menu]) pendingConfirmation();
			else acceptSubMenu();
		}
		else if(menuOn && !pendingResponse)
		{
      if(reset_count_menu[pos_menu]){
        //printf("\n\n\n\nRESETING COUNTERS!!\n\n\n\n");
        resetCounters();
      }
			if(in_menu[pos_menu]) inputMode = true;
			else if(as_sub_menu[pos_menu])
			{
				pos_sub_menu = 0;
				tmp_max_sub = 0;
				for(int x=0;x<maxSubMenu+1;x++)
				{
					//printf("Checkin %d sub_menu_id: %d to %d\n",x,sub_menu_id[x],menu_id[pos_menu]);
					if(sub_menu_id[x]==menu_id[pos_menu])
					{
						strcpy(tmp_sub_menu[tmp_max_sub],sub_menu[x]);
						tmp_sub_in_menu[tmp_max_sub] = sub_in_menu[x];
						tmp_sub_confirm_menu[tmp_max_sub] = sub_confirm_menu[x];
						tmp_sub_reset_menu[tmp_max_sub] = sub_reset_menu[x];

						tmp_max_sub++;
						//printf("Found Sub Menu: %s\n",sub_menu[x]);
						onSubMenu = true;
					}
				}
				tmp_max_sub--;

			}
			else
			{
				if(confirm_menu[pos_menu]) pendingConfirmation();
				else {

					acceptMenu();
				}
			}

		}
		else if(pendingResponse) // Está pendente de resposta e foi afirmativa
		{
			if(onSubMenu) acceptSubMenu();
			else if(menuOn) {

				acceptMenu();
			}
			pendingResponse = false;
		}
		else
		{
			//printf("None Action\n");
		}
		//Se não está dentro de um Sub Menu ou um Menu o botão 1 não faz nada!

		break;
		case 1://BT2 Click

		if(inputMode)
		{
			menuOn = false;
			menuSelection = false;
			inputMode = false;
		}
		else if(!menuOn && !onSubMenu){
      //printf("MAX MENU: %d\n",maxMenu);
      if(maxMenu>0) enterMenu();
    }
		else if(onSubMenu && !pendingResponse) //Já está no SubMenu scroll
		{
			if(pos_sub_menu==tmp_max_sub) pos_sub_menu = -1;
			pos_sub_menu++;
			//printf("Pos_sub_menu: %d\n",pos_sub_menu);
		}
		else if(!pendingResponse) //Já está no menu scroll
		{
			if(pos_menu==maxMenu) pos_menu = -1;
			pos_menu++;
			//printf("PosMenu: %d\n",pos_menu);
		}
		else if(pendingResponse) // Está pendente de resposta e foi negativa
		{
			menuOn = false;
			onSubMenu = false;
			menuSelection = false;
			pendingResponse = false;
		}
		break;
	}
}

static void keypadTask(void *pvParameters)
{
    while(1)
	{
		char customKey = customKeypad.getKey();

		if (customKey){
      last_click = String(customKey);
			switch (customKey)
			{
				case '1':
				keyClick(1,'1');
				break;
				case '2':
				keyClick(2,'2');
				break;
				case '3':
				keyClick(3,'3');
				break;
				case '4':
				keyClick(4,'4');
				break;
				case '5':
				keyClick(5,'5');
				break;
				case '6':
				keyClick(6,'6');
				break;
				case '7':
				keyClick(7,'7');
				break;
				case '8':
				keyClick(8,'8');
				break;
				case '9':
				keyClick(9,'9');
				break;
				case '0':
				if(inputMode)
				{
					inputString[inputPos] = '0';
					inputPos++;
				}
				break;
				case '*':

				break;
				case '#':

				break;

				case 'S':

				onSubMenu = false;
				menuOn = false;
				inputMode = false;

				break;

				case 'L':
				if(inputMode)
				{
					inputString[inputPos-1] = '_';
					if(inputPos>0)inputPos--;
				}
				else
				{
					if(onSubMenu) onSubMenu = false;
					else if(menuOn) menuOn = false;
				}
				break;

				case 'R':
				actionDecider(0);
				break;

				case 'B':

				if(!menuOn);
				else //Já está no menu scroll
				{
					if(!onSubMenu)
					{
						if(pos_menu==maxMenu) pos_menu = -1;
						pos_menu++;
						menu_roling(pos_menu);
						//printf("PosMenu: %d\n",pos_menu);
					}
					else
					{
						if(pos_sub_menu==tmp_max_sub) pos_sub_menu = -1;
						pos_sub_menu++;
						menu_roling2(pos_sub_menu);
						//printf("Pos_sub_menu: %d\n",pos_sub_menu);
					}

				}

				break;

				case 'C':

				if(!menuOn);
				else //Já está no menu scroll
				{
					if(!onSubMenu)
					{
						if(pos_menu==0) pos_menu = maxMenu+1;
						pos_menu--;
						menu_roling(pos_menu);
						//printf("PosMenu: %d\n",pos_menu);
					}
					else
					{
						if(pos_sub_menu==0) pos_sub_menu = tmp_max_sub+1;
						pos_sub_menu--;
						menu_roling2(pos_sub_menu);
						//printf("Pos_sub_menu: %d\n",pos_sub_menu);
					}

				}

				break;

				case 'E':
				actionDecider(0);
				break;

				default:

				break;
			}
			//printf("Pressed: %c\n",customKey);
		}

		vTaskDelay(100 / portTICK_RATE_MS);
	}
}

void keyClick(int num,char c)
{
	if(inputMode)
	{
		inputString[inputPos] = c;
		inputPos++;
	}
	else if(onSubMenu)
	{
		if(tmp_max_sub>(num-1))
		{
			pos_sub_menu = (num-1);
			actionDecider(0);
		}
	}
	else if(menuOn)
	{
		if(maxMenu>(num-1))
		{
			pos_menu = (num-1);
			actionDecider(0);
		}
	}
}

static void keypadTask2(void *pvParameters)
{
    while(1)
	{
		char customKey = customKeypad.getKey();

		if (customKey){
      last_click = String(customKey);
			switch (customKey)
			{
				case '1':
        network_name_su[pos_net] = '1';
        pos_net = 11;
				break;
				case '2':
        network_name_su[pos_net] = '2';
        pos_net = 11;
				break;
				case '3':
        network_name_su[pos_net] = '3';
        pos_net = 11;
				break;
				case '4':
        network_name_su[pos_net] = '4';
        pos_net = 11;
				break;
				case '5':
        network_name_su[pos_net] = '5';
        pos_net = 11;
				break;
				case '6':
        network_name_su[pos_net] = '6';
        pos_net = 11;
				break;
				case '7':
        network_name_su[pos_net] = '7';
        pos_net = 11;
				break;
				case '8':
        network_name_su[pos_net] = '8';
        pos_net = 11;
				break;
				case '9':
        network_name_su[pos_net] = '9';
        pos_net = 11;
				break;
				case '0':
        network_name_su[pos_net] = '0';
        pos_net = 11;
				break;
				case '*':

				break;
				case '#':

				break;

				case 'S':
        net_show = false;
				break;

				case 'L':
        pos_net = 10;
				break;

				case 'R':
				break;

				case 'B':

				break;

				case 'C':

				break;

				case 'E':
        if(net_show)
        {
          if(network_name_su == "Saved on memory")
          {
              network_name_su = network;
          }
          else
          {
            memory.putString("NT",network_name_su);
            //printf("Saved NT: %s\n",network_name_su.c_str());
            network = network_name_su;
            network_name_su = "Saved on memory";
          }
        }
        else net_show = true;
				break;

				default:

				break;
			}
			//printf("Pressed: %c\n",customKey);
		}

		vTaskDelay(100 / portTICK_RATE_MS);
	}
}

//###################### END BUTTONS FUNCIONS #################//

//######################## TEST MODE ##############################//

//readInputs function by polling
static void readInputsTestMenu(void *pvParameters)
{
	while(1)
	{
    //read all input state
    for(int i=0;i<MAX_INPUTS;i++) INPUT_STATES[i] = digitalRead(INPUT_PINS[i]);

    if (millis() - HeartTimeStamp > HeartLimiteTime)
    {
  	  if(heart)heart = false;
  	  else heart = true;
  	  digitalWrite(rele,heart);
  	  HeartTimeStamp = millis();
    }

	  vTaskDelay(200 / portTICK_RATE_MS);
	}

  vTaskDelete(NULL);
}

static void writeLCDTaskTestMode(void *pvParameters)
{
	while(1)
	{
    clearMain();

  	if(heart) piscaOFF();
  	else piscaON();

    drawLineFX(1,(char*)"Modo de testes:",true);
    String macn = "MAC: "+String(macStr);
    drawLineFX(2,(char*)macn.c_str(),true);

    String onIOS = "";
    for(int i=0;i<MAX_INPUTS;i++)
    {
      if(INPUT_STATES[i]) onIOS += String(i+1);
      else onIOS += " ";
      onIOS += " ";
    }
    onIOS += " > "+last_click;

  	drawLineFX(3,(char*)onIOS.c_str(),true);

    String actual = "SSID: "+network;
    drawLineFX(4,(char*)actual.c_str(),true);
    if(net_show) drawLineFX(5,(char*)network_name_su.c_str(),true);

    float h = dht.readHumidity();
    // Read temperature as Celsius (the default)
    float t = dht.readTemperature();

    // Check if any reads failed and exit early (to try again).
    if (isnan(h) || isnan(t))
    {

      //printf("Failed to read from DHT sensor mac: %s!\n",macStr);
    }
    else
    {
      humidity = String(h,1);
      temperature = String(t,1);
    }

    String nowt = "HUM: "+humidity+" % TEMP:"+temperature +" ª";
    drawLineFX(6,(char*)nowt.c_str(),true);

  	u8g2.sendBuffer();

    vTaskDelay(150 / portTICK_RATE_MS);
  }

  vTaskDelete(NULL);
}

//###################### END TEST MODE #########################//


//########################### MEMORY ##########################//

void read_offline_config()
{
  id_module = memory.getUInt("ID",0);
  //printf("Read ID = %d\n",id_module);
  network = memory.getString("NT","fluxotecDS01");
  //printf("Read NT = %s\n",network.c_str());
  network_pass = memory.getString("NTP","fluxotec01");
  //printf("Read NTP = %s\n",network_pass.c_str());
  hardware_version = memory.getString("HV","0");
  //printf("Read hardware_version = %s\n",hardware_version.c_str());
  counter_event = memory.getString("CE","CE:");
  //printf("Read CE = %s\n",counter_event.c_str());
  io_event = memory.getString("IE","IE:");
  //printf("Read IE = %s\n",io_event.c_str());
  mn_event = memory.getString("MN","MN:");
  //printf("Read MN = %s\n",mn_event.c_str());
  timeoutRequest = memory.getUInt("TR",1000);
  //printf("Read TR = %d\n",timeoutRequest);
  offline_saving = memory.getUInt("OS",10);
  //printf("Read OS = %d\n",offline_saving);
  factor = memory.getFloat("FC",1.00000);
  //printf("Read FC = %5f\n",factor);
  offline_counter = memory.getUInt("OC",0);
  //printf("Read OC = %d\n",offline_counter);
  bottom_offline = memory.getUInt("OB",0);
  //printf("Read OB = %d\n",bottom_offline);

  //Read INPUT_FUNCS and define PCNT MODE
  for(int i=0;i<MAX_INPUTS;i++)
  {
    String FName = "F"+String(i+1);
    INPUT_FUNCS[i] = memory.getUInt(FName.c_str(),1);
    //printf("Read %s = %d\n",FName.c_str(),INPUT_FUNCS[i]);

    configure_encoder_type(i);

    String IName = "I"+String(i+1);
    INPUT_CHANGE_TIME[i] = memory.getUInt(IName.c_str(),MAX_COUNTER_VALUE);
    //printf("Read %s = %d\n",IName.c_str(),INPUT_CHANGE_TIME[i]);

  }//End for


  FX1Line = memory.getString("FX1","Welcome");
  //printf("Read FX1 = %s\n",FX1Line.c_str());
  FX2Line = memory.getString("FX2","To");
  //printf("Read FX2 = %s\n",FX2Line.c_str());
  FX3Line = memory.getString("FX3","Fluxodata");
  //printf("Read FX3 = %s\n",FX3Line.c_str());

}

void saveOfflineEvent(int typeOf,String desc)
{
		String offlineString = String(id_module)+",";

		if(typeOf==1) offlineString += io_event;
		else if(typeOf==2) offlineString += mn_event;
		else if(typeOf==3) offlineString += counter_event;

		offlineString += desc+","+String(digitalRead(rele))+",";

    for(int i=0;i<MAX_INPUTS;i++)
    {
      if(INPUT_FUNCS[i]!=0) offlineString += String(COUNTERS[i]);
  		else offlineString += String(digitalRead(INPUT_PINS[i]));
  		offlineString += ",";
      if(i==2) offlineString += "dataUSB,";
    }

		offlineString += ","+String(millis());

    if(offline_counter>=LIMIT_OFFLINE_SAVE)
    {
      memory.remove(String(bottom_offline).c_str());
      //printf("Removed %d\n",bottom_offline);
      bottom_offline++;

      memory.putUInt("OB",bottom_offline);
      //printf("Saved OB: %d\n",bottom_offline);
    }

    memory.putString(String(offline_counter).c_str(),offlineString);
    //printf("Saved offline counter %d = %s\n",offline_counter,offlineString.c_str());

		offline_counter++;

    memory.putUInt("OC",offline_counter);
    //printf("Saved OC: %d\n",offline_counter);

    #ifdef LCD_MODULE
		drawOfflineCounter();
    #endif
}

void resetCounters()
{
  for(int i=0;i<MAX_INPUTS;i++)
  {
    COUNTERS[i] = 0;
    //COUNTERS_OLD[i] = 0;
    cnt[i] = 0;
    cnt_old[i] = 0;

    String CName = "CNT"+String(i);
    memory.putUInt(CName.c_str(),0);
    //printf("Saved %s = 0\n",CName.c_str());

    pcnt_counter_clear((pcnt_unit_t)PCNT_UNIT_NUM[i]);

    INPUT_CHANGE_TIME_STAMP[i] = millis();

  }
}

void readOverflowResto()
{
  for(int i=0;i<MAX_INPUTS;i++)
  {
    String CName = "CNT"+String(i);
    COUNTERS[i] = memory.getUInt(CName.c_str(),0);
    //printf("Read Counter %d = %lu\n",i,COUNTERS[i]);
  }
}

//####################### END MEMORY #########################//

//####################### PCNT ##############################//

void configure_encoder_type(int index)
{
  if(index<7 && INPUT_FUNCS[index]==2)
  {
    INPUT_FUNCS[index+1] = 3;
    pcnt_example_init(index,1);
    pcnt_example_init(index+1,2);
  }
}

static void pcnt_example_init(int index,int encoder)
{
    gpio_num_t PCNT_INPUT_CTRL_IO = (gpio_num_t)-1;
    //If second encoder pin control pin is previous pin
    if(encoder==2) PCNT_INPUT_CTRL_IO = (gpio_num_t) INPUT_PINS[index-1];
    //Else if first encoder pin control pin is next pin
    else PCNT_INPUT_CTRL_IO = (gpio_num_t) INPUT_PINS[index+1];

    pcnt_config_t pcnt_config;
    /*Set PCNT_INPUT_SIG_IO as pulse input gpio */
    pcnt_config.pulse_gpio_num = (gpio_num_t)INPUT_PINS[index];
    /*Choose channel 0 */
    pcnt_config.channel = PCNT_CHANNEL_0;
    /*Choose unit 0 */
    pcnt_config.unit = (pcnt_unit_t) PCNT_UNIT_NUM[index];
    /*Set counter and control mode*/
    /*Counter increase for positive edge on pulse input GPIO*/
    pcnt_config.pos_mode = PCNT_COUNT_INC;
    /*Counter decrease for negative edge on pulse input GPIO*/
    pcnt_config.neg_mode = PCNT_COUNT_DIS;

    pcnt_config.counter_h_lim = PCNT_H_LIM_VAL;

    if(encoder!=0)
    {
      /*set PCNT_INPUT_CTRL_IO as control gpio */
      pcnt_config.ctrl_gpio_num = PCNT_INPUT_CTRL_IO;
      //PCNT_MODE_DISABLE
      pcnt_config.lctrl_mode = PCNT_MODE_DISABLE;
      /*Counter mode does not change when control input is high level*/
      pcnt_config.hctrl_mode = PCNT_MODE_KEEP;      //when control signal is high,keep the primary counter mode
    }
    else
    {
      /*set PCNT_INPUT_CTRL_IO as control gpio */
      pcnt_config.ctrl_gpio_num = (gpio_num_t)0;

      pcnt_config.neg_mode = PCNT_COUNT_INC;
      //PCNT_MODE_DISABLE
      pcnt_config.lctrl_mode = PCNT_MODE_KEEP;
      /*Counter mode does not change when control input is high level*/
      pcnt_config.hctrl_mode = PCNT_MODE_KEEP;      //when control signal is high,keep the primary counter mode
    }

    /*Initialize PCNT unit */
    pcnt_unit_config(&pcnt_config);
    /*Configure input filter value*/
    pcnt_set_filter_value((pcnt_unit_t) PCNT_UNIT_NUM[index], FILTER);
    /*Enable input filter*/
    pcnt_filter_enable((pcnt_unit_t) PCNT_UNIT_NUM[index]);
    /*Enable watch point event of h_lim*/
    pcnt_event_enable((pcnt_unit_t) PCNT_UNIT_NUM[index], PCNT_EVT_H_LIM);
    /*Pause counter*/
    pcnt_counter_pause((pcnt_unit_t) PCNT_UNIT_NUM[index]);
    /*Reset counter value*/
    pcnt_counter_clear((pcnt_unit_t) PCNT_UNIT_NUM[index]);
    /*Register ISR handler*/
    pcnt_isr_register(pcnt_example_intr_handler, NULL, 0, NULL);
    /*Enable interrupt for PCNT unit*/
    pcnt_intr_enable((pcnt_unit_t) PCNT_UNIT_NUM[index]);
    /*Resume counting*/
    pcnt_counter_resume((pcnt_unit_t) PCNT_UNIT_NUM[index]);
}

static void IRAM_ATTR pcnt_example_intr_handler(void* arg)
{
    uint32_t intr_status = PCNT.int_st.val;
    int i;
    pcnt_evt_t evt;
    portBASE_TYPE HPTaskAwoken = pdFALSE;

    for(i = 0; i < PCNT_UNIT_MAX; i++) {
        if(intr_status & (BIT(i))) {
            evt.unit = i;
            evt.status = PCNT.status_unit[i].val;
            PCNT.int_clr.val = BIT(i);
            if(PCNT.status_unit[i].h_lim_lat) {
                cnt_old[i] = -1;
            }
            xQueueSendFromISR(pcnt_evt_queue, &evt, &HPTaskAwoken);
            if(HPTaskAwoken == pdTRUE) {
                portYIELD_FROM_ISR();
            }
        }
    }
}

static void pcnt_task(void *pcnt_task)
{
  pcnt_evt_t evt;
  while(1)
  {
      xQueueReceive(pcnt_evt_queue, &evt, 10 / portTICK_PERIOD_MS);

      for(int i=0;i<MAX_INPUTS;i++)
      {
        pcnt_get_counter_value((pcnt_unit_t)i, &cnt[i]);

        if(cnt[i]!=cnt_old[i])
        {
          if(cnt[i]-cnt_old[i]>0) COUNTERS[i] += cnt[i] - cnt_old[i];

          /*
          if(COUNTERS_OLD[i]>COUNTERS[i]) //Se antes maior bug count
          {
            COUNTERS[i] += PCNT_H_LIM_VAL;
          }
          COUNTERS_OLD[i] = COUNTERS[i];
          */

          String CName = "CNT"+String(i);
          memory.putUInt(CName.c_str(),COUNTERS[i]);

          //////////////////////////////////////////////////////////////////////////////

          if(INPUT_FUNCS[i] == 0)//Evento no pin INPUT_PINS[i]
          {
            if (millis() - INPUT_CHANGE_TIME_STAMP[i]>INPUT_CHANGE_TIME[i]  && INPUT_CHANGE_TIME[i]>0)
            {
              INPUT_STATES[i] = digitalRead(INPUT_PINS[i]);
              //printf("interrupt IO %d: %d\n",i+1,INPUT_STATES[i]);
              sendInterrupt(i+1);
              INPUT_CHANGE_TIME_STAMP[i] = millis();
            }
          }
          else//Contador ou Encoder
          {
            if(COUNTERS[i]>=INPUT_CHANGE_TIME[i] && millis() - INPUT_OVERFLOW_TIMESTAMP[i] > INPUT_OVERFLOW_GUARD[i])
            {
              sendOverFlow(i+1);
              resetCounters();
              //reset encoders
              INPUT_OVERFLOW_TIMESTAMP[i] = millis();
            }
          }

          cnt_old[i] = cnt[i];
        }
      }
      vTaskDelay(100 / portTICK_RATE_MS);
  }
}

//##################### END PCNT ############################//

//int count_rest = 0;

static void DHT22Task(void *pvParameters)
{
  while(1)
  {


      if(ONLINE)
      {
        //count_rest++;
        //if(count_rest>11) esp_restart();


        int rssi = 0;
        wifi_ap_record_t wifidata;
        if (esp_wifi_sta_get_ap_info(&wifidata)==0) rssi = wifidata.rssi;

        if(rssi>-30) signal_quality = 4;
        else if(rssi>-67) signal_quality = 3;
        else if(rssi>-70) signal_quality = 2;
        else if(rssi>-80) signal_quality = 1;
        else if(rssi>-90) signal_quality = 0;

        //printf("RSSI: %d\n",rssi);
      }

      if(hardware_version=="1")
      {

        float h = dht.readHumidity();
        // Read temperature as Celsius (the default)
        float t = dht.readTemperature();

        // Check if any reads failed and exit early (to try again).
        if (isnan(h) || isnan(t))
        {
          //printf("Failed to read from DHT sensor mac: %s!\n",macStr);
        }
        else
        {
          humidity = String(h,1);
          temperature = String(t,1);
        }
      }
      else if(hardware_version=="2")
      {
        float h = sht1x.readHumidity();
        // Read temperature as Celsius (the default)
        float t = sht1x.readTemperatureC();

        // Check if any reads failed and exit early (to try again).
        if (isnan(h) || isnan(t))
        {
          //printf("Failed to read from sht1x sensor mac: %s!\n",macStr);
        }
        else if (h>200 || h <-35 || t >200 || t< -35)
        {
          //printf("Failed to read from sht1x 00 sensor mac: %s!\n",macStr);
          humidity = "0.00";
          temperature = "0.00";
        }
        else
        {
          humidity = String(h,1);
          temperature = String(t,1);
        }
      }

      //printf("cnt[1]: %d\n",cnt[1]);

      vTaskDelay(2000 / portTICK_RATE_MS);
  }
}

static void bar_code_task(void *pvParameters)
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_0, &uart_config);
    //uart_set_pin(UART_NUM_1, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS);
    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0);

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

    while (1) {
        // Read data from the UART
        int len = uart_read_bytes(UART_NUM_0, data, BUF_SIZE, 20 / portTICK_RATE_MS);
        // Write data back to the UART
        //uart_write_bytes(UART_NUM_0, (const char *) data, len);
        if(len>0)
				{
					inputString = "";
					for(int i=0;i<len-2;i++)
					{
						inputString += (char)data[i];
					}
          inputPos = len-2;
          check_bar_code = true;
				}

    }
}


extern "C" void app_main()
{
  // Initialize NVS.
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
      // OTA app partition table has a smaller NVS partition size than the non-OTA
      // partition table. This size mismatch may cause NVS initialization to fail.
      // If this happens, we erase NVS partition and initialize NVS again.
      const esp_partition_t* nvs_partition = esp_partition_find_first(
              ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS, NULL);
      assert(nvs_partition && "partition table must have an NVS partition");
      ESP_ERROR_CHECK( esp_partition_erase_range(nvs_partition, 0, nvs_partition->size) );
      err = nvs_flash_init();
  }
  ESP_ERROR_CHECK( err );

  //begin namespace
  memory.begin(NamespacePreferences, false);

  pinMode(rele,OUTPUT);

  pinMode(BT1,INPUT);

  pinMode(BT2,INPUT);

  //#ifdef LCD_MODULE
  //initialize lcd
  u8g2.begin();
  //clear lcd
  u8g2.clearBuffer();



  //#endif

  //if BT1 and BT2 are clicked enter test menu
  if(digitalRead(BT1) && digitalRead(BT2))
  {
    network = memory.getString("NT","fluxotecDS01");
    //printf("Read NT = %s\n",network.c_str());
    //if define LCD MODULE accept entering test menu
    #ifdef LCD_MODULE
    //begin wifi
    initialise_wifi();
    //define input pins as input only
    for(int i=0;i<MAX_INPUTS;i++) pinMode(INPUT_PINS[i],INPUT);
    //creat task to read inputs by polling
    xTaskCreatePinnedToCore(&readInputsTestMenu, "readInputsTestMenu", 2048, NULL, 1, NULL,0);
    //creat task to write on LCD test mode
    xTaskCreatePinnedToCore(&writeLCDTaskTestMode, "writeLCDTaskTestMode", 2048, NULL, 1, NULL,0);
    //creat task to read keypad on super menu
    xTaskCreatePinnedToCore(&keypadTask2, "keypadTask2", 2048, NULL, 1, NULL,0);
    #endif //endif LCD_MODULE

  }
  else
  {

    //Task to write on LCD
    xTaskCreatePinnedToCore(&writeLCDTask, "writeLCDTask", 2048, NULL, 1, &xHandle,0);

    //printf("\n\nEND CREAT writeLCDTask\n\n");

    watchTimeStamp = millis();

    //Task to check LCD fail
    xTaskCreatePinnedToCore(&watchTask, "watchTask", 2048, NULL, 1, NULL,0);

    //printf("\n\nEND CREAT watchTask\n\n");


     //Configure PCNT for all pins
     for(int i=0;i<MAX_INPUTS;i++) {
       pcnt_example_init(i,0);
       INPUT_OVERFLOW_TIMESTAMP[i] = millis();
     }

     readOverflowResto();

     //read offline configurations
     read_offline_config();

     /*Init PCNT event queue */
     pcnt_evt_queue = xQueueCreate(100, sizeof(pcnt_evt_t));

     //Task to check counters and events
     xTaskCreatePinnedToCore(&pcnt_task, "pcnt_task", 2048, NULL, 1, NULL,0);

     getMenusOFF();

     //printf("\n\nEND GET MENUS OFF\n\n");

     getSubMenusOFF();

     //printf("\n\nEND GET SUBMENUS OFF\n\n");

     //Task to check button click
     xTaskCreatePinnedToCore(&touchTask, "touchTask", 2048, NULL, 1, NULL,0);

     //printf("\n\nEND CREAT touchTask\n\n");

     xTaskCreatePinnedToCore(&keypadTask, "keypadTask", 2048, NULL, 1, NULL,0);

     //printf("\n\nEND CREAT keypadTask\n\n");

     menuOn = false;

     //begin wifi
     initialise_wifi();

     //printf("\n\nEND initialise_wifi\n\n");

     //info send task
     xTaskCreatePinnedToCore(&clientTask, "clientTask", 4096, NULL, 1, NULL,0);

     //printf("\n\nEND CREAT clientTask\n\n");

     //check online status task
     xTaskCreatePinnedToCore(&checkOffline, "checkOffline", 2048, NULL, 1, NULL,0);

     //printf("\n\nEND CREAT checkOffline\n\n");

     //Server task
     xTaskCreatePinnedToCore(&mongooseTask, "mongooseTask", 4096, NULL, 1, NULL,0);

     //printf("\n\nEND CREAT mongooseTask\n\n");

     //Server DHT22 and RSSI task
     xTaskCreatePinnedToCore(&DHT22Task, "DHT22Task", 4096, NULL, 1, NULL,0);

     //printf("\n\nEND CREAT DHT22Task\n\n");

     xTaskCreate(bar_code_task, "bar_code_task", 1024, NULL, 10, NULL);

     ////printf("\n\nEND CREAT bar_code_task\n\n");

  }

}
