/*
 *  MQTT Switch
 *
 *  This firmware is meant for remote control with the ESP8266 via WIFI of the Sony As/20 action camera
 *
 *
 *  (c) 2015 by Rein Velt (rein@mechanicape.com)
 *
 */
#include "ets_sys.h"
#include "driver/uart.h"
#include "osapi.h"
#include "wifi.h"
#include "user_interface.h"
#include "mem.h"
#include "espconn.h"

#define CAMERA_HOST			"192.168.122.1" 
#define CAMERA_PORT			10000
#define CAMERA_BUF_SIZE		        1024
#define CAMERA_KEEPALIVE	        120	 /*second*/
#define CAMERA_PATH                     "/sony/camera/"
#define CAMERA_IP                       "192.168.122.1"   // fixed IP of camera
#define CAMERA_SSID                     "DIRECT-WXH4:HDR-AS20"
#define CAMERA_PWD                      "WjNpHNke"
#define CAMERA_ENCRYPTION_TYPE          AUTH_WPA2_PSK
#define CAMERA_SERVER_PATH              "/sony/camera"
#define CAMERA_COMMAND_GETVERSIONS      "{\"version\":\"1.0\",\"id\":1,\"method\":\"getVersions\",\"params\":[]}"
#define CAMERA_COMMAND_STARTRECMODE     "{\"version\":\"1.0\",\"id\":1,\"method\":\"startRecMode\",\"params\":[]}"
#define CAMERA_COMMAND_STARTLIVEVIEW    "{\"version\":\"1.0\",\"id\":1,\"method\":\"startLiveview\",\"params\":[]}"
#define CAMERA_COMMAND_STOPLIVEVIEW     "{\"version\":\"1.0\",\"id\":1,\"method\":\"stopLiveview\",\"params\":[]}"
#define CAMERA_COMMAND_ACTTAKEPICTURE   "{\"version\":\"1.0\",\"id\":1,\"method\":\"actTakePicture\",\"params\":[]}"
char json_data[ 256 ];
char buffer[ 2048 ];
struct espconn camera_conn;
ip_addr_t this_ip;
esp_tcp this_tcp;
void data_received( void *arg, char *pdata, unsigned short len )
{
    struct espconn *conn = arg;
    os_printf( "%s: %s\n", __FUNCTION__, pdata );
    espconn_disconnect(conn);
  
}

void take_picture(void *arg)
{
    struct espconn *conn = arg;
    os_printf( "%s\n", __FUNCTION__ );
    espconn_regist_recvcb( conn, data_received );
    os_sprintf( json_data, CAMERA_COMMAND_ACTTAKEPICTURE);
    os_sprintf( buffer, "POST %s HTTP/1.1\r\nHost: %s\r\nConnection: keep-alive\r\nContent-Type: application/json\r\nContent-Length: %d\r\n\r\n%s", CAMERA_SERVER_PATH, CAMERA_HOST, os_strlen( json_data ), json_data );   
    os_printf( "Sending: %s\n", buffer );
    espconn_sent( conn, buffer, os_strlen( buffer ) );
}

void tcp_disconnected( void *arg )
{
    struct espconn *conn = arg;
    
    os_printf( "%s\n", __FUNCTION__ );
    //wifi_station_disconnect();
    os_delay_us(1000000);
      espconn_connect(conn);
}

void camera_connect( struct espconn *camera_con, ip_addr_t *ipaddr )
{
    struct espconn *conn = camera_con;
    
    os_printf( "%s\n", __FUNCTION__ );
    
    if ( ipaddr == NULL) 
    {
        os_printf("DNS lookup failed\n");
        wifi_station_disconnect();
    }
    else
    {
        os_printf("Connecting to camera...\n" );
        
        conn->type = ESPCONN_TCP;
        conn->state = ESPCONN_NONE;
        conn->proto.tcp=&this_tcp;
        conn->proto.tcp->local_port = espconn_port();
        conn->proto.tcp->remote_port = CAMERA_PORT;
        os_memcpy( conn->proto.tcp->remote_ip, &ipaddr->addr, 4 );

        espconn_regist_connectcb( conn, take_picture );
        espconn_regist_disconcb( conn, tcp_disconnected );
        
        espconn_connect( conn );
        camera_con=conn;
    }
}

void wifi_callback( System_Event_t *evt )
{
    os_printf( "%s: %d\n", __FUNCTION__, evt->event );
    
    switch ( evt->event )
    {
        case EVENT_STAMODE_CONNECTED:
        {
            os_printf("wifi connect to ssid %s, channel %d\n",
                        evt->event_info.connected.ssid,
                        evt->event_info.connected.channel);
            break;
        }

        case EVENT_STAMODE_DISCONNECTED:
        {
            os_printf("wifi disconnect from ssid %s, reason %d\n",
                        evt->event_info.disconnected.ssid,
                        evt->event_info.disconnected.reason);
            
            
            break;
        }

        case EVENT_STAMODE_GOT_IP:
        {
            os_printf("ip:" IPSTR ",mask:" IPSTR ",gw:" IPSTR,
                        IP2STR(&evt->event_info.got_ip.ip),
                        IP2STR(&evt->event_info.got_ip.mask),
                        IP2STR(&evt->event_info.got_ip.gw));
            os_printf("\n");
            
            camera_connect(&camera_conn, &evt->event_info.got_ip.gw);
            
            break;
        }
        
        default:
        {
            break;
        }
    }
}




void ICACHE_FLASH_ATTR
user_init(void)
{
    static struct station_config config;
	uart_init(BIT_RATE_115200, BIT_RATE_115200);
	os_printf("\r\nSDK version: %s\n", system_get_sdk_version());
	os_printf("System init...\r\n");
	system_set_os_print(1);
	os_delay_us(1000000);
	//config_load();
	
    wifi_station_set_hostname( "camremote" );
    wifi_set_opmode_current( STATION_MODE );
    config.bssid_set = 0;
    os_memcpy( &config.ssid,CAMERA_SSID, 32 );
    os_memcpy( &config.password, CAMERA_PWD, 64 );
    wifi_station_set_config( &config );
    wifi_set_event_handler_cb( wifi_callback );
    os_printf("\r\nSystem started ...\r\n");
}

void ICACHE_FLASH_ATTR
ICACHE_FLASH_ATTRuser_rf_pre_init(void) {}
