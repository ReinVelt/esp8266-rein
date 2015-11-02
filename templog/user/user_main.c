#include "ets_sys.h"
#include "osapi.h"
#include "gpio.h"
#include "os_type.h"
#include "ip_addr.h"
#include "espconn.h"
#include "user_interface.h"
#include "user_config.h"


struct espconn logger_conn;
ip_addr_t *logger_ip;
esp_tcp logger_tcp;

char logger_host[] = "gorilla.fritz.box";
char logger_path[] = "/logger.php";
char json_data[ 256 ];
char buffer[ 2048 ];


void user_rf_pre_init( void )
{
}


void data_received( void *arg, char *pdata, unsigned short len )
{
    struct espconn *conn = arg;
    os_printf( "%s: %s\n", __FUNCTION__, pdata );
    espconn_disconnect( conn );
}


void tcp_connected( void *arg )
{
    int tLow = 55;   // test data
    int tHigh = 83;
    int tAmbient = 20;
    
    struct espconn *conn = arg;
    
    os_printf( "%s\n", __FUNCTION__ );
    espconn_regist_recvcb( conn, data_received );

    os_sprintf( json_data, "{\"temperature\":{\"low\": \"%d\",\"high\": \"%d\",\"ambient\": \"%d\" }}", tLow,tHigh,tAmbient );
    os_sprintf( buffer, "POST %s HTTP/1.1\r\nHost: %s\r\nConnection: close\r\nContent-Type: application/json\r\nContent-Length: %d\r\n\r\n%s", 
                         logger_path, logger_host, os_strlen( json_data ), json_data );
    
    os_printf( "Sending: %s\n", buffer );
    espconn_sent( conn, buffer, os_strlen( buffer ) );
}


void tcp_disconnected( void *arg )
{
    struct espconn *conn = arg;
    os_printf( "%s\n", __FUNCTION__ );
    wifi_station_disconnect();
}


void dns_done( const char *name, ip_addr_t *ipaddr, void *arg )
{
    struct espconn *conn = arg;
    os_printf( "%s\n", __FUNCTION__ );
    
    if ( ipaddr == NULL) 
    {
        os_printf("DNS lookup failed\n");
        wifi_station_disconnect();
    }
    else
    {
        os_printf("Connecting...\n" );
        conn->type = ESPCONN_TCP;
        conn->state = ESPCONN_NONE;
        conn->proto.tcp=&logger_tcp;
        conn->proto.tcp->local_port = espconn_port();
        conn->proto.tcp->remote_port = 80;
        os_memcpy( conn->proto.tcp->remote_ip, &ipaddr->addr, 4 );
        espconn_regist_connectcb( conn, tcp_connected );
        espconn_regist_disconcb( conn, tcp_disconnected );
        espconn_connect( conn );
    }
}


void wifi_callback( System_Event_t *evt )
{
    os_printf( "%s: %d\n", __FUNCTION__, evt->event );
    switch ( evt->event )
    {
        case EVENT_STAMODE_CONNECTED:
        {
            os_printf("connect to ssid %s, channel %d\n",
                        evt->event_info.connected.ssid,
                        evt->event_info.connected.channel);
            break;
        }

        case EVENT_STAMODE_DISCONNECTED:
        {
            os_printf("disconnect from ssid %s, reason %d\n",
                        evt->event_info.disconnected.ssid,
                        evt->event_info.disconnected.reason);
            
            deep_sleep_set_option( 0 );
            system_deep_sleep( 60 * 1000 * 1000 );  // 60 seconds
            break;
        }

        case EVENT_STAMODE_GOT_IP:
        {
            os_printf("ip:" IPSTR ",mask:" IPSTR ",gw:" IPSTR,
                        IP2STR(&evt->event_info.got_ip.ip),
                        IP2STR(&evt->event_info.got_ip.mask),
                        IP2STR(&evt->event_info.got_ip.gw));
            os_printf("\n");
            os_printf("resolving host :s\n",logger_host);
            espconn_gethostbyname( &logger_conn, logger_host, logger_ip, dns_done );
            break;
        }
        
        default:
        {
            break;
        }
    }
}


void user_init( void )
{
    static struct station_config config;
    uart_div_modify( 0, UART_CLK_FREQ / ( 115200 ) );
    os_printf( "\n** INIT **\n%s\n", __FUNCTION__ );
    wifi_station_set_hostname( "logger" );
    wifi_set_opmode_current( STATION_MODE );
    gpio_init();
    config.bssid_set = 0;
    os_memcpy( &config.ssid, "M3CH4N1C4P3", 32 );
    os_memcpy( &config.password, "mechanicape", 64 );
    wifi_station_set_config( &config );
    wifi_set_event_handler_cb( wifi_callback );
}

