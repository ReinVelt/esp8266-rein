#include "math.h"
#include "ets_sys.h"
#include "osapi.h"
#include "gpio.h"
#include "os_type.h"
#include "ip_addr.h"
#include "espconn.h"
#include "user_interface.h"
#include "user_config.h"
#include "driver/adc.h"
#define user_procTaskPrio        0
#define user_procTaskQueueLen    1
#define samples_max     60

#define STATE_STARTUP 0
#define STATE_AP_DISCONNECTED 1
#define STATE_AP_CONNECTED    2
#define STATE_DNS_RESOLV 3
#define STATE_CONNECT         4
#define STATE_CONNECTED       5
#define STATE_ACQUISITION     6
#define STATE_ACQUISITION_READY 7
#define STATE_TRANSMISSION    8
#define STATE_TRANSMISSION_READY 9
int STATE=0;
int OLDSTATE=0;
struct espconn *connectionHandle;
ip_addr_t logger_ip;
esp_tcp logger_tcp;


char logger_host[] = "gorilla.fritz.box";
char logger_path[] = "/logger.php";
char json_data[ 256 ];
char buffer[ 2048 ];
int  sensor1Buffer[samples_max];
int  sensor2Buffer[samples_max];
int  sensor3Buffer[samples_max];
int  sensor4Buffer[samples_max];
int bufferPointer=0;


os_event_t  user_procTaskQueue[user_procTaskQueueLen];
static void loop(os_event_t *events);
static void user_procTask(os_event_t *events);
void ICACHE_FLASH_ATTR user_init( void );
 
// Loop function
static void ICACHE_FLASH_ATTR user_procTask(os_event_t *events)
{
    // Do stuff
    os_delay_us(1000);
    system_os_post(user_procTaskPrio, 0, 0 );
} 

void startLoop()
{
    //Start os task
    system_os_task(loop, user_procTaskPrio,user_procTaskQueue, user_procTaskQueueLen);
    system_os_post(user_procTaskPrio, 0, 0 );
}

void user_rf_pre_init( void )
{
}


void data_received( void *arg, char *pdata, unsigned short len )
{
    connectionHandle=arg;
    os_printf( "%s: %s\n", __FUNCTION__, pdata );
    espconn_disconnect( connectionHandle );
    STATE=STATE_TRANSMISSION_READY;
    
}

int getMedian(int x[samples_max],int maxValue){
   os_printf( "%s\n", __FUNCTION__ );
   //create frequention table
   int frequentions[samples_max]; //memory-eater, to be refactored
   int mcount=0;
   int median=0;
   int i,j,k=0;
   for (i=0;i<maxValue;i++){frequentions[i]=0;} //init array
   for (j=0;i<samples_max;j++){frequentions[x[j]]++; } //fill array
   for (k=0;k<maxValue;k++){ //loop over frequentions
     if (mcount<frequentions[k]) 
     { 
       mcount=frequentions[k];
       median=k;
     }
   }
   //int median=x[0];
   return median;
}

void sendPayload(void *arg)
{
     

     connectionHandle=arg;
     os_printf( "%s\n", __FUNCTION__ );
     STATE=STATE_TRANSMISSION;   
     uint8 macaddr[6]; 
     wifi_get_macaddr(0,macaddr);
     int tLow=getMedian(sensor1Buffer,1023);
     int tHigh=getMedian(sensor2Buffer,1023);
     int tAmbient=getMedian(sensor3Buffer,1023);
    os_sprintf( json_data, "{\"message\":{\"sensor\":{\"temperature\":{\"incoming\": \"%d\",\"outgoing\": \"%d\",\"ambient\": \"%d\" }}},\"nodeid\":\"%s\"}", tLow,tHigh,tAmbient,macaddr );
    os_sprintf( buffer, "POST %s HTTP/1.1\r\nHost: %s\r\nConnection: keep-alive\r\nContent-Type: application/json\r\nContent-Length: %d\r\n\r\n%s", logger_path, logger_host, os_strlen( json_data ), json_data );
     espconn_regist_recvcb( connectionHandle, data_received );
    espconn_sent( connectionHandle, buffer, os_strlen(buffer ) );

   
}

void tcp_connected( void *arg )
{

    os_printf( "%s\n", __FUNCTION__ );
    connectionHandle=arg;
    //STATE=STATE_CONNECTED;
    STATE=STATE_ACQUISITION;
    
    
    
}






void tcp_disconnected( void *arg )
{
     connectionHandle=arg;
    os_printf( "%s\n", __FUNCTION__ );
    wifi_station_disconnect();
    os_printf("Disconnected\n");
    STATE=STATE_AP_DISCONNECTED;

    
    
}


void dns_done( const char *name, ip_addr_t *ipaddr, void *arg )
{
    connectionHandle = arg;
    os_printf( "%s\n", __FUNCTION__ );
    
    if ( ipaddr == NULL) 
    {
        os_printf("DNS lookup failed\n");
        wifi_station_disconnect();
        STATE=STATE_AP_DISCONNECTED;
    }
    else
    {
        os_printf("Connecting...\n" );
        connectionHandle->type = ESPCONN_TCP;
        connectionHandle->state = ESPCONN_NONE;
        connectionHandle->proto.tcp=&logger_tcp;
        connectionHandle->proto.tcp->local_port = espconn_port();
        connectionHandle->proto.tcp->remote_port = 80;
        os_memcpy( connectionHandle->proto.tcp->remote_ip, &ipaddr->addr, 4 );
        espconn_regist_connectcb( connectionHandle, tcp_connected );
        espconn_regist_disconcb( connectionHandle, tcp_disconnected );
        STATE=STATE_CONNECT;
        espconn_connect( connectionHandle );
       
       
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
                        STATE=STATE_AP_CONNECTED;
            break;
        }

        case EVENT_STAMODE_DISCONNECTED:
        {
            os_printf("disconnect from ssid %s, reason %d\n",
                        evt->event_info.disconnected.ssid,
                        evt->event_info.disconnected.reason);
                        STATE=STATE_AP_DISCONNECTED;
            
            
            break;
        }

        case EVENT_STAMODE_GOT_IP:
        {
            os_printf("ip:" IPSTR ",mask:" IPSTR ",gw:" IPSTR,
                        IP2STR(&evt->event_info.got_ip.ip),
                        IP2STR(&evt->event_info.got_ip.mask),
                        IP2STR(&evt->event_info.got_ip.gw));
            os_printf("\n");
            STATE=STATE_DNS_RESOLV;
            os_printf("resolving host :%s\n",logger_host);
            espconn_gethostbyname( connectionHandle, logger_host, &logger_ip, dns_done );
            break;
        }
        
        default:
        {
            break;
        }
    }
}



void doAcquisition()
{
        
        if (bufferPointer<samples_max)
        {
          bufferPointer++;
          int sensor=system_adc_read();
          os_printf(".");
          sensor1Buffer[bufferPointer]=sensor;
          sensor2Buffer[bufferPointer]=sensor;
          sensor3Buffer[bufferPointer]=sensor;
          sensor4Buffer[bufferPointer]=sensor;
          STATE= STATE_ACQUISITION;
         }
         else
         { 
          STATE= STATE_ACQUISITION_READY;
          bufferPointer=0;
          }
}



//Main code function
static void ICACHE_FLASH_ATTR loop(os_event_t *events) {
 
 switch (STATE)
 {
        case STATE_STARTUP:  break;
        case STATE_AP_DISCONNECTED:     wifi_station_connect(); break;
        case STATE_AP_CONNECTED: break; //is handled by conn callback
        case STATE_DNS_RESOLV: break; //is handled by dns callback
        case STATE_CONNECT:break; //is handled by dns callback
        case STATE_CONNECTED: doAcquisition() ; break;
        case STATE_ACQUISITION:  doAcquisition(); break; 
        case STATE_ACQUISITION_READY: sendPayload(connectionHandle); break;
        case STATE_TRANSMISSION:break; //is handled by receivedata callback;
        case STATE_TRANSMISSION_READY: ; break;
        default:break;
 }  
 if (OLDSTATE!=STATE)
 {
   os_printf("STATE %d -> %d\n",OLDSTATE,STATE);
 }
 os_delay_us(10000);
 system_os_post(user_procTaskPrio, 0, 0 );

}



void ICACHE_FLASH_ATTR user_init( void )
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
    STATE=STATE_STARTUP;
   
    startLoop();
    
}




