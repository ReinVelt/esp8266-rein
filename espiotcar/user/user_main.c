/* main.c 
*
* Copyright (c) 2014-2015, Rein Velt <rein at mechanicape dot com>
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of Redis nor the names of its contributors may be used
* to endorse or promote products derived from this software without
* specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

#include "ets_sys.h"
#include "driver/uart.h"
#include "osapi.h"
#include "mqtt.h"
#include "wifi.h"
#include "config.h"
#include "debug.h"
#include "pwm.h"
#include "gpio.h"
#include "user_interface.h"
#include "mem.h"
#include "driver/gpio16.h"

#define LOW                             0
#define HIGH                            1

#define COMMUNICATION_INTERVAL          100 //mSecs

#define MQTT_SENSOR_COLLISSION          "/car/sensor/collission"
#define MQTT_MOTOR_LEFT                 "/car/motor/left"
#define MQTT_MOTOR_RIGHT                "/car/motor/right"
#define MQTT_REMOTE_DIRECTION           "/car/remote/direction"


#define GPIO_PIN_MOTOR_LEFT_PWM            5
#define GPIO_PIN_MOTOR_RIGHT_PWM           4
#define GPIO_PIN_MOTOR_LEFT_ENABLE         0
#define GPIO_PIN_MOTOR_RIGHT_ENABLE        2

#define GPIO_PIN_COLLISSION1            14
#define GPIO_PIN_COLLISSION2            12
#define GPIO_PIN_COLLISSION3            13
#define GPIO_PIN_COLLISSION4            15

#define DIRECTION_STOP                      0
#define DIRECTION_FORWARD                   1
#define DIRECTION_BACKWARD                  2
#define DIRECTION_CCW                       3
#define DIRECTION_CW                        4

#define STATE_BOOT                      0
#define STATE_STARTUP                   1
#define STATE_NETWORK_INIT              2
#define STATE_MQTT_INIT                 3
#define STATE_CONNECTED                 4
#define STATE_READY                     5
#define STATE_MOVE                      6
#define STATE_COLLISSION                7
#define STATE_OVERRIDE                  8
#define STATE_ERROR                     9

LOCAL os_timer_t interval_timer;

MQTT_Client mqttClient;
int state;
int sequenceCounter;
uint8_t pwm_channels[]= {0};
uint16_t pwm_frequency = 1000;
int collission=0;







//===============WIFI FUNCTIONS====================
void wifiConnectCb(uint8_t status)
{
	if(status == STATION_GOT_IP){
	        state=STATE_MQTT_INIT;
		MQTT_Connect(&mqttClient);
	} else {
		MQTT_Disconnect(&mqttClient);
	}
	
}

//===============MQTT FUNCTIONS ====================
void mqttConnectedCb(uint32_t *args)
{
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Connected\r\n");
	MQTT_Subscribe(client, MQTT_REMOTE_DIRECTION, 0);
	state=STATE_CONNECTED;
	state=STATE_READY;
	
}

void mqttDisconnectedCb(uint32_t *args)
{
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Disconnected\r\n");
	state=STATE_MQTT_INIT;
}

void mqttPublishedCb(uint32_t *args)
{
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Published\r\n");
}

void mqttDataCb(uint32_t *args, const char* topic, uint32_t topic_len, const char *data, uint32_t data_len)
{
	char *topicBuf = (char*)os_zalloc(topic_len+1),*dataBuf = (char*)os_zalloc(data_len+1);
	MQTT_Client* client = (MQTT_Client*)args;
	os_memcpy(topicBuf, topic, topic_len);
	topicBuf[topic_len] = 0;
	os_memcpy(dataBuf, data, data_len);
	dataBuf[data_len] = 0;
	INFO("Receive topic: %s, data: %s \r\n", topicBuf, dataBuf);
	if (topicBuf==MQTT_REMOTE_DIRECTION)
	{
	        //REMOTE CONTROL
	        //if (state!=STATE_COLLISSION && dataBuf!=DIRECTION_FORWARD)
	        //{ 
	                state=STATE_OVERRIDE; 
	                motor_run(dataBuf,255);
	        //}
	        
	}
	os_free(topicBuf);
	os_free(dataBuf);
}


void mqttWrite()
{       
       
        char collissionStr[1];
        os_sprintf(collissionStr,"%x",collission);
        INFO("collission: %x\n",collission);
        MQTT_Publish(&mqttClient,MQTT_SENSOR_COLLISSION, collissionStr, 1, 0, 0);
        //os_free(collissionStr);
        
	
}


int getCollissionState()
{
        int collissionOld=collission;
        PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U,FUNC_GPIO14);             //d5/gpio14 - sens0 - shuts down automagically	
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U,FUNC_GPIO12);             //d6/gpio12 - sens1
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U,FUNC_GPIO13);             //d7/gpio13 - sens2
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U,FUNC_GPIO15);             //d8/gpio15 - sens3 -conflict with flashing, keep high
	set_gpio_mode(GPIO_PIN_COLLISSION1, GPIO_INPUT,  GPIO_PULLUP);
	set_gpio_mode(GPIO_PIN_COLLISSION2, GPIO_INPUT,  GPIO_PULLUP);
	set_gpio_mode(GPIO_PIN_COLLISSION3, GPIO_INPUT,  GPIO_PULLUP);
	set_gpio_mode(GPIO_PIN_COLLISSION4, GPIO_INPUT,  GPIO_PULLUP);
        collission=(1-GPIO_INPUT_GET(GPIO_PIN_COLLISSION1))*8+
                   (1-GPIO_INPUT_GET(GPIO_PIN_COLLISSION2))*4+
                   (1-GPIO_INPUT_GET(GPIO_PIN_COLLISSION3))*2+
                   (1-GPIO_INPUT_GET(GPIO_PIN_COLLISSION4))*1;
        if (collission!=collissionOld)
        {
                mqttWrite();
        }
        
       
}



//==================== MOTOR FUNCTIONS ========================

int motor_run(int direction,int speed)
{
    int leftEnable=LOW;
    int rightEnable=LOW;
    switch (direction)
    {
        case DIRECTION_STOP: leftEnable=LOW; rightEnable=LOW; break;
        case DIRECTION_FORWARD: leftEnable=HIGH; rightEnable=HIGH; break;
        case DIRECTION_BACKWARD: leftEnable=LOW; rightEnable=LOW; break;
        case DIRECTION_CCW: leftEnable=LOW; rightEnable=HIGH; break;
        case DIRECTION_CW: leftEnable=HIGH; rightEnable=LOW; break;
    }  
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO5_U,FUNC_GPIO5);             //d1/gpio5  - motor l pwm
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO4_U,FUNC_GPIO4);             //d2/gpio4  - motor r pwm
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U,FUNC_GPIO0);             //d3/gpio0  - motor l direction
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U,FUNC_GPIO2);             //d4/gpio2  - motor r direction
    
    gpio_write(GPIO_PIN_MOTOR_LEFT_ENABLE,leftEnable);
    gpio_write(GPIO_PIN_MOTOR_RIGHT_ENABLE,rightEnable);
    gpio_write(GPIO_PIN_MOTOR_LEFT_PWM,speed);
    gpio_write(GPIO_PIN_MOTOR_RIGHT_PWM,speed);
        
}

int motor_stop()
{
        motor_run(DIRECTION_STOP,0);
}




//========STATE MACHINE HANDLER ============
void runStateMove()
{
        motor_run(DIRECTION_FORWARD,1);
}

void runStateCollission()
{
       motor_run(DIRECTION_CW,1);      
}

LOCAL void ICACHE_FLASH_ATTR timer_cb(void *arg)
{
          os_timer_disarm(&interval_timer);
          if (state>STATE_CONNECTED)
          {
                
                int collission=getCollissionState();;
                if (collission>0) { state=STATE_COLLISSION;} else {state=STATE_MOVE;}
                switch (state)
                {
                        
                        case STATE_MOVE:        runStateMove(); break;
                        case STATE_COLLISSION:  runStateCollission(); break;
                        case STATE_ERROR:      break;
                        case STATE_OVERRIDE:   break;
                }
        }
        os_timer_arm(&interval_timer, COMMUNICATION_INTERVAL, 1);
        	
}

LOCAL void ICACHE_FLASH_ATTR gpio_interrupt_cb(void *arg)
{
        
}


//====================== STARTUP ===============================

void user_init(void)
{
	uart_init(BIT_RATE_115200, BIT_RATE_115200);
	os_delay_us(1000000); //wait 1 sec 
	INFO("\n# ******** ESPIOTCAR robotcar firmware ********\n");
	CFG_Load();
	
        //MQTT
	MQTT_InitConnection(&mqttClient, sysCfg.mqtt_host, sysCfg.mqtt_port, sysCfg.security);
	MQTT_InitClient(&mqttClient, sysCfg.device_id, sysCfg.mqtt_user, sysCfg.mqtt_pass, sysCfg.mqtt_keepalive, 1);
	MQTT_InitLWT(&mqttClient, "/lwt", "offline", 0, 0);
	MQTT_OnConnected(&mqttClient, mqttConnectedCb);
	MQTT_OnDisconnected(&mqttClient, mqttDisconnectedCb);
	MQTT_OnPublished(&mqttClient, mqttPublishedCb);
	MQTT_OnData(&mqttClient, mqttDataCb);


        //WIFI
	WIFI_Connect(sysCfg.sta_ssid, sysCfg.sta_pwd, wifiConnectCb);


        //GPIO
	gpio_init();
	                                                               //d0/gpio16 - unused
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO5_U,FUNC_GPIO5);             //d1/gpio5  - motor l pwm
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO4_U,FUNC_GPIO4);             //d2/gpio4  - motor r pwm
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U,FUNC_GPIO0);             //d3/gpio0  - motor l direction
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO2_U,FUNC_GPIO2);             //d4/gpio2  - motor r direction

	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U,FUNC_GPIO14);             //d5/gpio14 - sens0 - shuts down automagically	
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U,FUNC_GPIO12);             //d6/gpio12 - sens1
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U,FUNC_GPIO13);             //d7/gpio13 - sens2
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U,FUNC_GPIO15);             //d8/gpio15 - sens3 -conflict with flashing, keep high
	
	set_gpio_mode(GPIO_PIN_MOTOR_LEFT_ENABLE,  GPIO_OUTPUT, 0);
	set_gpio_mode(GPIO_PIN_MOTOR_RIGHT_ENABLE, GPIO_OUTPUT, 0);
	set_gpio_mode(GPIO_PIN_MOTOR_LEFT_PWM,     GPIO_OUTPUT, 0);
	set_gpio_mode(GPIO_PIN_MOTOR_RIGHT_PWM,    GPIO_OUTPUT, 0);
	
	set_gpio_mode(GPIO_PIN_COLLISSION1, GPIO_INPUT,  GPIO_PULLUP);
	set_gpio_mode(GPIO_PIN_COLLISSION2, GPIO_INPUT,  GPIO_PULLUP);
	set_gpio_mode(GPIO_PIN_COLLISSION3, GPIO_INPUT,  GPIO_PULLUP);
	set_gpio_mode(GPIO_PIN_COLLISSION4, GPIO_INPUT,  GPIO_PULLUP);
	

	
	//communication
	os_timer_disarm(&interval_timer);
	os_timer_setfn(&interval_timer, (os_timer_func_t *)timer_cb, (void *)0);
	os_timer_arm(&interval_timer, COMMUNICATION_INTERVAL, 1);
	state=STATE_STARTUP;
}
