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

#define COMMUNICATION_INTERVAL          1000 //mSecs

#define MQTT_SENSOR_COLLISSION_FRONT          "/car/sensor/collission/front"
#define MQTT_SENSOR_COLLISSION_LEFT          "/car/sensor/collission/left"
#define MQTT_SENSOR_COLLISSION_RIGHT          "/car/sensor/collission/right"
#define MQTT_SENSOR_COMPASS             "/car/sensor/compass"
#define MQTT_ACTUATOR_MOTOR_LEFT        "/car/actuator/motor/left"
#define MQTT_ACTUATOR_MOTOR_RIGHT       "/car/actuator/motor/right"
#define MQTT_REMOTE_DIRECTION           "/car/remote/direction"


#define GPIO_PIN_MOTOR_LEFT_ENABLE      0
#define GPIO_PIN_MOTOR_LEFT_PWM         5
#define GPIO_PIN_MOTOR_RIGHT_ENABLE     4
#define GPIO_PIN_MOTOR_RIGHT_PWM        2
#define GPIO_PIN_COLLISSION_FRONT       14
#define GPIO_PIN_COLLISSION_LEFT        12
#define GPIO_PIN_COLLISSION_RIGHT       13
#define GPIO_PIN_SPEAKER                15

#define DIRECTION_STOP                      0
#define DIRECTION_FORWARD                   1
#define DIRECTION_BACKWARD                  2
#define DIRECTION_CCW                       3
#define DIRECTION_CW                        4

#define STATE_BOOT                      0
#define STATE_STARTUP                   1
#define STATE_MOVE                      2
#define STATE_COLLISSION                3
#define STATE_OVERRIDE                  4
#define STATE_ERROR                     5

LOCAL os_timer_t interval_timer;

MQTT_Client mqttClient;
int state;
int sequenceCounter;
uint8_t pwm_channels[]= {0};
uint16_t pwm_frequency = 1000;
uint8_t collission=0;
uint8_t collissionOld=-1;







//===============WIFI FUNCTIONS====================
void wifiConnectCb(uint8_t status)
{
	if(status == STATION_GOT_IP){
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
	
}

void mqttDisconnectedCb(uint32_t *args)
{
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Disconnected\r\n");
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
	        state=STATE_OVERRIDE;
	}
	os_free(topicBuf);
	os_free(dataBuf);
}


void mqttWrite()
{       
       
        char collissionStr[16];
        os_sprintf(collissionStr,"%B",collission);
        MQTT_Publish(&mqttClient,MQTT_SENSOR_COLLISSION_FRONT, collissionStr, 2, 0, 0);
        os_free(collissionStr);
        
	
}


int getCollissionState()
{
        int a=1-GPIO_INPUT_GET(GPIO_PIN_COLLISSION_FRONT);
        int b=1-GPIO_INPUT_GET(GPIO_PIN_COLLISSION_LEFT);
        int c=1-GPIO_INPUT_GET(GPIO_PIN_COLLISSION_RIGHT);
        collission=100*a+10*b+c;
        if (collission>0)
        {
                state=STATE_COLLISSION;
        }
        
        if (collission!=collissionOld)
        {
                mqttWrite();
        }
        collissionOld=collission;
       
}



//==================== MOTOR FUNCTIONS ========================

int motor_run(int direction,int speed)
{
    int leftEnable=LOW;
    int rightEnable=LOW;
    switch (direction)
    {
        case DIRECTION_STOP: leftEnable=LOW; rightEnable=LOW; break;
        case DIRECTION_FORWARD: leftEnable=HIGH; rightEnable=LOW; break;
        case DIRECTION_BACKWARD: leftEnable=LOW; rightEnable=HIGH; break;
        case DIRECTION_CCW: leftEnable=LOW; rightEnable=LOW; break;
        case DIRECTION_CW: leftEnable=HIGH; rightEnable=HIGH; break;
    }  
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
        motor_run(DIRECTION_FORWARD,255);
}

void runStateCollission()
{
       motor_run(DIRECTION_CW,255);      
}

LOCAL void ICACHE_FLASH_ATTR timer_cb(void *arg)
{
        os_timer_disarm(&interval_timer);
        int collission=getCollissionState();;
        if (collission) { state=STATE_COLLISSION;}
        switch (state)
        {
                case STATE_STARTUP:     break;
                case STATE_MOVE:        runStateMove(); break;
                case STATE_COLLISSION:  runStateCollission(); break;
                case STATE_ERROR:       break;
                case STATE_OVERRIDE:   break;
        }
        
        os_timer_arm(&interval_timer, COMMUNICATION_INTERVAL, 1);
        	
}


//====================== STARTUP ===============================

void user_init(void)
{
	uart_init(BIT_RATE_115200, BIT_RATE_115200);
	os_delay_us(1000000); //wait 1 sec 
	INFO("\n# ******** ESPIOTCAR robotcar firmware ********\n");
        INFO("# load config...\n");
	CFG_Load();
	
        //MQTT
	MQTT_InitConnection(&mqttClient, sysCfg.mqtt_host, sysCfg.mqtt_port, sysCfg.security);
	MQTT_InitClient(&mqttClient, sysCfg.device_id, sysCfg.mqtt_user, sysCfg.mqtt_pass, sysCfg.mqtt_keepalive, 1);
	MQTT_InitLWT(&mqttClient, "/lwt", "offline", 0, 0);
	MQTT_OnConnected(&mqttClient, mqttConnectedCb);
	MQTT_OnDisconnected(&mqttClient, mqttDisconnectedCb);
	MQTT_OnPublished(&mqttClient, mqttPublishedCb);
	MQTT_OnData(&mqttClient, mqttDataCb);
	INFO("# MQTT loaded...\n");

        //WIFI
	WIFI_Connect(sysCfg.sta_ssid, sysCfg.sta_pwd, wifiConnectCb);
	INFO("# WIFI initialized...\n");

        //GPIO
	gpio_init();
	//PIN_FUNC_SELECT(PHERIPS_IO_MUX_GPIO0_U,FUNC_GPIO0);
	//PIN_FUNC_SELECT(PHERIPS_IO_MUX_U0_U,FUNC_GPIO1);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U,FUNC_GPIO12);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U,FUNC_GPIO13);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U,FUNC_GPIO14);
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U,FUNC_GPIO15);
	
	
	set_gpio_mode(GPIO_PIN_MOTOR_LEFT_ENABLE, GPIO_OUTPUT, 0);
	set_gpio_mode(GPIO_PIN_MOTOR_RIGHT_ENABLE, GPIO_OUTPUT, 0);
	set_gpio_mode(GPIO_PIN_MOTOR_LEFT_PWM, GPIO_OUTPUT, 0);
	set_gpio_mode(GPIO_PIN_MOTOR_RIGHT_PWM, GPIO_OUTPUT, 0);
	
	set_gpio_mode(GPIO_PIN_COLLISSION_FRONT, GPIO_INPUT,  GPIO_PULLDOWN);
	set_gpio_mode(GPIO_PIN_COLLISSION_LEFT,  GPIO_INPUT,  GPIO_PULLDOWN);
	set_gpio_mode(GPIO_PIN_COLLISSION_RIGHT, GPIO_INPUT,  GPIO_PULLDOWN);
	set_gpio_mode(GPIO_PIN_SPEAKER, GPIO_OUTPUT, 0);
	

	INFO("# GPIO initialized...\n");
	
	//PWM
	//pwm_init(1000000, pwm_frequency,GPIO_PIN_SPEAKER,*pin_info_list[3]);
	//INFO("# PWM initialized...\n");
	
	//communication
	os_timer_disarm(&interval_timer);
	os_timer_setfn(&interval_timer, (os_timer_func_t *)timer_cb, (void *)0);
	os_timer_arm(&interval_timer, COMMUNICATION_INTERVAL, 1);
	INFO("# Timer callback initialized ...\n");
	INFO("\n# System started.\n");
	state=STATE_STARTUP;
}
