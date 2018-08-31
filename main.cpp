/* WiFi Example
 * Copyright (c) 2016 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "mbed.h"
#include "mbed_events.h"
#include "UDPSocket.h"
#include "platform/CircularBuffer.h"

#define WIFI_ESP8266    1
#define WIFI_IDW0XX1    2

#define BUFFER_SIZE 536

//Thread recv_thread;
Thread send_thread;
Thread toggle_thread;
EventQueue queue(8 * EVENTS_EVENT_SIZE);
InterruptIn sw(D3);
Serial pc(USBTX, USBRX);
//InterruptIn ld(SW1);
Thread spi_thread;
DigitalOut led(LED_BLUE);
UDPSocket udpsocket;
SocketAddress remote_address;
SPISlave spi(SPI_MOSI, SPI_MISO, SPI_SCK, SPI_CS);

Mutex stdio_send;
Timer t;
volatile bool _isError = false;
int dt=0, n=0;
// Allocate 1K of data
uint16_t *in_buffer =  new uint16_t[BUFFER_SIZE/2];
char *out_buffer = new char[BUFFER_SIZE];
CircularBuffer<uint16_t, BUFFER_SIZE> buf;

#if TARGET_UBLOX_EVK_ODIN_W2
#include "OdinWiFiInterface.h"
OdinWiFiInterface wifi;

#elif TARGET_REALTEK_RTL8195AM
#include "RTWInterface.h"
RTWInterface wifi;

#else // External WiFi modules

#if MBED_CONF_APP_WIFI_SHIELD == WIFI_ESP8266
#include "ESP8266Interface.h"
ESP8266Interface wifi(MBED_CONF_APP_WIFI_TX, MBED_CONF_APP_WIFI_RX);
#elif MBED_CONF_APP_WIFI_SHIELD == WIFI_IDW0XX1
#include "SpwfSAInterface.h"
SpwfSAInterface wifi(MBED_CONF_APP_WIFI_TX, MBED_CONF_APP_WIFI_RX);
#endif // MBED_CONF_APP_WIFI_SHIELD == WIFI_IDW0XX1

#endif

const char *sec2str(nsapi_security_t sec)
{
    switch (sec) {
        case NSAPI_SECURITY_NONE:
            return "None";
        case NSAPI_SECURITY_WEP:
            return "WEP";
        case NSAPI_SECURITY_WPA:
            return "WPA";
        case NSAPI_SECURITY_WPA2:
            return "WPA2";
        case NSAPI_SECURITY_WPA_WPA2:
            return "WPA/WPA2";
        case NSAPI_SECURITY_UNKNOWN:
        default:
            return "Unknown";
    }
}

/*
void receiveUDP() {

    while(!_isError) {
        // recvfrom blocks until there is data
        nsapi_size_or_error_t size = udpsocket.recvfrom(&remote_address, in_buffer, BUFFER_SIZE);
        if (size <= 0) {
            if (size == NSAPI_ERROR_WOULD_BLOCK) {  // Would block... that's fine (no data on the line)
                wait_us(1);
                continue;
            }

            printf("Error while receiving data from TCP socket, probably it's closed now? (%d)\n", size);
            _isError = true;
            break;
        }
        // turn into valid C string
        printf("Received size: %d\n",size);
    }
}
*/


void sendUDP() {
	/*Timer*/
	int dt = t.read_ms();
	out_buffer[4] = (char)	(dt >> 24);
	out_buffer[5] = (char)	(dt >> 16);
	out_buffer[6] = (char)	(dt >> 8);
	out_buffer[7] = (char)	dt;

	if(buf.full()){
		uint16_t data=0;
		/*Synchronizing the buffer data*/
		while(data!=0xAA0A){
			buf.pop(data);
			if(buf.empty()) return;
		}
		out_buffer[8]	=(char) (data>>8);
		out_buffer[9]	=(char)	data;
		for(int i=5;i<BUFFER_SIZE/2-4;i++){
			buf.pop(data);
			if(buf.empty()) return;
			out_buffer[2*i]		=(char) (data>>8);
			out_buffer[2*i+1]	=(char)	data;
		}
		buf.reset();
		nsapi_size_or_error_t size = udpsocket.sendto(remote_address, out_buffer, BUFFER_SIZE);
//		printf("Sent; First Data %X, Last Data %X\n", out_buffer[0], out_buffer[BUFFER_SIZE-1]);
		wait_ms(1);
		if (size <= 0) {
			if (size == NSAPI_ERROR_WOULD_BLOCK) {  // Would block... that's fine (no data on the line)
				wait_us(1);
				return;
		}
			printf("Error while sending data from TCP socket, probably it's closed now? (%d)\n", size);
			_isError = true;
			return;
		}
		buf.reset();
	}
	else {
		return;
	}
//	t.stop();
	//printf("Time:%d\tSent Size: %d\n", t.read_ms()-dt, size);
//	dt = t.read_ms();
//	t.start();
}

void spi_read(){
	if(!buf.full()){
		buf.push((uint16_t) spi.read());
	}
}

void closeTCP(nsapi_size_or_error_t s, nsapi_error_t err){
    printf("Closing the socket and Wifi Connection %d | %d\n", s, err);
    udpsocket.close();
    wifi.disconnect();
}


void led_toggle(){
	uint16_t data;
	printf("led toggle runs\n");
	if(pc.getc()=='y'){
		led = !led;
		printf("%d\n", n);
		for(int j=n-10;j<n;j++){
			printf("%X\n", in_buffer[j]);
		}
		send_thread.signal_set(0x01);
		return;
	}
	else if(pc.getc()=='x'){
		led = !led;
		printf("%d\n", n);
		for(int j=0;j<n;j++){
			printf("%X\n", in_buffer[j]);
		}
		printf("Buffer Content: ");
		bool ret = true;
		while(!buf.empty() && ret){
			ret = buf.pop(data);
			printf("%X ", data);
		}
		send_thread.signal_set(0x01);
		return;
	}
	else {
		send_thread.signal_set(0x01);
		return;
	}
}

int main()
{
//	EventQueue *queue = mbed_event_queue();
    spi.format(16,0);
    spi.frequency(5000000);
//   int count = 0;
    printf("UDP Client\n\n");
//    wait_ms(1000);
    printf("\nConnecting to %s...\n", MBED_CONF_APP_WIFI_SSID);
    int ret = wifi.connect(MBED_CONF_APP_WIFI_SSID, MBED_CONF_APP_WIFI_PASSWORD, NSAPI_SECURITY_WPA_WPA2);
    if (ret != 0) {
        printf("\nConnection error\n");
        wifi.disconnect();
        return -1;
    }
    wait(1);
    printf("Success\n\n");
    printf("MAC: %s\n", wifi.get_mac_address());
    printf("IP: %s\n", wifi.get_ip_address());
    printf("Netmask: %s\n", wifi.get_netmask());
    printf("Gateway: %s\n", wifi.get_gateway());
    printf("RSSI: %d\n\n", wifi.get_rssi());

    nsapi_error_t response;
    /*Open the server on wifi stack*/
    response = udpsocket.open(&wifi);
    if(response != NSAPI_ERROR_OK){
    	printf("error initialized port %d\n", response);
    	wifi.disconnect();
    	return -1;
    }

    udpsocket.set_blocking(false);
    if(!remote_address.set_ip_address("192.168.2.220")){
    	printf("error setting remote ip address\n");
    	wifi.disconnect();
    	return -2;
    }
    remote_address.set_port(5555);
    wait_ms(100);

/******************** Data Framing ********************/
	/*Start Sync Bytes*/
	out_buffer[0]=0x0A;
	out_buffer[1]=0x05;
	out_buffer[2]=0x0A;
	out_buffer[3]=0x05;
	/*Stop Sync Bytes*/
	out_buffer[BUFFER_SIZE-1]=0xAA;
	out_buffer[BUFFER_SIZE-2]=0x55;
	out_buffer[BUFFER_SIZE-3]=0xAA;
	out_buffer[BUFFER_SIZE-4]=0x55;


/************** ISR and Queue Initialize **************/

    send_thread.start(callback(sendUDP));
	sw.mode(PullDown);
	sw.fall(&spi_read);
//    sw.rise(queue.event(sendUDP));
    queue.call_every(1, sendUDP);
    t.start();
	queue.dispatch_forever();
    wait(osWaitForever);
}
