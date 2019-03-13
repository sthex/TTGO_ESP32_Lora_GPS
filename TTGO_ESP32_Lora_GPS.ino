//https://github.com/LilyGO/TTGO-LORA32-V2.0
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>  
#include <SSD1306.h>

#include <TinyGPS++.h>  // http://arduiniana.org/libraries/tinygpsplus/

//GPIO34-39 can only be set as input mode and do not have software pullup or pulldown 
#define BTN_PRESS 37 
#define BTN_DOWN  38
#define BTN_UP    39
#define BTN_LEFT  34
#define BTN_RIGHT 36

// Lora
#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISnO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
#define RST     14   // GPIO14 -- SX1278's RESET
#define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define BAND  868E6  // Lora frequency
#define OLED_ADR 0x3c
#define OLED_SDA 4
#define OLED_SCL 15
#define OLED_RST 16

#define uS_TO_Min_FACTOR 60000000  /* Conversion factor for micro seconds to seconds */
#define SLEEP_AFTER_X_SENT 5
RTC_DATA_ATTR int minutesToSleep = 0;        // Time span ESP32 will go to sleep (in minutes) 
RTC_DATA_ATTR int bootCount = 0;   // Number of boot from deep sleep

HardwareSerial Serial2(2);
#define RXD2 23   // Gps RX. NB. GPIO16 is occupied by OLED_RST 
#define TXD2 17

TinyGPSPlus gps;

unsigned int teller = 0;
byte msgCount = 0;            // count of outgoing messages
int interval = 12000;          // interval between sends
long lastSendTime = 0;        // time of last packet send
long lastDisplayUpdate = 0;
int lastRSSI;
int numSendt = 0;
String messageOut = "";
String incoming = "";

SSD1306 display(OLED_ADR, OLED_SDA, OLED_SCL);
//String rssi = "RSSI --";
//String packSize = "--";
//String packet;

int requestSleep = -1;  // Pending sleep request if >= 0

void setup() {
	Serial.begin(115200);
	while (!Serial);
	++bootCount;
	Serial.println("Boot number: " + String(bootCount));

	pinMode(OLED_RST, OUTPUT);
	pinMode(2, OUTPUT);

	pinMode(BTN_PRESS, INPUT);
	pinMode(BTN_DOWN, INPUT);
	pinMode(BTN_LEFT, INPUT);
	pinMode(BTN_RIGHT, INPUT);
	pinMode(BTN_UP, INPUT);

	digitalWrite(OLED_RST, LOW);    // set GPIO16 low to reset OLED
	delay(50);
	digitalWrite(OLED_RST, HIGH); // while OLED is running, must set GPIO16 in high

	SPI.begin(SCK, MISO, MOSI, SS);
	LoRa.setPins(SS, RST, DI0);
	if (!LoRa.begin(BAND)) {
		Serial.println("Starting LoRa failed!");
		while (1);
	}

	LoRa.setSyncWord(0xF3);           // ranges from 0-0xFF, default 0x34, see API docs
	Serial.println("LoRa init succeeded.");

	Serial.println("init ok");
	display.init();
	display.flipScreenVertically();
	display.setFont(ArialMT_Plain_16);

	display.drawString(0, 20, "LoRa Duplex GPS");
	display.display();


	//setupTime();
	//delay(1000);

	Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
	Serial.println("Serial Txd is on pin: " + String(TXD2));
	Serial.println("Serial Rxd is on pin: " + String(RXD2));

}

void loop() {
	while (Serial2.available()) {
		//Serial.print(char(Serial2.read()));
		if (gps.encode(Serial2.read()))
			;// displayInfo();
	}

	if (digitalRead(BTN_PRESS) == HIGH)
	{
		digitalWrite(2, HIGH);
		display.clear();
		display.drawString(0, 20, "Button press");
		display.display();
		delay(1000);
		digitalWrite(2, LOW);
	}
	if (digitalRead(BTN_LEFT) == HIGH)
	{
		display.clear();
		display.drawString(0, 30, "Button left");
		display.display();
		delay(1000);
	}
	if (digitalRead(BTN_RIGHT) == HIGH)
	{
		display.clear();
		display.drawString(0, 30, "Button right");
		display.display();
		delay(1000);
	}
	if (digitalRead(BTN_UP) == HIGH)
	{
		display.clear();
		display.drawString(0, 30, "Button up");
		requestSleep++;
		display.drawString(0, 40, "Request sleep:");
		display.drawString(80, 40, String(requestSleep));
		display.display();
		delay(1000);
	}
	if (digitalRead(BTN_DOWN) == HIGH)
	{
		display.clear();
		display.drawString(0, 30, "Button down");
		if (requestSleep > 0)
			requestSleep--;
		display.drawString(0, 40, "Request sleep:");
		display.drawString(80, 40, String(requestSleep));
		display.display();
		delay(1000);
	}

	if (millis() - lastSendTime > interval) {
		if (messageOut.startsWith("OK"))
		{
		}
		else if (requestSleep >= 0)
		{
			messageOut = "sleep";
			messageOut += String(requestSleep);
		}
		else if (gps.location.isValid())
		{
			if (gps.time.isValid())
			{
				char buf[40];
				sprintf(buf, "%f,%f %02u:%02u:%02u",
					gps.location.lat(), gps.location.lng(),
					gps.time.hour(), gps.time.minute(), gps.time.second()
				);
				messageOut = String(buf);
			}
			else
			{
				messageOut = String(gps.location.lat(), 6);
				messageOut += ",";
				messageOut += String(gps.location.lng(), 6);
			}
		}
		else
		{
			messageOut = "No GPS fix";
			numSendt--;
		}
		sendMessage(messageOut);
		UpdateOLED();
		messageOut = "";
		lastSendTime = millis();            // timestamp the message
		interval = random(2000) + 10000;    // 10-12 seconds
	}
	if (millis() - lastDisplayUpdate > 1000) {
		//UpdateOLED();
	}
	// parse for a packet, and call onReceive with the result:
	onLoraReceive(LoRa.parsePacket());
	delay(10);
}
void displayInfo()
{
	Serial.print(F("Location: "));
	if (gps.location.isValid())
	{
		Serial.print(gps.location.lat(), 6);
		Serial.print(F(","));
		Serial.print(gps.location.lng(), 6);
	}
	else
	{
		Serial.print(F("INVALID"));
	}

	Serial.print(F("  Date/Time: "));
	if (gps.date.isValid())
	{
		Serial.print(gps.date.month());
		Serial.print(F("/"));
		Serial.print(gps.date.day());
		Serial.print(F("/"));
		Serial.print(gps.date.year());
	}
	else
	{
		Serial.print(F("INVALID"));
	}

	Serial.print(F(" "));
	if (gps.time.isValid())
	{
		if (gps.time.hour() < 10) Serial.print(F("0"));
		Serial.print(gps.time.hour());
		Serial.print(F(":"));
		if (gps.time.minute() < 10) Serial.print(F("0"));
		Serial.print(gps.time.minute());
		Serial.print(F(":"));
		if (gps.time.second() < 10) Serial.print(F("0"));
		Serial.print(gps.time.second());
		Serial.print(F("."));
		if (gps.time.centisecond() < 10) Serial.print(F("0"));
		Serial.print(gps.time.centisecond());
	}
	else
	{
		Serial.print(F("INVALID"));
	}

	Serial.println();
}
void sendMessage(String outgoing) {
	Serial.print(" - Sending: ");
	Serial.println(outgoing);

	LoRa.beginPacket();                   // start packet
	LoRa.print(outgoing);                 // add payload
	LoRa.endPacket();                     // finish packet and send it
	numSendt++;
	delay(500);
	if (minutesToSleep > 0 && numSendt > SLEEP_AFTER_X_SENT)
	{
		esp_sleep_enable_timer_wakeup(minutesToSleep * uS_TO_Min_FACTOR);

		Serial.println("Going to sleep now");
		Serial.flush();
		esp_deep_sleep_start();
	}
}

void onLoraReceive(int packetSize) {
	if (packetSize == 0) return;          // if there's no packet, return
	//digitalWrite(2, HIGH);				  // turn the LED on (HIGH is the voltage level)
	if (packetSize > 30)
		Serial.println("RECEIVED UNKNOWN MESSAGE! Lenght=" + String(packetSize) + ":");

	incoming = "";
	while (LoRa.available()) {
		//incoming += (char)LoRa.read();
		int chr = LoRa.read();
		incoming += (char)chr;
		if (packetSize > 30)
			Serial.println(chr, HEX);

	}
	Serial.print("Received: ");
	Serial.println(incoming);
	lastRSSI = LoRa.packetRssi();
	if (incoming.startsWith("OK sleep"))
	{
		requestSleep = -1;
	}
	else if (incoming.startsWith("sleep"))
	{
		int minutes = incoming.substring(5).toInt();
		if (minutes >= 0 && minutes <= 60 * 12)
		{
			Serial.print(minutes);
			Serial.println(" minutes");

			minutesToSleep = minutes;
			display.clear();
			display.setTextAlignment(TEXT_ALIGN_LEFT);
			display.setFont(ArialMT_Plain_16);
			display.drawString(0, 20, "Set sleep");
			display.drawString(0, 20, String(minutes));
			messageOut = "OK sleep";               // Confirm
		}
	}
	UpdateOLED();
	Serial.println("RSSI: " + String(lastRSSI));
	Serial.println("Snr: " + String(LoRa.packetSnr()));
	Serial.println();
	//digitalWrite(2, LOW);
}
String DistanceFrom(String inp)
{
	if (!gps.location.isValid())
		return String("--");

	int comma = incoming.indexOf(',');
	int space = incoming.indexOf(' ');
	String latS = inp.substring(0, comma);
	String lngS = inp.substring(comma+1, space);
	double lat = latS.toFloat();
	double lng = lngS.toFloat();

	unsigned long distanceM =
		(unsigned long)TinyGPSPlus::distanceBetween(
			gps.location.lat(), gps.location.lng(),
			lat, lng);
	double courseToTarget =
		TinyGPSPlus::courseTo(
			gps.location.lat(), gps.location.lng(),
			lat, lng);
	const char *cardinalToLondon = TinyGPSPlus::cardinal(courseToTarget);
	char buf[40];
	sprintf(buf, "TD: %dm %s %0.0f", distanceM, cardinalToLondon, courseToTarget);


	Serial.print("TD:");
	Serial.print(distanceM);
	Serial.print("meter CTT:");
	Serial.println(courseToTarget);
	Serial.println(gps.location.isValid() ? cardinalToLondon : "*** ");
	return String(buf);
}

void UpdateOLED(void)
{
	display.clear();
	display.setTextAlignment(TEXT_ALIGN_LEFT);
	display.setFont(ArialMT_Plain_10);

	char time_buf[40];
	if (gps.date.isValid() && gps.time.isValid())
	{
		sprintf(time_buf, "%02u/%02u %02u:%02u:%02u",
			gps.date.day(), gps.date.month(),
			gps.time.hour(), gps.time.minute(), gps.time.second()
		);
		display.drawString(0, 10, time_buf);
	}
	else
	{
		display.drawString(0, 0, "No GPS data");
	}

	unsigned int len = incoming.length();
	if (len > 15)
	{
		display.drawString(0, 30, DistanceFrom(incoming));
	}
	else if (len > 0)
		display.drawString(0, 30, incoming);
	else if (minutesToSleep > 0)
	{
		display.drawString(0, 30, "Sleep=");
		display.drawString(50, 30, String(minutesToSleep));
	}
	//display.setFont(ArialMT_Plain_10);
	display.drawString(0, 40, "Out:");
	display.drawString(0, 50, messageOut);
	display.display();
	lastDisplayUpdate = millis();
}
