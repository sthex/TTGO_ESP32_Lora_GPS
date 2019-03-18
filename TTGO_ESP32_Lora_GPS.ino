//#include <WiFi.h>
#include <time.h>		
#include <sys/time.h>		
#include <SPI.h>		// Serial communication used by Lora
#include <LoRa.h>
#include <Wire.h>		
#include <SSD1306.h>	// OLED display
#include <TinyGPS++.h>  // http://arduiniana.org/libraries/tinygpsplus/
#include <Ticker.h>

//#include "GPSCode.x"

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
#define GPS Serial2 // Hardware serial port for GPS 
TinyGPSPlus gps;
byte gps_set_sucess = 0;

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
String lastRxMsg;
String lastRxDist;
String lastRxTime;
bool _isTimeSet = false;


//#define TZ_INFO "MST7MDT6,M3.2.0/02:00:00,M11.1.0/02:00:00" //"America/Denver" 
#define TZ_UTZ "UTC0UTC" //"UTC" 
#define TZ_LOCAL "CET-1CEST,M3.5.0/2,M10.5.0/3" //"Central European Time" 
// see http://www.gnu.org/software/libc/manual/html_node/TZ-Variable.html
#define PRINT_DELAY 5 //seconds

//#define POSBUFLEN 100
//double latBuf[POSBUFLEN];
//double lngBuf[POSBUFLEN];
//int posPos = 0;
//bool bufFilled = false;

Ticker time_print;

void setTimeFromGps() {
	if (gps.time.isValid() && gps.date.isValid())
	{
		setenv("TZ", TZ_UTZ, 1);
		tzset(); // Assign UTC timezone from setenv

		struct tm time_gps_tm; // 
		time_gps_tm.tm_year = gps.date.year() - 1900;// time since 1900
		time_gps_tm.tm_mon = gps.date.month() - 1; // months sice January
		time_gps_tm.tm_mday = gps.date.day(); //day of the month
		time_gps_tm.tm_hour = gps.time.hour(); // hours
		time_gps_tm.tm_min = gps.time.minute(); // minutes
		time_gps_tm.tm_sec = gps.time.second(); // seconds
		time_t epoch_time = mktime(&time_gps_tm);
		Serial.println(String(gps.time.hour()));
		Serial.println(&time_gps_tm, "== Set time from GPS: %B %d %Y %H:%M:%S (%A)");
		Serial.flush();

		timeval epoch = { epoch_time, 0 };
		const timeval *tv = &epoch;
		timezone utc = { 0,0 };
		const timezone *tz = &utc;
		settimeofday(tv, tz);  //Set time
		_isTimeSet = true;
	}
}


time_t tmConvert_t(int YYYY, byte MM, byte DD, byte hh, byte mm, byte ss)
{
	struct tm tmSet;
	tmSet.tm_year = YYYY - 1900;
	tmSet.tm_mon = MM;
	tmSet.tm_mday = DD;
	tmSet.tm_hour = hh;
	tmSet.tm_min = mm;
	tmSet.tm_sec = ss;
	return mktime(&tmSet);
}

//void bufPos()
//{
//	if (gps.location.isValid())
//	{
//		latBuf[posPos] = gps.location.lat();
//		lngBuf[posPos] = gps.location.lng();
//		if (++posPos >= POSBUFLEN)
//		{
//			posPos = 0;
//			bufFilled = true;
//		}
//	}
//}
//double Lat()
//{
//	if (bufFilled)
//	{
//		double ret = 0;
//		for (int i = 0; i < POSBUFLEN; i++)
//			ret += latBuf[i];
//		ret /= POSBUFLEN;
//		Serial.print(" Lat ");
//		Serial.print( ret,6);
//		Serial.print(" Raw ");
//		Serial.println(gps.location.lat(),6);
//		return ret;
//	}
//	else if (gps.location.isValid())
//		return gps.location.lat();
//	return 0.0;
//}
//double Lng()
//{
//	if (bufFilled)
//	{
//		double ret = 0;
//		for (int i = 0; i < POSBUFLEN; i++)
//			ret += lngBuf[i];
//		ret /= POSBUFLEN;
//	/*	Serial.print(" Lng ");
//		Serial.println(ret,6);
//		Serial.print(" Raw ");
//		Serial.println(gps.location.lng(),6);*/
//		return ret;
//	}
//	else if (gps.location.isValid())
//		return gps.location.lng();
//	return 0.0;
//}

void timePrint() {
	Serial.println("________________");
	struct tm now;
	getLocalTime(&now, 0);
	Serial.println(&now, "%B %d %Y %H:%M:%S (%A)"); // May 25 1970 21:42:31 (Monday)
}
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

	display.init();
	display.flipScreenVertically();
	display.setFont(ArialMT_Plain_16);

	display.drawString(0, 20, "LoRa Duplex GPS");
	display.display();

	Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
	//while (!Serial2);
	//GpsDynamicMode(3);
	//time_print.attach(PRINT_DELAY, timePrint); // Schedule the timeprint function
}

void loop() {
	while (Serial2.available()) {
		//Serial.print(char(Serial2.read()));
		if (gps.encode(Serial2.read()))
			;// debugInfo();
	}
	if (_isTimeSet == false)
		setTimeFromGps();

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
		display.drawString(0, 50, "GPS low power");
		//GpsSleep(); //64mA, normal >100mA
		//GpsOff();  // 62mA, red led on
		GpsPowerLow(); // ~70mA (55mA is ESP)
		//GpsPeriod10A(); // ~68mA 
		display.display();
		delay(1000);
	}
	if (digitalRead(BTN_RIGHT) == HIGH)
	{
		display.clear();
		display.drawString(0, 30, "Button right");
		//display.drawString(0, 50, "GPS On");
		//GpsOn();
		display.drawString(0, 50, "GPS full power");
		GpsPowerFull();
		//GpsPeriod10B(); // ~68mA 
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

	////incoming = String("59.912017,10.580750 21:14:38");
	if (millis() - lastSendTime > interval) {
		if (messageOut.startsWith("OK"))
		{
			// Send this ackowledge message to other Lora unit
		}
		else if (requestSleep >= 0)
		{
			// send command 'sleep<Min>' to other Lora unit 
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
		UpdateOLED();
	}
	// parse for a packet, and call onReceive with the result:
	onLoraReceive(LoRa.parsePacket());
	delay(10);
}

void debugInfo()
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
	if (gps.satellites.isValid())
	{
		Serial.print(F(" Sat:"));
		Serial.print(gps.satellites.value());
	}
	if (gps.hdop.isValid())
	{
		Serial.print(F(" Hdop:"));
		Serial.print(gps.hdop.hdop());
	}
	{
		Serial.print(F(" Fail:"));
		Serial.print(gps.failedChecksum());
		Serial.print(F(" of "));
		Serial.print(gps.passedChecksum());
	}
	//Serial.print(F("  Date/Time: "));
	//if (gps.date.isValid())
	//{
	//	Serial.print(gps.date.month());
	//	Serial.print(F("/"));
	//	Serial.print(gps.date.day());
	//	Serial.print(F("/"));
	//	Serial.print(gps.date.year());
	//}
	//else
	//{
	//	Serial.print(F("INVALID"));
	//}

	//Serial.print(F(" "));
	//if (gps.time.isValid())
	//{
	//	if (gps.time.hour() < 10) Serial.print(F("0"));
	//	Serial.print(gps.time.hour());
	//	Serial.print(F(":"));
	//	if (gps.time.minute() < 10) Serial.print(F("0"));
	//	Serial.print(gps.time.minute());
	//	Serial.print(F(":"));
	//	if (gps.time.second() < 10) Serial.print(F("0"));
	//	Serial.print(gps.time.second());
	//	Serial.print(F("."));
	//	if (gps.time.centisecond() < 10) Serial.print(F("0"));
	//	Serial.print(gps.time.centisecond());
	//}
	//else
	//{
	//	Serial.print(F("INVALID"));
	//}

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
	digitalWrite(2, HIGH);				  // turn the LED on (HIGH is the voltage level)
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
	Serial.print("\n* Received: ");
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
			minutesToSleep = minutes;
			messageOut = "OK sleep";               // Confirm in next tranmit
		}
	}
	UpdateOLED();
	Serial.println("RSSI: " + String(lastRSSI));
	Serial.println("Snr: " + String(LoRa.packetSnr()));
	Serial.println();
	digitalWrite(2, LOW);
}
String DistanceFrom(String inp)
{
	if (!gps.location.isValid())
		return String("--");

	int comma = incoming.indexOf(',');
	int space = incoming.indexOf(' ');
	String latS = inp.substring(0, comma);
	String lngS = inp.substring(comma + 1, space);
	String time = inp.substring(space + 1);
	uint8_t hour = time.substring(0, 2).toInt() & 0xFF;
	uint8_t min = time.substring(3, 5).toInt() & 0xFF;
	uint8_t sec = time.substring(6, 8).toInt() & 0xFF;

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

	String deltatime = "?";
	{
		struct tm now;
		getLocalTime(&now, 0);

		int dh = now.tm_hour - hour;
		int dm = now.tm_min - min;
		int ds = now.tm_sec - sec;

		if (dh != 0)
		{
			deltatime = String(hour) + ":" + String(min);
		}
		else
		{
			int mytime = 60 * now.tm_min + now.tm_sec;
			int histime = 60 * min + sec;
			int dtime = mytime - histime;

			if (dtime > 3 * 60)
			{
				deltatime = String(dtime / 60, 2) + "m";
			}
			else
			{
				deltatime = String(dtime) + "s";
			}
		}
	}

	const char *cardinalToLondon = TinyGPSPlus::cardinal(courseToTarget);
	char buf[80];
	sprintf(buf, "TD: %dm %s %0.0f %s", distanceM, cardinalToLondon, courseToTarget, deltatime.c_str());


	Serial.print("TD:");
	Serial.print(distanceM);
	Serial.print("meter CTT:");
	Serial.print(courseToTarget);
	Serial.print(" ");
	Serial.println(gps.location.isValid() ? cardinalToLondon : "*** ");
	//Serial.print("Time of remote fix: ");
	//Serial.println(time);
	return String(buf);
}

void UpdateOLED(void)
{
	display.clear();
	display.setTextAlignment(TEXT_ALIGN_LEFT);
	display.setFont(ArialMT_Plain_10);

	char buf[40];
	struct tm now;
	getLocalTime(&now, 0);
	sprintf(buf, "%02u:%02u:%02u", now.tm_hour, now.tm_min, now.tm_sec);
	display.drawString(0, 0, buf);

	/*if (gps.date.isValid() && gps.time.isValid())
	{
		sprintf(time_buf, "%02u:%02u:%02u  %02u/%02u",
			gps.time.hour(), gps.time.minute(), gps.time.second(),
			gps.date.day(), gps.date.month()
		);
		display.drawString(0, 0, time_buf);
	}
	else
	{
		display.drawString(0, 0, "No time");
	}*/

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

	display.drawString(0, 54, "Out:");
	if (messageOut.length() > 20)
		display.drawString(30, 54, "'my pos'");
	else
		display.drawString(30, 54, messageOut);
	display.display();
	lastDisplayUpdate = millis();
}


//ref. https://ukhas.org.uk/guides:ublox_psm
// and https://ukhas.org.uk/guides:ublox6
//Set GPS to backup mode (sets it to never wake up on its own)
// 62mA, red led on
void GpsOff()
{
	Serial.println("gps GpsOff.");
	uint8_t GPSoff[] = { 0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x4D, 0x3B };
	sendUBX(GPSoff, sizeof(GPSoff) / sizeof(uint8_t));
}
//Start GPS after GpsOff
void GpsOn()
{
	Serial.println("gps GpsOn.");
	uint8_t GPSon[] = { 0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x4C, 0x37 };
	sendUBX(GPSon, sizeof(GPSon) / sizeof(uint8_t));
}

//Switch the RF GPS section off, draws about 5mA, retains its settings, wakes on serial command. 
//64mA, normal >100mA
void GpsSleep()
{
	Serial.println("gps GpsSleep.");
	uint8_t GPSoff[] = { 0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00,0x08, 0x00, 0x16, 0x74 };
	sendUBX(GPSoff, sizeof(GPSoff) / sizeof(uint8_t));
}

//Turn GPS RF Section on 
void GpsWakeup()
{
	Serial.println("gps GpsWakeup.");
	uint8_t GPSon[] = { 0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00,0x09, 0x00, 0x17, 0x76 };
	sendUBX(GPSon, sizeof(GPSon) / sizeof(uint8_t));
}

//Setup for Power Save Mode (Default Cyclic 1s)
//Setting power saving with the module that has a lock with only 4 satellites can lead to unstable performance.
//Its advise you only engage power saving mode when you have more than 5 satellites, and add delay after.
// ~70mA (55mA is ESP)
void GpsPowerLow()
{
	Serial.println("gps GpsPowerLow.");
	uint8_t setPSM[] = { 0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22, 0x92 };
	sendUBX(setPSM, sizeof(setPSM) / sizeof(uint8_t));
}
//Max Performance Mode (default)
void GpsPowerFull()
{
	Serial.println("gps GpsPowerFull.");
	uint8_t setPSM[] = { 0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x00, 0x21, 0x91 };
	sendUBX(setPSM, sizeof(setPSM) / sizeof(uint8_t));
}

//Dynamic Model '3' Pedestrian Small Deviation
//Much more accurate position and works below 9000 meters altitude and as long as the
//vertical velocity < 20m / s and horizontal velocity < 30 / ms.
// 2=stationary, 3=pedestrian, 4=auto, 5=Sea, 6=airborne 1g, 7=air 2g, 8=air 4g
// see. https://github.com/Kaasfabriek/GPS-Lora-Balloon-rfm95-TinyGPS/blob/master/Balloon-rfm95/Balloon-rfm95.ino
void GpsDynamicMode(uint8_t mode)
{
	Serial.print("gps GpsDynamicMode ");
	Serial.print(mode);
	Serial.println(".");
	uint8_t setPSM[] = { 0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, mode, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0x76 };
	sendUBX(setPSM, sizeof(setPSM) / sizeof(uint8_t));
}

//Update Period 10 seconds, do not enter 'inactive for search' state when no fix checked
void GpsPeriod10A()
{
	Serial.println("gps GpsPeriod10A.");
	uint8_t setPSM[] = { 0xB5, 0x62, 0x06, 0x3B, 0x2C, 0x00, 0x01, 0x06, 0x00, 0x00, 0x00, 0x90, 0x03, 0x00, 0x10, 0x27, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x4F, 0xC1, 0x03, 0x00, 0x87, 0x02, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x64, 0x40, 0x01, 0x00, 0xE4, 0x8B };
	sendUBX(setPSM, sizeof(setPSM) / sizeof(uint8_t));
}

//Update Period 10 seconds, do not enter 'inactive for search' state when no fix unchecked
void GpsPeriod10B()
{
	Serial.println("gps GpsPeriod10B.");
	uint8_t setPSM[] = { 0xB5, 0x62, 0x06, 0x3B, 0x2C, 0x00, 0x01, 0x06, 0x00, 0x00, 0x00, 0x90, 0x02, 0x00, 0x10, 0x27, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x4F, 0xC1, 0x03, 0x00, 0x86, 0x02, 0x00, 0x00, 0xFE, 0x00, 0x00, 0x00, 0x64, 0x40, 0x01, 0x00, 0xE1, 0x51 };
	sendUBX(setPSM, sizeof(setPSM) / sizeof(uint8_t));
}

// this factory reset erases all and wipes usefull info
void gps_requestColdStart() {
	Serial.println("gps Request Cold Start.");
	uint8_t setPSM[] = { 0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0xFF, 0x87, 0x00, 0x00, 0x94, 0xF5 };
	sendUBX(setPSM, sizeof(setPSM) / sizeof(uint8_t));
}

// Send a byte array of UBX protocol to the GPS and ack
void sendUBX(uint8_t *MSG, uint8_t len) {
	int retry = 10;
	while (!gps_set_sucess && --retry > 0)
	{
		for (int i = 0; i < len; i++) {
			GPS.write(MSG[i]);
		}
		GPS.println();
		gps_set_sucess = getUBX_ACK(MSG);
	}
	gps_set_sucess = 0;
}
// Calculate expected UBX ACK packet and parse UBX response from GPS
boolean getUBX_ACK(uint8_t *MSG) {
	uint8_t b;
	uint8_t ackByteID = 0;
	uint8_t ackPacket[10];
	unsigned long startTime = millis();
	Serial.print(" * Reading ACK response: ");

	// Construct the expected ACK packet    
	ackPacket[0] = 0xB5;	// header
	ackPacket[1] = 0x62;	// header
	ackPacket[2] = 0x05;	// class
	ackPacket[3] = 0x01;	// id
	ackPacket[4] = 0x02;	// length
	ackPacket[5] = 0x00;
	ackPacket[6] = MSG[2];	// ACK class
	ackPacket[7] = MSG[3];	// ACK id
	ackPacket[8] = 0;		// CK_A
	ackPacket[9] = 0;		// CK_B

	// Calculate the checksums
	for (uint8_t i = 2; i < 8; i++) {
		ackPacket[8] = ackPacket[8] + ackPacket[i];
		ackPacket[9] = ackPacket[9] + ackPacket[8];
	}

	while (1) {

		// Test for success
		if (ackByteID > 9) {
			// All packets in order!
			Serial.println(" (SUCCESS!)");
			return true;
		}

		// Timeout if no valid response in 3 seconds
		if (millis() - startTime > 3000) {
			Serial.println(" (FAILED!)");
			return false;
		}

		// Make sure data is available to read
		if (GPS.available()) {
			b = GPS.read();

			// Check that bytes arrive in sequence as per expected ACK packet
			if (b == ackPacket[ackByteID]) {
				ackByteID++;
				Serial.print(b, HEX);
			}
			else {
				ackByteID = 0;	// Reset and look again, invalid order
			}

		}
	}
}