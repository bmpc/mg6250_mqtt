/*
  Paradox MG6250 serial to MQTT for the ESP8266 microcontroller.

  Reads messages sent from the MG6250 console and sends the following events to an MQTT broker:
   * Alarm arming, disarming and triggering
   * Zone open and close

  Sending messages to the console is not supported.

  This script is assuming that the serial messages are already decrypted.
  Encryption can be disabled on the MG62 console direclty. Refer to the console's section programming manual.

  This script uses the D7 pin as RX. Pins 2 and 3 are used for debuging with SoftawreSerial.

  The following variables must be replaced:
    - <WIFI_SSID>
    - <WIFI_PWD>
    - <MQTT_IP>
    - <MQTT_user>
    - <MQTT_pwd>
*/

#include <stdio.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>

#define DEBUG 1 // debug on USB Serial

const char* ssid = "<WIFI_SSID>";
const char* password = "<WIFI_PWD>";

const char* mqtt_broker = "<MQTT_IP>";
const char* mqtt_user = "<MQTT_user>";
const char* mqtt_pwd = "<MQTT_pwd>";

const long BAUDRATE = 57600; // initialize serial communication at 57600 bits per second

const int CTR_BYTES = 4; // number of control bytes in a message

const int EVENT_BYTES = 37;

const byte ZONE_OK_EVENT = 0;
const byte ZONE_OPEN_EVENT = 1;
const byte PARTITION_STATUS_EVENT = 2;

Stream* logger;
#ifdef DEBUG
  #define debug_println(x) logger->println(x)
  #define debug_print(x) logger->print(x)
#else
  #define debug_println(x)
  #define debug_print(x)
#endif

/**
 * Supported Events.
 */
enum EventDef
{
  DISARM_PARTITION = 0x0B,
  ARM_PARTITION = 0x0C,
  ALARM_STOPPED = 0x07,
  SILENT_ALARM = 0x02,
  BUZZER_ALARM = 0x03,
  STEADY_ALARM = 0x04,
  PULSE_ALARM = 0x05
};

struct Event
{
  byte seq;
  byte unknown8;
  byte unknown9;
  byte unknown10;
  byte unknown11;
  byte unknown12;
  byte event;
  byte sub_event;
  byte area;
  byte century;
  byte year;
  byte month;
  byte day;
  byte hour;
  byte minute;
  byte unknown13;
  byte unknown14;
  byte unknown15;
  byte unknown16;
  byte unknown17;
  byte typed;
  char label[17];
};

struct Message
{
   byte unknown1;
   byte unknown2;
   byte unknown3;
   byte length;
   byte unknown4;
   byte unknown5;
   byte unknown6;
   struct Event events[10]; // more that enough ... I think
   byte checksum1;
   byte checksum2;
   unsigned int event_count;
};

struct Message message;

WiFiClient espClient;
PubSubClient client(espClient);

void setup_wifi() {
  delay(100);
  // We start by connecting to the WiFi network
  debug_println();
  debug_print("Connecting to ");
  debug_println(ssid);

  WiFi.setAutoConnect(true);
  WiFi.setAutoReconnect(true);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    debug_print(".");
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
  }

  randomSeed(micros());

  debug_println("");
  debug_println("WiFi connected");
  debug_print("IP address: ");
  debug_println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  debug_print("MQTT Message arrived [");
  debug_print(topic);
  debug_print("] ");
  for (int i = 0; i < length; i++) {
    debug_print((char)payload[i]);
  }
  debug_println();

}

void serial_flush() {
  while(Serial.available() > 0) {
    Serial.read();
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(BAUDRATE);
  Serial.swap(); // use D7 as RX; default serial pin2 (1,3) was not working for some reason

  // using HardwareSerial0 pins, so we can still log to the regular usbserial chips
  SoftwareSerial* ss = new SoftwareSerial(3, 1, false);
  ss->begin(115200);
  ss->enableIntTx(false);
  logger = ss;

  setup_wifi();

  client.setServer(mqtt_broker, 1883);
  client.setCallback(callback);

  connect();

  debug_println("Ready.");

  serial_flush();
}

// forever and ever baby
void loop() {
  if (Serial.available() > 0) {
    debug_println("Receiving message...");

    int r = read();
    if (r == 0) { // message is OK
      #ifdef DEBUG
      debugMessage();
      #endif

      processMessage();

      reportSuccess();
    } else { // timeout or invalid message
      debug_println("Invalid message!");

      serial_flush();

      reportError();
    }

    message = Message(); // reset message
  }
}

void reportSuccess() {
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH);
}

void reportError() {
  unsigned long startMillis = millis();
  while (millis() - startMillis < 3000) {
    digitalWrite(LED_BUILTIN, LOW);
    delay(50);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
  }
}

void connect() {
  // Loop until we're connected. Max retries: 5
  uint8_t retries = 0;
  while (!client.connected() && retries < 5) {
    retries++;
    debug_println("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_pwd)) {
      debug_println("connected");
    } else {
      debug_print("failed, rc=");
      debug_print(client.state());
      debug_println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

int read() {
  // read control bytes (4)
  int ctr_i = 0;
  byte ctr_bytes[CTR_BYTES];
  unsigned long ctrStartMillis = millis();
  while (ctr_i < CTR_BYTES) {
    if (Serial.available() > 0) {
      *(ctr_bytes + (ctr_i++)) = Serial.read();
    }
    if (millis() - ctrStartMillis > 1000) {
      debug_println("Timeout while reading control bytes!");
      return -1; // timeout
    }
  }

  int length = ctr_bytes[3];

  // read remaining bytes
  byte bytes[length];
  bytes[0] = ctr_bytes[0];
  bytes[1] = ctr_bytes[1];
  bytes[2] = ctr_bytes[2];
  bytes[3] = ctr_bytes[3];

  int remainingBytes = length - CTR_BYTES;

  int i = 4; // we already have the first 4 bytes

  unsigned long startMillis = millis();
  while (remainingBytes > 0) {
    if (Serial.available()) {
      *(bytes + (i++)) = Serial.read();
      --remainingBytes;
    }
    if (millis() - startMillis > 1000) {
      debug_println("Timeout while reading message event bytes!");
      return -1; // timeout
    }
  }

  // verify crc
  if (!verify_crc(bytes, length)) {
    return -2;
  }

  // create message
  createMessage(bytes);

  return 0;
}

bool verify_crc(byte bytes[], int length) {
  int calc = calc_crc(bytes, length - 2);

  // little endian
  int high = bytes[length - 1];
  int low = bytes[length - 2];
  unsigned int expected = low | (high << 8);

  if (expected != calc) {
    char buffer[128];
    snprintf(buffer, sizeof(buffer), "CRC check failed! Calc: %d Expected: %d", calc, expected);
    debug_println(buffer);
  }

  return expected == calc;
}

// CRC-16/MODBUS
unsigned int calc_crc(byte bytes[], int length) {
  unsigned int crc = 0xFFFF;
  for( unsigned int i = 0; i < length; i++ ) {
    crc ^= bytes[i];
    for( unsigned int j = 0; j < 8; j++ ) {
      if ((crc & 1) != 0) {
          crc >>= 1;
          crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }

  return crc;
}

void createMessage(byte bytes[]) {
  message.unknown1 = bytes[0];
  message.unknown2 = bytes[1];
  message.unknown3 = bytes[2];
  int length = message.length = bytes[3];
  message.unknown4 = bytes[4];
  message.unknown5 = bytes[5];
  message.unknown6 = bytes[6];
  message.checksum1 = bytes[length - 1];
  message.checksum2 = bytes[length];

  // Events
  int events_length = bytes[3] - 9;
  unsigned int event_count = message.event_count = int(events_length / EVENT_BYTES);

  byte *events_arr = bytes + 7;
  for (unsigned int ev = 0; ev < event_count; ev++) {
    byte event_bytes[EVENT_BYTES];
    for (unsigned int i = 0; i < EVENT_BYTES; i++) {
      event_bytes[i] = *(events_arr + i + (EVENT_BYTES * ev));
    }
    message.events[ev] = createEvent(event_bytes);
  }
}

Event createEvent(byte bytes[]) {
  Event e;
  e.seq = bytes[0];
  e.unknown8 = bytes[1];
  e.unknown9 = bytes[2];
  e.unknown10 = bytes[3];
  e.unknown11 = bytes[4];
  e.unknown12 = bytes[5];
  e.event = bytes[6];
  e.sub_event = bytes[7];
  e.area = bytes[8];
  e.century = bytes[9];
  e.year = bytes[10];
  e.day = bytes[11];
  e.hour = bytes[12];
  e.minute = bytes[13];
  e.unknown13 = bytes[14];
  e.unknown14 = bytes[15];
  e.unknown15 = bytes[16];
  e.unknown16 = bytes[17];
  e.unknown17 = bytes[18];
  e.typed = bytes[19];

  // label
  for (unsigned int i = 0; i < 16; i++) {
    e.label[i] = bytes[i + 20];
  }
  e.label[16] = '\0';

  return e;
}

void debugMessage() {
  char buffer[128];
  snprintf(buffer, sizeof(buffer), "Received message with %d events.", message.event_count);
  debug_println(buffer);
  for (unsigned short i = 0; i < message.event_count; i++) {
    Event e = message.events[i];
    char buffer2[128];
    snprintf(buffer2, sizeof(buffer2), "[%d-%d-%d %d:%d] Event: %d; Sub-Event: %d; Area: %d; Label %s", e.year, e.month, e.day, e.hour, e.minute, e.event, e.sub_event, e.area, e.label);
    debug_println(buffer2);
  }
}

void processMessage() {
  for (unsigned short i = 0; i < message.event_count; i++) {
    Event e = message.events[i];
    if (e.event == PARTITION_STATUS_EVENT) {
      switch(e.sub_event)
      {
        case DISARM_PARTITION:
          sendMQTTevent("home/alarm", "disarmed");
          break;
        case ARM_PARTITION:
        case ALARM_STOPPED:
          sendMQTTevent("home/alarm", "armed_away");
          break;
        case SILENT_ALARM:
        case BUZZER_ALARM:
        case STEADY_ALARM:
        case PULSE_ALARM:
          sendMQTTevent("home/alarm", "triggered");
          break;
      }
    } else if (e.event == ZONE_OPEN_EVENT) { // also register zone open events for statistics
      sendMQTTevent("home/alarm/zone_open", e.label);
    } else if (e.event == ZONE_OK_EVENT) {
      sendMQTTevent("home/alarm/zone_ok", e.label);
    }
  }
}

void sendMQTTevent(char* topic, char* body) {
  if (!client.connected()) {
    connect();
  }
  debug_println("Sending MQTT event...");
  client.publish(topic, body);
}
