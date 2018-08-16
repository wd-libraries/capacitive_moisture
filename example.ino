#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h> // don't forget to install the MQTT library

#define SOIL_THRESHOLD_LEVEL 500 // delete this line and replace with your calibrated measurements for each sensor
#define SOIL_READING_INTERVAL 60 // In seconds. Check the soil once per minute.
#define MAX_RELAY_DURATION 10 // In seconds. We'll allow the relay to close for for a maximum of ten seconds.
// note: MAX_RELAY_DURATION should be < SOIL_READING_INTERVAL
#define ENABLE_PIN D7 // the EN pin on the mux

#define WIFI_SSID "your_ssid"
#define WIFI_PASSWORD "your_pw"
#define MQTT_SERVER "mqtt.wombatdashboard.com"
#define MQTT_PORT 8883
#define TLS_FINGERPRINT "replace_this" // see wombatdashboard.com/tutorials/tls
#define DEVICE_PRIVATE_IDENTIFIER "your_private_device_id"
#define DEVICE_PUBLIC_IDENTIFIER "your_public_device_id"

#define IRRIGATE_COMMAND_RELAY1 "your_command1"
#define IRRIGATE_COMMAND_RELAY2 "your_command2"

const char* fingerprint = TLS_FINGERPRINT;

// declare sensor and relay types
struct Relay {
    char name[8]; // max 7 chars per name + null termination
    uint8_t pin;  // the connection to ESP-12E ditigal IO pin number
    uint16_t counter; // how long the relay has been closed for, in seconds. 0 means open.
    char command[10]; // for if you want to close using a MQTT message from the WombatDashboard interface
};
struct Sensor {
    char name[8]; // max 7 chars per name + null termination
    uint8_t muxPins;  // the bits used to select the sensor through the mux
    Relay *relay; // pointer to associated relay to activate if soil is too dry
    uint16_t soilThreshold; // probe reading when soil is too dry
};

// declare all our global variables
Relay r1, r2;
Relay relays[2] = {r1, r2};
Sensor s1 = {"probe1", 0b00000000, &r1, SOIL_THRESHOLD_LEVEL}; 
Sensor s2 = {"probe2", 0b00000001, &r1, SOIL_THRESHOLD_LEVEL}; 
Sensor s3 = {"probe3", 0b00000010, &r2, SOIL_THRESHOLD_LEVEL}; 
Sensor s4 = {"probe4", 0b00000011, &r2, SOIL_THRESHOLD_LEVEL}; 
Sensor sensors[4] = {s1, s2, s3, s4};

// N pins are required to represent 2^N sensors through the mux
uint8_t muxPins[2] = {D1, D2};
uint8_t nb_pins = 2;

// we'll keep count of our main loop using this variable
uint16_t loop_counter = 0;

// clients for internet connectivity
WiFiClientSecure wc;
PubSubClient client(wc);

/*
 * Utility function to handle connecting to wifi
 */
void setupWiFi(void) {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting");
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print("...waiting");
    }
    Serial.println();
    WiFi.printDiag(Serial);
}

/*
 * Utility function for verifying our secure connection
 */
 bool verifyFingerprint(void) {
    const char* host = MQTT_SERVER;
    if (!wc.connect(host, MQTT_PORT)) {
        return false; // connection failed
    }
    if (wc.verify(fingerprint, host)) {
        return true;
    } else {
        return false; // fingerprint verification failed
    }
}

/*
 * Utility function to handle reconnecting to MQTT server
 */
void reconnect(void) {
    // Loop until we're reconnected
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        if (client.connect(DEVICE_PUBLIC_IDENTIFIER)) { // Attempt to connect
            Serial.println("connected");
            client.subscribe(DEVICE_PRIVATE_IDENTIFIER);
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}

/* 
 * Sets mux input pins high corresponding to bits representing the mux channel to select
 */
void convert_bits_to_pins(uint8_t pins) {
    // limit 4 pins max (for 16-channel mux)
    for(uint8_t i=0; i<4; i++) {
        uint8_t bitmask = 0b00000001 << i; // create the mask for the current pin
        if(pins & bitmask) { // if the bit was HIGH in current position
            digitalWrite(muxPins[i], HIGH);
        } else {
            digitalWrite(muxPins[i], LOW);
        }
    }
}

/*
 * Helper function to determine if two char arrays have the same contents
 * Used instead of strcmp to avoid the memory penalty of the cstring library
 */
bool stringcompare(char a[],char b[]){
    for(uint8_t i=0; a[i] != '\0' || b[i] != '\0'; i++) {
        if(a[i] != b[i]) return false;
    }
    return true;
}

/*
 * Callback function for processing incoming MQTT messages
 * Don't forget to null-terminate any payload string you send to your device
 */
void callback(char* topic, byte* payload, unsigned int length) {
    if(!stringcompare(topic, DEVICE_PRIVATE_IDENTIFIER)) return;
    for(Relay r: relays) {
        if(stringcompare(r.command, (char *)payload)) {
            digitalWrite(r.pin, HIGH);
            // start the relay counter
            r.counter = 1;
            char message[12];
            sprintf(message, "{\"%s\":1}", r.name); // json to indicate that we closed relay
            client.publish(DEVICE_PRIVATE_IDENTIFIER, message);
        }     
    }
}

/*
 * The setup function is run once after power-on by the bootloader
 */
void setup(void) {

    r1 = {NULL, D5, 0, NULL};
    sprintf(r1.name, "relay1");
    sprintf(r1.command, IRRIGATE_COMMAND_RELAY1);
    r2 = {NULL, D6, 0, NULL};
    sprintf(r2.name, "relay2");
    sprintf(r2.command, IRRIGATE_COMMAND_RELAY2);
  
    // set up the relevant digital pins to output mode
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, HIGH); // disactivate the mux inputs
    for(Relay r: relays) {
        pinMode(r.pin, OUTPUT);
    }
    for(auto p: muxPins) {
        pinMode(p, OUTPUT);
    }

    setupWiFi();
    if(!verifyFingerprint()) {
        Serial.println("SHA1 fingerprint doesn't match.");
        abort();
    }
    client.setServer("mqtt.wombatdashboard.com", 1883);
    client.setCallback(callback);
    client.subscribe(DEVICE_PRIVATE_IDENTIFIER);

    Serial.begin(9600); // don't forget to set the baud rate to 9600 in your serial monitor
}

/*
 * The loop function is run continuously after setup
 */
void loop(void) {
    // handle connectivity checking to MQTT broker
    if (!client.connected()) {
        reconnect();
    }
    client.loop(); // this maintains communication with the MQTT server

    if(loop_counter > SOIL_READING_INTERVAL) {
        for(Sensor s: sensors) {
            convert_bits_to_pins(s.muxPins); // convert binary number i into pin representation for mux
            delay(2); // allow all the transistors between MCU and sensor time to trigger
            digitalWrite(ENABLE_PIN, LOW); // activate the mux
            delay(2);
            uint16_t level = analogRead(A0); // read the sensor's value
            digitalWrite(ENABLE_PIN, HIGH); // disactivate the mux
            if(level > s.soilThreshold) {
                digitalWrite(s.relay->pin, HIGH);
                // start the relay counter
                s.relay->counter = 1;
                char message[12];
                sprintf(message, "{\"%s\":1}", s.relay->name);
                client.publish(DEVICE_PRIVATE_IDENTIFIER, message, false);
                char serialMessage[40];
                sprintf(serialMessage, "%s level = %d, triggered %s", s.name, level, s.relay->name);
                Serial.println(serialMessage);
            } else {
                char serialMessage[24];
                sprintf(serialMessage, "%s level = %d", s.name, level);
                Serial.println(serialMessage);
            }
        }
        loop_counter = 0; // reset the loop counter

    } else { // otherwise, if it's not time to take another soil reading yet
        loop_counter++;
        // traverse the relays to increment the counters for any acive relays
        for(Relay r : relays) {
            if(r.counter != 0) r.counter++; // increment counter for current relay
            if(r.counter > MAX_RELAY_DURATION) {
                // open the relay if it has been closed for longer than MAX_RELAY_DURATION
                digitalWrite(r.pin, LOW);
                r.counter = 0;
                // construct json message to record that relay was switched off
                char message[12];
                sprintf(message, "{\"%s\":0}", r.name);
                client.publish(DEVICE_PRIVATE_IDENTIFIER, message, false);
            }
        }
    }
    delay(1000); // delay a second
}
