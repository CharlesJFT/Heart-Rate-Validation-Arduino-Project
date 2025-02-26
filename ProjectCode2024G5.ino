#include <ESP8266WiFi.h> //ESP8266 wifi library
#include <Wire.h> //wire library for I2C connection with MAX30101
#include <AdafruitIO_WiFi.h> //Adafruit IO library for API connectiona and display
#include "MAX30105.h" // MAX sensor library, this also works for the MAX30101
#include "spo2_algorithm.h" //Algorithm to get Sp02 measurements from the MAX30101
#include <PulseSensorPlayground.h> //Pulse sensor function library

// Adafruit IO Configuration
#define IO_USERNAME    "Your IO username here"  // Adafruit username definition
#define IO_KEY         "put IO key here" // Adafruit password definition

// WiFi credentials
#define WIFI_SSID "Cisco54191" //Wifi SSID definition 
#define WIFI_PASS "ABCD1234" //Wifi Password definition

// Create the Adafruit IO client
AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS); // Creates the adafruit client with your four definitions

AdafruitIO_Feed *heartRateFeedMAX30101 = io.feed("heart-rate-max30101"); //Creates feed for MAX30101 heart rate 
AdafruitIO_Feed *heartRateFeedPulseSensor = io.feed("heart-rate-pulsesensor"); //Creates feed for Pulse Sensor heart rate
AdafruitIO_Feed *spo2Feed = io.feed("spo2"); //Creates feed for MAX30101 Sp02 

// MAX30101 setup
MAX30105 particleSensor;
#define MAX_BRIGHTNESS 60 //lowered from 100

// Pulse Sensor setup
PulseSensorPlayground pulseSensor;
const int pulsePin = A0; // Pin for the pulse sensor signal connection
int bpm = 0;             // BPM value from the pulse sensor
int threshold = 400;     // Threshold for the pulse sensor (modify as needed, 550 is the standard, we used 400)

// Buffers for MAX30101 data
#define MAX_BUFFER_SIZE 50 //reduced buffer size from 100 for quicker calculations 
uint32_t irBuffer[MAX_BUFFER_SIZE];
uint32_t redBuffer[MAX_BUFFER_SIZE];
int bufferLength = MAX_BUFFER_SIZE;
int32_t heartRateMAX30101 = 0;
int32_t spo2 = 0;
int8_t validHeartRate = 0;
int8_t validSPO2 = 0;

// Timing variables
unsigned long lastPublishTime = 0;         // Last time data was published
const unsigned long publishInterval = 5000; // Publish interval (5 seconds)
unsigned long lastCalculationMillis = 0;  // Last time calculations were done
const unsigned long calculationInterval = 200; // Interval for calculations (200ms)

void setup() {
    Serial.begin(115200); //communcation baud rate for serial monitor

    // Connect to Wi-Fi
    connectToWiFi();

    // Connect to Adafruit IO
    connectToAdafruitIO();

    // Initialize MAX30101 sensor
    if (!particleSensor.begin()) {
        Serial.println("MAX30101 not found!"); //prints if the sensor is not initialized properly
        while (1);
    }
    particleSensor.setup(MAX_BRIGHTNESS, 4, 2, 100, 411, 4096); //These brightness variables conform to the analog reading from your finger. 

    // Initialize Pulse Sensor
    pulseSensor.analogInput(pulsePin);
    pulseSensor.blinkOnPulse(LED_BUILTIN); // Blink onboard LED with heartbeat
    pulseSensor.setThreshold(threshold);
    if (pulseSensor.begin()) {
        Serial.println("PulseSensor initialized!");
    }
}

void loop() {
    unsigned long currentMillis = millis();

    // Check for a heartbeat from the Pulse Sensor
    if (pulseSensor.sawStartOfBeat()) {
        bpm = pulseSensor.getBeatsPerMinute();
        Serial.print("Pulse Sensor BPM: ");
        Serial.println(bpm);
    }

    // Perform MAX30101 calculations every 200ms
    if (currentMillis - lastCalculationMillis >= calculationInterval) {
        lastCalculationMillis = currentMillis;

        // Read MAX30101 data for heart rate and SpO2 calculations
        for (int i = 0; i < bufferLength; i++) {
            redBuffer[i] = particleSensor.getRed();
            irBuffer[i] = particleSensor.getIR();
            particleSensor.nextSample();
        }
        maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRateMAX30101, &validHeartRate);
    }

    // Publish data to Adafruit IO every 5 seconds
    if (millis() - lastPublishTime >= publishInterval) {
        lastPublishTime = millis();

        heartRateFeedMAX30101->save(heartRateMAX30101); // how the feed data is saved
        heartRateFeedPulseSensor->save(bpm); // how the fooed data is saved
        spo2Feed->save(spo2); // how the feed data is saved. 

        Serial.println("Data published to Adafruit IO."); //this should correspond with a change on your dashboard/feeds on Adafruit IO
    }

    // Run Adafruit IO
    io.run();

    // Debugging on the serial monitor so we can see what is working properly
    Serial.print("Heart Rate (MAX30101): ");
    Serial.println(heartRateMAX30101);
    Serial.print("SpO2 (MAX30101): ");
    Serial.println(spo2);
}

// Helper function to connect to Wi-Fi
void connectToWiFi() {
    unsigned long startAttemptTime = millis();
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
        delay(500);
        Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("Connected to WiFi!");
    } else {
        Serial.println("WiFi connection failed. Continuing...");
    }
}

// Helper function to connect to Adafruit IO
void connectToAdafruitIO() {
    io.connect();
    while (io.status() < AIO_CONNECTED) {
        Serial.print(".");
        delay(1000);
    }
    Serial.println("Connected to Adafruit IO!");
}
