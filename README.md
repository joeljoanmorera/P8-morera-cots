# Práctica 8. Gerard Cots y Joel J. Morera

## Estandares de comunicación RS232, RS485 y RS422

RS232, RS485 y RS422 son estándares de comunicación serial utilizados para la transmisión de datos en una variedad de dispositivos electrónicos.

RS232 es un estándar de comunicación serial que transmite datos en un solo hilo. Es comúnmente utilizado para conectar dispositivos periféricos a una computadora, como módems, impresoras y scanners. RS232 utiliza un protocolo de comunicación asincrónico, lo que significa que no hay una señal de reloj de sincronización que se transmita junto con los datos.

RS485 y RS422 son estándares de comunicación serial diferencial, lo que significa que utilizan dos hilos para la transmisión de datos. A diferencia de RS232, estos estándares pueden transmitir datos a largas distancias (hasta 1200 metros) y a una velocidad máxima de 10 Mbps para RS485 y 10 Mbps para RS422.

RS485 utiliza un esquema de comunicación de tipo maestro/esclavo, lo que significa que un dispositivo maestro controla la comunicación con varios dispositivos esclavos. RS422, por otro lado, utiliza un esquema de comunicación de tipo punto a punto o multipunto, lo que significa que un dispositivo puede comunicarse directamente con otro dispositivo o con varios dispositivos.

En resumen, RS232 es adecuado para la comunicación a corta distancia entre dispositivos periféricos y una computadora, mientras que RS485 y RS422 son más adecuados para la comunicación a larga distancia y entre dispositivos industriales y sistemas de control.

### Funciones principales de la API Serial
#### if(Serial)
Indica si el puerto des Seria seleccionado está libre.
#### avaible()
Nos da el numero de bytes disponibles para leer del puerto. Són datos almacenados en el buffer del receptor serial (64 bytes).
#### availableForWrite()
Nos da el número de bytes disponibles para escribir en el buffer del serial sin bloquear la operación escritura.
## Ejercicio practico 1 : Bucle de comuniación UART2

###### **Funcionamiento**
```
En este ejercicio se implementa un bucle de comunicación entre el puerto serie 2 y el puerto serie 0. El programa lee los datos que llegan por el puerto serie 2 y los envía por el puerto serie 0. Para ello se utiliza la función `Serial2.read()` para leer los datos del puerto serie 2 y la función `Serial.write()` para enviar los datos por el puerto serie 0.
```
**TEXT PROVISIONAL**

###### **Código del programa**

```cpp
#include <Arduino.h>
#include <HardwareSerial.h>




void setup() {
  Serial.begin(115200);
  Serial2.begin(115200); 
  uint8_t e = 1;
  Serial2.write(e);
}

void loop() {
 
  if(Serial2.available()>0) {
    uint8_t c = Serial2.read();
    Serial.println(c);
    Serial2.write(c+1);
    
  }
 

    delay(1000);
}

```

###### **Salida del puerto serie**

En la salida del puerto serie se puede observar como se reciben los datos que se envían por el puerto serie 2.

```
1
2
3
4
...
```

***

## Ejercicio optativo 2 : Módulo GPS

###### **Funcionamiento**

En este ejecticio se descibre un módulo GPS que se conecta al ESP32 mediante el puerto serie 2. El módulo GPS envía una secuencia de datos que se pueden leer mediante el puerto serie 2. Para ello se utiliza la librería `TinyGPS` que permite leer los datos que envía el módulo GPS. En este caso se utiliza la función `gps.encode()` para leer los datos que envía el módulo GPS y la función `gps.f_get_position()` para obtener la posición del GPS.

###### **Código del programa**

- platformio.ini:

```

```

- main.cpp:

```cpp
#include <SoftwareSerial.h>
#include <TinyGPS.h>

TinyGPS gps;

SoftwareSerial softSerial(4, 3);

void setup()
{
    Serial.begin(115200);
    softSerial.begin(9600);
}

void loop()
{
    bool newData = false;
    unsigned long chars;
    unsigned short sentences, failed;
    // Intentar recibir secuencia durante un segundo
    for (unsigned long start = millis(); millis() - start < 1000;)
    {
        while (softSerial.available())
        {
            char c = softSerial.read();
            if (gps.encode(c)) // Nueva secuencia recibida
                newData = true;
        }
    }

    if (newData)
    {
        float flat, flon;
        unsigned long age;
        gps.f_get_position(&flat, &flon, &age);
        Serial.print("LAT=");
        Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
        Serial.print(" LON=");
        Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
        Serial.print(" SAT=");
        Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.sate
        Serial.print(" PREC=");
        Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
    }
    gps.stats(&chars, &sentences, &failed);
    Serial.print(" CHARS=");
    Serial.print(chars);
    Serial.print(" SENTENCES=");
    Serial.print(sentences);
    Serial.print(" CSUM ERR=");
    Serial.println(failed);
}
```

###### **Salida del puerto serie**

Paragraph

```
```

***

## Ejercicio optativo 3 : Módulo GPRS -GSM

###### **Funcionamiento**

En este ejercicio se utiliza un módulo GPRS para enviar datos a un servidor MQTT. Para ello se utiliza la librería `TinyGSM` que permite enviar datos mediante el módulo GPRS. En este caso se utiliza la función `modem.init()` para inicializar el módulo GPRS, la función `modem.gprsConnect()` para conectarse a la red GPRS, la función `mqtt.connect()` para conectarse al servidor MQTT, la función `mqtt.publish()` para publicar datos en el servidor MQTT y la función `mqtt.loop()` para mantener la conexión con el servidor MQTT.

###### **Código del programa**

- platformio.ini:

``` 
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
monitor_port = /dev/ttyUSB0
```

- main.cpp:

```cpp
// Select your modem:
#define TINY_GSM_MODEM_SIM800 // Modem is SIM800L
// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial
// Set serial for AT commands
#define SerialAT Serial1
// Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG SerialMon
// set GSM PIN, if any
#define GSM_PIN ""
// Your GPRS credentials, if any
const char apn[] = ""; // APN (example: internet.vodafone.pt) use https://wiki.apncha
const char gprsUser[] = "";
const char gprsPass[] = "";
// SIM card PIN (leave empty, if not defined)
const char simPIN[]
= "";
// MQTT details
const char* broker = "XXX.XXX.XXX.XXX";// Public IP address or do
const char* mqttUsername = "REPLACE_WITH_YOUR_MQTT_USER";// MQTT username
const char* mqttPassword = "REPLACE_WITH_YOUR_MQTT_PASS";// MQTT password
const char* topicOutput1 = "esp/output1";
const char* topicOutput2 = "esp/output2";
const char* topicTemperature = "esp/temperature";
const char* topicHumidity = "esp/humidity";
// Define the serial console for debug prints, if needed
//#define DUMP_AT_COMMANDS
#include <Wire.h>
#include <TinyGsmClient.h>
#ifdef DUMP_AT_COMMANDS
    #include <StreamDebugger.h>
    StreamDebugger debugger(SerialAT, SerialMon);
    TinyGsm modem(debugger);
#else
    TinyGsm modem(SerialAT);
#endif
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
TinyGsmClient client(modem);
PubSubClient mqtt(client);
// TTGO T-Call pins
#define MODEM_RST5
#define MODEM_PWKEY4
#define MODEM_POWER_ON23
#define MODEM_TX27
#define MODEM_RX26
#define I2C_SDA21
#define I2C_SCL22
// BME280 pins
#define I2C_SDA_218
#define I2C_SCL_219
#define OUTPUT_12
#define OUTPUT_215
uint32_t lastReconnectAttempt = 0;
// I2C for SIM800 (to keep it running when powered from battery)
TwoWire I2CPower = TwoWire(0);
TwoWire I2CBME = TwoWire(1);
Adafruit_BME280 bme;
#define IP5306_ADDR0x75
#define IP5306_REG_SYS_CTL00x00
float temperature = 0;
float humidity = 0;
long lastMsg = 0;

bool setPowerBoostKeepOn(int en){
    I2CPower.beginTransmission(IP5306_ADDR);
    I2CPower.write(IP5306_REG_SYS_CTL0);
    if (en) {
        I2CPower.write(0x37); // Set bit1: 1 enable 0 disable boost keep on
} else {
I2CPower.write(0x35); // 0x37 is default reg value
}
return I2CPower.endTransmission() == 0;
}
void mqttCallback(char* topic, byte* message, unsigned int len) {
Serial.print("Message arrived on topic: ");
Serial.print(topic);
Serial.print(". Message: ");
String messageTemp;
for (int i = 0; i < len; i++) {
Serial.print((char)message[i]);
messageTemp += (char)message[i];
}
Serial.println();
// Feel free to add more if statements to control more GPIOs with MQTT
// If a message is received on the topic esp/output1, you check if the message is e
// Changes the output state according to the message
if (String(topic) == "esp/output1") {
Serial.print("Changing output to ");
if(messageTemp == "true"){
Serial.println("true");
digitalWrite(OUTPUT_1, HIGH);
}
else if(messageTemp == "false"){
Serial.println("false");
digitalWrite(OUTPUT_1, LOW);
}
}
else if (String(topic) == "esp/output2") {
Serial.print("Changing output to ");
if(messageTemp == "true"){
Serial.println("true");
digitalWrite(OUTPUT_2, HIGH);
}
else if(messageTemp == "false"){
Serial.println("false");
digitalWrite(OUTPUT_2, LOW);
}
}
}
boolean mqttConnect() {
SerialMon.print("Connecting to ");
SerialMon.print(broker);
// Connect to MQTT Broker without username and password
//boolean status = mqtt.connect("GsmClientN");
// Or, if you want to authenticate MQTT:
boolean status = mqtt.connect("GsmClientN", mqttUsername, mqttPassword);
if (status == false) {
SerialMon.println(" fail");
ESP.restart();
return false;
}
SerialMon.println(" success");
mqtt.subscribe(topicOutput1);
mqtt.subscribe(topicOutput2);
return mqtt.connected();
}
void setup() {
// Set console baud rate
SerialMon.begin(115200);
delay(10);
// Start I2C communication
I2CPower.begin(I2C_SDA, I2C_SCL, 400000);
I2CBME.begin(I2C_SDA_2, I2C_SCL_2, 400000);
// Keep power when running from battery
bool isOk = setPowerBoostKeepOn(1);
SerialMon.println(String("IP5306 KeepOn ") + (isOk ? "OK" : "FAIL"));
// Set modem reset, enable, power pins
pinMode(MODEM_PWKEY, OUTPUT);
pinMode(MODEM_RST, OUTPUT);
pinMode(MODEM_POWER_ON, OUTPUT);
digitalWrite(MODEM_PWKEY, LOW);
digitalWrite(MODEM_RST, HIGH);
digitalWrite(MODEM_POWER_ON, HIGH);
pinMode(OUTPUT_1, OUTPUT);
pinMode(OUTPUT_2, OUTPUT);
SerialMon.println("Wait...");
// Set GSM module baud rate and UART pins
SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
delay(6000);
// Restart takes quite some time
// To skip it, call init() instead of restart()
SerialMon.println("Initializing modem...");
modem.restart();
// modem.init();
String modemInfo = modem.getModemInfo();
SerialMon.print("Modem Info: ");
SerialMon.println(modemInfo);
// Unlock your SIM card with a PIN if needed
if ( GSM_PIN && modem.getSimStatus() != 3 ) {
modem.simUnlock(GSM_PIN);
}
// You might need to change the BME280 I2C address, in our case it's 0x76
if (!bme.begin(0x76, &I2CBME)) {
Serial.println("Could not find a valid BME280 sensor, check wiring!");
while (1);
}
SerialMon.print("Connecting to APN: ");
SerialMon.print(apn);
if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
SerialMon.println(" fail");
ESP.restart();
}
else {
SerialMon.println(" OK");
}
if (modem.isGprsConnected()) {
SerialMon.println("GPRS connected");
}
// MQTT Broker setup
mqtt.setServer(broker, 1883);
mqtt.setCallback(mqttCallback);
}
void loop() {
if (!mqtt.connected()) {
SerialMon.println("=== MQTT NOT CONNECTED ===");
// Reconnect every 10 seconds
    uint32_t t = millis();
    if (t - lastReconnectAttempt > 10000L) {
        lastReconnectAttempt = t;
        if (mqttConnect()) {
            lastReconnectAttempt = 0;
        }
    }
        delay(100);
        return;
    }
    long now = millis();
    if (now - lastMsg > 30000) {
        lastMsg = now;
        // Temperature in Celsius
        temperature = bme.readTemperature();
        // Uncomment the next line to set temperature in Fahrenheit
        // (and comment the previous temperature line)
        //temperature = 1.8 * bme.readTemperature() + 32; // Temperature in Fahrenheit
        // Convert the value to a char array
        char tempString[8];
        dtostrf(temperature, 1, 2, tempString);
        Serial.print("Temperature: ");
        Serial.println(tempString);
        mqtt.publish(topicTemperature, tempString);
        humidity = bme.readHumidity();
        // Convert the value to a char array
        char humString[8];
        dtostrf(humidity, 1, 2, humString);
        Serial.print("Humidity: ");
        Serial.println(humString);
        mqtt.publish(topicHumidity, humString);
    }
    mqtt.loop();
}

```

###### **Salida del puerto serie**

Paragraph

```
```

***
