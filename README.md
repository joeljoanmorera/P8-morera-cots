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

En este ejercicio se implementa un bucle de comunicación entre el puerto serie 2 y el puerto serie 0. El programa lee los datos que llegan por el puerto serie 2 y los envía por el puerto serie 0. Para ello se utiliza la función `Serial2.read()` para leer los datos del puerto serie 2 y la función `Serial.write()` para enviar los datos por el puerto serie 0.

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
