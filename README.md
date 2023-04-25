# Práctica 8. Gerard Cots y Joel J. Morera

## Ejercicio practico 1 : Bucle de comuniación UART2

###### **Funcionamiento**

Hulaaa m'agradaria fer aquesta part si no et sap greu, així faig algo i no em sento un inútil. La intentaré fer avui al vespre.

###### **Código del programa**

- platformio.ini

```

```

- main.cpp

```cpp

```

###### **Salida del puerto serie**

Paragraph

```

```

***

## Ejercicio optativo 2 : Módulo GPS

###### **Funcionamiento**

Paragraph

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

Paragraph

###### **Código del programa**

- platformio.ini:

``` 
```

- main.cpp:

```cpp

```

###### **Salida del puerto serie**

Paragraph

```
```

***
