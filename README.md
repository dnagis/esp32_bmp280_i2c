# Lecture capteurs Bosch (temp pression hum)

bm[p,e]280 (attention selon le modèle il faut changer adresse register (0x77 ou 0x76) dans bmx280.h)  

Basé sur esp-idf/examples/get-started/hello_world et sur https://github.com/openairproject/sensor-esp32  

PINS:  
esp32 | bmp280  

3v3 | Vin  
GND | GND  
D18 | SDI   
D19 | SCK  

n°s des pins configuré dans bmx280.h 
