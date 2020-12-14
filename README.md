# ESP32: Lecture capteurs Bosch bm[p,e]280 (temp pression hum)

2020: Nouvelle stratégie pour accéder en i2c:
il faut installer la librairie esp-idf-lib : recette install dans mtox/exp32

 
 
git clone https://github.com/UncleRus/esp-idf-lib dans un DIR et ajouter dans la Makefile:
EXTRA_COMPONENT_DIRS := $DIR/esp-idf-lib/components


ce code basé sur les exemples disponibles dans esp-idf-lib/components/bmp280

n.b. ancienne technique est archivée dans ce repo avec les fichiers



PINS:  
esp32 | bmp280  

3v3 | Vin  
GND | GND  
SDA | SDI   
SCL | SCK  

