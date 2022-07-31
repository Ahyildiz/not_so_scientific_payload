#include <Wire.h>
#include <SPI.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h> 
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp;

TinyGPSPlus gps;
SoftwareSerial ss(11, 10);


bool test_mode = false;
bool gps_precise = true;

int buzzer_on_frequency = 4; //Buzzer beeping frequency in Hz
int buzzer_off_frequency = 2.5; //Buzzer quiet frequency in Hz
int message_frequency = 10; //LoRa messages per second

int counter = 0;
float cur_time;
char tmp_str[20];
char ch_message[150];
float base_alt = 0;

int buzzPin = 12;

int sensorValue;

typedef struct
{
    float bmp_alt, bmp_pres, bmp_temp;
    double gps_alt , gps_lat, gps_lon;
    float gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z;
    float rc_ang, rc_rot, rc_yaw,rc_speed;
    unsigned long int rc_time;
}ROCKET_STAT;

ROCKET_STAT Astra,Astra_prev;



void setup()
{
  Serial.begin(9600);
  ss.begin(9600);
  bmp.begin(0x76,0x58);
  /*
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     
                  Adafruit_BMP280::SAMPLING_X2,     
                  Adafruit_BMP280::SAMPLING_X16,    
                  Adafruit_BMP280::FILTER_X16,      
                  Adafruit_BMP280::STANDBY_MS_500); 
                  */
                  
  pinMode(buzzPin,OUTPUT); // Set buzzer-pin as output

  if (!test_mode){  
    for (cur_time = millis(); millis() - cur_time <100; base_alt += (1.0 - pow(bmp.readPressure() / 101325.0F, 0.1903F)) * ( (bmp.readTemperature() + 273.15F)/0.0065F), counter++) {
      
      Serial.print(base_alt/counter);
      Serial.print("   ");
      Serial.println(counter);
      }
    base_alt /= (counter);
    Serial.println(base_alt);
    smartDelay(1000);
  }
}
void loop()
{
  Astra_prev = Astra;
  add_hash();
  add_gps_values();
  add_bmp_values();
  add_quality_values();
  ring_buzzer();
  broadcast_message();
  

  smartDelay(0);
}

void ring_buzzer(){
  digitalWrite(buzzPin, LOW); // Tone ON
  delay(1000/buzzer_on_frequency); // Tone length
  digitalWrite(buzzPin, HIGH); // Tone OFF
  delay(1000/buzzer_off_frequency); // Tone length
  }

void save_float(float input) {
    char tmp_str[10];
    dtostrf(input, 1, 2, tmp_str);
    strcat(ch_message, tmp_str);
    strcat(ch_message, ",");
    free(tmp_str);
}

void save_gps(float input) {
    char tmp_str[10];
    dtostrf(input, 1, 6, tmp_str);    
    strcat(ch_message, tmp_str);
    strcat(ch_message, ",");
    free(tmp_str);
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

void add_gps_values() {
    if (!test_mode) {
        if (gps.location.isValid()) {
            Astra.gps_lat = gps.location.lat();
            Astra.gps_lon = gps.location.lng();
            Astra.gps_alt = gps.altitude.meters();
        }
        else if (gps_precise) {
            Astra.gps_lat = Astra_prev.gps_lat;
            Astra.gps_lon = Astra_prev.gps_lon;
            Astra.gps_alt = Astra_prev.gps_alt;
        }
        else{
          Astra.gps_lat = (float)random(40993660, 40994000)/1000000;
          Astra.gps_lon = (float)random(28832000, 28834000)/1000000;
          Astra.gps_alt = (float)random(5000, 6000)/100;
        }
    }
    else {
      
        Astra.gps_lat = (float)random(40991660, 41001660)/1000000;
        Astra.gps_lon = (float)random(28832000, 28837000)/1000000;
        Astra.gps_alt = (float)random(5000, 6000)/100;        
    }
    save_gps(Astra.gps_lat);
    save_gps(Astra.gps_lon);
    save_float(Astra.gps_alt);
}

void broadcast_message() {
  if (millis() > cur_time + (1000/message_frequency)){
    cur_time = millis();
    ch_message[strlen(ch_message) - 1] = '\0';
    Serial.println(ch_message);
    smartDelay(0);
  }
}
void add_hash() {
    Astra.rc_time = millis();
    strcpy(ch_message, "$FY;");
}

float tmp_alt = 0;
float tmp_alt2 = 0;
float tmp_alt3 = 0;

void add_bmp_values() {
  if (!test_mode) {
    tmp_alt3 = tmp_alt2;
    tmp_alt2 = tmp_alt;
    tmp_alt = Astra.bmp_alt;


    Astra.bmp_pres = (bmp.readPressure() / 100.0F);
    Astra.bmp_temp = bmp.readTemperature();
    Astra.bmp_alt = (1.0 - pow(Astra.bmp_pres / 1013.25F, 0.1903F)) * ( (Astra.bmp_temp + 273.15F)/0.0065F) - base_alt;

    
    Astra.bmp_alt = (Astra.bmp_alt + Astra_prev.bmp_alt + tmp_alt + tmp_alt2 + tmp_alt3) / 5.0F;
  }
  else {
    Astra.bmp_alt = (float)random(000, 50000)/100;
    Astra.bmp_pres = (float)random(90000, 100000)/100;
    Astra.bmp_temp = (float)random(2000, 3000)/100;
    }

  save_float(Astra.bmp_alt);
  save_float(Astra.bmp_pres);
  save_float(Astra.bmp_temp);
}

void add_quality_values() {
  sensorValue = analogRead(A0);
  save_float(sensorValue/100.0F);
  }
