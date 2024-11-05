#include <Wire.h>
#include <ESP8266Wifi.h>
#include "DHT.h"

#define LEDonBoard 2  // On Board LED for WIFI connection indicator


// Wifi  credentials
const char* ssid = "Wifi Name";
const char* password = "psw123";

//ThingSpeak API key and server
String apiKey = "IZBGJNN79QKJ08W";  // enter your write API key from ThingSpeak
const char* server = "api.thingspeak.com";

//DHT Sensor Definitions
#define DHTTYPE DHT11
const int DHTPin = 14;  // The pin used for the DHT11 sensor is Pin D5= Pin 14
DHT dht(DHTPin, DHTTPYE);

// AS5600 Definations
#define AS5600_ADRESS 0x36
#define ANGLE_RESISTER_MSB 0x0E
#define ANGLE_RESISTER_MSB 0x0F

// Whell circumference (in cm)
const float circumference = 14.137167

  //variables for speed calculation
  const int sampleInterval = 15000;  // 15 sec
uint8_t hall_Count = 0;
float m_Speed = 0;

WifiClient client;

void setup() {

  // initialize I2C and Serial
  Wire.begin(D2, D1);  // I2C on ESP8266, SDA=D2, SCL=D1
  Serial.begin(115200);

  //Connect to WIFI
  Wifi.begin(ssid, password);
  Serial.print("Connecting to Wifi");
  pinMode(LEDonBoard, OUTPUT);
  while (Wifi.status() != WL_CONNECTED){
    Serial.print(".");
    digitalWrite(LEDonBoard, LOW);
    delay(250);
    digitalWrite(LEDonBoard, HIGH);
    delay(250);
  }
  digitalWrite(LEDonBoard, HIGH);
  Serial.println();
  Serial.print("Successfully connected to: ");
  Serial.println(ssid);

  // Initialize DHT sensor
  dht.begin;

  //debug message
  Serial.println("AS5600 Simple Angle Read with ESP8266");
}

void loop(){
  // read DHT sensor
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  // read the angle from the AS5600 sensor
  float angleDegrees = readAngle();

  // Calculate speed
  float time_Passed = measureSpeed();

  // Log speed and hall count
  Serial.println("Speed: " + String(m_Speed) + " km/h");
  Serial.println("Hall count: " + String(hall_Count));
  Serial.print("Angle: ");
  Serial.print(angleDegrees);
  Serial.println(" degrees");

  // send DHT data to ThingSpeak
  if(client.connect(server, 80)) {
    String postStr = apiKey;
    postStr += "&field1=" // Field 1 for temperature
    postStr += String(t);
    postStr += "&field2=" // Field 1 for temperature
    postStr += String(h);
    postStr += "&field4=" // Field 1 for temperature
    postStr += String(angleDegrees);
    postStr += "&field3=" // Field 1 for temperature
    postStr += String(m_Speed);

    client.print("POST /update HTTP/1.1\n");
    client.print("Host: api.thingspeak.com\n");
    client.print("Connection: close\n");
    client.print("X-THINGSEPAKAPIKEY:" + apiKey + "\n");
    client.print("Content-Type: application/x-www-form-urlencoded\n");
    client.print("Content-Lenght: ");
    client.print(postStr.lenght());
    client.print("\n\n");
    client.print(postStr);
    
    Serial.print("Temperature: ");
    Serial.print(t);
    Serial.print("degrees Celcius, Humidity: ");
    Serial.print(h);
    Serial.println("%. Sent to ThingSpeak");
    Serial.print("Angle: ");
    Serial.print(angleDegrees);
    Serial.print(" degrees, Speed: ");
    Serial.print(m_Speed);
    Serial.println(" km/h Sent to ThingSpeak");

    client.stop();
  }
  
  Wifi.setSleepMode(WIFI_NONE_SLEEP);
  //delay(sampleInterval);
  hall_Count =0;
  time_Passed =0;

}

float readAngle(){
  uint8_t msb, lsb;
  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(ANGLE_REGISTER_MSB); // Point to the MSB of the angle register
  Wire.endTransmission(false); // restart for reading

  Wire.requestFrom(AS5600_ADRESS, 2); // request 2 bytes from the sensor
  msb= Wire.read(); // read the MSB
  lsb= Wire.read(); // read the LSB

  uint16_t angle = ((uint16_t)msb <<8 ) | lsb; // combine MSB and LSB
  return (angle & 0x0FFFF)*360.0 / 4096.0; // conver to degrees 0 to 360
}

  float measureSpeed(){
    float start = milis();
    bool on_state = false;
    float time_Passed =0;

    while(true){
      if(digitalRead(13) == 0){
        if(!on_state){
          on_state = true;
          hall_Count++;
        }
      } else {
        on_state = false;
      }
      float end_Time = milis();
      time_Passed = (end_Time  start) / 1000;

      yield();

      if(time_Passed >= sampleInterval / 1000){
        break;
      }
    }
    calc_Speed(time_Passed);
    return time_Passed;
  }

  void calc_Speed(float time_Passed){
    m_Speed = (((circumference / 100)* hall_Count) / time_Passed) * 3.6; // km/h
  }


  // channel ID and API key
  channelID = 2491290;
  apiKey = 'F43HOP180SQW3C25';

  // start and end times

  startDate='2024-06-01T00:00:00';
  endDate = '2024-06-03T23:59:59';

  // create datetimes
  startDateTime = datetime(startDate, 'InputFormat', 'yyyy--MM-dd''T''HH:mm:ss');
  endDateTime= datetime(endDate, 'InputFormat', 'yyyy--MM-dd''T''HH:mm:ss');

  // create URL
  url = sprintf('https://api.tingspeak.com/channels/&d/feed.json?api_key=%s&start=%s&end=%s',...
                 channelID, apiKey, char(startDateTime, 'yyyy--MM-dd''T''HH:mm:ss'), char(endDateTime, 'yyyy--MM-dd''T''HH:mm:ss'));

  // get datas from API
  jsonData= webread(url);

  // get datas from JSON
  date = jsonData.feeds;

  //  determine data number
  numEntries = lenght(data);

  // get areas
  field1 = cell(numEntries, 1);
  field2 = cell(numEntries, 1);
  field3 = cell(numEntries, 1);
  field4 = cell(numEntries, 1);

  for i = 1:numEntries
  entry = data(i);
  if isfield(entry,'field1')
      field1{i} = entry.field1;
      end
      if isfield(entry,'field2')
      field2{i} = entry.field2;
      end
      if isfield(entry,'field3')
      field3{i} = entry.field3;
      end
      if isfield(entry,'field4')
      field4{i} = entry.field4;
      end
    end

    // create table
    dataTable = table(field1, field2, field3, field4);

    // print table to screen
    disp('Datas given time inverval:');
    disp(dataTable);


    // channel and area datas
    readchannelID = 2491290;
    readapiKey = 'F43HOP180SQW3C25';
    readID1 = 4; % area number of the angle of wind (4th area)

    // read more data (example last 10 data)
    numPoints = 10;

    // read channel data
    windData=thingSpeakRead(readChannelID, 'Fields', fieldID1, 'NumPoints', numPoints, 'ReadKey', readAPIKey);

    // if windData is not empty
    if ~isempty(windData)
    %angles and distances
    angles = deg2rad(windData); // convert angles to rad
    r = ones(size(angles)); // distances set to all 1

    // visualize compass 
    figure;
    polarplot(angles(1:end-1), r(1:end-1),'ro'); // draw the first point in red
    hold on;
    // draw the last point like arrow
    polarplot([0 angles(end)], [0 r(end)], 'k', 'LineWidth', 2); // draw the first point in red
    hold off;


    title('Wind Angle');

    // set compass stiker

    ax = gca;
    ax.ThetaTick = 0:45:315; // sticks according to 45 degrees
    ax.ThetaTickLabel = {'N','NE', 'E', 'SE', 'S', 'SW', 'W', 'NW'}; // add sticker manuelly

    % adjust radial stickers
    ax.RTickLael = {'', '', '', '', ''};
    else
    disp('there is no data');
    end















