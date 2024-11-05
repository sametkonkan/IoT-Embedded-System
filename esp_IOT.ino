#include <Wire.h>
#include <ESP8266WiFi.h>
#include "DHT.h"

#define LEDonBoard 2  // On-board LED for WiFi connection indicator

// WiFi credentials
const char* ssid = "Wifi Name";
const char* password = "psw123";

// ThingSpeak API key and server
String apiKey = "IZBGJNN79QKJ08W";  // enter your write API key from ThingSpeak
const char* server = "api.thingspeak.com";

// DHT Sensor Definitions
#define DHTTYPE DHT11
const int DHTPin = 14;  // The pin used for the DHT11 sensor is Pin D5 = Pin 14
DHT dht(DHTPin, DHTTYPE);

// AS5600 Definitions
#define AS5600_ADDRESS 0x36
#define ANGLE_REGISTER_MSB 0x0E
#define ANGLE_REGISTER_LSB 0x0F

// Wheel circumference (in cm)
const float circumference = 14.137167;

// Variables for speed calculation
const int sampleInterval = 15000;  // 15 sec
uint8_t hall_Count = 0;
float m_Speed = 0;

WiFiClient client;

void setup() {
  // Initialize I2C and Serial
  Wire.begin(D2, D1);  // I2C on ESP8266, SDA = D2, SCL = D1
  Serial.begin(115200);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  pinMode(LEDonBoard, OUTPUT);
  while (WiFi.status() != WL_CONNECTED) {
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
  dht.begin();

  // Debug message
  Serial.println("AS5600 Simple Angle Read with ESP8266");
}

void loop() {
  // Read DHT sensor
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  // Read the angle from the AS5600 sensor
  float angleDegrees = readAngle();

  // Calculate speed
  float time_Passed = measureSpeed();

  // Log speed and hall count
  Serial.println("Speed: " + String(m_Speed) + " km/h");
  Serial.println("Hall count: " + String(hall_Count));
  Serial.print("Angle: ");
  Serial.print(angleDegrees);
  Serial.println(" degrees");

  // Send DHT data to ThingSpeak
  if (client.connect(server, 80)) {
    String postStr = apiKey;
    postStr += "&field1="; // Field 1 for temperature
    postStr += String(t);
    postStr += "&field2="; // Field 2 for humidity
    postStr += String(h);
    postStr += "&field3="; // Field 3 for speed
    postStr += String(m_Speed);
    postStr += "&field4="; // Field 4 for angle
    postStr += String(angleDegrees);

    client.print("POST /update HTTP/1.1\n");
    client.print("Host: api.thingspeak.com\n");
    client.print("Connection: close\n");
    client.print("X-THINGSPEAKAPIKEY: " + apiKey + "\n");
    client.print("Content-Type: application/x-www-form-urlencoded\n");
    client.print("Content-Length: ");
    client.print(postStr.length());
    client.print("\n\n");
    client.print(postStr);

    Serial.print("Temperature: ");
    Serial.print(t);
    Serial.print("°C, Humidity: ");
    Serial.print(h);
    Serial.println("% sent to ThingSpeak.");
    Serial.print("Angle: ");
    Serial.print(angleDegrees);
    Serial.print(" degrees, Speed: ");
    Serial.print(m_Speed);
    Serial.println(" km/h sent to ThingSpeak");

    client.stop();
  }

  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  hall_Count = 0;
  time_Passed = 0;
}

float readAngle() {
  uint8_t msb, lsb;
  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(ANGLE_REGISTER_MSB);  // Point to the MSB of the angle register
  Wire.endTransmission(false);      // Restart for reading

  Wire.requestFrom(AS5600_ADDRESS, 2);  // Request 2 bytes from the sensor
  msb = Wire.read();                    // Read the MSB
  lsb = Wire.read();                    // Read the LSB

  uint16_t angle = ((uint16_t)msb << 8) | lsb;  // Combine MSB and LSB
  return (angle & 0x0FFF) * 360.0 / 4096.0;     // Convert to degrees 0 to 360
}

float measureSpeed() {
  float start = millis();
  bool on_state = false;
  float time_Passed = 0;

  while (true) {
    if (digitalRead(13) == 0) {
      if (!on_state) {
        on_state = true;
        hall_Count++;
      }
    } else {
      on_state = false;
    }
    float end_Time = millis();
    time_Passed = (end_Time - start) / 1000.0;

    yield();

    if (time_Passed >= sampleInterval / 1000.0) {
      break;
    }
  }
  calc_Speed(time_Passed);
  return time_Passed;
}

void calc_Speed(float time_Passed) {
  m_Speed = (((circumference / 100) * hall_Count) / time_Passed) * 3.6;  // km/h
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















