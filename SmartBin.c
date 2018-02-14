#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <HX711_ADC.h>

int sensorPin = A0; //analog read pin
int sensorVal = 0; //analog sensor value (dist in cm is 0.718*sensorVal)
int URTRIG = 5; // PWM trigger pin
long fillLevel[20]; //array to store the fill level percentages
long massLevel[20];//array to store the mass of each measuring point
int levelTracker = 0;//variable to store index location of next fill level
unsigned long trashVol[20];//array to store the volume of the trash
double echoA[20];//array of sensor measurements
double mult = 0.718*0.393701*0.9*0.7565;//multipying factor to get inches
//offset distance (in) of sensor from top of can
#define OFFSET 0
//container dimensions in
#define DEPTH 14.75
#define LENGTH 13
#define WIDTH 9.3
double maxVolume = DEPTH*LENGTH*WIDTH;
double avgDist = 0;
double trVol = 0;
double fillPerc = 0;


uint8_t EnPwmCmd[4]={0x44,0x02,0xbb,0x01};    // distance measure command

HX711_ADC scale(D8, D13);

long t;
float loadcellVal;

const char index_html[] PROGMEM = {"<!DOCTYPE html>\n"
"<html>\n"
"  <head>\n"
"    <style>\n"
"       #map {\n"
"        height: 400px;\n"
"        width: 100%;\n"
"       }\n"
"    </style>\n"
"  </head>\n"
"  <body>\n"
"    <h3>STRUMS</h3>\n"
"    <div id=\"map\"></div>\n"
"    <script>\n"
"      function initMap() {\n"
"        var location = {lat: 41.5021, lng: -81.6075};\n"
"        var map = new google.maps.Map(document.getElementById('map'), {\n"
"          zoom: 15,\n"
"          center: location\n"
"        });\n"
"        var marker = new google.maps.Marker({\n"
"          position: location,\n"
"          map: map\n"
"        });\n"
"\n"
"        var info = new google.maps.InfoWindow({content: '<h1> Trash bin 1</<h1>'});\n"
"        marker.addListener('click', function(){\n"
"          info.open(map,marker);\n"
"        });\n"
"\n"
"        /*\n"
"        function addMarker(props)\n"
"        {\n"
"        var marker = new google.maps.Marker({\n"
"        position:props.coords,\n"
"        map:map,\n"
"        icon: props.iconImage\n"
"      })\n"
"      }\n"
"        */\n"
"      }\n"
"    </script>\n"
"    <script async defer\n"
"    src=\"https://maps.googleapis.com/maps/api/js?key=AIzaSyBwuJPk1vgsFu6Ko92OMVow8M5QINqZ8hA&callback=initMap\">\n"
"    </script>\n"
"    <table align= \"center\" border = \"5\" cellspacing = \"5\" cellpadding = \"5\">\n"
"      <<tr align = \"center\">\n"
"        <<th>Location</th>\n"
"        <<th>Weight</th>\n"
"        <<th>Fill Level</th>\n"
"      </tr>\n"
"      <<tr align = \"center\">\n"
"        <<td>41.5021, -81.6075</td>\n"
"        <td>@@loadcellVal@@</td>\n"
"        <td>@@fillperc@@</td>\n"
"      </tr>\n"
"    </table>\n"
"  </body>\n"
"</html>"""};
const char* ssid = "CaseGuest";
const char* password = "";

ESP8266WebServer server(80);

const int led = 13;

void handleRoot() {
  String s = index_html;
  digitalWrite(led, 1);
  s.replace("@@fillperc@@",String(fillPerc));
  s.replace("@@loadcellVal@@", String(loadcellVal));
  server.send(200, "text/HTML", s);
  digitalWrite(led, 0);
}

void handleNotFound(){
  digitalWrite(led, 1);
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET)?"GET":"POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i=0; i<server.args(); i++){
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
  digitalWrite(led, 0);
}

void setup(void){
  Serial.begin(9600);
  Serial.println("HX711 Demo");

  Serial.println("Initializing the scale");
  scale.begin();
  long stabilisingtime = 2000;
  scale.start(stabilisingtime);
  scale.setCalFactor(696.0);

  pinMode(led, OUTPUT);
  digitalWrite(led, 0);
  Serial.begin(9600);

  pinMode(URTRIG,OUTPUT);
  for(int i=0;i<4;i++)
  {
      Serial.write(EnPwmCmd[i]);
  }
  digitalWrite(URTRIG,HIGH);
  delay(500);

  WiFi.begin(ssid, password);
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  if (MDNS.begin("esp8266")) {
    Serial.println("MDNS responder started");
  }

  server.on("/", handleRoot);

  server.on("/inline", [](){
    server.send(200, "text/plain", "this works as well");
  });

  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("HTTP server started");
}

void loop(void){
  takeMeasurement();
  scale.update();

  //get smoothed value from data set + current calibration factor
  if (millis() > t + 250) {
    float i = scale.getData();
    //scaleing to pounds function: y=(x-0.7)/145.7;
    loadcellVal = (i-0.7)/145.7;
    Serial.print("Load_cell output val: ");
    Serial.println(loadcellVal);
    t = millis();
  }

    //receive from serial terminal
  if (Serial.available() > 0) {
    float i;
    char inByte = Serial.read();
    if (inByte == 't') scale.tareNoDelay();
  }

  //check if last tare operation is complete
  if (scale.getTareStatus() == true) {
    Serial.println("Tare complete");
  }
  server.handleClient();
  doDelay();
}

void takeMeasurement()
{

  //ultrasound measurement
  //******
  unsigned long measureTimeStart = micros();
Serial.print("measure time start: ");
Serial.println(measureTimeStart);

  unsigned long sum = 0;

  digitalWrite(URTRIG,LOW);//sends a pulse through ultrasound sensor triggering a sensor reading
  digitalWrite(URTRIG,HIGH);

  for(int x = 0;x<20;x++)
  {
  unsigned long timeA = micros();
  unsigned long echoTimeA = analogRead(sensorPin);
Serial.print("timeA: ");
Serial.println(timeA);
Serial.println("Sensor reading A: ");
Serial.println(echoTimeA);//distance measurement from top

    echoA[x] = echoTimeA*mult;
    trashVol[x] = LENGTH*WIDTH*(DEPTH-((avgDist*mult)-OFFSET));
    fillLevel[x] = trashVol[x]/maxVolume;
    sum = sum+echoTimeA;
  }

  avgDist = (sum/20.00)*mult;
  trVol = LENGTH*WIDTH*(DEPTH-((avgDist*mult)-OFFSET));
  fillPerc = (trVol/maxVolume)*100.00;
  //*****
  //end ultrasound measurement
}

void doDelay()
{
  //Ouput values to serial.function
  //******
  //output fill level%
  Serial.print("Fill level: ");
  Serial.print(fillPerc);
  Serial.print("%");
  Serial.println();
  //******
/* //
  //output mass
  Serial.print("Mass Reading: ");
  Serial.print(scale.get_units(), 1); //scale.get_units() returns a float
  Serial.print(" lbs"); //You can change this to kg but you'll need to refactor the calibration_factor
  Serial.println();
  //******
  //end output values
*/

//for testing
Serial.print(trVol);
Serial.println("in^3");
Serial.print("AVG dist(in): ");
Serial.println(avgDist);
Serial.println(mult);
Serial.println();
Serial.println();
//******
//end for testing




  //delay based on previous fill level measurements
    //unsigned long startTime = millis();//millis can be ~50days before reseting
    //while(millis()-startTime < 1800000);//delays for a half hour (1800000ms)

  if(levelTracker<19){
  levelTracker = levelTracker + 1;
  }
  else{
    levelTracker = 0;
  }

  //delay function for life cycle
  //*****
//  unsigned long delayTime = 180*pow(2.71828,-0.02485*fillPerc)*60*1000;//delay time in milliseconds based on fill%
//  unsigned long startTime = millis();
//  while(millis()-startTime < delayTime);
  //*****
  //end delay function for life cycle



  //delay for testing
  //*****
  unsigned long startTime = millis();
  while(millis()-startTime < 5000){
    server.handleClient();
  }//delay for 5 seconds
  //*****
  //end delay for testing
}
