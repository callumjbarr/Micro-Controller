#include <WiFi.h>
#include <LittleFS.h>
#include <WebServer.h>

///////////////////////////////////////////////////////////////////////
// WiFi AP 
///////////////////////////////////////////////////////////////////////
const char* ssid = "ESP32-StripTil";
const char* password = "12345678";
WebServer server(80);

///////////////////////////////////////////////////////////////////////
// Serve index.html manually 
///////////////////////////////////////////////////////////////////////
void handleRoot() {
  if(!LittleFS.exists("/index.html")){
    server.send(404, "text/plain", "File not found");
    return;
  }
  
  File file = LittleFS.open("/index.html", "r");
  server.streamFile(file, "text/html");
  file.close();
}

///////////////////////////////////////////////////////////////////////
// Pins
///////////////////////////////////////////////////////////////////////

// proximatiy switch data input
#define rpmPin 27
int motor1 = 25;
int motor2 = 26;
int pwm = 14;

///////////////////////////////////////////////////////////////////////
// Motor and PID
///////////////////////////////////////////////////////////////////////
// time variables
volatile unsigned long lastTime = 0;
volatile unsigned long curTime = 0;

int count = 0;
int calCount = 0;

// rpm variable
double rpm = 0;
double targetRPM = 0.0;
double calWeight = 0.0;
int calTurns = 100;
double meterRate = 0.0;
int targetRate = 0;
float workingWidth = 0; // current implement width
double speed = 10.0;
double workRate = 0.0;

// state variable to make sure it triggers between correct points
int lastState = HIGH;
int lastStateCal = HIGH;

// motor variables
int f = 3000;
int channel = 0;
int res = 8;

// PID variables
unsigned long PIDrefresh = 10;
unsigned long lastPIDtime = 0;
unsigned long curPIDtime = 0;
double error = 0;
double integral = 0;
double prevError = 0;
double pwmOut = 0;
double derivative =0;

///////////////////////////////////////////////////////////////////////
// PID Tuning
///////////////////////////////////////////////////////////////////////

double Kp = 1.2;
double Ki = 0.5;
double Kd = 0.1;   

///////////////////////////////////////////////////////////////////////
// Button bools
///////////////////////////////////////////////////////////////////////

bool cal = false;
bool run = false;
bool calFinished = false;



void setup() {
  // Set input and begin serial
  Serial.begin(115200);
  
  // Pins
  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);
  pinMode(pwm, OUTPUT);
  pinMode(rpmPin, INPUT_PULLUP);
  ledcAttach(pwm, f, res);

  ///////////////////////////////////////////////////////////////////////
  // File systen
  ///////////////////////////////////////////////////////////////////////
  if(!LittleFS.begin(true)){
    Serial.println("LittleFS Mount Failed");
    return;
  }

  ///////////////////////////////////////////////////////////////////////
  // start AP
  ///////////////////////////////////////////////////////////////////////
  WiFi.softAP(ssid, password);
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP()); // usually 192.168.4.1

  // Routes 
  server.on("/", handleRoot);       // serve index.html on root
  server.serveStatic("/style.css", LittleFS, "/style.css");  // serve CSS
  server.serveStatic("/script.js", LittleFS, "/script.js");  // serve JS
  server.begin();
  Serial.println("Web server started!");
  
  server.on("/rpm", HTTP_GET, []() {
    String json = "{";
    json += "\"rpm\":" + String(rpm, 1) + ",";
    json += "\"target\":" + String(targetRPM, 1);
    json += "}";
    server.send(200, "application/json", json); // <-- OK with WebServer  
  });

  // calibration btn
  server.on("/toggleCal", HTTP_GET, []() {
    cal = !cal;        // toggle calibration on/off
    if(cal){
        calCount = 0;  // reset count when starting
    }
    server.send(200, "text/plain", cal ? "Calibration started" : "Calibration stopped");
  });

  // Return calibration progress as a percentage
  server.on("/calProgress", HTTP_GET, []() {
  int progress = map(calCount, 0, 100, 0, 100); // map count 0-100 to percent
  server.send(200, "text/plain", String(progress));
});

 // meter on off btn
  server.on("/setRun", HTTP_GET, []() {
    if (server.hasArg("value")) {
      run = server.arg("value") == "1";

      Serial.print("Run state changed to: ");
      Serial.println(run);
    }

    server.send(200, "text/plain", "OK");
  });

// calibration weight input
server.on("/setCalWeight", HTTP_GET, []() {
  if (server.hasArg("value")) {
    calWeight = server.arg("value").toFloat();

    Serial.print("Calibration weight set to: ");
    Serial.println(calWeight);
  }

  server.send(200, "text/plain", "OK");
});

// target application rate
server.on("/setTargetRate", HTTP_GET, []() {
  if (server.hasArg("value")) {
    targetRate = server.arg("value").toInt();

    Serial.print("Target rate set to: ");
    Serial.println(targetRate);
  }

  server.send(200, "text/plain", "OK");
});

// Implement Manager: set number of rows


server.on("/setWorkingWidth", HTTP_GET, []() {
  if (server.hasArg("value")) {
    workingWidth = server.arg("value").toFloat(); // get width in meters

    Serial.print("Working width set to: ");
    Serial.println(workingWidth);
  }

  server.send(200, "text/plain", "OK");
});




}



void loop() {
  server.handleClient(); // handle web requests
  readRPM();
  rateCalc();
  calibration();
  PID();
  
Serial.print(pwmOut);

Serial.print("\t");  // tab
Serial.print(rpm);
Serial.print("\t");
Serial.print(targetRPM);
Serial.print("\t");
Serial.print(cal);
Serial.print("\t");
Serial.println(calCount);
Serial.print("\t");
Serial.println(calWeight);
Serial.print("\t");
Serial.println(targetRate);


}

  ///////////////////////////////////////////////////////////////////////////////////
  // Fuction: RPM sensor
  ///////////////////////////////////////////////////////////////////////////////////

void readRPM(){
    // get sensor state, if LOW metal is detected
  int sensorState = digitalRead(rpmPin); 
  unsigned long now = micros();
  // every flash
  
  if (sensorState == LOW && lastState == HIGH) {
    curTime = micros();

    if (count > 0){  
      double deltaTime = curTime - lastTime;
      rpm = (int)round(60.0 * 1000000.0/ deltaTime); 
      //unsigned long timeSinceLastPulse = now - lastTime;
    }
    else{
        rpm = 0;
        Serial.println("First Spin, skip rpm.");
    }
    
    lastTime = curTime;
    count++;
    if (cal) calCount++;
  }

  const unsigned long rpmTimeOut = 60000000UL;
  if (!run){
    rpm = 0;
  }

  lastState = sensorState;
}

  ///////////////////////////////////////////////////////////////////////////////////
  // Fuction: calibration
  ///////////////////////////////////////////////////////////////////////////////////

void calibration(){
  
  //skip if not calibrating
  if (!cal){
    pwmOut = 0;
    digitalWrite(motor1, LOW);
    digitalWrite(motor2, HIGH); 
    ledcWrite(pwm,int(pwmOut));
    calCount=0;
    return;
  }

  // 50% const power for calibration
  pwmOut = 128;
  digitalWrite(motor1, LOW);
  digitalWrite(motor2, HIGH); 
  ledcWrite(pwm,int(pwmOut));

  if (calCount > calTurns) {
    pwmOut = 0;
    cal = false;
    calCount = 0;
    pwmOut = 0;
    calFinished = true;
    digitalWrite(motor1, LOW);
    digitalWrite(motor2, HIGH); 
    ledcWrite(pwm,int(pwmOut));
    Serial.println("Calibration complete. Measure weight now.");
    return;
    }

}

  ///////////////////////////////////////////////////////////////////////////////////
  // Fuction: PID
  ///////////////////////////////////////////////////////////////////////////////////

void PID(){

  // skip if meter not on
  if(!run) return;

    // this makes the PID loop slower than void loop so that PID isnt too sensitive.
  if(millis()-lastPIDtime >= PIDrefresh){
    curPIDtime = millis();
    double dt = (curPIDtime - lastPIDtime)/1000.0;
    if(dt<=0) dt = 0.001;
    lastPIDtime = curPIDtime;

    // calcs proportional part of PID
    error = targetRPM - rpm;
    // calcs intergral of PID
    integral += error*dt;
    // calcs derivative of PID
    derivative = (error - prevError) / dt;

    double minPWM = 0.0;
    
    //PWM output but also contrain
    pwmOut = Kp*error + Ki*integral + Kd*derivative;
    if(isnan(pwmOut)) pwmOut =minPWM;
    pwmOut = constrain(pwmOut, minPWM, 255);
    digitalWrite(motor1, LOW);
    digitalWrite(motor2, HIGH); 
    ledcWrite(pwm,int(pwmOut));

    // update prev error
    prevError = error;

  }
}


void rateCalc(){
  workRate = (speed*1000*workingWidth)/10000;
  meterRate = calWeight/calTurns;
  targetRPM = ((targetRate*workRate)/meterRate)/60;
}





