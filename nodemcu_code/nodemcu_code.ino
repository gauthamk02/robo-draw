#define WHEEL_BASE 11.3  //cm
#define WHEEL_DIA 6.3      //cm

#define SERVO_PIN D8

// ULN2003 Motor Driver Pins
#define IN1 D0
#define IN2 D1
#define IN3 D2
#define IN4 D3

#define IN5 D4
#define IN6 D5
#define IN7 D6
#define IN8 D7

#include <Servo.h>
#include <AccelStepper.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>

const double wheelCircumference = WHEEL_DIA * PI;
const int stepsPerRevolution = 2*2048;  // change this to fit the number of steps per revolution
const double distPerStep = wheelCircumference / stepsPerRevolution;

const char* ssid = "Gk119";
const char* password = "nyanparayula";

Servo penServo;  // create servo object to control a servo
ESP8266WebServer server(80);

AccelStepper motor1(AccelStepper::HALF4WIRE, IN1, IN3, IN2, IN4);
AccelStepper motor2(AccelStepper::HALF4WIRE, IN8, IN6, IN7, IN5);

bool isRunning = false;
bool isPenUp = true;
String httpResp = "";

void penUp() {
  penServo.write(0);
}

void penDown() {
  penServo.write(90);
}

int getSteps(double dist) {
  // return int(dist * stepsPerRevolution / wheelCircumference);
  return int(dist / distPerStep);
}

void moveDist(double dist) {
  // dist = steps * circum / stepsperev
  int steps = getSteps(dist);
  Serial.println("Moving Steps: " + String(steps));
  motor1.move(steps);
  motor2.move(steps);
}

void turnAngle(double angle) {
  double arcLength = (WHEEL_BASE / 2) * angle * PI / 180;
  int steps = getSteps(arcLength);
  Serial.println("Turning Steps: " + String(steps));
  motor1.move(steps);
  motor2.move(-steps);
}

void handleMove() {
  if (server.hasArg("distance") == false) {  //Check if body received

    server.send(400, "text/plain", "Body parameter 'distance' not received");
    return;
  }

  double dist = server.arg("distance").toDouble();
  Serial.println(dist);
  moveDist(dist);
  // if (!isPenUp) penUp();
  httpResp = "{\"distance\": \"" + String(dist) + "\"}";
  isRunning = true;
  if (!isPenUp) penDown();
}

void handleTurn() {
  if (server.hasArg("angle") == false) {  //Check if body received

    server.send(400, "text/plain", "Body parameter 'angle' not received");
    return;
  }

  double dist = server.arg("angle").toDouble();
  Serial.println(dist);
  turnAngle(dist);
  httpResp = "{\"angle\": \"" + String(dist) + "\"}";
  isRunning = true;
}

void handlePen() {
  if (server.hasArg("position") == false) {  //Check if body received

    server.send(400, "text/plain", "Body parameter 'position' not received");
    return;
  }

  String pos = server.arg("position");
  Serial.println(pos);
  if (pos.equals("up")) isPenUp = true;
  else if (pos.equals("down")) isPenUp = false;
  else server.send(400, "text/plain", "Unidentified value for 'position' argument");

  httpResp = "{\"angle\": \"" + pos + "\"}";
  sendResponse(httpResp, 200);
}

void sendResponse(String resp, int code) {
  server.send(code, "text/json", resp);
}

void restServerRouting() {
  server.on("/", HTTP_GET, []() {
    server.send(200, F("text/html"),
                F("Welcome to the REST Web Server"));
  });
  server.on(F("/move"), HTTP_POST, handleMove);
  server.on(F("/turn"), HTTP_POST, handleTurn);
  server.on(F("/pen"), HTTP_POST, handlePen);
}

void handleNotFound() {
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
}

void setup() {
  penServo.attach(SERVO_PIN);  // attaches the servo on GIO2 to the servo object

  if (isPenUp) penUp();

  motor1.setMaxSpeed(700);
  motor1.setAcceleration(200);

  motor2.setMaxSpeed(700);
  motor2.setAcceleration(200);

  Serial.begin(115200);
  Serial.println("\nCircumference: " + String(wheelCircumference));
  Serial.println("Dist per Step: " + String(distPerStep));
  WiFi.mode(WIFI_STA);
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

  // Set server routing
  restServerRouting();
  // Set not found response
  server.onNotFound(handleNotFound);
  // Start server
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  if (!isRunning)
    server.handleClient();

  motor1.run();
  motor2.run();

  if (motor1.distanceToGo() == 0 && motor2.distanceToGo() == 0 && isRunning) {
    isRunning = false;
    penUp();
    sendResponse(httpResp, 200);
  }
}
