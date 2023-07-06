#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include "index.h"
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>

// On the ESP32S2 SAOLA GPIO is the NeoPixel.
#define PIN        18 

//Single NeoPixel
Adafruit_NeoPixel pixels(1, PIN, NEO_GRB + NEO_KHZ800);

//Max
//Max Y=300 Min Y = 120

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

#define total_point 200
#define EEPROM_SIZE 4096

const char* ssid = "SCARA V1";
const char* password = "12345678";

// define two tasks for Blink & AnalogRead
void TaskSensor( void *pvParameters );
void TaskDisplay( void *pvParameters );

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Link lengths in centimeters
const float L1 = 145.0;
const float L2 = 155.5;

IPAddress IP;

//L1=145
//L2=155.5

// Define pin connections & motor's steps per revolution
const int J1dirPin =  33;
const int J1stepPin = 34;
const int J2dirPin =  35;
const int J2stepPin = 36;
const int J3dirPin =  37;
const int J3stepPin = 38;
const int J4dirPin =  39;
const int J4stepPin = 40;
const int enPin1 = 20;

const int SW1 = 1;
const int SW2 = 2;
const int SW3 = 3;

const int stepsPerRevolution = 200*32*4;
const int stepsDelay = 200;
const int homeDelay = 150;
const int z_step = 200*32*1;

WebServer server(80);

float wx[total_point];
float wy[total_point];
float z_offset=0.0;
int w_prt = 0;

String task_status = "";
boolean bg_flag = false;
boolean run_flag = false;

int
THIS_J1 = 0,
THIS_J2 = 0,
THIS_J3 = 0,
LAST_J1 = 0,
LAST_J2 = 0,
LAST_J3 = 0;


float
X,                                //gcode float values held here
Y,
Z,
J1,
J2,
J3,
J4,
J5
;

float XT[2];
float JT[2];

void setup()
{
  Serial.begin(115200);
  
  //This pixel is just way to bright, lower it to 10 so it does not hurt to look at.
  pixels.setBrightness(100);
  pixels.begin(); // INITIALIZE NeoPixel (REQUIRED)
  //pixels.clear();
  pixels.setPixelColor(0, pixels.Color(0, 255, 0)); //RGB
  pixels.show();   // Send the updated pixel colors to the hardware.
    
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  
  // Declare pins as Outputs
  pinMode(J1stepPin, OUTPUT);
  pinMode(J1dirPin, OUTPUT);
  pinMode(J2stepPin, OUTPUT);
  pinMode(J2dirPin, OUTPUT);
  pinMode(J3stepPin, OUTPUT);
  pinMode(J3dirPin, OUTPUT);
  pinMode(enPin1, OUTPUT);
  digitalWrite(enPin1, LOW);

  pinMode(SW1, INPUT_PULLUP);
  pinMode(SW2, INPUT_PULLUP);
  pinMode(SW3, INPUT_PULLUP);

  // Now set up two tasks to run independently.
  xTaskCreatePinnedToCore(
    TaskSensor
    ,  "TaskSensor"   // A name just for humans
    ,  4096  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  3  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    TaskDisplay
    ,  "TaskDisplay"
    ,  4096  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);

  if (!EEPROM.begin(EEPROM_SIZE))
  {
    Serial.println("failed to initialise EEPROM");
  }

  ee_load();
  //ee_store();

  //WiFi.begin(ssid, password);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  IP = WiFi.softAPIP();
  start_server();
  
  show_display();
  bg_flag = true;
  go_home();
  bg_flag = false;
  THIS_J1 = 0,
  THIS_J2 = 0,
  THIS_J3 = 0,
  LAST_J1 = 0,
  LAST_J2 = 0,
  LAST_J3 = 0;
  delay(200);
  //digitalWrite(enPin1, HIGH);
}
void loop()
{
  if (run_flag){
    test();
  }
  else{
    go_position();
  }
}

void go_position(){
  XT[0]=X; //x
  XT[1]=Y; //y
  XT[2]=Z; //Z
  move_z(XT[2]);
  InverseK(XT, JT);
  if (((JT[0]>-180)&&(JT[0]<180))&&((JT[1]>-180)&&(JT[1]<180))){
    move_xy(JT[0], JT[1]); //200,250
  }
  else{
    Serial.println("IK math error");
    Serial.print(JT[0]);
    Serial.print(",");
    Serial.println(JT[1]);
  }
}

void test(){
  bg_flag = true;
  task_status = "Progress: " + String(0) + " %";
  for (int i=0;i<w_prt;i++){
    X = wx[i];
    Y = wy[i];
    Z = 55;
    XT[0]=X; //x
    XT[1]=Y; //y
    XT[2]=Z; //Z
    InverseK(XT, JT);
    if (((JT[0]>-180)&&(JT[0]<180))&&((JT[1]>-180)&&(JT[1]<180))){
      move_xy(JT[0], JT[1]); //200,250
    }
    else{
      Serial.println("IK math error");
      Serial.print(JT[0]);
      Serial.print(",");
      Serial.println(JT[1]);
    }
    move_z(XT[2]);
    delay(1000);
    Z = 0;
    XT[2]=Z; //Z
    move_z(XT[2]);
    int percentage = map(i, 0, w_prt, 0, 100);
    task_status = "Progress: " + String(percentage) + " %";
  }
  task_status = "Progress: " + String(100) + " %";
  X=0;
  Y=100;
  Z=0;
  run_flag=false;
  bg_flag=false;
}

void ee_load(){
  int i=0,
  address=0;
  
  for (int i=0;i<total_point;i++){
    wx[i] = EEPROM.readFloat(address); 
    address += sizeof(float);
  }
  
  for (int i=0;i<total_point;i++){
    wy[i] = EEPROM.readFloat(address); 
    address += sizeof(float);
  }

  w_prt = EEPROM.readInt(address); 
  address += sizeof(int);

  z_offset = EEPROM.readInt(address); 
  address += sizeof(float);
  
  Serial.print("Read: ");
  Serial.println(address);

}

void ee_store(){
  
  int i=0,
  address=0;
  
  for (int i=0;i<total_point;i++){
    EEPROM.writeFloat(address, wx[i]); 
    address += sizeof(float);
  }
  
  for (int i=0;i<total_point;i++){
    EEPROM.writeFloat(address, wy[i]); 
    address += sizeof(float);
  }
  EEPROM.commit();
  
  EEPROM.writeInt(address, w_prt);
  address += sizeof(int);
  EEPROM.commit();

  EEPROM.writeInt(address, z_offset);
  address += sizeof(float);
  EEPROM.commit();
  
  Serial.print("Write: ");
  Serial.println(address);
  
}

void show_display(){
  display.clearDisplay();
  display.drawRect(0, 0, display.width()-1, display.height()-1, SSD1306_WHITE);
  display.setTextSize(1); // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  
  display.setCursor(5, 5);
  display.print("X:");
  display.print(XT[0]);
  display.setCursor(5, 15);
  display.print("Y:");
  display.print(XT[1]);
  display.setCursor(5, 25);
  display.print("Z:");
  display.print(XT[2]);

  display.setCursor(58, 5);
  display.print("J1:");
  display.print(JT[0]);
  display.print("\xF7"); 
  display.setCursor(58, 15);
  display.print("J2:");
  display.print(JT[1]);
  display.print("\xF7"); 
  display.setCursor(58, 25);
  display.print("J3:");
  display.print(JT[2]);
  display.print("\xF7");

  display.setCursor(5, 37);
  display.print("http://");
  display.print(IP);
  
  display.setCursor(5, 50);
  display.print("SW: ");

  if (digitalRead(SW1)==LOW) display.print(" OFF");
  else display.print(" ON ");
  if (digitalRead(SW2)==LOW) display.print(" OFF");
  else display.print(" ON ");
  if (digitalRead(SW3)==LOW) display.print(" OFF");
  else display.print(" ON ");
  
  display.display();      // Show initial text
}

void go_home(){
  
  while(digitalRead(SW3)==LOW){
    z_incr();
  }
  //delay(200);
  //move_z(5);
  delay(200);
  while(digitalRead(SW2)==LOW){
    digitalWrite(J2dirPin, HIGH);
    digitalWrite(J2stepPin, HIGH);
    delayMicroseconds(homeDelay);
    digitalWrite(J2stepPin, LOW);
    delayMicroseconds(homeDelay);
  }
  delay(200);
  while(digitalRead(SW1)==LOW){
    digitalWrite(J1dirPin, HIGH);
    digitalWrite(J1stepPin, HIGH);
    delayMicroseconds(homeDelay);
    digitalWrite(J1stepPin, LOW);
    delayMicroseconds(homeDelay);
  }
  delay(200);
  int cnt=1900;
  while (cnt){
    digitalWrite(J1dirPin, LOW);
    digitalWrite(J1stepPin, HIGH);
    delayMicroseconds(homeDelay);
    digitalWrite(J1stepPin, LOW);
    delayMicroseconds(homeDelay);
    cnt--;
  }
  cnt=6400; //90'
  while (cnt){
    digitalWrite(J1dirPin, LOW);
    digitalWrite(J1stepPin, HIGH);
    delayMicroseconds(homeDelay);
    digitalWrite(J1stepPin, LOW);
    delayMicroseconds(homeDelay);
    cnt--;
  }
  cnt=4500;
  while (cnt){
    digitalWrite(J2dirPin, LOW);
    digitalWrite(J2stepPin, HIGH);
    delayMicroseconds(homeDelay);
    digitalWrite(J2stepPin, LOW);
    delayMicroseconds(homeDelay);
    cnt--;
  }
  cnt=6400; //90'
  while (cnt){
    digitalWrite(J2dirPin, LOW);
    digitalWrite(J2stepPin, HIGH);
    delayMicroseconds(homeDelay);
    digitalWrite(J2stepPin, LOW);
    delayMicroseconds(homeDelay);
    cnt--;
  }
  THIS_J1 = 0, 
  THIS_J2 = 0, 
  THIS_J3 = 0, 
  LAST_J1 = 0, 
  LAST_J2 = 0, 
  LAST_J3 = 0; 
  
  delay(200);
  Serial.print(THIS_J1);
  Serial.print(",");
  Serial.println(THIS_J2);
  X=0;
  Y=100;
  Z=0;
}

void z_incr() {
  //Serial.println("Z++");
  digitalWrite(J3dirPin, LOW);
  digitalWrite(J3stepPin, HIGH);
  delayMicroseconds(stepsDelay);
  digitalWrite(J3stepPin, LOW);
  delayMicroseconds(stepsDelay);
}

void z_decr() {
  //Serial.println("Z--");
  digitalWrite(J3dirPin, HIGH);
  digitalWrite(J3stepPin, HIGH);
  delayMicroseconds(stepsDelay);
  digitalWrite(J3stepPin, LOW);
  delayMicroseconds(stepsDelay);
}

void j1_step_cw(){
  // Set motor direction clockwise
  digitalWrite(J1dirPin, HIGH);
  digitalWrite(J1stepPin, HIGH);
  delayMicroseconds(stepsDelay);
  digitalWrite(J1stepPin, LOW);
  delayMicroseconds(stepsDelay);
}

void j1_step_ccw(){
  // Set motor direction counterclockwise
  digitalWrite(J1dirPin, LOW);
  digitalWrite(J1stepPin, HIGH);
  delayMicroseconds(stepsDelay);
  digitalWrite(J1stepPin, LOW);
  delayMicroseconds(stepsDelay);
}


void j2_step_cw(){
  // Set motor direction clockwise
  digitalWrite(J2dirPin, LOW);
  digitalWrite(J2stepPin, HIGH);
  delayMicroseconds(stepsDelay);
  digitalWrite(J2stepPin, LOW);
  delayMicroseconds(stepsDelay);
}

void j2_step_ccw(){
  // Set motor direction counterclockwise
  digitalWrite(J2dirPin, HIGH);
  digitalWrite(J2stepPin, HIGH);
  delayMicroseconds(stepsDelay);
  digitalWrite(J2stepPin, LOW);
  delayMicroseconds(stepsDelay);
}

void move_z(float z){
  THIS_J3 = round(z * 90);
  int dz = LAST_J3 - THIS_J3;                         //distance
  int l = abs(dz);
  for (int i = 0; i < l; i++) {
    if (dz < 0) {
      z_decr();                         //move pen 1 step down
    } else {
      z_incr();                        //move pen 1 step up
    }
  }
  LAST_J3 = THIS_J3;
}



void move_xy(float j1, float j2) {                        //x,y are absolute co-ordinates

  // ----- apply scale factor
  THIS_J1 = round(j1 * (stepsPerRevolution/360.0));      //scale x and y
  THIS_J2 = round(j2 * (stepsPerRevolution/360.0));  //THIS_Z = round(z * STEPS_PER_MM * SCALE_FACTOR);

  //Serial.print(THIS_J1);
  //Serial.print(",");
  //Serial.println(THIS_J2);
  //move_z(LAST_Z,THIS_Z);
  // ----- draw a line between these "scaled" co-ordinates
  xy_line(LAST_J1, LAST_J2, THIS_J1, THIS_J2);

  // ----- remember last "scaled" co-ordinate
  LAST_J1 = THIS_J1;
  LAST_J2 = THIS_J2;
  
}

void xy_line(int j1_a, int j2_a, int j1_b, int j2_b) {          //these are "scaled" co-ordinates

  // ----- locals
  int
  j1 = j1_a,                               //current "scaled" X-axis position
  j2 = j2_a,                               //current "scaled" Y-axis position
  dj1,                                   //line slope
  dj2,
  slope,
  longest,                              //axis lengths
  shortest,
  maximum,
  error,                                //bresenham thresholds
  threshold;

  // ----- find longest and shortest axis
  dj1 = j2_b - j2_a;                         //vertical distance
  dj2 = j1_b - j1_a;                         //horizontal distance
  longest = max(abs(dj2), abs(dj1));      //longest axis
  shortest = min(abs(dj2), abs(dj1));     //shortest axis

  // ----- scale Bresenham values by 2*longest
  error = -longest;                     //add offset to so we can test at zero
  threshold = 0;                        //test now done at zero
  maximum = (longest << 1);             //multiply by two
  slope = (shortest << 1);              //multiply by two ... slope equals (shortest*2/longest*2)

  // ----- initialise the swap flag
  /*
     The XY axes are automatically swapped by using "longest" in
     the "for loop". XYswap is used to decode the motors.
  */
  bool XYswap = true;                    //used for motor decoding
  if (abs(dj1) >= abs(dj2)) XYswap = false;

  // ----- pretend we are always in octant 0
  /*
     The current X-axis and Y-axis positions will now be incremented (decremented) each time
     through the loop. These intermediate steps are parsed to the plot(x,y) function which calculates
     the number of steps required to reach each of these intermediate co-ordinates. This effectively
     linearises the plotter and eliminates unwanted curves.
  */
  for (int i = 0; i < longest; i++) {

    // ----- move left/right along X axis
    if (XYswap) {   //swap
      if (dj2 < 0) {
        j1--;
        j1_step_ccw();
      } else {
        j1++;
        j1_step_cw();
      }
    } else {    //no swap
      if (dj1 < 0) {
        j2--;
        j2_step_ccw();
      } else {
        j2++;
        j2_step_cw();
      }
    }

    // ----- move up/down Y axis
    error += slope;
    if (error > threshold) {
      error -= maximum;

      // ----- move up/down along Y axis
      if (XYswap) {  //swap
        if (dj1 < 0) {
          j2--;
          j2_step_ccw();
        } else {
          j2++;
          j2_step_cw();
        }
      } else {  //no swap
        if (dj2 < 0) {
          j1--;
          j1_step_ccw();
        } else {
          j1++;
          j1_step_cw();
        }
      }
    }
  }
}

void InverseK(float* Xik, float* Jik)
{
    // Define the inverse kinematics variables
    float r1 = 0.0;
    float phi_1 = 0.0;
    float phi_2 = 0.0;
    float phi_3 = 0.0;
    float theta_1 = 0.0;
    float theta_2 = 0.0;

    float x_pos = Xik[0];
    float y_pos = Xik[1];
    /* Calculate the inverse kinematics*/
    // (1) r1  = square_root((x_0_2)^2 + (y_0_2)^2) 
    // Value is in centimeters
    r1 = sqrt((x_pos * x_pos) + (y_pos * y_pos));
 
    // (2) ϕ1 = arccos((L2^2 - r1^2 - L1^2)/(-2*r1*L1))
    // The returned value is in the range [0, pi] radians. 
    phi_1 = acos(((L2 * L2) - (r1 * r1) - (L1 * L1))/(-2.0 * r1 * L1));
 
    // (3) ϕ2 = arctan((y_0_2) / (x_0_2)) 
    // The atan2() function computes the principal value of the 
    // arc tangent of y/x, using the signs of both arguments to 
    // determine the quadrant of the return value. 
    // The returned value is in the range [-pi, +pi] radians.
    phi_2 = atan2(y_pos,x_pos);
 
    // (4) θ1 =  ϕ1 - ϕ2
    // Value is in radians
    theta_1 = phi_2 - phi_1;
 
    // (5) ϕ3 = arccos((r1^2 - L1^2 - L2^2)/(-2*L1*L2))
    // Value is in radians
    phi_3 = acos(((r1 * r1) - (L1 * L1) - (L2 * L2))/(-2.0 * L1 * L2));
 
    //(6) θ2 = 180° - ϕ3 
    theta_2 = PI - phi_3;
     
    /* Convert the joint (servo) angles from radians to degrees*/
    theta_1 = theta_1 * RAD_TO_DEG; // Joint 1
    theta_2 = theta_2 * RAD_TO_DEG; // Joint 2

    theta_1 = 90 - theta_1;
  
    Jik[0] = theta_1;
    Jik[1] = theta_2;
}

void handleRoot() 
{
  String s = webpage;
  server.send(200, "text/html", s);
}

void monitor() 
{
  String mpos = String(X)+","+String(Y)+","+String(Z) + "<br>";
  //mpos = "";
  mpos += task_status + "<br>";
  server.send(200, "text/plane", mpos);
}

void send_points() 
{
  //String set_points = String(THIS_X/STEPS_PER_MM_F)+","+String(THIS_Y/STEPS_PER_MM_F)+","+String(THIS_Z/STEPS_PER_MM_F);
  String set_points = "";
  for (int i=0;i<w_prt;i++){
    set_points = set_points + "P" + String(i+1) + ": " + String(wx[i]) + "," + String(wy[i]) + "<br>";
  }
  
  server.send(200, "text/plane", set_points);
}

void control() 
{
  String button = server.arg("state");
  //Serial.println(button);
  if (button.equals("mon")) {
    ;
    //motor_enable();
  }
  else if (button.equals("mof")) {
    ;
    //motor_disable();
  }
  else if (button.equals("xp1")) {
    X=X+0.1;
    //move_xy(X,Y);
  }
  else if (button.equals("xn1")) {
    X=X-0.1;
    //move_xy(X,Y);
  }
  else if (button.equals("yp1")) {
    Y=Y+0.1;
    //move_xy(X,Y);
  }
  else if (button.equals("yn1")) {
    Y=Y-0.1;
    //move_xy(X,Y);
  }
  else if (button.equals("xp2")) {
    X=X+1;
    //move_xy(X,Y);
  }
  else if (button.equals("xn2")) {
    X=X-1;
    //move_xy(X,Y);
  }
  else if (button.equals("yp2")) {
    Y=Y+1;
    //move_xy(X,Y);
  }
  else if (button.equals("yn2")) {
    Y=Y-1;
    //move_xy(X,Y);
  }
  else if (button.equals("xp3")) {
    X=X+5;
    //move_xy(X,Y);
  }
  else if (button.equals("xn3")) {
    X=X-5;
    //move_xy(X,Y);
  }
  else if (button.equals("yp3")) {
    Y=Y+5;
    //move_xy(X,Y);
  }
  else if (button.equals("yn3")) {
    Y=Y-5;
    //move_xy(X,Y);
  }
  else if (button.equals("xp4")) {
    X=X+10;
    //move_xy(X,Y);
  }
  else if (button.equals("xn4")) {
    X=X-10;
    //move_xy(X,Y);
  }
  else if (button.equals("yp4")) {
    Y=Y+10;
    //move_xy(X,Y);
  }
  else if (button.equals("yn4")) {
    Y=Y-10;
    //move_xy(X,Y);
  }
  else if (button.equals("xp5")) {
    X=X+50;
    //move_xy(X,Y);
  }
  else if (button.equals("xn5")) {
    X=X-50;
    //move_xy(X,Y);
  }
  else if (button.equals("yp5")) {
    Y=Y+50;
    //move_xy(X,Y);
  }
  else if (button.equals("yn5")) {
    Y=Y-50;
    //move_xy(X,Y);
  }
  else if (button.equals("zp3")) {
    Z=Z+5;
    //move_z(Z);
  }
  else if (button.equals("zp2")) {
    Z=Z+1;
    //move_z(Z);
  }
  else if (button.equals("zp1")) {
    Z=Z+0.1;
    //move_z(Z);
  }
  else if (button.equals("zn3")) {
    Z=Z-5;
    //move_z(Z);
  }
  else if (button.equals("zn2")) {
    Z=Z-1;
    //move_z(Z);
  }
  else if (button.equals("zn1")) {
    Z=Z-0.1;
    //move_z(Z);
  }
  else if (button.equals("feed")) {
    ;
    //motor_enable();
    //feed_out(1);
  }
  else if (button.equals("run")) {
    //motor_enable();
    //task();
    run_flag=true;
  }
  else if (button.equals("xy0")) {
    ;
    //THIS_X = 0;
    //THIS_Y = 0;
    //LAST_X = 0;
    //LAST_Y = 0;
  }
  else if (button.equals("z0")) {
    ;
    //THIS_Z = 0;
    //LAST_Z = 0;
  }
  else if (button.equals("add")) {
    wx[w_prt]= X;
    wy[w_prt]= Y;
    w_prt=w_prt+1;
  }
  else if (button.equals("del")) {
    wx[w_prt]= 0.0;
    wy[w_prt]= 0.0;
    w_prt=w_prt-1;
  }
  else if (button.equals("str")) {
    ee_store();
  }
  else if (button.equals("rst")) {
    for (int i=0;i<=w_prt;i++){
      wx[i]= 0.0;
      wy[i]= 0.0;
    }
    w_prt=0;
  }
  else;
  
  //move_xy(x,y); xy0, z0
  //move_z(z);
  server.send(200, "text/plane", "OK");
}

void start_server(){
  server.on("/", handleRoot);
  server.on("/led_set", control);
  server.on("/mpos", monitor);
  server.on("/points", send_points);
  server.begin();

  Serial.println("HTTP Server Started");
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskSensor(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  for (;;) // A Task shall never return or exit.
  {
    server.handleClient();
    vTaskDelay(15);  // one tick delay (15ms) in between reads for stability
  }
}

void TaskDisplay(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  for (;;)
  {
    show_display();
    if (bg_flag){
      pixels.setPixelColor(0, pixels.Color(255, 0, 0));
      pixels.show();
      vTaskDelay(200);
      pixels.setPixelColor(0, pixels.Color(0, 0, 0));
      pixels.show();
      vTaskDelay(200);
    }
    else{
      pixels.setPixelColor(0, pixels.Color(0, 0, 0));
      pixels.show();
    }
    //vTaskDelay(100);  // one tick delay (100ms) in between reads for stability
  }
}
