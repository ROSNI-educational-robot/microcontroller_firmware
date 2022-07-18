#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

// Commands
// Requests
#define HOTSPOT_ON 15
#define ROS_INIT 25
#define ROS_KILL 35
// Responses
#define FAIL 0
#define CONNECTED 10
#define IP_RECIEVED 20
#define ROS_OK 30

// ESTATES
int E1, E2, E3, E4, E5, E6, E7;

// Hardware variables
int led = 11;
int button = 12;
const int battery = A1;
//int battery_charge = 100;

// Program variables
int serial_connection = 0;
int ros_ok = 0;
int ros_kill_ok = 0;
int ip_recieved = 0;
String current_ip = "";

int temps_ant = 0;
int temps_new = 0;
int t_pressed = 0;

int puls = 1;
int led_blink = 0;
int temps_blink = 0;
int temps_blink_ant = 0;
int temps_blink_new = 0;

LiquidCrystal_I2C lcd(0x27, 16, 2);  // Inicia el LCD en la dirección 0x27, con 16 caracteres y 2 líneas

void check_battery()
{
  int a = ((analogRead(battery)-769)*100/267);
  if (a>=95)
  {
    a = 10;
  }
  else
  {
    a = a/10;
  }
  lcd.setCursor(12, 1);
  lcd.print(String(a*10));
  lcd.print("%");
  //return a;
}

String serial_read()
{
  if (Serial.available() > 0) {
    //Se crea una variable que servirá como buffer
    String bufferString = "";
    //int n=0;
    char a='0';
    a=Serial.read();
    delay(2);
    if (a=='X'){
      a=Serial.read();
      while(a!='Y'){
        delay(2);
        bufferString += a;
        a=Serial.read();
      }
      //Serial.println(bufferString);
      //n=bufferString.toInt();
      //Serial.print("Recieved int:");
      //Serial.println(n);
      return bufferString;
    }
  }
  return "";
}

void serial_write(int req)
{
  String msg = "X"+String(req)+"Y";
  Serial.print(msg);
}

void setup() // A0
{
  Serial.begin(9600);
  Serial.println("Estat 1");
  E1 = 1;
  E2 = 0; E3 = 0; E4 = 0; E5 = 0; E6 = 0; E7 = 0;
  
  pinMode(led, OUTPUT);
  pinMode(button, INPUT);

  digitalWrite(led, LOW);
  
  lcd.backlight();
  lcd.begin();                      
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("    Welcome!");
  check_battery();
  delay(2000);
}

void loop()
{
  // ~~~~~~~~~~~~~~~ Estats ~~~~~~~~~~~~~~~~
  if(digitalRead(button) == LOW and (E2 or E4 or E6))
  {
    Serial.print("pressed");
    temps_new = millis();
    temps_ant = temps_new;
    bool done = false;
    while (digitalRead(button) == LOW and !done)
    {
      temps_new = millis();
      t_pressed = temps_new - temps_ant;
      // Wait button to start Hotspot
      if (E2 and t_pressed>=1000)
      {
        Serial.println("Estat 3");
        E3 = 1;
        E2 = 0;
        done = true;

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Starting...");
        lcd.setCursor(0, 1);
        lcd.print("Plz wait");

        led_blink = 1;

        serial_write(HOTSPOT_ON);
        delay(10);
      }
      // Wait button to start ROS
      if (E4 and t_pressed>=2000)
      {
        Serial.println("Estat 5");
        E5 = 1;
        E4 = 0;
        done = true;
        
        serial_write(ROS_INIT);
        delay(10);
      }
      // ROS ON, wait button to kill it
      if (E6 and t_pressed>=5000)
      {
        Serial.println("Estat 7");
        E7 = 1;
        E6 = 0;
        done = true;

        serial_write(ROS_KILL);
        delay(10);
      }
    }
  }

  // Wait serial communication
  if (E1 and serial_connection)
  {
    Serial.println("Estat 2");
    E2 = 1;
    E1 = 0;

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("<- Hold Button");
    lcd.setCursor(0, 1);
    lcd.print("to start!");
    serial_connection = 0;
  }
  
  // Send Hotspot Request and wait
  if ((E3 and ip_recieved) or (E7 and ros_kill_ok))
  {
    Serial.println("Estat 4");
    E4 = 1;
    E3 = 0;
    E7 = 0;
    ros_kill_ok = 0;
    led_blink = 1;
    lcd.setCursor(0, 1);
    lcd.print("ROS OFF");
  }
  
  // Send ROS init Request
  if (E5 and ros_ok)
  {
    Serial.println("Estat 6");
    E6 = 1;
    E5 = 0;
    ros_ok = 0;
    led_blink = 0;
  }

  // ~~~~~~~~~~~~~~~~ Actions ~~~~~~~~~~~~~~~~~
  if (E1) // Wait serial communication + print "Welcome" + print battery
  {
    int msg = serial_read().toInt();
    if (msg == CONNECTED)
    {
      serial_connection = 1; 
    }
    check_battery();
  }
  
  if (E2) // Nothing to do, just wait to the button to be pressed + Print "Press button to start" + battery
  {
    check_battery();
  }
  
  if (E3) // Send Hotspot start request and wait ip response + print "Starting... Please Wait" + battery
  {
    
    int msg = serial_read().toInt();
    if (msg == IP_RECIEVED)
    {
      current_ip = serial_read();
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(current_ip);
      lcd.setCursor(0, 1);
      lcd.print("ROS OFF");
      ip_recieved = 1;
    }
    check_battery();
  }
  
  if (E4) // wait button to be pressed + get ip from computer + print ip + battery
  {
    int msg = serial_read().toInt();
    if (msg == IP_RECIEVED)
    {
      delay(5);
      current_ip = serial_read();
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(current_ip);
      lcd.setCursor(0, 1);
      lcd.print("ROS OFF");
    }
    check_battery();
  }
  
  if (E5) // Send Ros_Init request and wait response
  {
    int msg = serial_read().toInt();
    if (msg == ROS_OK)
    {
      ros_ok = 1;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(current_ip);
      lcd.setCursor(0, 1);
      lcd.print("ROS ON");
    }
    check_battery();
  }
  
  if (E6) // Nothing to do, just wait untill button is pressed + battery
  {
    digitalWrite(led, HIGH);
    check_battery();
  }
  if (E7) // Send Ros_Kill request and wait response
  {
    int msg = serial_read().toInt();
    if (msg == ROS_OK)
    {
      ros_kill_ok = 1;
    }
    check_battery();
  }

  // Led Blinking
  if (led_blink)
  {
    temps_blink_new = millis();
    temps_blink = temps_blink_new - temps_blink_ant;
    if (E3 and temps_blink >= 200) // Fast
    {
      puls = !puls;
      digitalWrite(led, puls);
      temps_blink_ant = temps_blink_new;
    }
    if (E4 and temps_blink >=600) // Slow
    {
      puls = !puls;
      digitalWrite(led, puls);
      temps_blink_ant = temps_blink_new;
    }
  }
}
