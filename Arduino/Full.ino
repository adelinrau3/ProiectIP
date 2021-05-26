//Libraries
#include <DHT.h>
#include <SoftwareSerial.h>  

//Constants
#define DHTPIN 4     // what pin we're connected to
#define DHTTYPE DHT11   // DHT 11  (AM2302)
// Initialize DHT sensor for normal 16mhz Arduino
DHT dht(DHTPIN, DHTTYPE);
int bluetoothTx = 2;  // TX-O pin of bluetooth mate, Arduino D2
int bluetoothRx = 3;  // RX-I pin of bluetooth mate, Arduino D3
SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);
//Variables
int chk;
float hum;  //Stores humidity value
float temp; //Stores temperature value

int PulseSensorPurplePin = 1;        // Pulse Sensor PURPLE WIRE connected to ANALOG PIN 0
int Signal;                // holds the incoming raw data. Signal value can range from 0-1024

void setup()
{
    Serial.begin(9600);
    dht.begin();
    pinMode(10, INPUT); // Setup for leads off detection LO +
    pinMode(11, INPUT); // Setup for leads off detection LO -
    bluetooth.begin(115200);  // The Bluetooth Mate defaults to 115200bps
    bluetooth.print("$");  // Print three times individually
    bluetooth.print("$");
    bluetooth.print("$");  // Enter command mode
    delay(100);  // Short delay, wait for the Mate to send back CMD
    bluetooth.println("U,9600,N");  // Temporarily Change the baudrate to 9600, no parity
    // 115200 can be too fast at times for NewSoftSerial to relay the data reliably
    bluetooth.begin(9600);  // Start bluetooth serial at 9600
}

void loop()
{
  //Temperature and humidity
  //Read data and store it to variables hum and temp
  hum = dht.readHumidity();
  temp= dht.readTemperature();
  //Print temp and humidity values to serial monitor
  Serial.print("Humidity: ");
  Serial.print(hum);
  Serial.println(" %");
  Serial.print("Temp: ");
  Serial.print(temp);
  Serial.println(" Celsius");

  //EKG
  Serial.print("EKG: ");
  Serial.println(analogRead(A0));
  //delay(1000); //Delay 2 sec.

  //Pulse
  Signal = analogRead(PulseSensorPurplePin);  // Read the PulseSensor's value.
                                              // Assign this value to the "Signal" variable.                                            
  Serial.print("Pulse: ");
  Serial.println(analogRead(A1)); // Send the Signal value to Serial Plotter.
  Serial.println("");

  //datele spre aplicatie
  bluetooth.println(temp);
  bluetooth.println(hum);
  bluetooth.println(analogRead(A0));
  bluetooth.println(analogRead(A1));
  bluetooth.println("");
  
  delay(1000);
}
//Humidity: hum
//Temperature: temp
//EKG: analogRead(A0)
//Pulse: analogRead(A1)
   
