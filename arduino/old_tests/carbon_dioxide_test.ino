// Basic demo for readings from Adafruit SCD30
#include <Adafruit_SCD30.h>

Adafruit_SCD30  scd30;
// set to the pins connected to SEL on the multiplexor
const int SEL_PIN[3] = {22, 24, 26};

const int NUM_OF_SENS = 2;

float temperature[NUM_OF_SENS];
float humidity[NUM_OF_SENS];
float CO2[NUM_OF_SENS];


// prototypes
void print_data(float temperature[NUM_OF_SENS], float humidity[NUM_OF_SENS], float CO2[NUM_OF_SENS]);
void sel_numbers(int n, int sel_settings[3]);
void sel_pins(int sel_settings[3]);

void setup(void) {
  //pin modes
  pinMode(SEL_PIN[0], OUTPUT);
  pinMode(SEL_PIN[1], OUTPUT);
  pinMode(SEL_PIN[2], OUTPUT);

  digitalWrite(SEL_PIN[0], LOW);
  digitalWrite(SEL_PIN[1], LOW);
  digitalWrite(SEL_PIN[2], LOW);


  Serial.begin(9600);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit SCD30 test!");

  //Try to initialize!
  if (!scd30.begin()) {
    Serial.println("Failed to find SCD30 chip");
    while (1) { delay(10); }
  }
  Serial.println("bruh");


  // if (!scd30.setMeasurementInterval(10)){
  //   Serial.println("Failed to set measurement interval");
  //   while(1){ delay(10);}
  // }
  Serial.print("Measurement Interval: "); 
  Serial.print(scd30.getMeasurementInterval()); 
  Serial.println(" seconds");

  


  }


void loop() {
  for (int i = 0; i < NUM_OF_SENS; i++){
    // sets the SDA using the multiplexor
    sel_pins(i);

    // error reading check
    if (!scd30.read()){ Serial.println("Error reading sensor data"); 
    delay(1000);
    return; }

    // saves current info into the arrays
    temperature[i] = scd30.temperature;
    humidity[i] = scd30.relative_humidity;
    CO2[i] = scd30.CO2;
  } 

  print_data(temperature, humidity, CO2);


}

void print_data(float temperature[NUM_OF_SENS], float humidity[NUM_OF_SENS], float CO2[NUM_OF_SENS]){
  for (int i = 0; i < NUM_OF_SENS; i++){
    // prints out all sensors, repeating until all have been printed
    Serial.print ("Sensor #");
    Serial.println (i);
    Serial.print("Temperature: ");
    Serial.print(temperature[i]);
    Serial.println(" degrees C");
    
    Serial.print("Relative Humidity: ");
    Serial.print(humidity[i]);
    Serial.println(" %");
    
    Serial.print("CO2: ");
    Serial.print(CO2[i], 3);
    Serial.println(" ppm");
    Serial.println("");
  }
}

void sel_numbers(int n, int sel_settings[3]){  
  if (n > 8|| n < 0)
  Serial.println("Invalid sensor number");
  int i = 0; 
  while (n > 0) { 
    // storing remainder in binary array 
    sel_settings[i] = n % 2; 
    n = n / 2; 
    i++; 
  } 
}

void sel_pins(int num){
  int sel_settings[3] = {0};
  sel_numbers(num, sel_settings);

  for (int i = 0; i < 3; i++){
    if (sel_settings[i] == 0)
      digitalWrite(SEL_PIN[i], LOW);

    else if (sel_settings[i] == 1)
      digitalWrite(SEL_PIN[i], HIGH);
  }

  delay(1000);


}
