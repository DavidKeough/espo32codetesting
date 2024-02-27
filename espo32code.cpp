#include <SPI.h>
#include <max6675.h>

// Define the pins connected to the MAX6675K
#define MAX6675_CS_PIN 5
#define MAX6675_SO_PIN 19
#define MAX6675_SCK_PIN 18

// Define the MOSFET pin
#define MOSFET_PIN 4  // Example pin for MOSFET control

// Define temperature setpoint
#define SETPOINT_TEMPERATURE 210.0  // Desired temperature in Celsius

// PID constants (adjust as needed)
#define KP 1.0
#define KI 0.1
#define KD 0.1

float currentTemperature = 0.0;
float previousError = 0.0;
float integralTerm = 0.0;


MAX6675 thermocouple(MAX6675_SCK_PIN, MAX6675_CS_PIN, MAX6675_SO_PIN);

void setup() {
	Serial.begin(9600);
	pinMode(MOSFET_PIN, OUTPUT);
	Serial.println("MAX6675K Thermocouple Test");
}

void loop() {

	// Read temperature from the thermocouple
	float temperature = thermocouple.readCelsius();


	if (temperature == NAN) {
		Serial.println("Unable to read temperature :(");

		//Send shutoff signal to mosfet on bad temp reading
		analogWrite(MOSFET_PIN, 255.0);
	}

	
	//currentTemperature = temperature;  // Update current temperature

	////Perform PID control
	//float error = SETPOINT_TEMPERATURE - temperature;
	//integralTerm += error; // Accumulate error for integral term
	//float derivativeTerm = error - previousError; // Calculate derivative term
	//previousError = error; // Update previous error


	//float output = (KP * error) + (KI * integralTerm) + (KD * derivativeTerm); // Calculate PID output

	//Serial.print("PID output");
	//Serial.println(output);

	//// Clip the output to ensure it's within valid range
	//output = constrain(output, 0.0, 255.0);

	// Apply PWM to MOSFET pin to control heating element
	
	if (temperature >= SETPOINT_TEMPERATURE) {
		analogWrite(MOSFET_PIN, 255);
	}

	// Print temperature and PID components to serial monitor
	Serial.print("Temperature: ");
	Serial.print(temperature);
	Serial.println(" °C");
	Serial.print("PID Output: ");
	Serial.println(output);
	delay(1000); // Delay for readability
}