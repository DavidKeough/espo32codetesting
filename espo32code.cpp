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

// PID variables
int PID_P = 0;
int PID_I = 0;
int PID_D = 0;
int PID_Error = 0;
int PID_Value = 0;

float currentTemperature = 0.0;
float previousError = 0.0;
// float integralTerm = 0.0;
float elapsedTime, Time, timePrev;


MAX6675 thermocouple(MAX6675_SCK_PIN, MAX6675_CS_PIN, MAX6675_SO_PIN);

void setup() {
	Serial.begin(9600);
	pinMode(MOSFET_PIN, OUTPUT);
	Time = millis();
	Serial.println("MAX6675K Thermocouple Test");
}

void loop() {

	// Read temperature from the thermocouple
	float temperature = thermocouple.readCelsius();



	// Might help catch errors
	if (temperature == NAN) {
		Serial.println("Unable to read temperature :(");

		//Send shutoff signal to mosfet on bad temp reading
		analogWrite(MOSFET_PIN, 255.0);
	}

	
	currentTemperature = temperature;  // Update current temperature


	/*
	* ORIGNAL CODE
	*/

	//Perform PID control
	//float error = SETPOINT_TEMPERATURE - temperature;
	//integralTerm += error; // Accumulate error for integral term
	//float derivativeTerm = error - previousError; // Calculate derivative term
	//previousError = error; // Update previous error


	//float output = (KP * error) + (KI * integralTerm) + (KD * derivativeTerm); // Calculate PID output


	/*
	* STOLEN CODE - Incorporates time into the derivative equation (rate of change)
	*/

	PID_Error = SETPOINT_TEMPERATURE - temperature;
	
	//Calculate the P value
	PID_P = KP * PID_Error;

	//Calculate the I value in a range on +-3 //? Not sure why this is necessary but im assuming it is. 
	if (-3 < PID_Error < 3) 
	{
		PID_I = PID_I + (KI * PID_Error);
	}

	//Derivative equation using real time.
	timePrev = Time;                            // the previous time is stored before the actual time read
	Time = millis();                            // actual time read
	elapsedTime = (Time - timePrev) / 1000;

	//Now we can calculate the D calue
	PID_D = KD * ((PID_Error - previousError) / elapsedTime);
	//Final total PID value is the sum of P + I + D
	PID_Value = PID_P + PID_I + PID_D;

	
	float PMW_Out = constrain(PID_Value, 0.0, 255.0); //clip output PMW signal

	analogWrite(MOSFET_PIN, PMW_Out); //write PMW to mosfet 
	previousError = PID_Error;
	Serial.print("PID output");
	Serial.println(PMW_Out);


	// Print temperature and PID components to serial monitor
	Serial.print("Temperature: ");
	Serial.print(temperature);
	Serial.println(" °C");
	Serial.print("PID Output: ");
	Serial.println(PMW_Out);
	delay(1000); // Delay for readability
}
