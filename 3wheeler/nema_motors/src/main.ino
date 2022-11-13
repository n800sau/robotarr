#include <Arduino.h>
//#include "BasicStepperDriver.h"
#include "A4988.h"
#include "MultiDriver.h"
#include "SyncDriver.h"
#include <SimpleSerialProtocol.h>

// declare callbacks (this is boilerplate code but needed for proper compilation of the sketch)
void onError(uint8_t errorNum);
void onReceivedValues();

// inintialize hardware constants
const long BAUDRATE = 115200; // speed of serial connection
const long CHARACTER_TIMEOUT = 500; // wait max 500 ms between single chars to be received

// initialize command constants
const byte COMMAND_MOVE = 'm';
const byte COMMAND_STEPS = 's';

// Create instance. Pass Serial instance. Define command-id-range within Simple Serial Protocol is listening (here: a - z)
SimpleSerialProtocol ssp(Serial, BAUDRATE, CHARACTER_TIMEOUT, onError, 'a', 'z'); // ASCII: 'a' - 'z' (26 byes of RAM is reserved)


// If microstepping is set externally, make sure this matches the selected mode
// 1=full step, 2=half step etc.
#define MICROSTEPS 16

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 200
// Target RPM for motor
#define MOTOR_RPM 20

const int stepper1_enable_pin = PB7;
const int stepper1_dir_pin = PA12;
const int stepper1_step_pin = PA15;

const int stepper2_enable_pin = PB6;
const int stepper2_dir_pin = PB4;
const int stepper2_step_pin = PB3;


// 2-wire basic config, microstepping is hardwired on the driver
// Other drivers can be mixed and matched but must be configured individually
//BasicStepperDriver stepper1(MOTOR_STEPS, stepper1_dir_pin, stepper1_step_pin);
//BasicStepperDriver stepper2(MOTOR_STEPS, stepper2_dir_pin, stepper2_step_pin);

A4988 stepper1(MOTOR_STEPS, stepper1_dir_pin, stepper1_step_pin, stepper1_enable_pin);
A4988 stepper2(MOTOR_STEPS, stepper2_dir_pin, stepper2_step_pin, stepper2_enable_pin);

// Pick one of the two controllers below
// each motor moves independently, trajectory is a hockey stick
// MultiDriver controller(stepper1, stepper2);
// OR
// synchronized move, trajectory is a straight line
SyncDriver controller(stepper1, stepper2);

void setup() {
//	Serial.begin(115200);
//	Serial.println("Start");

	// prepare LED and set it off
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW);
	// init ssp. ssp is calling 'Serial.begin(9600)' behind the scenes
	ssp.init();
	// if message command with 'r' is received, the given callback will be called
	ssp.registerCommand(COMMAND_MOVE, onMoveCmd);
	ssp.registerCommand(COMMAND_STEPS, onStepsCmd);
	/*
	 * Set target motors RPM.
	 */
	pinMode(stepper1_enable_pin, OUTPUT);
	digitalWrite(stepper1_enable_pin, LOW);

	pinMode(stepper2_enable_pin, OUTPUT);
	digitalWrite(stepper2_enable_pin, LOW);

	stepper1.begin(MOTOR_RPM, MICROSTEPS);
	stepper1.setSpeedProfile(BasicStepperDriver::LINEAR_SPEED, 2000, 2000);
	stepper2.begin(MOTOR_RPM, MICROSTEPS);
	stepper2.setSpeedProfile(BasicStepperDriver::LINEAR_SPEED, 2000, 2000);
	// if using enable/disable on ENABLE pin (active LOW) instead of SLEEP uncomment next two lines
	// stepper1.setEnableActiveState(LOW);
	// stepper2.setEnableActiveState(LOW);
}

void loop()
{
	// motor control loop - send pulse and return how long to wait until next pulse
	unsigned wait_time_micros = controller.nextAction();

	// 0 wait time indicates the motor has stopped
	if (wait_time_micros <= 0) {
		controller.disable();	   // comment out to keep motor powered
	}

	// (optional) execute other code if we have enough time
	if (wait_time_micros > 100){
		// other code here
	}
	ssp.loop();

//	Serial.println("Step1");
//	controller.rotate(90*5, 60*15);
//	delay(1000);
//	Serial.println("Step2");
//	controller.rotate(-90*5, -30*15);
//	delay(1000);
//	Serial.println("Step3");
//	controller.rotate(0, -30*15);
//	delay(30000);
}

// callbacks implementation
void onMoveCmd()
{
	int s1val = ssp.readInt32();
	int s2val = ssp.readInt32();
	int pval = ssp.readInt32();
	ssp.readEot(); // read and expect the end-of-transmission byte. important, don't forget!

	//
	// Immediately send back all received and interpreted values
	//
	ssp.writeCommand(COMMAND_MOVE); // start command with command id

	ssp.writeInt32(s1val);
	ssp.writeInt32(s2val);
	ssp.writeInt32(pval);

	ssp.writeEot(); // end command with end-of-transmission byte. important, don't forget!

	controller.setRPM(pval);
	controller.startMove(s1val, s2val);

}

// callbacks implementation
void onStepsCmd()
{
	ssp.readEot();

	int s1val = controller.getMotor(0).getStepsRemaining();
	int s2val = controller.getMotor(1).getStepsRemaining();

	ssp.writeCommand(COMMAND_STEPS); // start command with command id

	ssp.writeInt32(s1val);
	ssp.writeInt32(s2val);

	ssp.writeEot(); // end command with end-of-transmission byte. important, don't forget!


}

void onError(uint8_t errorNum) {
  digitalWrite(LED_BUILTIN, HIGH);
}
