#include <SimpleSerialProtocol.h>

// inintialize hardware constants
const long BAUDRATE = 9600; // speed of serial connection
const long CHARACTER_TIMEOUT = 500; // wait max 500 ms between single chars to be received

// initialize command constants
const byte CMD_PING = 'p';
const byte CMD_MOVE = 'm';
const byte CMD_STEPS = 's';
const byte CMD_RESET = 'r';

// declare callbacks (this is boilerplate code but needed for proper compilation of the sketch)
void onError(uint8_t errorNum);

// Create instance. Pass Serial instance. Define command-id-range within Simple Serial Protocol is listening (here: a - z)
SimpleSerialProtocol ssp(Serial, BAUDRATE, CHARACTER_TIMEOUT, onError, 'a', 'z'); // ASCII: 'a' - 'z' (26 byes of RAM is reserved)

const int stepper1_enable_pin = PB7;
const int stepper1_dir_pin = PA12;
//const int stepper1_step_pin = PB1; //PA15;

PinName stepper1_step_pin = PB_1;

const int stepper2_enable_pin = PB6;
const int stepper2_dir_pin = PB3;
//const int stepper2_step_pin = PB8; //PB3;

PinName stepper2_step_pin = PB_8;

// 50 - too much
const uint32_t perc = 1;

// 20000 too fast
const uint32_t freq_max = 4000;
const uint32_t freq_min = 800;

HardwareTimer *t1;
HardwareTimer *t2;

uint32_t last_freq_t1 = 0;
uint32_t last_freq_t2 = 0;

volatile bool last_fwd1 = true;
volatile bool last_fwd2 = true;

volatile int64_t counter1 = 0;
volatile int64_t counter2 = 0;

void periodCallback1()
{
//	counter1 += is_fwd1() ? 1 : -1;
	counter1 += last_fwd1 ? 1 : -1;
}

void periodCallback2()
{
//	counter2 += is_fwd2() ? 1 : -1;
	counter2 += last_fwd2 ? 1 : -1;
}

HardwareTimer *t_begin(PinName pin, uint32_t perc, callback_function_t periodCallback)
{
  TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(pin, PinMap_TIM);
  HardwareTimer *HT = new HardwareTimer((TIM_TypeDef *)pinmap_peripheral(pin, PinMap_TIM));
  uint32_t channel = STM_PIN_CHANNEL(pinmap_function(pin, PinMap_TIM));
  HT->setMode(channel, TIMER_OUTPUT_COMPARE_PWM1, pin);
  HT->setCaptureCompare(channel, perc, MICROSEC_COMPARE_FORMAT);
  HT->pause();
//  auto period_callback = std::bind(&periodCallback, pcounter);
  HT->attachInterrupt(periodCallback);
  return HT;
}

void setup()
{

//	const PinName p1 = PB_1;
//	const PinName p2 = PB_8;

//	Serial.begin(115200);
//	Serial.println("Start");

//	Serial.print("timer ");
//	Serial.print(get_timer_index((TIM_TypeDef *)pinmap_peripheral(p1, PinMap_TIM)));
//	Serial.print(", channel ");
//	Serial.print(STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(p1), PinMap_PWM)));
//	Serial.print(", timer ");
//	Serial.print(get_timer_index((TIM_TypeDef *)pinmap_peripheral(p2, PinMap_TIM)));
//	Serial.print(", channel ");
///	Serial.print(STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(p2), PinMap_PWM)));
//	Serial.println();

	pinMode(stepper1_enable_pin, OUTPUT);
	digitalWrite(stepper1_enable_pin, LOW);
	pinMode(stepper1_dir_pin, OUTPUT);
//	pinMode(stepper1_step_pin, OUTPUT);
//	digitalWrite(stepper1_step_pin, LOW);

	pinMode(stepper2_enable_pin, OUTPUT);
	digitalWrite(stepper2_enable_pin, LOW);
	pinMode(stepper2_dir_pin, OUTPUT);
//	pinMode(stepper2_step_pin, OUTPUT);
//	digitalWrite(stepper2_step_pin, LOW);

	// init ssp. ssp is calling 'Serial.begin(9600)' behind the scenes
	ssp.init();
	// if message command with 'r' is received, the given callback will be called
	ssp.registerCommand(CMD_MOVE, onMoveCmd);
	ssp.registerCommand(CMD_STEPS, onStepsCmd);
	ssp.registerCommand(CMD_RESET, onResetCmd);
	ssp.registerCommand(CMD_PING, onPingCmd);

	set_dir(true, true);

	t1 = t_begin(stepper1_step_pin, perc, periodCallback1);
	t2 = t_begin(stepper2_step_pin, perc, periodCallback2);
}

void set_dir(bool fwd1, bool fwd2)
{
	last_fwd1 = fwd1;
	last_fwd2 = fwd2;
	digitalWrite(stepper1_dir_pin, fwd1 ? HIGH : LOW);
	digitalWrite(stepper2_dir_pin, fwd2 ? LOW : HIGH);
}

bool is_fwd1()
{
	return digitalRead(stepper1_dir_pin) == HIGH;
}

bool is_fwd2()
{
	return digitalRead(stepper2_dir_pin) == LOW;
}

void change_freq_to(uint32_t to)
{
	change_freq(last_freq_t1, to);
	change_freq(last_freq_t2, to);
}

void change_freq(uint32_t from, uint32_t to)
{
	if(from < to)
	{
		uint32_t step = 100;
		uint32_t sig = from > to ? -1 : 1;
//		Serial.print("From ");
//		Serial.print(from);
//		Serial.print(" To ");
//		Serial.println(to);
		for(uint32_t freq=from; freq*sig<=to*sig; freq+=step*sig) {
//			Serial.print("Freq ");
//			Serial.println(freq);
			t1->setOverflow(freq, HERTZ_FORMAT);
			t2->setOverflow(freq, HERTZ_FORMAT);
			t1->resume();
			t2->resume();
			delay(10);
		}
	} else {
		t1->setOverflow(to, HERTZ_FORMAT);
		t2->setOverflow(to, HERTZ_FORMAT);
		t1->resume();
		t2->resume();
	}
	last_freq_t1 = to;
	last_freq_t2 = to;
}

void loop()
{
/*	change_freq(freq_min, freq_max);
	delay(2000);
	t1->pause();
	t2->pause();
	delay(4000);
	set_dir(false, false);
	change_freq(freq_min, freq_max);
	delay(2000);
	t1->pause();
	t2->pause();
	delay(4000);
	set_dir(true, true);
*/	ssp.loop();
}

// callbacks implementation
void onMoveCmd()
{
	int s1val = ssp.readInt32();
	int s2val = ssp.readInt32();
	ssp.readEot(); // read and expect the end-of-transmission byte. important, don't forget!

	//
	// Immediately send back all received and interpreted values
	//
	ssp.writeCommand(CMD_MOVE); // start command with command id

	ssp.writeInt32(s1val);
	ssp.writeInt32(s2val);

	ssp.writeEot(); // end command with end-of-transmission byte. important, don't forget!

	t1->pause();
	t2->pause();
	set_dir(s1val >= 0, s2val >= 0);
	if(s1val != 0)
	{
		t1->setOverflow(abs(s1val), HERTZ_FORMAT);
		t1->resume();
	}
	if(s2val != 0)
	{
		t2->setOverflow(abs(s2val), HERTZ_FORMAT);
		t2->resume();
	}

}

// callbacks implementation
void onStepsCmd()
{
	ssp.readEot();

	ssp.writeCommand(CMD_STEPS);

	ssp.writeInt64(counter1);
	ssp.writeInt64(counter2);

	ssp.writeEot();


}

void onResetCmd()
{
	counter1 = 0;
	counter2 = 0;
	ssp.readEot();
}

void onPingCmd()
{
	char ping_char = ssp.readChar();
	ssp.readEot();
	if(ping_char == 'A') {
		ssp.writeCommand(CMD_PING);
		ssp.writeChar('B');
		ssp.writeEot();
	}
}

void onError(uint8_t errorNum) {
  digitalWrite(LED_BUILTIN, HIGH);
}
