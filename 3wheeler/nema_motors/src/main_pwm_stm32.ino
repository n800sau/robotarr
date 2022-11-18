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

HardwareTimer *t_begin(PinName pin, uint32_t perc)
{
  TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(pin, PinMap_TIM);
  HardwareTimer *HT = new HardwareTimer((TIM_TypeDef *)pinmap_peripheral(pin, PinMap_TIM));
  uint32_t channel = STM_PIN_CHANNEL(pinmap_function(pin, PinMap_TIM));
  HT->setMode(channel, TIMER_OUTPUT_COMPARE_PWM1, pin);
  HT->setCaptureCompare(channel, perc, MICROSEC_COMPARE_FORMAT);
  HT->pause();
  return HT;
}

void setup()
{

	const PinName p1 = PB_1;
	const PinName p2 = PB_8;

	Serial.begin(115200);
	Serial.println("Start");

	Serial.print("timer ");
	Serial.print(get_timer_index((TIM_TypeDef *)pinmap_peripheral(p1, PinMap_TIM)));
	Serial.print(", channel ");
	Serial.print(STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(p1), PinMap_PWM)));
	Serial.print(", timer ");
	Serial.print(get_timer_index((TIM_TypeDef *)pinmap_peripheral(p2, PinMap_TIM)));
	Serial.print(", channel ");
	Serial.print(STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(p2), PinMap_PWM)));
	Serial.println();

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

	set_dir(true, true);

	t1 = t_begin(stepper1_step_pin, perc);
	t2 = t_begin(stepper2_step_pin, perc);
}

void set_dir(bool fwd1, bool fwd2)
{
	digitalWrite(stepper1_dir_pin, fwd1 ? HIGH : LOW);
	digitalWrite(stepper2_dir_pin, fwd2 ? LOW : HIGH);
}

void change_freq(uint32_t from, uint32_t to)
{
	if(from != to)
	{
		uint32_t step = 100;
		uint32_t sig = from > to ? -1 : 1;
		Serial.print("From ");
		Serial.print(from);
		Serial.print(" To ");
		Serial.println(to);
		for(uint32_t freq=from; freq*sig<=to*sig; freq+=step*sig) {
			Serial.print("Freq ");
			Serial.println(freq);
			t1->setOverflow(freq, HERTZ_FORMAT);
			t2->setOverflow(freq, HERTZ_FORMAT);
			t1->resume();
			t2->resume();
			delay(10);
		}
	}
}

void loop()
{
	change_freq(freq_min, freq_max);
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
}
