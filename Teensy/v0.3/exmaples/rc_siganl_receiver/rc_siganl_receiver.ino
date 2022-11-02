#define num_of_channel 6

bool is_connected = false;
bool serialDebug = true;

const unsigned short connnection_timeout = 940; // in us

char ch_pinNum[num_of_channel] = {39, 40, 41, 42, 43, 44};
unsigned short ch_high_time[num_of_channel] = {0, 0, 0, 0, 0, 0};
unsigned short ch_max[num_of_channel] = {1750, 1750, 2000, 1750, 2000, 2000};
unsigned short ch_min[num_of_channel] = {1250, 1250, 1000, 1250, 1000, 1000};
int ch_value[num_of_channel] = {0, 0, 0, 0, 0, 0};


void setup() {
  Serial.begin(9600);
  for (int ch = 0; ch < num_of_channel; ch++) {
    pinMode(ch_pinNum[ch], INPUT);
  }
}

void loop() {
  readPwm();
  print_debug();
}

bool readPwm() {
  for (int ch = 0; ch < 4; ch++) {
    ch_high_time[ch] = pulseIn(ch_pinNum[ch], HIGH, 14000); // read pwm pulse(high signal)

    if (ch_high_time[ch] < connnection_timeout) {
      is_connected = false;
      return false; // check connection of RC transmitter
    }
    ch_value[ch] = (ch_high_time[ch] - ch_min[ch]) * 100 / (ch_max[ch] - ch_min[ch]); //scale pwm sinal (0~100)

    if (ch_value[ch] < 0) ch_value[ch] = 0; // ignore negative values

  }
  return true;
}

void print_debug() {
   is_connected = readPwm();

  if (serialDebug == true) {
    Serial.print("CH1: ");
    Serial.print(ch_value[0]);
    Serial.print("  CH2: ");
    Serial.print(ch_value[1]);
    Serial.print("  CH3: ");
    Serial.print(ch_value[2]);
    Serial.print("  CH4: ");
    Serial.print(ch_value[3]);
    Serial.print("  CH5: ");
    Serial.print(ch_value[4]);
    Serial.print("  CH6: ");
    Serial.print(ch_value[5]);
    Serial.println((is_connected == true) ? "  CONNECTED" : "  CONNECTION LOST");
  }
}

int8_t getValue(const uint8_t &ch) {
  return ch_value[ch];
}
