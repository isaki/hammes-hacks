/*
  Intro to Arduino Shield: Rainbow Nightlight
  Author: isaki-x@github
  License: Apache 2.0

  Acknowledgments:

  - Emily Hammes -
  This code was inspired by the sample code provided by Emily Hammes and is
  designed for the hardware you build as part of her "Introduction to Arduino
  Shield" class. Please check out hammeshacks.com and check out the fun
  projects she has available!

  Description:

  This code is a fun toy that utilizes the hardware designed in the
  aforementioned class (see Acknowledgments).
*/

/*
   STRUCTS/CLASSES/ENUMS
*/

/**
   This is the base class for LED output management.
*/
class LEDManager {

  public:

    /**
       This is a base class so its destructor should be virtual; especially
       since we delete child classes via pointers of this class.
    */
    virtual ~LEDManager() {}

    /**
       Optionally, a child class can implement this if they want to process
       color shifts based on the configured shift interval.
    */
    virtual void shift() {}

    /**
       This method sets the brightness for the managed LED.

       @param brightness
    */
    void set(const int brightness) {
      if (processRequired(brightness)) {
        process(brightness);
        m_brightness = brightness;
      }
    }

  protected:

    /**
       Base class constructor.
    */
    LEDManager() {}

    /**
       This method must be implemented by child classes to handle the
       brightness requested. Note that the value passed here will be the same as
       the value passed to processRequired(const int) if that method returns
       true.

       @param brightness
    */
    virtual void process(const int) = 0;

    /**
       This determines of the process method will be invoked; by default this
       checks the last set brightness level and returns true if a new value has
       been requested. This can be overridden as required.

       @param brightness
       @return
    */
    virtual bool processRequired(const int brightness) {
      return brightness != m_brightness;
    }

  private:

    // Prevent copies of this object
    LEDManager(const LEDManager&) = delete;
    LEDManager& operator=(const LEDManager&) = delete;

    // Internal object state
    int m_brightness;
};

/**
   This class manages a single color.
*/
class PrimaryLEDManager : public LEDManager {

  public:

    /**
       Constructor for a single color manager.

       @param led
    */
    PrimaryLEDManager(const int led) : LEDManager(), m_led(led) {}

    /**
       Destructor.
    */
    ~PrimaryLEDManager() {
      analogWrite(m_led, 0);
    }

  protected:

    /**
       {@inheritDoc}
    */
    void process(const int brightness) override {
      analogWrite(m_led, brightness);
    }

  private:

    // No copies allowed
    PrimaryLEDManager() = delete;
    PrimaryLEDManager(const PrimaryLEDManager&) = delete;
    PrimaryLEDManager& operator=(const PrimaryLEDManager&) = delete;

    // Internal object state
    const int m_led;
};

/**
  Activates two LED colors at half power each.
*/
class CompoundLEDManager : public LEDManager {

  public:

    /**
       Constructor for compound light activation.

       @param ledA
       @param ledB
    */
    CompoundLEDManager(const int ledA, const int ledB) : LEDManager(), m_a(ledA), m_b(ledB) {}

    /**
       Destructor.
    */
    ~CompoundLEDManager() {
      analogWrite(m_a, 0);
      analogWrite(m_b, 0);
    }

  protected:

    /**
       {@inheritDoc}
    */
    void process(const int brightness) override {
      // Binary is awesome, if we shift everything by one, it is the same as
      // multiplying by 0.5 without having to do any casts or use float point
      // calculations. It is also WAY faster than dividing by 2 (unless the
      // compiler optimizes the division by turning it into this code) as
      // division is actually really slow.
      const int writeB = brightness >> 1;

      // Since we are dealing with a divisor of 2, it is faster to do a quick
      // bit mask than to calculate a modulus.
      // NOTE: We are adding one as the static cast is lossy, if brightness is
      // an odd number, our total LED output would be one less than brightness.
      const int writeA = ((brightness & 1) == 0) ? writeB : writeB + 1;
      analogWrite(m_a, writeA);
      analogWrite(m_b, writeB);
    }

  private:
    // No object copies
    CompoundLEDManager() = delete;
    CompoundLEDManager(const CompoundLEDManager&) = delete;
    CompoundLEDManager& operator=(const CompoundLEDManager&) = delete;

    // Internal object state
    const int m_a;
    const int m_b;
};

/**
  Activates all LED colors at 1/3 power each.
*/
class WhiteLEDManager : public LEDManager {
  public:

    /**
       Constructor.

       @param red
       @param green
       @param blue
    */
    WhiteLEDManager(const int red, const int green, const int blue) : LEDManager(), m_r(red), m_g(green), m_b(blue) {}

    /**
       Destructor.
    */
    ~WhiteLEDManager() {
      analogWrite(m_r, 0);
      analogWrite(m_g, 0);
      analogWrite(m_b, 0);
    }

  protected:

    /**
       {@inheritDoc}
    */
    void process(const int brightness) override {
      const float val = static_cast<float>(brightness) * BRIGHTNESS_MOD;
      const int rem = brightness % DIVISOR;
      const int writeR = static_cast<int>(val);

      int writeG, writeB;
      switch (rem) {
        case 1:
          writeG = writeR + 1;
          writeB = writeR;
          break;
        case 2:
          writeG = writeR + 1;
          writeB = writeG;
          break;
        default:
          writeG = writeR;
          writeB = writeR;
          break;
      }

      analogWrite(m_r, writeR);
      analogWrite(m_g, writeG);
      analogWrite(m_b, writeB);
    }

  private:

    // No object copies
    WhiteLEDManager() = delete;
    WhiteLEDManager(const WhiteLEDManager&) = delete;
    WhiteLEDManager& operator=(const WhiteLEDManager&) = delete;

    // Class constants
    static const int DIVISOR = 3;
    static constexpr float BRIGHTNESS_MOD = 1.0 / static_cast<float>(DIVISOR);

    // Internal object state
    const int m_r;
    const int m_g;
    const int m_b;
};

/**
  A no-op LED manager.
*/
class NoneLEDManager : public LEDManager {

  public:

    /**
       Constructor.
    */
    NoneLEDManager() : LEDManager() {}

    /**
       Destructor.
    */
    ~NoneLEDManager() {}

  protected:

    /**
       {@inheritDoc}
    */
    void process(const int) override {}

  private:

    // No object copies
    NoneLEDManager(const NoneLEDManager&) = delete;
    NoneLEDManager& operator=(const NoneLEDManager&) = delete;
};

/**
   An advanced LED manager for beautiful color transitions.
*/
class RainbowLEDManager : public LEDManager {

  public:

    /**
       Constructor.

       @param red
       @param green
       @param blue
    */
    RainbowLEDManager(const int red, const int green, const int blue) : LEDManager(),
      m_r(red),
      m_g(green),
      m_b(blue),
      m_mod(0.0),
      m_state(0),
      m_changed(true) {

      // Set initial pointers
      m_dptr = &m_r;
      m_iptr = &m_g;
      m_zptr = &m_b;
    }

    /**
       Destructor.
    */
    ~RainbowLEDManager() {
      m_dptr = nullptr;
      m_iptr = nullptr;
      m_zptr = nullptr;

      analogWrite(m_r, 0);
      analogWrite(m_g, 0);
      analogWrite(m_b, 0);
    }

    /**
       {@inheritDoc}
    */
    void shift() override {
      m_mod += SHIFT_STEP;

      if (m_mod > 1.0) {
        m_mod = 0.0;

        switch (m_state) {
          case 0:
            m_dptr = &m_g;
            m_iptr = &m_b;
            m_zptr = &m_r;
            m_state = 1;
            break;
          case 1:
            m_dptr = &m_b;
            m_iptr = &m_r;
            m_zptr = &m_g;
            m_state = 2;
            break;
          default:
            m_dptr = &m_r;
            m_iptr = &m_g;
            m_zptr = &m_b;
            m_state = 0;
        };
      }

      m_changed = true;
    }

  protected:

    /**
       {@inheritDoc}
    */
    void process(const int brightness) override {
      const float bf = static_cast<float>(brightness);

      // m_mod is incrementing, thus our incrementing value should be applied to
      // our brightness. By that logic, the decrement is the 'opposite' of our
      // increment, or for our purposes: 1.0 - m_mod. Thus, our wdf value should
      // be (1.0 - m_mod) * bf. If we use the distributive property, we end up
      // with bf - bf * m_mod. Since wif is equal to bf * m_mod, we can simply
      // subtract wif from bf to get the wdf value (which makes sense, so the
      // combined value of wif + wdf should be the total requested brightness
      // value of bf).
      const float wif = m_mod * bf;
      const float wdf = bf - wif;

      int writeD = static_cast<int>(wdf);
      int writeI = static_cast<int>(wif);

      // float calcs can be lossy
      const int diff = brightness - writeD - writeI;
      if (diff > 0) {
        const int mod = diff >> 1;
        writeD += ((diff & 1) == 0) ? mod : mod + 1;
        writeI += mod;
      }

      analogWrite(*m_dptr, writeD);
      analogWrite(*m_iptr, writeI);
      analogWrite(*m_zptr, 0);

      m_changed = false;
    }

    /**
       {@inheritDoc}

       This version will return true if shift has been invoked in addition to
       respecting brightness changes.
    */
    bool processRequired(const int brightness) override {
      return m_changed || LEDManager::processRequired(brightness);
    }

  private:

    // No copies.
    RainbowLEDManager() = delete;
    RainbowLEDManager(const WhiteLEDManager&) = delete;
    RainbowLEDManager& operator=(const WhiteLEDManager&) = delete;

    // Constant: SHIFT_STEP = 1 / 256 (as there are 256 numbers from 0 - 255)
    static constexpr float SHIFT_STEP = 0.00390625;

    // Internal object state
    const int m_r;
    const int m_g;
    const int m_b;

    float m_mod;

    char m_state;

    bool m_changed;

    // We want our pointers to be mutable, but point to constant memory, hence
    // the 'mutable' keyword.
    mutable const int* m_dptr;
    mutable const int* m_iptr;
    mutable const int* m_zptr;
};

/**
  bool timefunc(unsigned long now, const unsigned long last);
*/
typedef bool(*timefunc)(unsigned long, unsigned long);

/**
  Timer utility for managing asynchronous processing without the use of threads.
*/
class Timer {

  public:

    /**
       Constructor.
    */
    Timer() : m_length(0) {
      m_callback = nullptr;
      m_last = nullptr;
    }

    /**
       Destructor.
    */
    ~Timer() {
      if (m_length != 0) {
        delete[] m_last;
        delete[] m_callback;
        m_length = 0;
      }
    }

    /**
       Registers a callback to be executed on every loop invocation.
    */
    void registerCallback(const timefunc callback) {
      const size_t old = m_length++;

      unsigned long* tmpLast = new unsigned long[m_length];
      timefunc* tmpCallback = new timefunc[m_length];

      if (old != 0) {
        memcpy(tmpLast, m_last, sizeof(unsigned long) * old);
        memcpy(tmpCallback, m_callback, sizeof(timefunc) * old);
        delete[] m_last;
        delete[] m_callback;
      }

      m_last = tmpLast;
      m_callback = tmpCallback;

      m_last[old] = 0;
      m_callback[old] = callback;
    }

    /**
       This should be called by the loop method for Adruino.
    */
    void execute() {
      const unsigned long now = millis();
      for (size_t i = 0; i < m_length; ++i) {
        bool cache = m_callback[i](now, m_last[i]);
        if (cache) {
          m_last[i] = now;
        }
      }
    }

  private:
    // No copies!
    Timer(const Timer&) = delete;
    Timer& operator=(const Timer&) = delete;

    // Internal object state
    unsigned long * m_last;
    timefunc * m_callback;
    size_t m_length;
};

//---------- END CLASSES/STRUCTS ----------//

/*
   DIGITAL SETTINGS
*/

// LED Pins
static const int LED_BLUE = 3;
static const int LED_GREEN = 5;
static const int LED_RED = 6;

// Other Pins
static const int GND = 2;
static const int BUTTON = 4;

// UNO Built-Ins we are co-opting
static const int LED_UNO_L = 13;

//---------- END DIGITAL SETTINGS ----------//

/*
   ANALOG SETTINGS
*/

// Voltage and our LDR are wired to analog pins
static const int LDR_SENSOR = A0;
static const int VCC = A1;

//---------- END ANALOG SETTINGS ----------//

/*
   APPLICATION CONSTANTS
*/

// LDR Constraints
// NOTE: I spoke to Emily, these values were based on some testing she did for
// herself measuring resistance at different light/distances.
static const int LDR_READ_MIN = 50;
static const int LDR_READ_MAX = 500;

// Light Pin Constraints: Digital pins have a square wave voltage controlled by
// an 8-bit value.
// NOTE: Arduino ints are int16_t, as such we don't need unsigned.
static const int VAL_LOW = 0;
static const int VAL_HIGH = 255;

// Button Constants; millis are tracking in uint32_t (unsigned long for Arduino Uno)
static const unsigned long DEBOUNCE_DELAY = 5;

// Rainbow frequency
static const unsigned long SHIFT_DELAY = 250;

//---------- END APPLICATION CONSTANTS ----------//

/*
   APPLICATION STATE
*/

int nextState = 8;
int prevButton;
int currButton;

// Helper objects
Timer timer;
LEDManager * lman = nullptr;

//---------- END APPLICATION STATE ----------//

/*
   ARDUINO METHODS
*/

void setup() {

  // SHIELD

  //Set our 3 LEDs as OUTPUT
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);

  // Button is input, with a pullup resistor
  pinMode(BUTTON, INPUT_PULLUP);

  // Set the LDR as INPUT
  pinMode(LDR_SENSOR, INPUT);

  // Set our "ground" pin
  pinMode(GND, OUTPUT);
  digitalWrite(GND, LOW);

  // Set vcc to OUTPUT
  pinMode(VCC, OUTPUT);
  digitalWrite(VCC, HIGH);

  // Debugging and error helper
  pinMode(LED_UNO_L, OUTPUT);
  digitalWrite(LED_UNO_L, LOW);

  // Base state is no voltage
  analogWrite(LED_RED, 0);
  analogWrite(LED_GREEN, 0);
  analogWrite(LED_BLUE, 0);

  // Load button state
  currButton = digitalRead(BUTTON);
  prevButton = currButton;

  // Force a state change
  init_next_state();

  // Register timer callbacks
  timer.registerCallback(debounce_callback);
  timer.registerCallback(shift_callback);
}

void loop() {
  // Take care of any timed tasks
  timer.execute();

  const int brightness = determine_led_brightness(LDR_SENSOR);
  lman->set(brightness);
}

//---------- END ARDUINO METHODS ----------//

/*
   APPLICATION METHODS
*/

/**
   Function designed to work with Timer based on:
   https://www.arduino.cc/en/Tutorial/Debounce

   @param now
   @param last
   @return
*/
bool debounce_callback(const unsigned long now, const unsigned long last) {
  const int reading = digitalRead(BUTTON);
  const bool ret = prevButton != reading;
  if (ret) {
    prevButton = reading;
  } else if ((now - last) > DEBOUNCE_DELAY && reading != currButton) {
    currButton = reading;

    // Engage on the instant the button is pressed.
    if (currButton == LOW) {
      init_next_state();
    }
  }

  return ret;
}

/**
   Function designed to shift colors for the LED managers that support it.

   @param now
   @param last
   @return
*/
bool shift_callback(const unsigned long now, const unsigned long last) {
  const bool ret = (now - last) > SHIFT_DELAY;
  if (ret) {
    lman->shift();
  }

  return ret;
}

/*
   Controls state settings.

   0: WHITE
   1: OFF
   2: RED
   3: ORANGE
   4: GREEN
   5: CYAN
   6: BLUE
   7: MAGENTA
   8: RAINBOW
*/
void init_next_state() {
  const int x = nextState++;

  LEDManager* tmpptr = lman;
  switch (x) {
    case 0:
      lman = new WhiteLEDManager(LED_RED, LED_GREEN, LED_BLUE);
      break;
    case 1:
      lman = new NoneLEDManager();
      break;
    case 2:
      lman = new PrimaryLEDManager(LED_RED);
      break;
    case 3:
      lman = new CompoundLEDManager(LED_RED, LED_GREEN);
      break;
    case 4:
      lman = new PrimaryLEDManager(LED_GREEN);
      break;
    case 5:
      lman = new CompoundLEDManager(LED_GREEN, LED_BLUE);
      break;
    case 6:
      lman = new PrimaryLEDManager(LED_BLUE);
      break;
    case 7:
      lman = new CompoundLEDManager(LED_BLUE, LED_RED);
      break;
    case 8:
      lman = new RainbowLEDManager(LED_RED, LED_GREEN, LED_BLUE);
      nextState = 0;
      break;
    default:
      lman = new NoneLEDManager();
      error();
      nextState = 0;
      break;
  };

  // Need to cleanup any old heap allocations
  if (tmpptr != nullptr) {
    delete tmpptr;
  }
}

/**
   Convenience method to pull and translate LDR data into something that can be
   applied to LED brightness. Basically, this inverts the LDR value and massages
   it into an analogWrite friendly value.

   @return The value representing the total brightness our LED should be using.
*/
static int determine_led_brightness(const int sensor) {
  int ret = analogRead(sensor);
  ret = map(ret, LDR_READ_MIN, LDR_READ_MAX, VAL_HIGH, VAL_LOW);
  return constrain(ret, VAL_LOW, VAL_HIGH);
}

/**
   Simple method for alerting you to problems.
*/
inline static void error() {
  digitalWrite(LED_UNO_L, HIGH);
}

//---------- END APPLICATION METHODS ----------//

// (g|mac)vim settings to play nice with the auto-format of the Aruino IDE //
// vim: set tabstop=2 shiftwidth=2 softtabstop=2 expandtab : //
