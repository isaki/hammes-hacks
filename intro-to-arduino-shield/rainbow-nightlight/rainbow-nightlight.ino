/*
  Intro to Arduino Shield: Rainbow Nightlight
  Author: isaki@github
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
  CLASSES/TYPES
*/

/**
   This is the type used for pin addressing.
   @see arduino.h for pin args to write and read functions
*/
#if defined (__AVR_ATmega328P__)
typedef uint8_t pin_t;
#else
#error "Integral type for pin_t has not been configured for this processor"
#endif

/**
  bool timefunc(const unsigned long& now, const unsigned long& last);
*/
typedef bool(*timefunc)(const unsigned long&, const unsigned long&);

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
    PrimaryLEDManager(const pin_t led) : LEDManager(), m_led(led) {}

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
    const pin_t m_led;
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
    CompoundLEDManager(const pin_t ledA, const pin_t ledB) : LEDManager(), m_a(ledA), m_b(ledB) {}

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

      // This will either be writeB if brightness was even, or writeB + 1 if
      // brightness was odd.
      const int writeA = brightness - writeB;

      analogWrite(m_a, writeA);
      analogWrite(m_b, writeB);
    }

  private:

    // No object copies
    CompoundLEDManager() = delete;
    CompoundLEDManager(const CompoundLEDManager&) = delete;
    CompoundLEDManager& operator=(const CompoundLEDManager&) = delete;

    // Internal object state
    const pin_t m_a;
    const pin_t m_b;
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
    WhiteLEDManager(const pin_t red, const pin_t green, const pin_t blue) : LEDManager(), m_r(red), m_g(green), m_b(blue) {}

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
      const int writeR = brightness / DIVISOR;
      const int rem = brightness - (DIVISOR * writeR);

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

    // Internal object state
    const pin_t m_r;
    const pin_t m_g;
    const pin_t m_b;
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
    RainbowLEDManager(const pin_t red, const pin_t green, const pin_t blue) : LEDManager(),
      m_r(red),
      m_g(green),
      m_b(blue),
      m_state(0),
      m_mod(0.0),
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
            break;
        };
      }

      m_changed = true;
    }

  protected:

    /**
       {@inheritDoc}
    */
    void process(const int brightness) override {
      const float inc = m_mod * static_cast<float>(brightness);
      const int writeI = static_cast<int>(inc);
      const int writeD = brightness - writeI;

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
    const pin_t m_r;
    const pin_t m_g;
    const pin_t m_b;

    unsigned char m_state;

    float m_mod;

    bool m_changed;

    // We want our pointers to be mutable, but point to constant memory, hence
    // the 'mutable' keyword.
    mutable const pin_t* m_dptr;
    mutable const pin_t* m_iptr;
    mutable const pin_t* m_zptr;
};

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
        const bool cache = m_callback[i](now, m_last[i]);
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
    unsigned long* m_last;
    timefunc* m_callback;
    size_t m_length;
};

//---------- END CLASSES/TYPES ----------//

/*
   DIGITAL SETTINGS
*/

// LED Pins
static const pin_t LED_BLUE = 3;
static const pin_t LED_GREEN = 5;
static const pin_t LED_RED = 6;

// Other Pins
static const pin_t GND = 2;
static const pin_t BUTTON = 4;

//---------- END DIGITAL SETTINGS ----------//

/*
   ANALOG SETTINGS
*/

// Voltage and our LDR are wired to analog pins
static const pin_t LDR_SENSOR = A0;
static const pin_t VCC = A1;

//---------- END ANALOG SETTINGS ----------//

/*
   APPLICATION CONSTANTS
*/

// LDR Constraints (NOTE: Orignal values from shield designer were 50 and 500)

// LDR_READ_MIN: The higher the value, the sooner we hit max brightness as the
// ambient light decreases.
static const int LDR_READ_MIN = 50;

// LDR_READ_MAX: The lower the value, the quicker we hit zero brightness as the
// ambient light increases.
static const int LDR_READ_MAX = 305;

// Light Pin Constraints: Digital pins have a square wave voltage controlled by
// an 8-bit value.
// NOTE: Arduino ints are int16_t, as such we don't need unsigned.
static const int VAL_LOW = 0;
static const int VAL_HIGH = 255;

// Button Constants; millis are tracking in uint32_t (unsigned long for Arduino Uno)
static const unsigned long DEBOUNCE_DELAY = 5;

// Rainbow frequency
static const unsigned long SHIFT_DELAY = 250;

// Derived Constants

// This constant is used to eliminate use of map which costs too many CPU cycles
// when we know its constaints are constant
static constexpr float LDR_MAP_FACTOR = static_cast<float>(VAL_LOW - VAL_HIGH) / static_cast<float>(LDR_READ_MAX - LDR_READ_MIN);

//---------- END APPLICATION CONSTANTS ----------//

/*
   APPLICATION STATE
*/

// digitalRead returns an int
int prevButton;
int currButton;

// LED state tracking
unsigned char nextState = 0;

// Object State
Timer timer;
LEDManager* lman = nullptr;

//---------- END APPLICATION STATE ----------//

/*
   ARDUINO METHODS
*/

void setup() {

  // Shield PWM Setup

#if defined (__AVR_ATmega328P__)
  // (WARN: This section is specifically for the ATmega328P on the Aruino UNO)

  // NOTE: Pins 5&6 (and millis()) run off Timer0, which is FastPWM with a
  // CLK/64 prescaler. In contrast, Pins 3&11 run off Timer2 which uses phase
  // correct modulation; this results in half the frequency seen from Timer0.
  // We need to ensure our prescaled square wave frequency matches to avoid LED
  // light jitter.

  // NOTE: We do NOT touch Timer0 as this messes up timer calculations. Thus,
  // manipulating the registers for Timer2 is our only safe option.

  // From the ATmega328P Data sheet:

  // COM2A1 = 1 with COM2A0 = 0 when in FastPWM:
  //   Clear OC2B on Compare Match, set OC2B at BOTTOM, (non-inverting mode).
  // WGM21 | WGM20 results in a WGM2:3, which is FastPWM with 0xff max
  // CS22: clkT2S/64 (From prescaler); this gives us the same frequency as
  // Timer0 (used by pins 5, 6, and millis).
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS22);
#endif

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

  // Alternate mode notifications
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

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

  // Note that on the Uno, digital pins 2 and 3 support interrupts. Our button
  // is on pin 4. As such, we can only use a timer based approach that polls
  // the pin state on every loop.
  timer.registerCallback(debounce_callback);

  // This periodically invokes shift on the current LEDManager
  timer.registerCallback(shift_callback);
}

void loop() {
  // Take care of any timed tasks
  timer.execute();

  const int brightness = determine_led_brightness();
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
bool debounce_callback(const unsigned long& now, const unsigned long& last) {
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
bool shift_callback(const unsigned long& now, const unsigned long& last) {
  const bool ret = (now - last) > SHIFT_DELAY;
  if (ret) {
    lman->shift();
  }

  return ret;
}

/*
   Controls state settings.

   0: RAINBOW
   1: OFF
   2: RED
   3: ORANGE
   4: GREEN
   5: CYAN
   6: BLUE
   7: MAGENTA
   8: WHITE
*/
void init_next_state() {
  const unsigned char x = nextState++;

  if (lman != nullptr) {
    delete lman;
  }

  switch (x) {
    case 0:
      lman = new RainbowLEDManager(LED_RED, LED_GREEN, LED_BLUE);
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
    default:
      lman = new WhiteLEDManager(LED_RED, LED_GREEN, LED_BLUE);
      nextState = 0;
      break;
  };
}

/**
   Convenience method to pull and translate LDR data into something that can be
   applied to LED brightness. Basically, this inverts the LDR value and massages
   it into an analogWrite friendly value.

   @return The value representing the total brightness our LED should be using.
*/
static int determine_led_brightness() {
  int ret = analogRead(LDR_SENSOR);
  // Due to our constant definitions, this is the equivalent of:
  // map(ret, LDR_READ_MIN, LDR_READ_MAX, VAL_HIGH, VAL_LOW)
  // Note that is faster than map as we eliminate the division on every read.
  // It also uses less code space on our device.
  ret = static_cast<int>(static_cast<float>(ret - LDR_READ_MIN) * LDR_MAP_FACTOR) + VAL_HIGH;
  return constrain(ret, VAL_LOW, VAL_HIGH);
}

//---------- END APPLICATION METHODS ----------//

// (g|mac)vim settings to play nice with the auto-format of the Aruino IDE //
// vim: set tabstop=2 shiftwidth=2 softtabstop=2 expandtab : //
