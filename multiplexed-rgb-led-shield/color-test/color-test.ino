/*
  This exists to test the color handling of all LED bulbs at once.
*/

const uint8_t COLOR_MATRIX[3][4] = {
  { 2, 5, 8, 11 },
  { 3, 6, 9, 12 },
  { 4, 7, 10, 13}
};

const size_t COLOR_MOD = 3;
size_t nextCol = 0;

void setup() {
  for (uint8_t i = 2; i < 14; i++) {
    pinMode(i, OUTPUT);
    digitalWrite(i, 0);
  }
 
  for (uint8_t i = 14; i < 20; i++) {
    pinMode(i, OUTPUT);
    digitalWrite(i, 1);
  }
}

void loop() {
  const size_t col = nextCol;
  nextCol = (nextCol + 1) % COLOR_MOD;

  const uint8_t * entry = COLOR_MATRIX[col];

  // Enable all the requisite columns
  for (size_t i = 0; i < 4; ++i) {
    const uint8_t x = *(entry + i);
    digitalWrite(x, 1);
  }

  // Enable the rows for those colors
  for (uint8_t i = 14; i < 20; ++i) {
    digitalWrite(i, 0);
  }

  // This is a test application, so I didn't bother with timers.
  delay(500);

  // Disable the rows for those colors
  for (uint8_t i = 14; i < 20; ++i) {
    digitalWrite(i, 1);
  }

  // Disable all the requisite columns
  for (size_t i = 0; i < 4; ++i) {
    const uint8_t x = *(entry + i);
    digitalWrite(x, 0);
  }
}

// (g|mac)vim settings to play nice with the auto-format of the Aruino IDE //
// vim: set tabstop=2 shiftwidth=2 softtabstop=2 expandtab : //
