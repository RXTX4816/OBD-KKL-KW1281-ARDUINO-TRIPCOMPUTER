// Five direction joystick
const int buttonPin_RST = 13; // reset
const int buttonPin_SET = 2;  // set
const int buttonPin_MID = 3;  // middle
const int buttonPin_RHT = 4;  // right
const int buttonPin_LFT = 5;  // left
const int buttonPin_DWN = 6;  // down
const int buttonPin_UP = 7;   // up
void init_joystick() {
    // Setup pins
    //pinMode(pin_tx, OUTPUT); // TX
    //digitalWrite(pin_tx, HIGH);
    pinMode(buttonPin_RST, INPUT_PULLUP); // Joystick
    pinMode(buttonPin_SET, INPUT_PULLUP);
    pinMode(buttonPin_MID, INPUT_PULLUP);
    pinMode(buttonPin_RHT, INPUT_PULLUP);
    pinMode(buttonPin_LFT, INPUT_PULLUP);
    pinMode(buttonPin_DWN, INPUT_PULLUP);
    pinMode(buttonPin_UP, INPUT_PULLUP);
}

// Button state functions
bool reset_click()
{
    return digitalRead(buttonPin_RST) == LOW;
}
bool set_click()
{
    return digitalRead(buttonPin_SET) == LOW;
}
bool up_click()
{
    return digitalRead(buttonPin_UP) == LOW;
}
bool down_click()
{
    return digitalRead(buttonPin_DWN) == LOW;
}
bool left_click()
{
    return digitalRead(buttonPin_LFT) == LOW;
}
bool right_click()
{
    return digitalRead(buttonPin_RHT) == LOW;
}
bool mid_click()
{
    return digitalRead(buttonPin_MID) == LOW;
}