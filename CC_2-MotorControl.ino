/*
 * Title: Control Clear Core Motors
 *
 * Objective:
 *   Single program to control both MedX motors in one program.  move routing 
 *   adopted from example programs
 *   
 * Description:
 *    Enable ClearPath motors and executes a repeating loop to check for input During operation, various move statuses are
 *    written to the USB  port.
 *    
 *    Command syntax 
 *      - <RPM:XXX> where XXX is speed in rpm
 *      - <UP+:XXX> canula up where XXX doesn't matter
 *      - <DN-:XXX> cannula down XXX doesn't matter.
 *      
 *      
 * Requirements:
 * 1. A ClearPath motor must be connected to Connector M-0.
 * 2. The connected ClearPath motor must be configured through the MSP software
 *    for Manual Velocity Control mode (In MSP select Mode>>Velocity>>Manual
 *    Velocity Control, then hit the OK button).
 * 3. In the MSP software:
 *    * Define a Max Clockwise and Counter-Clockwise (CW/CCW) Velocity (On the
 *      main MSP window fill in the textboxes labeled "Max CW Velocity (RPM)"
 *      and "Max CCW Velocity (RPM)"). Any velocity commanded outside of this
 *      range will be rejected.
 *    * Set the Velocity Resolution to 2 (On the main MSP window check the
 *      textbox labeled "Velocity Resolution (RPM per knob count)" 2 is
 *      default). This means the commanded velocity will always be a multiple
 *      of 2. For finer resolution, lower this value and change
 *      velocityResolution in the sketch below to match.
 *    * Set Knob Direction to As-Wired, and check the Has Detents box (On the
 *      main MSP window check the dropdown labeled "Knob Direction" and the
 *      checkbox directly below it labeled "Has Detents").
 *    * On the main MSP window set the dropdown labeled "On Enable..." to be
 *      "Zero Velocity".
 *    * Set the HLFB mode to "ASG-Velocity w/Measured Torque" with a PWM carrier
 *      frequency of 482 Hz through the MSP software (select Advanced>>High
 *      Level Feedback [Mode]... then choose "ASG-Velocity w/Measured Torque" 
 *      from the dropdown, make sure that 482 Hz is selected in the "PWM Carrier
 *      Frequency" dropdown, and hit the OK button).
 *
 * Links:
 * ** ClearCore Documentation: https://teknic-inc.github.io/ClearCore-library/
 * ** ClearCore Manual: https://www.teknic.com/files/downloads/clearcore_user_manual.pdf
 * ** ClearPath Manual (DC Power): https://www.teknic.com/files/downloads/clearpath_user_manual.pdf
 * ** ClearPath Manual (AC Power): https://www.teknic.com/files/downloads/ac_clearpath-mc-sd_manual.pdf
 *
 *
 * Move routing portions are Copyright (c) 2020 Teknic Inc. This work is free to use, copy and distribute under the terms of
 * the standard MIT permissive software license which can be found at https://opensource.org/licenses/MIT
 */

#include "ClearCore.h"

// The INPUT_A_B_FILTER must match the Input A, B filter setting in
// MSP (Advanced >> Input A, B Filtering...)
#define INPUT_A_B_FILTER 20


// Defines the Stir motor's connector as ConnectorM0
#define motorS ConnectorM0
// Defines Cannula motor's connector as ConnectorM1
#define motorC ConnectorM1

// Specifies the Cannula home sensor connector
#define HomingSensor DI6

// Select the baud rate to match the target device.
#define baudRate 115200   // Try this and see how we do, slow it down if we start dropping stuff
#define isTtlInputPort  false

// This is the variable used to keep track of the current commanded velocity for Stir Motor
double commandedVelocityStir = 0;

// A reference to the maximum clockwise and counter-clockwise velocities set in
// the MSP software. These must match the values in MSP
int maxVelocityCW = 1000;
int maxVelocityCCW = 1000;

// Each velocity commanded will be a multiple of this value, which must match
// the Velocity Resolution value in MSP. Use a lower value here (and in MSP) to
// command velocity with a finer resolution
double velocityResolution = 2.0;

// Declares our user-defined Stir helper function, which is used to send a velocity
// command. The definition/implementation of this function is at the bottom of
// the sketch.
bool MoveAtVelocity(double velocity);

// Declares our user-defined Cannula functions, which are used to pass the state of the
// home sensor to the motor, and to send move commands. The
// definitions/implementations of these functions are at the bottom of the sketch
void HomingSensorCallback();
bool MoveToPosition(int positionNum);

const byte numChars = 32;       // max lengh of all our strings
char inputString[numChars] = "";         // a String to hold incoming data
char tempChars[numChars];               //used to parse and process data
char newCommand[numChars]={0};                //Used to hold command to change activities
int newValue = 0;                   // for commands that require a value

bool newData = false;           // Keep track of when to process incoming command

void setup() {
    // Put your setup code here, it will run once:
 
    // Set Stir and Canula motor connectors to the correct mode for Manual Velocity
    // mode.
    MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL,
                          Connector::CPM_MODE_A_DIRECT_B_DIRECT);


    // Set Stir and Canula motor's HLFB mode to bipolar PWM
    motorS.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
    motorC.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);

    // Set the HFLB carrier frequency to 482 Hz for Stir and Canula
    motorS.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);
    motorC.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);

    // Enforces the state of the Stir motor's A and B inputs before enabling
    // the motor.
    motorS.MotorInAState(false);
    motorS.MotorInBState(false);

    // Enforces the state of the Cannula motor's Input A before enabling the motor
    motorC.MotorInAState(false);

    // This  attaches the interrupt callback to the Cannula homing sensor pin,
    // set to trigger on any change of sensor state
    pinMode(HomingSensor, INPUT);
    attachInterrupt(digitalPinToInterrupt(HomingSensor), HomingSensorCallback, CHANGE);
    // Set input B to match the initial state of the sensor
    motorC.MotorInBState(digitalRead(HomingSensor));


    // Sets up serial communication and waits up to 5 seconds for a port to open.
    // Serial communication is not required for this example to run.
    Serial0.begin(baudRate);
    Serial0.ttl(isTtlInputPort);
    uint32_t timeout = 5000;
    uint32_t startTime = millis();
    while (!Serial0 && millis() - startTime < timeout) {
        continue;
    }

    // Enables the  Stir motor
    motorS.EnableRequest(true);
    Serial0.println("Stir Motor Enabled");

    // Enables the Cannula motor; homing will begin automatically
    motorC.EnableRequest(true);
    Serial0.println("Cannula Motor Enabled");

    // Waits for HLFB to assert for Stir Motor
    Serial0.println("Waiting for Stir HLFB...");
    while (motorS.HlfbState() != MotorDriver::HLFB_ASSERTED) {
        continue;
    }
    Serial0.println("StirMotor Ready");

    // Waits for Cannula HLFB to assert (waits for homing to complete if applicable)
    Serial0.println("Waiting for Cannula  HLFB...");
    while (motorC.HlfbState() != MotorDriver::HLFB_ASSERTED) {
        continue;
    }
    Serial0.println("Cannula Motor Ready");
}

void loop() {
    // Put your main code here, it will run repeatedly:
    // read some stuff
    recCommand();
    //getFeedback();
    if (newData == true) {
      strcpy(tempChars, inputString);   // necessary to protect original data from srtok()
      parseData();
      sendCommands();
      newData = false;
    }
    
}
void recCommand(){
  static bool recInProgress = false;
  static byte index = 0;
  char startMark = '<';
  char endMark = '>';
  char readChar;

  while (Serial0.available()>0 && newData == false){
    readChar =Serial0.read();
    
    if (recInProgress == true){
      if (readChar!= endMark){
        inputString[index] = readChar;
        index++;
        if (index>=numChars) {
          index=numChars-1;
        }
      }
      else {
        inputString[index] = '\0';  //Terminate string
        recInProgress = false;       // reset things
        index = 0;
        newData = true;             // Set to process command
      }
    }
    else if (readChar == startMark) {
      recInProgress = true;
    }
    
  }
}
// data receive end

void parseData(){     // Sort out the input to cmd and value  
  char *strtokIndx;   // used by strtok
  strtokIndx = strtok(tempChars,":");  // delimiter of command string, get command
  strcpy(newCommand, strtokIndx); //copy command over

  strtokIndx = strtok(NULL,":");
  newValue =atoi(strtokIndx);

}

void sendCommands() {
  Serial0.print(" new Command is - ");
  Serial0.println(newCommand);
  Serial0.print(" new Value is - ");
  Serial0.println(newValue);
  Serial0.print(" Done ");
      // Spin at 750 RPM in the CW direction.
    if (strcmp(newCommand,"RPM") == 0){
        MoveAtVelocity(newValue);
        delay(1000);
    }

    else if (strcmp(newCommand,"UP+") == 0){
      MoveToPosition(1);      // our move absolute motor
      Serial0.print(" Going Up  \n");
      
    }
    else if (strcmp(newCommand,"DN-") == 0){
      MoveToPosition(2);      // Our move absolute function
      Serial0.print(" Going Down  \n");
    }
    else {
      Serial0.print(" COMMAND ERROR!!!");
      Serial0.println(newCommand);
    }
}


/*------------------------------------------------------------------------------
 * MoveAtVelocity
 *
 *    From Arduio/CC Examples
 *    Triggers a quadrature output commanding the desired velocity.
 *    Prints the velocity and move status to the USB serial port.
 *    Returns when HLFB asserts (indicating move has successfully completed).
 *
 * Parameters:
 *    double velocity  - The velocity in RPM to command
 *
 * Returns: True/False depending on whether a new velocity was reached
 */
bool MoveAtVelocity(double velocity) {
    // If the same velocity is commanded there's nothing to do.
    if (velocity == commandedVelocityStir) {
        return false;
    }

    // Check to see if the requested velocity exceeds the valid range.
    if (velocity > maxVelocityCCW || velocity < -maxVelocityCW) {
        Serial0.print("An invalid velocity of ");
        Serial0.print(velocity);
        Serial0.println(" RPM has been requested.");
        return false;
    }

    // Check if an alert is currently preventing motion
    if (motorS.StatusReg().bit.AlertsPresent) {
        Serial0.println("Motor status: 'In Alert'. Move Canceled.");
        return false;
    }

    Serial0.print("Commanding ");
    Serial0.print(velocity);
    Serial0.println(" RPM");

    // Determine which order the quadrature must be sent by determining if the
    // new velocity is greater or less than the previously commanded velocity
    // If greater, Input A begins the quadrature. If less, Input B begins the
    // quadrature.
    int32_t currentVelocityRounded = round(commandedVelocityStir / velocityResolution);
    int32_t targetVelocityRounded = round(velocity / velocityResolution);
    int32_t velocityDifference = labs(targetVelocityRounded - currentVelocityRounded);
    for (int32_t i = 0; i < velocityDifference; i++) {
        if (velocity > commandedVelocityStir){
            // Toggle Input A to begin the quadrature signal.
            motorS.MotorInAState(true);
            // Command a 5 microsecond delay to ensure proper signal timing.
            delayMicroseconds(5);
            motorS.MotorInBState(true);
            delayMicroseconds(5);
            motorS.MotorInAState(false);
            delayMicroseconds(5);
            motorS.MotorInBState(false);
            delayMicroseconds(5);
        }
        else {
            motorS.MotorInBState(true);
            delayMicroseconds(5);
            motorS.MotorInAState(true);
            delayMicroseconds(5);
            motorS.MotorInBState(false);
            delayMicroseconds(5);
            motorS.MotorInAState(false);
            delayMicroseconds(5);
        }
    }

    // Keeps track of the new commanded velocity
    commandedVelocityStir = velocity;

    // Waits for HLFB to assert (signaling the motor has successfully reached
    // its target velocity).
    // Serial0.println("Ramping Speed... Waiting for HLFB");
    // Wait for some time so HLFB has time to transition.
    delay(1);
    while (motorS.HlfbState() != MotorDriver::HLFB_ASSERTED) {
        continue;
    }

   // Serial0.println("Target Velocity Reached");
    Serial0.print(velocity);
    return true;
}


//------------------------------------------------------------------------------

/*------------------------------------------------------------------------------
 * MoveToPosition
 *
 *    Move to position number positionNum (defined in MSP)
 *    Prints the move status to the USB serial port
 *    Returns when HLFB asserts (indicating the motor has reached the commanded
 *    position)
 *
 * Parameters:
 *    int positionNum  - The position number to command (defined in MSP)
 *
 * Returns: True/False depending on whether a valid position was
 * successfully commanded and reached.
 */
bool MoveToPosition(int positionNum) {
    // Check if an alert is currently preventing motion
    if (motorC.StatusReg().bit.AlertsPresent) {
        Serial0.println("Motor status: 'In Alert'. Move Canceled.");
        return false;
    }

    Serial0.print("Moving to position: ");
    Serial0.print(positionNum);

    switch (positionNum) {
        case 1:
            // Sets Input A "off" for position 1
            motorC.MotorInAState(false);
            Serial0.println(" (Input A Off)");
            break;
        case 2:
            // Sets Input A "on" for position 2
            motorC.MotorInAState(true);
            Serial0.println(" (Input A On)");
            break;
        default:
            // If this case is reached then an incorrect positionNum was entered
            return false;
    }

    // Ensures this delay is at least 2ms longer than the Input A, B filter
    // setting in MSP
    delay(2 + INPUT_A_B_FILTER);

    // Waits for HLFB to assert (signaling the move has successfully completed)
    Serial0.println("Moving.. Waiting for HLFB");
    while (motorC.HlfbState() != MotorDriver::HLFB_ASSERTED) {
        continue;
    }

    Serial0.print("Position - ");
    Serial0.print(positionNum);
    return true;
}
//------------------------------------------------------------------------------

/*------------------------------------------------------------------------------
 * HomingSensorCallback
 *
 *    Reads the state of the homing sensor and passes the state to the motor.
 */
void HomingSensorCallback() {
    // A 1 ms delay is required in order to pass the correct filtered sensor
    // state
    delay(1);
    motorC.MotorInBState(digitalRead(HomingSensor));
}
//------------------------------------------------------------------------------d


/*-----------------------------------------------------------------------------
 * * HLFB trial
 * get some feedback out the port

void getFeedback(){
   // Check the state of the HLFB.
    MotorDriver::HlfbStates hlfbState = motorS.HlfbState();

    // Print the HLFB state.
    if (hlfbState == MotorDriver::HLFB_HAS_MEASUREMENT) {
        // Get the measured speed as a percent of Max Speed.
        float hlfbPercent = motorS.HlfbPercent();

        //Serial0.print("Speed output: ");

        if (hlfbPercent == MotorDriver::HLFB_DUTY_UNKNOWN) {
            Serial0.println("UNKNOWN");
        }
        else {
            char hlfbPercentStr[10];
            // Convert the floating point duty cycle into a string representation.
            snprintf(hlfbPercentStr, sizeof(hlfbPercentStr), "%.0f%%", hlfbPercent);
            Serial0.print(hlfbPercentStr);
            Serial0.println(" of maximum speed");
        }
    }
    else if (hlfbState == MotorDriver::HLFB_DEASSERTED) {
        Serial0.print(hlfbState);
        //Serial0.print(hlfbPercent);
        Serial0.print(hlfbState);
    }

}

 */
