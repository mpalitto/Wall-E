/////////////////////////////////
// Wall-E Robot Project Phase 1//
/////////////////////////////////
// In this phase 1, Wall-E has its 8266 uC connected to the serial interface from which receives commands
// The intent is to be able to correctly control the different motors for propulsion and other movements
// Propulsion engines: Mr drives the right engine, Ml dirves the left engine, they are controlled through an H-bridge
// Arms step motors: Ar drives the right Arm step motor, Al drives the left Arm step motor
// Head step motors: Hud drives the Up/Down head movment, Hrl drives the clock or anti-clock head rotation




//checks if a given string represents a valid integer within a specified range.
//It iterates through the string and ensures each character is a digit (0-9) or a leading '-' (if signed integers are allowed).
//If the string is valid, it converts it to an integer (long) and checks if it falls within the provided minimum and maximum values.
//It returns different error codes for invalid input (-2) and out-of-range numbers (-1).
int checkIntegerRange(const String& inputString, int minValue, int maxValue, bool isSigned = true) {
  // Check if the string contains only digits or (optionally) a leading '-' for signed integers
  bool isValid = true;
  int startIndex = 0;

  if (isSigned && inputString.charAt(0) == '-') {
    startIndex = 1;  // Allow a leading '-' for signed integers
  }

  for (int i = startIndex; i < inputString.length(); i++) {
    if (!isDigit(inputString.charAt(i))) {
      isValid = false;        // Input is not a valid integer number
      return -2;              // Error code for invalid input
    }
  }

  // If the string is valid, convert to integer
  if (isValid) {
    long inputLong = inputString.toInt();

    // Check if the value is within the specified range (inclusive)
    if (inputLong >= minValue && inputLong <= maxValue) {
      return static_cast<int>(inputLong); // Valid number within the range
    }
  }

  // Return -1 if number is out of range
  return -1;  // Error code for out-of-range number
}





//This class represents an input buffer that reads characters from the serial port and builds a complete string until a termination character (\n) is received.
    /**
     * Method: processInput
     * Processes incoming serial characters and detects a complete string.
     * @return: True if a complete string (terminated by '\n') is detected, False otherwise.
     */
    /**
     * Method: getBuffer
     * Returns the completed input string.
     * @return: Pointer to the null-terminated input string.
     */
    /**
     * Method: reset
     * Resets the buffer and index to their initial state.
     */
class InputFromSerial {
#define TERMINATION_CHAR '\n'  // Termination character for the input string

private:
    char* buffer;     // Dynamically allocated buffer to store incoming characters
    int bufferSize;   // Maximum size of the buffer
    int index;        // Index for the next character to store

public:
    /**
     * Constructor: Initializes the buffer with a given size and resets the index.
     * @param size: Maximum size of the input buffer
     */
    InputFromSerial(int size) : bufferSize(size), index(0) {
        buffer = new char[bufferSize];  // Allocate memory for the buffer
        buffer[0] = '\0';              // Initialize with an empty string
    }

    /**
     * Destructor: Frees the dynamically allocated buffer.
     */
    ~InputFromSerial() {
        delete[] buffer;  // Free the allocated memory
    }

    /**
     * Method: processInput
     * Processes incoming serial characters and detects a complete string.
     * @return: True if a complete string (terminated by '\n') is detected, False otherwise.
     */
    bool processInput() {
        while (Serial.available() > 0) {
            char receivedChar = Serial.read();  // Read one character
            if (receivedChar == TERMINATION_CHAR) {
                buffer[index] = '\0';  // Null-terminate the string
                return true;           // Input string is complete
            } else {
                // Add character to buffer if there's space
                if (index < bufferSize - 1) {
                    buffer[index++] = receivedChar;
                } else {
                    Serial.println("Error: Input buffer overflow!");
                    reset();  // Reset the buffer
                    return false;  // Input string is incomplete and buffer is full
                }
            }
        }
        return false;  // Input string is incomplete, still waiting for more chars
    }

    /**
     * Method: getBuffer
     * Returns the completed input string.
     * @return: Pointer to the null-terminated input string.
     */
    String getBuffer() const {
        return String(buffer);
    }
   // const char* getBuffer() const {
   //     return buffer;
   // }

    /**
     * Method: reset
     * Resets the buffer and index to their initial state.
     */
    void reset() {
        index = 0;
        buffer[0] = '\0';  // Clear the buffer
    }
};




//This class controls two motors.
//The constructor initializes the motor pins as outputs.
//Methods are provided for stopping, moving forward/backward, turning clockwise/counterclockwise, and controlling individual motor speed and direction.
//The right() and left() functions are significantly improved to handle both positive and negative speed percentages correctly, using abs() and setting the direction pin accordingly.
class TwoEnginePropulsion {
  private:
    // Motor control pins
    int motorRDirPin, motorRSpeedPin; // Right motor (direction and speed)
    int motorLDirPin, motorLSpeedPin; // Left motor (direction and speed)

  public:
    // Constructor to initialize motor pins
    TwoEnginePropulsion(int motorRDirPin, int motorRSpeedPin, int motorLDirPin, int motorLSpeedPin) {
      this->motorRDirPin = motorRDirPin;
      this->motorRSpeedPin = motorRSpeedPin;
      this->motorLDirPin = motorLDirPin;
      this->motorLSpeedPin = motorLSpeedPin;

      // Initialize motor pins as OUTPUT
      pinMode(motorRDirPin, OUTPUT);
      pinMode(motorRSpeedPin, OUTPUT);
      pinMode(motorLDirPin, OUTPUT);
      pinMode(motorLSpeedPin, OUTPUT);
    }

    // Set speed for both motors
    void stop() {
      digitalWrite(motorRSpeedPin, LOW);
      digitalWrite(motorLSpeedPin, LOW);
      digitalWrite(motorRDirPin, LOW);  // Right motor forward
      digitalWrite(motorLDirPin, LOW);  // Left motor forward
     }

    void right(int percentage_of_full) {
      int pwmValue = 0;
      if ( percentage_of_full <= -2) {
        pwmValue = map(-percentage_of_full, 0, 100, 0, 255);
        analogWrite(motorRSpeedPin, pwmValue);
        digitalWrite(motorRDirPin, LOW);  // Right motor forward
      } else if ( percentage_of_full >= 0) {
        pwmValue = map(percentage_of_full, 0, 100, 0, 255);
        digitalWrite(motorRSpeedPin, LOW);
        analogWrite(motorRDirPin, pwmValue);  // Right motor backward
      }
    }

    void left(int percentage_of_full) {
      int pwmValue = 0;
      if ( percentage_of_full <= -2) {
        pwmValue = map(-percentage_of_full, 0, 100, 0, 255);
        analogWrite(motorLSpeedPin, pwmValue);
        digitalWrite(motorLDirPin, LOW);  // Left motor forward
      } else if ( percentage_of_full >= 0) {
        pwmValue = map(percentage_of_full, 0, 100, 0, 255);
        digitalWrite(motorLSpeedPin, LOW);
        analogWrite(motorLDirPin, pwmValue);  // Left motor backward
      }
    }

    // Go forward
    void goForward() {
      digitalWrite(motorRDirPin, HIGH);  // Right motor forward
      digitalWrite(motorLDirPin, HIGH);  // Left motor forward
    }

    // Go backward
    void goBackward() {
      digitalWrite(motorRDirPin, LOW);   // Right motor backward
      digitalWrite(motorLDirPin, LOW);   // Left motor backward
    }

    // Turn clockwise (right motor faster)
    void turnClockWise(int percentage_of_full_speed) {
      int pwmValue = map(percentage_of_full_speed, 0, 100, 0, 255);
      analogWrite(motorRSpeedPin, pwmValue); // Increase right motor speed
      analogWrite(motorLSpeedPin, 0); // Stop left motor
      digitalWrite(motorRDirPin, HIGH); // Right motor forward
      digitalWrite(motorLDirPin, LOW);  // Left motor backward
    }

    // Turn counterclockwise (left motor faster)
    void turnAntiClockWise(int percentage_of_full_speed) {
      int pwmValue = map(percentage_of_full_speed, 0, 100, 0, 255);
      analogWrite(motorLSpeedPin, pwmValue); // Increase left motor speed
      analogWrite(motorRSpeedPin, 0); // Stop right motor
      digitalWrite(motorRDirPin, LOW);  // Right motor backward
      digitalWrite(motorLDirPin, HIGH); // Left motor forward
    }
};

//////////////////////////////
// Global Objects and Setup //
//////////////////////////////
// Defines motor pins using #define for better readability. 
// Creates an instance of the TwoEnginePropulsion class. 
// The setup() function initializes serial communication and stops the motors at startup. 
// TODO: improve The prompt to provide usage examples.

// Create an object of the TwoEnginePropulsion class
#define Right_dir   D8
#define Right_Speed D7
#define Left_dir    D5
#define Left_Speed  D6
TwoEnginePropulsion WallE_propulsion(Right_dir, Right_Speed, Left_dir, Left_Speed);

void setup() {
    Serial.begin(9600);
    pinMode(Right_Speed, OUTPUT); // Set D7 as an output
    pinMode(Right_dir, OUTPUT);   // Set D8 as an output
    pinMode(Left_dir, OUTPUT);    // Set D5 as an output
    pinMode(Left_Speed, OUTPUT);  // Set D6 as an output
    WallE_propulsion.stop();      // Set motor speed to 0% of full speed as pins are not in a defined state at startup
    delay(1000);
    Serial.println("Enter a string:");  // Prompt user
}

// it abbreviates the command using its first character (left 100 --> l 100, right 50 --> r 50, stop --> s...)
// in this way I can specify a command either with its complete name or with just the 1st letter (for lazy typing)
// unless otherwise specified as "special treated command" to be used, for exemple, if more than one command starts with the same letter... 
// than I can choose a different 1 letter command for one of them
String transformCommand(String command) {
  String value = "";
  String type = "";

  // Find the first space in the command
  int spaceIndex = command.indexOf(' ');

  // Extract the command type and value
  if (spaceIndex == -1) { // if no space is found
    value =  ""; // Return empty string
    type = command;
  } else {
    value = command.substring(spaceIndex + 1);
    type = command.substring(0, spaceIndex);
  }

  // Return the transformed string based on the type
  if (type == "special treated type") {
    return "x " + value;
  } else {
    return command.substring(0, 1) + " " + value; // type's first char + space + [value]
  }
}

// parses the command and calls the functions to do it
void processCommand(String command) {

  String cmd = transformCommand(command);
  String val = cmd.substring(2);  // Extract everything after 1st char
  cmd = cmd.substring(0, 2);
  Serial.println("--> " + cmd + " " + val);
  int speed = 0;
  if (val != "") {
    speed = checkIntegerRange(val, -100, 100);
    Serial.print("Valid SPEED: ");
    Serial.println(speed);
  }

  

  bool isInvalidCMD = false;
  if (speed == -2) {  // value is not a number
    Serial.print("EXE Cmd: ");
    // ECHO command
    if (cmd == "E ") {
      // Execute the code for ECHO
      Serial.print("ECHOING: ");
      Serial.println(val);  // Echo the received string
    }  else isInvalidCMD = true;
  } else if (speed != -1) { // value its an interger number and in range
      // right speed[0-100]% command
      if (cmd == "r ") {    
        Serial.print("right");
        Serial.print(", Speed: ");
        Serial.println(speed);
        WallE_propulsion.right(speed);
        
      } else if (cmd == "l ") {  // left engine  command
        Serial.print("left");
        Serial.print(", Speed: ");
        Serial.println(speed);
        WallE_propulsion.left(speed);
        
      } else if (cmd == "s ") { // set speed command
        Serial.print("STOP");
        WallE_propulsion.stop();
      } else isInvalidCMD = true;
  } else {  // it is an integer number but out of range
      Serial.println("Error: speed must be an INT number between 0 - 100");
  }
  if (isInvalidCMD) {
    Serial.println("Command not recognized");
  }
}

// Command string initialization
InputFromSerial inputBuffer(64);  // Instantiate the InputFromSerial object with a buffer size of 64
// make code more readable
#define command inputBuffer.getBuffer()
#define getCMD inputBuffer.processInput()

//////////
// loop //
//////////

void loop() {
    // Call the method to process serial input
    if (getCMD) {  // check if command has been received from serial interface

        // give a feedback as debugging tool
        Serial.print("You entered: ");  
        Serial.println(command);  // Echo the received string
        
        processCommand(command); // process and execute the command
        
        inputBuffer.reset();  // Reset the buffer
    }

    // Perform other tasks here if needed
}
