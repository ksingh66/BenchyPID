#include <Servo.h>

// We make Servo objects
Servo panServo;  // Up and down
Servo tiltServo; // Left and right

// Here we define the pins for our servos
const int panPin = 3;  // Signal pin for the pan servo
const int tiltPin = 9; // Signal pin for the tilt servo

// Starting positions for both the servos- This makes them aim in the middle
int panPos = 90;
int tiltPos = 90;

const int smoothDelay = 0; // There will be no delay for the position updates
const int smallStep = 40;  // The larger the step the faster the servo moves

void setup()
{
    Serial.begin(115200); // We open a connection to our connected computer with 115200 baud rate

    // Now we attach the right pins to each servo object
    panServo.attach(panPin);
    tiltServo.attach(tiltPin);

    // Now we center our camera for startup
    panServo.write(panPos);
    tiltServo.write(tiltPos);
    delay(500); // We delay to allow our motors to get into position

    Serial.println("Pan-Tilt Camera Ready"); // This notifies our connected computer that we are ready
}

void loop()
{
    // We first need to check if data is ready to be read or if data has been sent
    if (Serial.available() > 0)
    {
        // If data is available to read
        String command = Serial.readStringUntil('\n'); // We will read whatever our input is until we encounter a newline which indicated end of message
        command.trim();                                // This line removed any whitespaces from our read command

        // If logic has reached here it means we have recieved a message and it has been cleaned
        // Now we need to corrospond each expected input with what kind movement we want with the sensor
        if (command.startsWith("PAN:"))
        {
            // This means the user wants to move our PAN motor to a certain angle
            int angle = command.substring(4).toInt(); // Here we are taking the substring after index 4 or in other words we are taking the 5th letter (because 0 indexed string) and onwards

            // Angle now is an integer which will be the angle the user wants to move the motor to
            angle = constrain(angle, 0, 180); // I am doing this incase we send an angle higher than 180 or less than 0, in case that happens itll set it to eiether 0 or 180
            // the contstrain is more of a safety thing

            // Now to actually move our motor
            panServo.write(angle);
            Serial.print("Pan set to: ");
            Serial.print(angle);
        }

        // Now to cover TILT and Pan and TILT both in one command
        else if (command.startsWith("TILT:"))
        {
            int angle = command.substring(5).toInt();
            angle = constrain(angle, 0, 180);
            tiltServo.write(angle);
            tiltPos = angle;
            Serial.print("Tilt set to: ");
            Serial.println(angle);
        }

        else if (command.startsWith("PT:"))
        {
            // Combined pan and tilt command this will be a bit more complicated with parsing
            int commaIndex = command.indexOf(',', 3); // This line just finds us the index at which the comma is located in our string, the 3 is there because we know the first 3 characters are not the comma so we start searching from 3
            // We will use this comma index to seperate our pan and tilt angle values we recieved from our computer
            if (commaIndex > 0)
            {
                // if our recieved message does infact have a comma then we take our angles
                int panAngle = command.substring(3, commaIndex).toInt();   // This find the string after the : in PT: and this string ends at the comma
                int tiltAngle = command.substring(commaIndex + 1).toInt(); // This one starts at the comma and ends at the end of the recieved message

                panAngle = constrain(panAngle, 0, 180);
                tiltAngle = constrain(tiltAngle, 0, 180);

                panServo.write(panAngle);
                tiltServo.write(tiltAngle);
                panPos = panAngle;
                tiltPos = tiltAngle;

                Serial.print("Pan set to: ");
                Serial.print(panAngle);
                Serial.print(", Tilt set to: ");
                Serial.println(tiltAngle);
            }
        }
    }
}