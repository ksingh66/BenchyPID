#include <Servo.h>

// PID class
class PIDController
{
public:
    PIDController(float p, float i, float d)
    {
        Kp = p;
        Ki = i;
        Kd = d;
        integral = 0;
        prevError = 0;
        maxIntegral = 20;
        lastTime = 0;
    }

    // This function computes the derivative and integral
    float compute_D_and_I(float error)
    {
        unsigned long now = millis(); // start counting the milliseconds here

        // dt = change in time from now to our last recording (lastTime is 0 for the first loop)
        float dt = (now - lastTime) / 1000.0; // We divide by 1000 here to get dt in seconds not milliseconds

        if (dt > 0.01)
        { // Minimum time step (100 Hz or 0.01 seconds): this means this part will only run every 0.01 seconds

            // Now we calculate our integral by multiplying our error with the time and adding it to integral: integral compounds to larger and larger values over time
            integral += error * dt;
            integral = constrain(integral, -maxIntegral, maxIntegral); // Since it compounds over time we need to set a limit to how high it can go

            // The effect integral creates is that of "momentum" where it will chase after the boat and if it cant catch it over time itll increase the speed at which its chasing

            // Derivative is found by finding the difference of our current error and our error 0.01 seconds ago and then dividing that by 0.01 which tells us if the error is inccreasing or decreasing
            // If the error is increasing (Derivative is +) then we speed up the chase and if its decreasing (Derivative is -) then we slow down a little to not overshoot

            float derivative = (dt > 0) ? (error - prevError) / dt : 0; // we could've made this simpler but we are tyring to not divide by 0

            // Note that the integral keeps us moving with momentum and the derivative slows us down, they kinda counteract each other like they do in math :)
            float output = Kp * error + Ki * integral + Kd * derivative; // Kp is how much the distance between where we are and where we wanna be matters, Ki is how much the time for which we have been chasing matters and Kd is how much the speed of wheather we are getting closer to the location or further matters.

            // kinda like incermenting we are setting our current error and time as our previos error and time to prepare for the next cycle
            prevError = error;
            lastTime = now;

            return output; // output is a number in degrees of movement that tells us how many degrees to move : Note that it does not tell us absolute posoition and tells us how to move relitivily
        }
        return 0;
    }

    // We need this function for when we lose tracking of an object and redetect it because we cant carry that old integral or else we will have crazy high momentum
    void reset()
    {
        integral = 0;
        prevError = 0;
        lastTime = millis();
    }

    // This is just to tweak the PID values
    void setTunings(float p, float i, float d)
    {
        Kp = p;
        Ki = i;
        Kd = d;
    }

private:
    float Kp, Ki, Kd;
    float integral;
    float prevError;
    unsigned long lastTime;
    float maxIntegral;
};

// Here we are making 2 Servo Objects for our two servos
Servo panServo;
Servo tiltServo;

// D3 is the pan signal wire and D9 is the tilt signal wire
const int panPin = 3;
const int tiltPin = 9;

// PID Controllers - much lower values for stability
PIDController panPID(0.2, 0.0, 0.05);  // Zero integral component for stability
PIDController tiltPID(0.2, 0.0, 0.05); // Zero integral component for stability

// Current positions
int currentPan = 90;
int currentTilt = 90;

// Target positions, We start with the target being the center
int targetPan = 90;
int targetTilt = 90;

// Smooth movement variables
unsigned long lastMoveTime = 0;
const unsigned long moveInterval = 20; // Update every 20ms for 50Hz (more stable): This here is what will affect the responsiveness of the camera

// How much the servos are allows to move in one cycle
int max_degrees_movement = 4;

// This will be the minimum number for the error for which the camera will begin to move
int deadzone_pan = 0.5;
int deadzone_tilt = 0.5;

// CRITICAL: Command timeout to prevent erratic movement when Python app closes
unsigned long lastCommandTime = 0;
const unsigned long commandTimeout = 1000; // 1 second timeout

void setup()
{
    Serial.begin(115200);
    panServo.attach(panPin);
    tiltServo.attach(tiltPin);

    // Initialize PID timers
    panPID.reset();
    tiltPID.reset();

    // Center servos and stay there
    targetPan = 90;
    targetTilt = 90;
    currentPan = 90;
    currentTilt = 90;

    // Move to initial position (center the camera)
    panServo.write(currentPan);
    tiltServo.write(currentTilt);
    delay(500); // Give the camera some time to center

    // Send ready signal to the Pi
    Serial.println("READY");

    // Set initial command time
    lastCommandTime = millis();
}

void loop()
{
    // Update motion at fixed intervals
    unsigned long currentTime = millis(); // We find the current time which we will use to compare to the last time we got a command

    // Check for command timeout - CRITICAL to prevent spazzing when Python closes - This is because when the Python script closes we won't recieve any message
    if (currentTime - lastCommandTime > commandTimeout)
    {
        // If no commands for a while, reset targets to center
        targetPan = 90;
        targetTilt = 90;

        // Reset PID controllers to prevent accumulated errors
        if (currentTime - lastCommandTime > commandTimeout + 500)
        {
            panPID.reset();
            tiltPID.reset();
        }
    }

    // Process movement at a regular interval
    if (currentTime - lastMoveTime >= moveInterval)
    {
        lastMoveTime = currentTime;

        // Calculate errors for both pan and tilt : Note that the error will be 0 on startup
        float panError = targetPan - currentPan;
        float tiltError = targetTilt - currentTilt;

        // Only move if outside a small deadzone
        if (abs(panError) > deadzone_pan || abs(tiltError) > deadzone_tilt)
        {
            // Calculate PID outputs
            float panOutput = panPID.compute(panError);
            float tiltOutput = tiltPID.compute(tiltError);

            // Limit maximum movement speed, this force the camera to move in these "steps" that are going to be as big as our max_degrees_movement variable
            panOutput = constrain(panOutput, -max_degrees_movement, max_degrees_movement);
            tiltOutput = constrain(tiltOutput, -max_degrees_movement, max_degrees_movement);

            // Apply output to current positions
            currentPan += panOutput;
            currentTilt += tiltOutput;

            // Constrain final positions: this is to not push the servo beyond what it can move
            currentPan = constrain(currentPan, 0, 180);
            currentTilt = constrain(currentTilt, 0, 180);

            // Move servos
            panServo.write(currentPan);
            tiltServo.write(currentTilt);
        }
    }

    // Check for commands
    if (Serial.available() > 0)
    {
        String command = Serial.readStringUntil('\n'); // Listening to the Pi for any messages
        command.trim();                                // remove whitespace

        if (command.startsWith("PT:"))
        {
            int commaIndex = command.indexOf(',', 3); // Start searching for where the servos should be aiming after the 3rd index
            if (commaIndex > 0)
            {
                // Extract target positions
                targetPan = command.substring(3, commaIndex).toInt();
                targetTilt = command.substring(commaIndex + 1).toInt();

                // Constrain to valid servo range : This part is important because if our servos are already at lets say 180 180, the PI does not know that and it will still tell us to turn the servos to 180 180 if the Benchy is on the edge of the screen.
                targetPan = constrain(targetPan, 0, 180);
                targetTilt = constrain(targetTilt, 0, 180);

                // Update command time - CRITICAL to track when last command was received
                lastCommandTime = millis();

                // Send feedback
                Serial.print("PT:");
                Serial.print(currentPan);
                Serial.print(",");
                Serial.println(currentTilt);
            }
        }
        else if (command.startsWith("TUNE:"))
        {
            // Format: TUNE:axis,P,I,D (axis: 0=pan, 1=tilt)
            // Example: TUNE:0,0.5,0.01,0.1
            int commaIndex1 = command.indexOf(',', 5);
            int commaIndex2 = command.indexOf(',', commaIndex1 + 1);
            int commaIndex3 = command.indexOf(',', commaIndex2 + 1);

            if (commaIndex3 > 0)
            {
                int axis = command.substring(5, commaIndex1).toInt();
                float p = command.substring(commaIndex1 + 1, commaIndex2).toFloat();
                float i = command.substring(commaIndex2 + 1, commaIndex3).toFloat();
                float d = command.substring(commaIndex3 + 1).toFloat();

                if (axis == 0)
                {
                    panPID.setTunings(p, i, d);
                }
                else if (axis == 1)
                {
                    tiltPID.setTunings(p, i, d);
                }

                // Update command time
                lastCommandTime = millis();

                Serial.println("TUNED");
            }
        }
        else if (command == "RESET")
        {
            // Emergency reset - center servos and reset PID
            targetPan = 90;
            targetTilt = 90;
            currentPan = 90;
            currentTilt = 90;
            panServo.write(currentPan);
            tiltServo.write(currentTilt);
            panPID.reset();
            tiltPID.reset();

            Serial.println("RESET_OK");
        }
    }
}