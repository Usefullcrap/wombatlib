#include <kipr/wallaby.h>
#include <time.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*
    THIS TEMPLATE CODE ASSUMES A 2 WHEELED BOT WITH 2 TOPHAT SENSORS ON EACH SIDE OF THE FRONT OF THE BOT
    Created by Matteo Tang at an unreasonable time in the morning. WIP
    I am more than sure that this bot will be really rigid due to it not being able to drift with gyros. I hope to fix this in the future.
*/

#define gmpc(port) get_motor_position_counter(port)
#define cmpc(port) clear_motor_position_counter(port)

/*
degrees/sec per raw unit, confirm the exact value in the KIPR docs
*/
const double GYRO_SCALE = 512.0;//512 for wombat

static double angle = 0.0;
static double gyroBias = 0.0;
static struct timespec lastTime;
static int initialized = 0;

//Change as needed, bot may not act as intended if not
static double wheelRadiusMM = 40;
static int motor1Ticks = 1200;
static int motor2Ticks = 1200;
static int averageTicks = 0;
static double distanceMM = 0.0;
static int leftPort = 0;
static int rightPort = 1;
static int leftSensor = 0;
static int rightSensor = 1;
static int lineFollowValue1 = 3875;
static int lineFollowValue2 = 3875;

static int lightPort = 3;
static int lightThreshold = 2500;



/*
    Computes variables
*/
void initDriveConstants(){
    averageTicks = (motor1Ticks + motor2Ticks) / 2;
    distanceMM = (2.0 * M_PI * wheelRadiusMM) / averageTicks;
}





/*
    GYRO RELATED
*/
//Call this once at startup while the robot is completely still.
void calibrateGyro(int numSamples){
    long sum = 0;

    for (int i = 0; i < numSamples; i++){
        sum += gyro_z();
        msleep(2);
    }

    gyroBias = (double)sum / numSamples;
}

//Resets the accumulated angle back to zero and restarts the timer.
void resetAngle(){
    angle = 0.0;
    clock_gettime(CLOCK_MONOTONIC, &lastTime);
    initialized = 1;
}

//Returns angle in degrees
double getAngle(){
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    if (!initialized){
        lastTime = now;
        initialized = 1;
        return angle;
    }

    double dt = (now.tv_sec - lastTime.tv_sec) + (now.tv_nsec - lastTime.tv_nsec) / 1e9;

    double raw = (double)gyro_z() - gyroBias;
    double angularVelocity = raw / GYRO_SCALE; //degrees per second

    angle += angularVelocity * dt;
    lastTime = now;

    return angle;
}





/*
    TRANSVERSAL RELATED
Power 1 should be left, power 2 should be right. Check variables.
*/

//Cumulative distance (mm) traveled since the last cmpc() reset on both ports.
double calculateDistance(){
    int distanceTick = (gmpc(leftPort) + gmpc(rightPort)) / 2;
    return distanceTick * distanceMM;
}

//Stops all motors instantly
void resetMotors(){
    for (int i = 0; i < 4; i++){
        motor(i, 0);
        msleep(1);
    }
}

//Time based movement, ms, omnidirectional, allows for manual rotation. Does not account for gyro
void moveT(int power1, int power2, int time){
    motor(leftPort, power1);
    motor(rightPort, power2);
    msleep(time);
    resetMotors();
}

//Distance based movement, mm, forward and backward only.
void moveD(int power, int distance){
    if (power == 0 || distance <= 0){
        return;
    }

    cmpc(leftPort);
    cmpc(rightPort);

    double startAngle = getAngle();
    double completedDistance = 0;

    const int correctionPower = power / 4;   //how hard to nudge back on course
    const double headingTolerance = 3.0;     //degrees of drift allowed before correcting

    while (completedDistance < distance){
        double headingError = getAngle() - startAngle;

        if (headingError > headingTolerance){
            //right
            motor(leftPort, power);
            motor(rightPort, power - correctionPower);
        }
        else if (headingError < -headingTolerance){
            //left
            motor(leftPort, power - correctionPower);
            motor(rightPort, power);
        }
        else{
            motor(leftPort, power);
            motor(rightPort, power);
        }

        msleep(20);
        completedDistance = calculateDistance();
    }

    resetMotors();
    printf("moved %i", distance);
}

//Positive is counter clockwise, negative is clockwise
void turn(int degrees, int power){
    if (degrees == 0){ return; }

    double startAngle = getAngle();

    if (degrees > 0){
        while (getAngle() < startAngle + degrees){
            moveT(power, -power, 10);
        }
    }
    else{
        //clockwise
        while (getAngle() > startAngle + degrees){
            moveT(-power, power, 10);
        }
    }

    resetMotors();
    printf("Turn completed. New bearing is %f", getAngle());
}

/*
behavior at an intersection:
-1 = turn left
0 = move forward
1 = turn right
2 = stop

Based upon GCER 2026 line follow code which uses time. I dont think I will add a distance one
*/
void lineFollowT(int power, int time, int behavior){
    int tick = 0;

    while (tick < time){
        int a0Value = analog(leftSensor);
        int a1Value = analog(rightSensor);

        //Stop when either sensor sees black
        if (behavior == 2){
            if (a0Value > lineFollowValue1 || a1Value > lineFollowValue2){
                resetMotors();
                printf("stopped at intersection\n");
                return;
            }

            moveT(power, power, 50);
            tick += 50;
            continue;
        }

        //Intersection
        if (a0Value > lineFollowValue1 && a1Value > lineFollowValue2){
            switch (behavior){
            case -1:
                moveT(0, power, 100);
                printf("intersection: left\n");
                break;

            case 0:
                moveT(power, power, 100);
                printf("intersection: forward\n");
                break;

            case 1:
                moveT(power, 0, 100);
                printf("intersection: right\n");
                break;

            default:
                resetMotors();
                printf("invalid intersection behavior\n");
                return;
            }
        }
        //Correct right
        else if (a0Value > lineFollowValue1){
            moveT(0, power, 100);
            printf("correct right\n");
        }
        //Correct left
        else if (a1Value > lineFollowValue2){
            moveT(power, 0, 100);
            printf("correct left\n");
        }
        //Forward
        else{y
            moveT(power, power, 100);
            printf("forward\n");
        }

        tick += 100;
    }
}





/*
    ESSENTIALS
*/

//Full bot reset, change based on task
void reset(){
    /*
    You may want to put servos back to default positions here
    */
    initDriveConstants();
    calibrateGyro(250);
    resetAngle();
    disable_servos();
    resetMotors();
    printf("Bot reset.\n");
    float currentBatteryLife = power_level();
    if (currentBatteryLife < 15) {
        printf("WARNING! Battery low: %.1f%%\n", currentBatteryLife);
    }
    else if (currentBatteryLife < 30) {
        printf("Battery approaching low: %.1f%%\n", currentBatteryLife);
    }
    else {
        printf("Battery is at: %i%%\n", (int)currentBatteryLife);
    }
    msleep(3000);
}

//Wait for light + autoshutoff. true true is competition ready, false true is perfect for time testing, and false false is good for first runs.
void startSequence(bool waitForLight, bool autoShutOff){
    while (waitForLight && analog(lightPort) < lightThreshold){
        printf("Current light value: %d\n", analog(lightPort));
        msleep(50);
    }
    if (autoShutOff){
        int shutDownDelay = 119;
        shut_down_in(shutDownDelay); //Limit during competition is 120 seconds but 119 is safer.
        printf("Will shut down in %d seconds", shutDownDelay);
    }
    printf("Running");
}

int main(void){
    reset();
    startSequence(true, true);
    return 0;
}
