package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.I2cAddr;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See TeambotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 */
public class HardwareTeambot
{
    /* Public OpMode members. */
    public DcMotor  leftMotor   = null;
    public DcMotor  rightMotor  = null;
    public DcMotor lift1Motor = null;
    public DcMotor  lift2Motor  = null;
    public DcMotor  claw1Motor   = null;
    public DcMotor  claw2Motor  = null;
    public Servo leftArm = null;
    public Servo rightArm = null;
    public OpticalDistanceSensor lightSensor = null;
    public ColorSensor leftColorSensor = null;
    public ColorSensor rightColorSensor = null;

    public TouchSensor leftTouchSensor = null;
    public TouchSensor rightTouchSensor = null;
    ModernRoboticsI2cGyro gyro    = null;

    public final static double ARM_HOME = 0.20;
    public final static double ARM_MIN_RANGE  = 0.10;
    public final static double ARM_MAX_RANGE  = 0.90;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareTeambot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftMotor   = hwMap.dcMotor.get("left_drive");
        rightMotor  = hwMap.dcMotor.get("right_drive");
//        lift1Motor = hwMap.dcMotor.get("lift1");
//        lift2Motor  = hwMap.dcMotor.get("lift2");
//        claw1Motor   = hwMap.dcMotor.get("claw1");
//        claw2Motor  = hwMap.dcMotor.get("claw2");

        leftArm = hwMap.servo.get("left_arm");

        rightArm = hwMap.servo.get("right_arm");

        leftColorSensor = hwMap.colorSensor.get("left_color");
        leftColorSensor.setI2cAddress(I2cAddr.create7bit(0x1e)); //7-bit address for 0x3c (Standard address)
        rightColorSensor = hwMap.colorSensor.get("right_color");
        rightColorSensor.setI2cAddress(I2cAddr.create7bit(0x26)); //7-bit address for 0x4c (New Address for 2nd color sensor)

        lightSensor = hwMap.opticalDistanceSensor.get("sensor_ods");

        leftTouchSensor = hwMap.touchSensor.get("sensor_left_touch");
        rightTouchSensor = hwMap.touchSensor.get("sensor_right_touch");


        // Initialize Wheel Motors
        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        // Initialize Lift Motors
//        lift1Motor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
//        lift2Motor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
//        lift1Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        lift2Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        lift1Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        lift2Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        lift1Motor.setPower(0);
//        lift2Motor.setPower(0);

        // Initialize Claw Motors
//        claw1Motor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
//        claw2Motor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
//        claw1Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        claw2Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        claw1Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        claw2Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        claw1Motor.setPower(0);
//        claw2Motor.setPower(0);

        // Gyro
        gyro = (ModernRoboticsI2cGyro)hwMap.gyroSensor.get("gyro");


        leftArm.setPosition(ARM_MIN_RANGE);
        rightArm.setPosition(ARM_MAX_RANGE);


        leftColorSensor.enableLed(true);
        rightColorSensor.enableLed(true);
        lightSensor.enableLed(true);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}

