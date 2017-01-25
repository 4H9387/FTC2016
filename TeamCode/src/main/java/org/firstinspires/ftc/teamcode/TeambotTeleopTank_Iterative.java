
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Teambot: Teleop Tank", group="Teambot")
//@Disabled
public class TeambotTeleopTank_Iterative extends OpMode{

    /* Declare OpMode members. */
    HardwareTeambot robot       = new HardwareTeambot(); // use the class created to define a Pushbot's hardware
                                                         // could also use HardwarePushbotMatrix class.
    double      leftArmPosition      = robot.ARM_MIN_RANGE;
    double      rightArmPosisiton    = robot.ARM_MAX_RANGE;
    double      leftPower            = 0;
    double      rightPower           = 0;
    double      ARM_SPEED            = 0.02;
    double      ACCELERATION         = 0.04;
    double      LIFT_GEAR_REDUCTION  = 10.0;
    int         COUNTS_PER_MOTOR_REV = 1440 ;    // eg: TETRIX Motor Encoder
    int         LIFT_MAX             = (int) ( COUNTS_PER_MOTOR_REV * 10 *  LIFT_GEAR_REDUCTION);
    int         CLAW_MAX             = (int) (COUNTS_PER_MOTOR_REV * 0.5);

    boolean isLatchOpen = false;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        robot.leftColorSensor.enableLed(true);
        robot.rightColorSensor.enableLed(true);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        RunWheels();

        // Run the Servos
        RunServos();

        // Run the Latch
        RunLatch();

        // Only run the lift and claws if the latch is open
        if(isLatchOpen) {
            // Run the claws
            RunClaws();

            // Run the lift
            //RunLift();
        }
    }

    // This method controls what happens with the servos for the button pushers for each iteration
    private void RunServos() {
        if (gamepad2.left_bumper)
            leftArmPosition += ARM_SPEED;
        else
            leftArmPosition -= ARM_SPEED;

        if (gamepad2.right_bumper)
            rightArmPosisiton -= ARM_SPEED;
        else
            rightArmPosisiton += ARM_SPEED;

        leftArmPosition = Range.clip(leftArmPosition, robot.ARM_MIN_RANGE, robot.ARM_MAX_RANGE);
        rightArmPosisiton = Range.clip(rightArmPosisiton, robot.ARM_MIN_RANGE, robot.ARM_MAX_RANGE);

        telemetry.addData("armL", leftArmPosition);
        telemetry.addData("armR", rightArmPosisiton);

        robot.leftArm.setPosition(leftArmPosition);
        robot.rightArm.setPosition(rightArmPosisiton);
    }

    // This method controls the acceleration of the robot.
    private double RampMotor(double value, double target, double rate)
    {
        double newValue=value;
        if(newValue < target)
        {
            if(target-newValue<rate)
                newValue = target;
            else
                newValue += rate;
        }
        else
        {
            if(newValue-target < rate)
                newValue = target;
            else
                newValue -= rate;
        }

        return Range.clip(newValue, -1.0, 1.0);
    }

    // This method controls what happens with the drive wheels for each iteration
    private void RunWheels() {
        double accel = ACCELERATION;
        double leftTarget = -gamepad1.left_stick_y;
        double rightTarget = -gamepad1.right_stick_y;

        // If one of the gamepad 1 bumpers is pushed, run at half speed
        if(gamepad1.left_bumper || gamepad1.right_bumper)
        {
            leftTarget *= 0.5;
            rightTarget *= 0.5;
            accel *= 0.5;
        }

        // Move the power at a slower rate to increase robot control
        leftPower = RampMotor(leftPower,leftTarget, accel);
        rightPower = RampMotor(rightPower,rightTarget, accel);

        // If a touch sensor is pushed, stop the wheels on that side
        if(robot.leftTouchSensor.isPressed() && gamepad1.left_stick_y < 0)
            leftPower=0;
        if(robot.rightTouchSensor.isPressed() && gamepad1.right_stick_y < 0)
            rightPower=0;

        telemetry.addData("left", leftPower);
        telemetry.addData("right", rightPower);
        robot.leftMotor.setPower(leftPower);
        robot.rightMotor.setPower(rightPower);
    }

    // This method controls what happens with the latch
    private void RunLatch() {

        // we can only open the latch
        if(isLatchOpen)
            return;

        if(gamepad1.y)
        {
            robot.liftLatch.setPosition(robot.LATCH_OPEN);
        }

        if(robot.liftLatch.getPosition() == robot.LATCH_OPEN)
            isLatchOpen=true;
    }

    // This method controls what happens with the Lift
    private void RunLift() {
        double power = gamepad2.left_stick_y;

        if(power >0)
            robot.lift1Motor.setTargetPosition(-LIFT_MAX);
        else
            robot.lift1Motor.setTargetPosition(0);

        robot.lift1Motor.setPower(power);
    }

    // This method controls what happens with the claws
    private void RunClaws() {

        double stick = -gamepad2.right_stick_y * 0.1;
        double power = 1;
        int currentPosition = robot.claw1Motor.getCurrentPosition();

        if(stick >0) {
            robot.claw1Motor.setTargetPosition(CLAW_MAX);
            robot.claw2Motor.setTargetPosition(CLAW_MAX);
            power = 1;
        }
        else if(stick <0){
            robot.claw1Motor.setTargetPosition(0);
            robot.claw2Motor.setTargetPosition(0);
            power = -1;
        }
        else
        {
            robot.claw1Motor.setTargetPosition(currentPosition);
            robot.claw2Motor.setTargetPosition(currentPosition);
            power = 1;
        }

        if(stick ==0 && currentPosition<10)
            power = 0;

        telemetry.addData("claw power", power);
        telemetry.addData("claw position", currentPosition);
        telemetry.addData("claw target", robot.claw1Motor.getTargetPosition());
        robot.claw1Motor.setPower(power);
        robot.claw2Motor.setPower(power);
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        robot.leftColorSensor.enableLed(false);
        robot.rightColorSensor.enableLed(false);
    }
}
