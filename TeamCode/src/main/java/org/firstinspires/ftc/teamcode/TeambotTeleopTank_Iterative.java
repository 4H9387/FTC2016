/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;


/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwareTeambot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Teambot: Teleop Tank", group="Teambot")
//@Disabled
public class TeambotTeleopTank_Iterative extends OpMode{

    /* Declare OpMode members. */
    HardwareTeambot robot       = new HardwareTeambot(); // use the class created to define a Pushbot's hardware
                                                         // could also use HardwarePushbotMatrix class.
    double leftArmPosition      = robot.ARM_MIN_RANGE;
    double rightArmPosisiton    = robot.ARM_MAX_RANGE;
    double leftPower            = 0;
    double rightPower           = 0;
    final double ARM_SPEED      = 0.02;
    final double ACCELERATION   = 0.04;


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
    }

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

    private double Accelerate(double value, double target, double rate)
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
    private void RunWheels() {

        double leftTarget = -gamepad1.left_stick_y;
        double rightTarget = -gamepad1.right_stick_y;

        if(gamepad1.left_bumper || gamepad1.right_bumper)
        {
            leftTarget *= 0.5;
            rightTarget *= 0.5;
        }

        leftPower = Accelerate(leftPower,leftTarget, ACCELERATION);
        rightPower = Accelerate(rightPower,rightTarget, ACCELERATION);

        if(robot.leftTouchSensor.isPressed())
            leftPower=0;
        if(robot.rightTouchSensor.isPressed())
            rightPower=0;

        telemetry.addData("left", leftPower);
        telemetry.addData("right", rightPower);
        robot.leftMotor.setPower(leftPower);
        robot.rightMotor.setPower(rightPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
