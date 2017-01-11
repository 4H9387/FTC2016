package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Push Ball Off Vortex Base", group="a_Teambot")
public class TeambotPushBall extends TeambotLinearOpModeBase {

    static final double     DRIVE_SPEED             = 0.7; // 0.7    // Nominal speed for better accuracy.

    @Override
    public void runOpMode() {

        InitializeRobot();
        encoderDrive(DRIVE_SPEED,50.0,50.0);
        gyroTurn(DRIVE_SPEED, 90);
        encoderDrive(DRIVE_SPEED, 40,40);
    }
}
