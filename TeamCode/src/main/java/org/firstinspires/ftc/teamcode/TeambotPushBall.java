package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Push Ball Off Vortex Base", group="a_Teambot")
public class TeambotPushBall extends TeambotLinearOpModeBase {

    static final int    DRIVE_SPEED             = 1440; // Nominal speed for better accuracy.
    static final double     TURN_SPEED          = 0.7;

    @Override
    public void runOpMode() {

        InitializeRobot();
        encoderDrive(DRIVE_SPEED,45.0,45.0);
        gyroTurn(TURN_SPEED, 90);
    }
}
