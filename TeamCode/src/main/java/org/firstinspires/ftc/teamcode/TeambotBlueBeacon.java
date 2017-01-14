package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Push BLUE Beacon", group="a_Teambot")
public class TeambotBlueBeacon extends TeambotLinearOpModeBase {

    static final double     DRIVE_SPEED             = 0.7; // 0.7    // Nominal speed for better accuracy.
    static final double     TURN_SPEED             = 0.5; // 0.7    // Nominal speed for better accuracy.

    @Override
    public void runOpMode() {
        boolean success;
        InitializeRobot();

        encoderDrive(DRIVE_SPEED,6.0,6.0);
        gyroTurn(TURN_SPEED,45.0);
        driveToLine(DRIVE_SPEED, 70.0);
        gyroTurn(TURN_SPEED, 90);

        success=PushBeaconButton(Color.Blue);
        if(!success)
            PushBeaconButton(Color.Blue);
    }

}
