package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Push BLUE Beacon", group="a_Teambot")
public class TeambotBlueBeacon extends TeambotLinearOpModeBase {

    static final double     DRIVE_SPEED             = 0.3; // 0.7    // Nominal speed for better accuracy.
    static final double     TURN_SPEED             = 0.3; // 0.7    // Nominal speed for better accuracy.

    @Override
    public void runOpMode() {
        boolean success;
        InitializeRobot();

        encoderDrive(DRIVE_SPEED,6.0,6.0);
        gyroTurn(TURN_SPEED,-45.0);
        //encoderDrive(DRIVE_SPEED,40.0,40.0);
        driveToLine(DRIVE_SPEED, 70.0);
        encoderDrive(DRIVE_SPEED,2.0,2.0);

        sleep(100);
        gyroTurn(TURN_SPEED, -90);

        //gyroTurn(TURN_SPEED, -90);
        encoderDrive(DRIVE_SPEED,7,7);

        sleep(1000);
        PushBeaconButton(Color.Blue);
        //if(!success)
        //    PushBeaconButton(Color.Blue);
    }
}
