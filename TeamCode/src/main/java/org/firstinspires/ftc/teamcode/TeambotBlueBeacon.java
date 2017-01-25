package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Push BLUE Beacon", group="a_Teambot")
public class TeambotBlueBeacon extends TeambotLinearOpModeBase {

    static final int     DRIVE_SPEED             = 1800;
    static final double     TURN_SPEED             = 0.5;

    @Override
    public void runOpMode() {
        boolean success;
        InitializeRobot();

        encoderDrive(DRIVE_SPEED,6.0,6.0);
        gyroTurn(TURN_SPEED,-47.0);

        driveToLine(DRIVE_SPEED, 70.0);
        encoderDrive(DRIVE_SPEED,2.0,2.0);

        //sleep(100);
        gyroTurn(TURN_SPEED, -90);

        encoderDrive(DRIVE_SPEED,12,12);

        //sleep(100);
        success = PushBeaconButton(Color.Blue);
        if(!success)
            PushBeaconButton(Color.Blue);

        // Push second beacon
        encoderDrive(DRIVE_SPEED, -7,-7);
        gyroTurn(TURN_SPEED, 0);
        driveToLine(DRIVE_SPEED,50.0);
        encoderDrive(DRIVE_SPEED,2.0,2.0);

        gyroTurn(TURN_SPEED, -90);

        encoderDrive(DRIVE_SPEED,8,8);

        sleep(100);
        success = PushBeaconButton(Color.Blue);
        if(!success)
            PushBeaconButton(Color.Blue);
    }
}
