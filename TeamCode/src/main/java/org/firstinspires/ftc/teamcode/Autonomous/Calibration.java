package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Robot.Drive;
import org.firstinspires.ftc.teamcode.Robot.Path;
import org.firstinspires.ftc.teamcode.Robot.Runner;

@Autonomous(name="calibration")
@Disabled
public class Calibration extends LinearOpMode {
    Drive robot = new Drive(this);
    Runner runner = new Runner(robot);
    Orientation angles = new Orientation();
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.motorsOn();
        robot.doIMU();
        waitForStart();
        robot.startPositionTracking();
        Path drive = new Path();
        drive.addPoint(0,8,0);
        drive.addPoint(90, 18, -135);
        drive.addPoint(90, 48, -180);

        runner.setPath(drive, 0.75, 1);
        runner.start();
        while (runner.currentPoint() < 1) {
            idle();
        }
        robot.foundationServo.setPosition(0.3);
        while (runner.isAlive()) {
            idle();
        }
        robot.foundationServo.setPosition(0);

        sleep(500);

        Path drive2 = new Path();
        drive2.addPoint(78, 12);
        robot.runPath(drive2, 0.7, 1);

        robot.turnTo(-90);
    }
}
