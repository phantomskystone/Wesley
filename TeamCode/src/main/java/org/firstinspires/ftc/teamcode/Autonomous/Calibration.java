package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Robot.Drive;
import org.firstinspires.ftc.teamcode.Robot.Line;
import org.firstinspires.ftc.teamcode.Robot.Path;
import org.firstinspires.ftc.teamcode.Robot.Point;
import org.firstinspires.ftc.teamcode.Robot.Runner;

@Autonomous(name="calibration")

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
        drive.addPoint(0,30);
        drive.addPoint(30, 30);
        drive.addPoint(30,0);
        drive.addPoint(0,0);
        drive.addPoint(12,24);
        drive.addPoint(0,36);
        drive.addPoint(0,24);
        drive.addPoint(30,24);
        robot.runPath(drive, 0.4, 15);
    }
}