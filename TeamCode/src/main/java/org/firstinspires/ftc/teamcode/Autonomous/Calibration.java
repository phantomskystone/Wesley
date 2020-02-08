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
        drive.addPoint(30,30);
        drive.addPoint(0, 60);
        robot.runPath(drive, 0.4, 1.0);
        Point target = new Point();
        Point pointOne = new Point(5,15);
        Point pointTwo = new Point(10,20);
        Point position = new Point(5,5);
        target.setPoint(new Line(pointOne.subtract(position), pointTwo.subtract(position)).pointAtDistance(position, pointTwo, 15));
        while(opModeIsActive()) {
        }
    }
}