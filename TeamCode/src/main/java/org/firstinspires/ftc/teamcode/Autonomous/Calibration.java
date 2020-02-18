package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Robot.Drive;
import org.firstinspires.ftc.teamcode.Robot.Line;
import org.firstinspires.ftc.teamcode.Robot.Path;
import org.firstinspires.ftc.teamcode.Robot.Point;
import org.firstinspires.ftc.teamcode.Robot.Runner;

@Autonomous(name="Calibration")

public class Calibration extends LinearOpMode {
    Drive robot = new Drive(this);
    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();

        robot.motorsOn();
        robot.startPositionTracking();

        telemetry.addData(">", "Move robot forwards");

    }
}