package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Robot.Drive;
import org.firstinspires.ftc.teamcode.Robot.Line;
import org.firstinspires.ftc.teamcode.Robot.Path;
import org.firstinspires.ftc.teamcode.Robot.Point;
import org.firstinspires.ftc.teamcode.Robot.Runner;

@Autonomous(name="Calibration")

public class Calibration extends LinearOpMode {

    Drive robot = new Drive(this);

    ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();

        robot.motorsOn();
        robot.startPositionTracking();

        runtime.reset();

        double valF;
        double valB;

        while (runtime.seconds() < 10) {
            telemetry.addData(">", "Move robot forwards");
            telemetry.addData("Seconds Left", 10-runtime.seconds());
            telemetry.addData("encoders", robot.encoderDistanceY());
        }

        valF = robot.encoderDistanceY();

        runtime.reset();

        while (runtime.seconds() < 10) {
            telemetry.addData(">", "Move robot backwards");
            telemetry.addData("Seconds Left", 10-runtime.seconds());
            telemetry.addData("encoders", robot.encoderDistanceY());
        }

        valB = robot.encoderDistanceY() - valF;
    }
}