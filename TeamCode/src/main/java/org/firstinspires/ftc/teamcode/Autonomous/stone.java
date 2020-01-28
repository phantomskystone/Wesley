package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.Drive;
@Autonomous(name="hello world")
@Disabled
public class stone extends LinearOpMode {
    Drive robot = new Drive(this);
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        robot.motorsOn();
        double position = 0.19;
        boolean grip = false;
        robot.leftBase.setPosition(-(Math.abs(position-0.24)+0.24) + 1);
        robot.rightBase.setPosition((Math.abs(position-0.24)+0.24));
        robot.leftStabilization.setPosition((Math.abs(position-0.24)+0.24)*1.13 - 0.07);
        robot.rightStabilization.setPosition(-(Math.abs(position-0.24)+0.24)*1.13 + 1.07);
        if (grip) {
            robot.leftGrip.setPosition(0.55);
            robot.rightGrip.setPosition(0.45);
        }
        else {
            robot.leftGrip.setPosition(0.87);
            robot.rightGrip.setPosition(0.07);
        }
        robot.frontLeft.setPower(0.4);
        robot.frontRight.setPower(0.4);
        robot.backLeft.setPower(0.4);
        robot.backRight.setPower(0.4);
        robot.intake1.setPower(-0.5);
        robot.intake2.setPower(-0.5);
        sleep(3000);
        robot.stop();
        position = 0.25;
        grip = true;
        robot.leftBase.setPosition(-(Math.abs(position-0.24)+0.24) + 1);
        robot.rightBase.setPosition((Math.abs(position-0.24)+0.24));
        robot.leftStabilization.setPosition((Math.abs(position-0.24)+0.24)*1.13 - 0.07);
        robot.rightStabilization.setPosition(-(Math.abs(position-0.24)+0.24)*1.13 + 1.07);
        if (grip) {
            robot.leftGrip.setPosition(0.55);
            robot.rightGrip.setPosition(0.45);
        }
        else {
            robot.leftGrip.setPosition(0.87);
            robot.rightGrip.setPosition(0.07);
        }
        sleep(2000);
        position = 0.7;
        robot.leftBase.setPosition(-(Math.abs(position-0.24)+0.24) + 1);
        robot.rightBase.setPosition((Math.abs(position-0.24)+0.24));
        robot.leftStabilization.setPosition((Math.abs(position-0.24)+0.24)*1.13 - 0.07);
        robot.rightStabilization.setPosition(-(Math.abs(position-0.24)+0.24)*1.13 + 1.07);
        if (grip) {
            robot.leftGrip.setPosition(0.55);
            robot.rightGrip.setPosition(0.45);
        }
        else {
            robot.leftGrip.setPosition(0.87);
            robot.rightGrip.setPosition(0.07);
        }
        sleep(3000);
    }
}
