package org.firstinspires.ftc.teamcode.RaameshBridge.proto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RaameshBridge.MecanumConfig;
import org.firstinspires.ftc.teamcode.RaameshBridge.MecanumRedux2;

public class DebugMotion {
    
    
   
    public static Boolean debug(LinearOpMode linearOpMode){
        MecanumConfig robotdeb = new MecanumConfig();
        robotdeb.init(linearOpMode.hardwareMap);
        if (linearOpMode.gamepad1.a==true && linearOpMode.gamepad1.b==true && linearOpMode.gamepad1.x==true && linearOpMode.gamepad1.y==true && linearOpMode.gamepad1.left_bumper==true && linearOpMode.gamepad1.right_bumper!=true) {
            MecanumRedux2 mr = new MecanumRedux2(linearOpMode);
            robotdeb.frontRight.setPower(0.5);
            linearOpMode.sleep(2500);
            robotdeb.stop();
            linearOpMode.sleep(2500);
            robotdeb.frontLeft.setPower(0.5);
            linearOpMode.sleep(2500);
            robotdeb.stop();
            linearOpMode.sleep(2500);
            robotdeb.backRight.setPower(0.5);
            linearOpMode.sleep(2500);
            robotdeb.stop();
            linearOpMode.sleep(2500);
            robotdeb.backLeft.setPower(0.5);
            linearOpMode.sleep(2500);
            robotdeb.stop();
            linearOpMode.sleep(2500);
            mr.drive(robotdeb, 0, 2.5f, 0.5f);
            linearOpMode.sleep(2500);
            mr.drive(robotdeb, 0, 2.5f, -0.5f);
            return true;
        }
        return false;
    }
    
}
