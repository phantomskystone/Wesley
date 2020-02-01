package org.firstinspires.ftc.teamcode.RaameshBridge;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RaameshBridge.MecanumConfig;
import org.firstinspires.ftc.teamcode.RaameshBridge.proto.BlueAuto;
import org.firstinspires.ftc.teamcode.RaameshBridge.proto.RedAuto;

public class MecanumRedux2 extends MecanumRedux {
    /**
     *
     *
     *MecanumRedux.java
     * Written by Ian S. W. in Oct. 2019
     * Some code stolen from Raamesh B. (middleDrive)
     * Updated by Ian S. W. in Nov. 2019
     *
     */

    public MecanumRedux2(LinearOpMode thisOpmode){
        e=thisOpmode;
    }
    protected MecanumRedux2(){}



    public void middleDrive(MecanumConfig robot, int angle, float power){
        ElapsedTime runtime = new ElapsedTime();runtime.reset();
        angle=angle+45; // this does the thing. Magic numbers are magic. -Ian
        //this should work, however we need a controller input to test
        //while (runtime.seconds() < time && !linearOpMode.isStopRequested()){
        robot.frontRight.setPower((Math.sin(Math.toRadians(angle)) * power));
        robot.frontLeft.setPower((Math.cos(Math.toRadians(angle)) * power));
        robot.backRight.setPower((Math.cos(Math.toRadians(angle)) * power));
        robot.backLeft.setPower((Math.sin(Math.toRadians(angle)) * power));
        e.telemetry.addData("MIDDLE","DRIVING");
        e.telemetry.update();
    }
    protected void msDrive(MecanumConfig robot, int angle, float power){


        //this should work, however we need a controller input to test
        //while (runtime.seconds() < time && !linearOpMode.isStopRequested()){
        e.telemetry.addData("MS","FORWARDING");
        e.telemetry.update();
        middleDrive(robot,sideify(angle),power);
    }
    public void drive(MecanumConfig robot, int angle, double time, double power){
        super.drive(robot,angle,(float)time,(float)power);
    }
    public void complexDrive(MecanumConfig robot, int angle, double distance, double power){
        //do not use this!
    }
    public void sdrive(MecanumConfig robot, int angle, float time, float power){
        drive(robot,sideify(angle),time,power);
    }
    public void sdrive(MecanumConfig robot, int angle, double time, double power){
        drive(robot,sideify(angle),time,power);
    }
    public int sideify(int angle){
        if (e instanceof RedAuto){
            RedAuto e2=(RedAuto)e;
            if (e2!=null){
                if (e2.getSide()=="red"){
                    return angle;
                }
            }
        }
        if (e instanceof BlueAuto){
            BlueAuto e2=(BlueAuto)e;
            if (e2!=null){
                if (e2.getSide()=="blue"){
                    return -angle;
                }
            }
        }

        return angle;

    }
}
