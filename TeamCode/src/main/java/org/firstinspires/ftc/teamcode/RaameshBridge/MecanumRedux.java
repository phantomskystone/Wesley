package org.firstinspires.ftc.teamcode.RaameshBridge;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RaameshBridge.MecanumConfig;

public class MecanumRedux {
    /**
     *
     *
     *
     * Written by Ian S. W. in Oct. 2019
     *
     *
     *
     */
    LinearOpMode e;
    boolean debug=true;
    protected MecanumRedux(){}
    public MecanumRedux(LinearOpMode thisOpmode){
        e=thisOpmode;
    }
    public void drive(MecanumConfig m, int angle, float time, float power){
        ElapsedTime runtime = new ElapsedTime();runtime.reset();
        angle=angle+45; // this does the thing. Magic numbers are magic. -Ian
        //this should work, however we need a controller input to test
        //while (runtime.seconds() < time && !e.isStopRequested()){
            m.frontRight.setPower((Math.sin(Math.toRadians(angle)) * power)); //We need a weight fix, if only we had a center of mass overlay.
            m.frontLeft.setPower((Math.cos(Math.toRadians(angle)) * power));
            m.backRight.setPower((Math.cos(Math.toRadians(angle)) * power));
            m.backLeft.setPower((Math.sin(Math.toRadians(angle)) * power));

        //}
        while (runtime.seconds() < time && !e.isStopRequested()){
            if (debug) {


                e.telemetry.addData("Timestamp: ", System.nanoTime());
                e.telemetry.addData("Front Right: ", Math.sin(Math.toRadians(angle)) * power);
                e.telemetry.addData("Front Left: ", Math.cos(Math.toRadians(angle)) * power);
                e.telemetry.addData("Back Right: ", Math.cos(Math.toRadians(angle)) * power);
                e.telemetry.addData("Back Left: ", Math.sin(Math.toRadians(angle)) * power);
                e.telemetry.update();
            }
        } //TODO
        m.frontLeft.setPower(0);
        m.frontRight.setPower(0);
        m.backLeft.setPower(0);
        m.backRight.setPower(0);
    }
    public void simpleDrive(MecanumConfig m, float time, float power){
        ElapsedTime runtime = new ElapsedTime();runtime.reset();
        //angle=angle+45; // this does the thing. Magic numbers are magic. -Ian
        //this should work, however we need a controller input to test
        //while (runtime.seconds() < time && !e.isStopRequested()){
        m.frontRight.setPower(power); //We need a weight fix, if only we had a center of mass overlay.
        m.frontLeft.setPower(power);
        m.backRight.setPower(power);
        m.backLeft.setPower(power);

        //}
        while (runtime.seconds() < time && !e.isStopRequested()){
            if (debug) {


                e.telemetry.addData("Timestamp: ", System.nanoTime());
                e.telemetry.addData("Front Right: ",  power);
                e.telemetry.addData("Front Left: ", power);
                e.telemetry.addData("Back Right: ", power);
                e.telemetry.addData("Back Left: ", power);
                e.telemetry.update();
            }
        } //TODO
        m.frontLeft.setPower(0);
        m.frontRight.setPower(0);
        m.backLeft.setPower(0);
        m.backRight.setPower(0);
    }
    /** This literally does the same thing as above. Good job. 10/10 -Ian
    public void driveSwitch(MecanumConfig m, int angle, int time, float power){
        ElapsedTime runtime = new ElapsedTime();runtime.reset();
        angle=-angle-45; // Wesley wrote this!
        angle=angle+90; //Magic numbers!
        //this should work, however we need a controller input to test
        while (runtime.seconds() < time && !e.isStopRequested()){
            m.frontRight.setPower((Math.cos(Math.toRadians(angle)) * power));
            m.frontLeft.setPower((Math.sin(Math.toRadians(angle)) * power));
            m.backRight.setPower((Math.sin(Math.toRadians(angle)) * power));
            m.backLeft.setPower((Math.cos(Math.toRadians(angle)) * power));

        }
        m.frontLeft.setPower(0);
        m.frontRight.setPower(0);
        m.backLeft.setPower(0);
        m.backRight.setPower(0);
    }

     **/

}
