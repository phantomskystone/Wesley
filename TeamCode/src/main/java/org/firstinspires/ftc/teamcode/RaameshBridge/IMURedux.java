package org.firstinspires.ftc.teamcode.RaameshBridge;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RaameshBridge.PhantomConfig4200X;

public class IMURedux extends MecanumRedux2 {

    PhantomConfig4200X robot;
    float gain;
    float IMUg;
    public IMURedux(LinearOpMode thisOpMode, PhantomConfig4200X robot,float IMUGain){
        e=thisOpMode;
        this.robot=robot;
        gain = IMUGain;
        IMUg = IMUGain;
    }

    public void resetAngle()
    {
        robot.lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        robot.globalAngle = 0;
    }
    protected IMURedux(){}
    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    public double getAngle()
    {
        //if (robot.IMUf) {


            // We experimentally determined the Z axis is the axis we want to use for heading angle.
            // We have to process the angle because the imu works in euler angles so the Z axis is
            // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
            // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

            // Who wrote the above comment? - Ian


            // is this the correct angle? Th Controller is mounted on it's side, so theoretically we need to use the X axis.  -Ian
            Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double deltaAngle = angles.firstAngle - robot.lastAngles.firstAngle;

            if (deltaAngle < -180)
                deltaAngle += 360;
            else if (deltaAngle > 180)
                deltaAngle -= 360;

            robot.globalAngle += deltaAngle;

            robot.lastAngles = angles;

            return robot.globalAngle;
        //}
        //return 0;
    }
    public void imuDrive(int angle, double timeS, double maxAllowedPower) {
       // if (robot.IMUf) {
            angle = sideify(angle);
            angle = -angle-45;

            ElapsedTime runtime = new ElapsedTime();
            runtime.reset();
            e.telemetry.addData("running","true");
            e.telemetry.addData("Status",runtime.seconds()+" "+timeS +" "+e.isStopRequested());
            e.telemetry.update();

            while (!e.isStopRequested() && runtime.seconds() < timeS) {
               // e.telemetry.addData("Angle X", getAngle());
                e.telemetry.addData("Correction", checkDirection());
                //e.telemetry.addData("lastangles", robot.lastAngles.firstAngle);
                e.telemetry.update();
                robot.frontRight.setPower((-Math.sin(Math.toRadians(angle)) * maxAllowedPower) - checkDirection());
                robot.frontLeft.setPower((Math.cos(Math.toRadians(angle)) * maxAllowedPower) + checkDirection());
                robot.backRight.setPower((Math.cos(Math.toRadians(angle)) * maxAllowedPower) - checkDirection());
                robot.backLeft.setPower((-Math.sin(Math.toRadians(angle)) * maxAllowedPower) + checkDirection());
                e.sleep(1);
            }
            robot.frontRight.setPower(0);
            robot.backRight.setPower(0);
            robot.frontLeft.setPower(0);
            robot.backLeft.setPower(0);
       // }else{
         //   e.telemetry.addData("IMUf","bad");
        //}


    }   //mecanumTimeDrive
    public void imuDrive2(int angle, double timeS, double maxAllowedPower, boolean s) {
       // if (robot.IMUf) {
            if (s){angle = sideify(angle);}
            angle = -angle-45;

            ElapsedTime runtime = new ElapsedTime();
            runtime.reset();
            e.telemetry.addData("running","true");
            e.telemetry.addData("Status",runtime.seconds()+" "+timeS +" "+e.isStopRequested());
            e.telemetry.update();

            while (!e.isStopRequested() && runtime.seconds() < timeS) {
                // e.telemetry.addData("Angle X", getAngle());
                e.telemetry.addData("Correction", checkDirection());
                //e.telemetry.addData("lastangles", robot.lastAngles.firstAngle);
                e.telemetry.update();
                robot.frontRight.setPower((-Math.sin(Math.toRadians(angle)) * maxAllowedPower) - checkDirection());
                robot.frontLeft.setPower((Math.cos(Math.toRadians(angle)) * maxAllowedPower) + checkDirection());
                robot.backRight.setPower((Math.cos(Math.toRadians(angle)) * maxAllowedPower) - checkDirection());
                robot.backLeft.setPower((-Math.sin(Math.toRadians(angle)) * maxAllowedPower) + checkDirection());
                e.sleep(1);
            }
            robot.frontRight.setPower(0);
            robot.backRight.setPower(0);
            robot.frontLeft.setPower(0);
            robot.backLeft.setPower(0);
        //}else{
            e.telemetry.addData("IMUf","bad");
        //}


    }   //mecanumTimeDrive
    public void calibrate(){
        robot.imu.initialize(robot.parameters);
        e.telemetry.addData("Calibration","Doing it!");
        e.telemetry.update();
         while (!e.isStopRequested() && !robot.imu.isGyroCalibrated())
        {
            e.sleep(50);
            e.idle();
        }
        //robot.imu.startAccelerationIntegration(new Position(DistanceUnit.INCH,0,0,0,0),new Velocity(DistanceUnit.INCH,0,0,0,0),10);
        e.telemetry.addData("Calibration","Done!");
        e.telemetry.update();
    }
    public void mIMUddleDrive(int angle, float power){
       // if (robot.IMUf) {
            angle = -angle-45;
           // e.telemetry.addData("Angle X", getAngle());
            e.telemetry.addData("Correction", checkDirection());
           // e.telemetry.addData("lastangles", robot.lastAngles.firstAngle);
            e.telemetry.update();
            robot.frontRight.setPower((-Math.sin(Math.toRadians(angle)) * power) - checkDirection());
            robot.frontLeft.setPower((Math.cos(Math.toRadians(angle)) * power) +   checkDirection());
            robot.backRight.setPower((Math.cos(Math.toRadians(angle)) * power) -   checkDirection());
            robot.backLeft.setPower((-Math.sin(Math.toRadians(angle)) * power) +   checkDirection());
        //}
    }
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction;
        //double gain = 0.025;

        //gain = mult * angle;

        correction = -getAngle() * gain;        // reverse sign of angle for correction.

        return correction;
    }
    private void rotate(int degrees, double rPower)
    {
        double  leftPower=0;
        double rightPower=0;
        boolean quit=false;
        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = rPower;
            rightPower = -rPower;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -rPower;
            rightPower = rPower;
        }
        else if (degrees==0){leftPower=0;rightPower=0;quit=true;}
        //return;
        if (!quit) {

            //return;

            // set power to rotate.
            robot.backLeft.setPower(leftPower);
            robot.frontLeft.setPower(leftPower);
            robot.backRight.setPower(rightPower);
            robot.frontRight.setPower(rightPower);
            // rotate until turn is completed.
            if (degrees < 0) {
                // On right turn we have to get off zero first.
                while (e.opModeIsActive() && getAngle() == 0) {
                    e.telemetry.addData("heading", robot.globalAngle);
                    e.telemetry.update();
                }

                while (e.opModeIsActive() && getAngle() > degrees) {
                    e.telemetry.addData("heading", robot.globalAngle);
                    e.telemetry.update();
                }
            } else    // left turn.
                while (e.opModeIsActive() && getAngle() < degrees) {
                    e.telemetry.addData("heading", robot.globalAngle);
                    e.telemetry.update();
                }

            // turn the motors off.
            robot.backLeft.setPower(0);
            robot.frontLeft.setPower(0);
            robot.backRight.setPower(0);
            robot.frontRight.setPower(0);
            // wait for rotation to stop.
            e.sleep(500);
        }else{robot.backLeft.setPower(0);
            robot.frontLeft.setPower(0);
            robot.backRight.setPower(0);
            robot.frontRight.setPower(0);}
            // reset angle tracking on new heading.
        resetAngle();

    }
    public void imuTurn (double angle, double maxTime) {
        robot.globalAngle = -angle;

        gain = 0.02f;

        ElapsedTime time = new ElapsedTime();

        time.reset();

        double x = 0;

        while (!e.isStopRequested() && time.seconds() <= maxTime) {
            robot.frontRight.setPower(-checkDirection());
            robot.frontLeft.setPower(checkDirection());
            robot.backRight.setPower(-checkDirection());
            robot.backLeft.setPower(checkDirection());
            x += 0.05;
            if (time.seconds() > x) {
                gain += 0.005;
            }
        }

        robot.backLeft.setPower(0);
        robot.frontLeft.setPower(0);
        robot.backRight.setPower(0);
        robot.frontRight.setPower(0);

        gain = IMUg;
    }
    public void imuTurn (double angle, double maxTime,boolean xy) {
        robot.globalAngle = -angle;

        gain = 0.0125f;

        ElapsedTime time = new ElapsedTime();

        time.reset();

        double x = 0;

        while (!e.isStopRequested() && time.seconds() <= maxTime) {
            robot.frontRight.setPower(-checkDirection());
            robot.frontLeft.setPower(checkDirection());
            robot.backRight.setPower(-checkDirection());
            robot.backLeft.setPower(checkDirection());
            x += 0.05;
            if (time.seconds() > x) {
                gain += 0.003;
            }
        }

        robot.backLeft.setPower(0);
        robot.frontLeft.setPower(0);
        robot.backRight.setPower(0);
        robot.frontRight.setPower(0);

        gain = IMUg;
    }
}
