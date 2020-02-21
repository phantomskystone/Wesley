package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.Robot.Line.vertical;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.lang.Math;


public class Drive extends Config {

    public LinearOpMode opMode;

    public Drive(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    //runtime is the timer
    private ElapsedTime runtime = new ElapsedTime();

    //lastAngles is used in the checkDirection() method
    public double lastAngles;

    PositionTracking tracker = new PositionTracking();

    //IMUAngle is used in getAngle()
    double IMUAngle;

    //assume the robot is at coordinates (0,0)
    Point robot = new Point(0,0);

    double velX = 0;
    double velY = 0;

    //current angle in radians
    double currentAngle = 0;
    //variable used for IMU direction correction
    double checkAngle;
    double setAngle = 0;

    //not used yet. Maybe later.
    double ROBOT_WEIGHT_KG = 10;
    double WHEEL_FRICTION = 3;

    double COUNTS_PER_REVOLUTION = 1440/1.5;
    double WHEEL_DIAMETER_INCHES = 1.5;

    double slipFactorY;
    double slipFactorX;

    //calculate distance based on encoder counts
    public double encoderDistanceY1() {
        //The encoder will be plugged into the same port as the frontLeft motor
        return frontLeft.getCurrentPosition() * PI * WHEEL_DIAMETER_INCHES / COUNTS_PER_REVOLUTION;
    }
    public double encoderDistanceY2() {
        //The encoder will be plugged into the same port as the frontRight motor
        return frontRight.getCurrentPosition() * PI * WHEEL_DIAMETER_INCHES / COUNTS_PER_REVOLUTION;
    }

    public double encoderDistanceX() {
        //The encoder will be plugged into the same port as the frontRight motor
        return backLeft.getCurrentPosition() * 3.1415926535 * WHEEL_DIAMETER_INCHES / COUNTS_PER_REVOLUTION;
    }

    public double encoderDistanceY() {
        return (encoderDistanceY1() + encoderDistanceY2())/2;
    }

    public void startPositionTracking() {
        tracker.start();
    }

    public void resetPosition() {
        tracker.interrupt();
    }

    //return calculated x and y positions

    public double x() {
        return robot.x;
    }

    public double y() {
        return robot.y;
    }

    public double getVelX() {
        return velX;
    }
    
    public double getVelY() {
        return velY;
    }
    

    public double getAngle() {
        return toDegrees(-currentAngle + PI/2);
    }

    double vX;
    double vY;

    public void setAngle(double angle) {
        setAngle = angle;
    }
    
    public void velocityReset() {
        vX = encoderDistanceX();
        vY = encoderDistanceY();
        runtime.reset();
    }
    public double getVelocity() {
        double d1 = encoderDistanceX() - vX;
        double d2 = encoderDistanceY() - vY;
        double v = sqrt(pow(d1, 2) + pow(d2, 2)) / runtime.seconds();
        return v;
    }

    // Sets power to drive at angle
    public void simpleDrive(double angle, double power) {

        //Change angle by 45
        angle = angle + 45;

        frontRight.setPower((sin(toRadians(angle)) * power));
        frontLeft.setPower((cos(toRadians(angle)) * power));
        backRight.setPower((cos(toRadians(angle)) * power));
        backLeft.setPower((sin(toRadians(angle)) * power));
    }

    //Calculates power to drive at angle with IMU correction
    private void angleDrive(double angle, double power) {

        //Change angle by 45
        angle = angle + 45;

        frontRight.setPower(sin(angle) * power - checkDirection());
        frontLeft.setPower(cos(angle) * power + checkDirection());
        backRight.setPower(cos(angle) * power - checkDirection());
        backLeft.setPower(sin(angle) * power + checkDirection());
    }

    public void relativeDrive(double angle, double distance, double power) {

        double d = sqrt(pow(encoderDistanceX(), 2) + pow(encoderDistanceY(), 2));

        while (d < distance) {
            d = sqrt(pow(encoderDistanceX(), 2) + pow(encoderDistanceY(), 2));
            angleDrive(angle, power);
        }
    }


    //Drive to a point on the robot
    public void pointDrive(double x, double y, double power) {
        double rX = x - robot.x;
        double rY = y - robot.y;
        double angle = atan2(rY, rX) - (currentAngle - PI/2);
        double powX = cos(angle);
        double powY = sin(angle) * 0.85;

        double maximizer = power / (abs(powX) + abs(powY));

        frontLeft.setPower(maximizer*(powY + powX) + checkDirection());
        frontRight.setPower(maximizer*(powY - powX) - checkDirection());
        backLeft.setPower(maximizer*(powY - powX) + checkDirection());
        backRight.setPower(maximizer*(powY + powX) - checkDirection());
    }

    public void pointDrive(Point point, double power) {
        double rX = point.x - robot.x;
        double rY = point.y - robot.y;
        double angle = atan2(rY, rX) - (currentAngle - PI/2);
        double powX = cos(angle);
        double powY = sin(angle) * 0.85;

        double maximizer = power / (abs(powX) + abs(powY) + abs(checkDirection()));

        frontLeft.setPower(maximizer*(powY + powX) + checkDirection());
        frontRight.setPower(maximizer*(powY - powX) - checkDirection());
        backLeft.setPower(maximizer*(powY - powX) + checkDirection());
        backRight.setPower(maximizer*(powY + powX) - checkDirection());
    }

    int currentPoint = 0;
    public static double dynamicAngle;
    public static double adjustedPower;

    public void runPath(Path pointList, double power, double drift) {
        motorsOn();

        boolean driving = true;

        Point crossDist = new Point(0,0);
        Point target = new Point(0,0);
        Point stopPoint = new Point(0,0);

        Line pathLine = new Line(0,0);
        Line robotLine;

        Line relativeLine = new Line(0,0);

        currentPoint = 0;

        Point pointOne = new Point(0,0);
        Point pointTwo = new Point(0,0);
        
        Point errorPoint = new Point(0,0);
        Point realErrorPoint = new Point(0,0);
        double error;
        double realError;

        dynamicAngle = 0;
        adjustedPower = power;

        pointOne.setPoint(robot);
        pointTwo.setPoint(pointList.get(0));

        while (driving && !opMode.isStopRequested() && !Thread.interrupted()) {

            pathLine.setLine(pointOne, pointTwo);

            robotLine = pathLine.perpendicularThrough(robot);

            stopPoint.setPoint(0.02 * velX, 0.02 * velY);

            relativeLine.setLine(pointOne.subtract(robot), pointTwo.subtract(robot));

            for (Point out : relativeLine.pointAtDistance(robot, pointTwo, drift)) {
                opMode.telemetry.addData("out x", out.x);
                opMode.telemetry.addData("out y", out.y);
            }

            if (relativeLine.pointAtDistance(robot, pointTwo, drift).get(0).distanceTo(pointTwo.subtract(robot)) <
                    relativeLine.pointAtDistance(robot, pointTwo, drift).get(1).distanceTo(pointTwo.subtract(robot))) {
                target.setPoint(relativeLine.pointAtDistance(robot, pointTwo, drift).get(0));
            } else {
                target.setPoint(relativeLine.pointAtDistance(robot, pointTwo, drift).get(1));
            }

            target = target.add(robot);
            target = target.subtract(stopPoint);

            errorPoint = target.subtract(robot);
            error = hypot(errorPoint.x, errorPoint.y);
            realErrorPoint = pointTwo.subtract(robot);
            realError = hypot(realErrorPoint.x, realErrorPoint.y);

            if (drift != 0) {
                crossDist.setPoint(pathLine.intersection(robotLine).subtract(robot));
                //target = target.add(crossDist);
            }


            if (pointTwo.hasAngle) {
                setAngle = pointTwo.angle;
            } else if (pointTwo.hasDynamicAngle) {
                dynamicAngle = pointTwo.dynamicAngle;
                setAngle = -toDegrees(atan2(errorPoint.y, errorPoint.x))+90 + pointTwo.dynamicAngle;
            } else {
                setAngle = -toDegrees(atan2(errorPoint.y, errorPoint.x))+90 + dynamicAngle;
            }
            if (pointTwo.hasPower) {
                adjustedPower = pointTwo.power;
            }

            pointDrive(target, adjustedPower);

            opMode.telemetry.addData("da", dynamicAngle);
            opMode.telemetry.addData("p2", pointTwo.x);
            opMode.telemetry.addData("p2", pointTwo.y);
            opMode.telemetry.addData("target", target.x);
            opMode.telemetry.addData("target", target.y);
            opMode.telemetry.addData("current point", currentPoint);
            opMode.telemetry.addData("robot x", robot.x);
            opMode.telemetry.addData("robot y", robot.y);
            opMode.telemetry.addData("angle", getAngle());
            opMode.telemetry.update();
            runtime.reset();

            if (realError < drift) {
                if (currentPoint == pointList.length() - 1) {
                    driving = false;
                } else {
                    pointOne = pointList.get(currentPoint);
                    pointTwo = pointList.get(currentPoint + 1);
                    currentPoint++;
                }
            }

            Thread.yield();
        }

        double errX;
        double errY;
        double finishPower;
        double integral = 0;
        double derivative;

        runtime.reset();

        ElapsedTime looptime = new ElapsedTime();

        while(!(pow(robot.x - pointTwo.x, 2) + pow(robot.y - pointTwo.y, 2) < 1) && !opMode.isStopRequested()
                && !Thread.interrupted() && runtime.seconds() < 3) {

            errX = robot.x - pointTwo.x;
            errY = robot.y - pointTwo.y;


            if (pointTwo.hasDynamicAngle) {
                dynamicAngle = pointTwo.angle;
                setAngle = -toDegrees(atan2(errY, errX))+90 + pointTwo.dynamicAngle;
            } else if (pointTwo.hasAngle) {
                setAngle = pointTwo.angle;
            }

            error = hypot(errX, errY);
            derivative = hypot(velX, velY);
            integral += error*looptime.seconds();
            looptime.reset();

            finishPower = error/30 + 0.07*integral - derivative * 0.004;
            if (finishPower > power) {
                finishPower = power;
            }
            if (!pointTwo.hasAngle) {
                setAngle = -toDegrees(atan2(pointTwo.subtract(pointOne).y, pointTwo.subtract(pointOne).x)) + 90 + dynamicAngle;
            }
            pointDrive(pointTwo, finishPower);
        }
        stop();
    }

    public void turnTo(double angle) {
        runtime.reset();
        setAngle = angle;
        while(checkDirection() > 0.05 && !opMode.isStopRequested() && runtime.seconds() < 2) {
            frontLeft.setPower(checkDirection());
            frontRight.setPower(- checkDirection());
            backLeft.setPower(checkDirection());
            backRight.setPower(- checkDirection());
        }
        stop();
    }


    //IMU angle correction methods

    public double checkDirection() {

        double correction;
        double gain = 0.015;

        double angles = toDegrees(-currentAngle + PI/2) % 360;

        correction = setAngle%360 - angles;

        if (abs(correction) > 180) {
            correction -= 360;
        }

        correction *= gain;

        return correction;
    }


    public void resetAngle() {
        lastAngles = 0;

        IMUAngle = 0;
    }

    public void stop() {
        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
    }


    protected class PositionTracking extends Thread {
        public void run() {
            try {

                double robotX = 0;
                double robotY = 0;
                
                double lastEncoderX = 0;
                double lastEncoderY = 0;

                double deltaEncoderX;
                double deltaEncoderY;

                double deltaX;
                double deltaY;

                double lastVelX = 0;
                double lastVelY = 0;

                ElapsedTime time = new ElapsedTime();

                final double encoderRadius = 13;

                int loopEvent = 0;

                while(!Thread.interrupted() && opMode.opModeIsActive()) {

                    currentAngle = (encoderDistanceY2()*2/3 - encoderDistanceY1()*2/3)/(encoderRadius) + (PI / 2);

                    deltaEncoderX = encoderDistanceX() - lastEncoderX;
                    deltaEncoderY = encoderDistanceY() - lastEncoderY;

                    deltaY = sin(currentAngle) * deltaEncoderY + sin(currentAngle - PI/2) * deltaEncoderX;
                    deltaX = cos(currentAngle) * deltaEncoderY + cos(currentAngle - PI/2) * deltaEncoderX;

                    robotX += deltaX;
                    robotY += deltaY;
                    robot.setPoint(robotX, robotY);

                    if(loopEvent==5) {
                        velX = (robot.x - lastVelX) / time.seconds();
                        velY = (robot.y - lastVelY) / time.seconds();
                        time.reset();
                        lastVelX = robot.x;
                        lastVelY = robot.y;
                        loopEvent = 0;
                    }

                    lastEncoderX = encoderDistanceX();
                    lastEncoderY = encoderDistanceY();

                    loopEvent++;

//                    opMode.telemetry.addData("robot x", robotX);
//                    opMode.telemetry.addData("robot y", robotY);
//                    opMode.telemetry.addData("robot.x", robot.x);
//                    opMode.telemetry.addData("robot.y", robot.y);
//                    opMode.telemetry.addData("currntAngle", currentAngle);
//                    opMode.telemetry.addData("angle degrees", getAngle());
//                    opMode.telemetry.update();

                    try {
                        Thread.sleep(10);
                    } catch(InterruptedException e) {
                        return;
                    }

                    Thread.yield();
                }

            } catch(Exception e) {
                //exception!
            }
        }
    }
}

