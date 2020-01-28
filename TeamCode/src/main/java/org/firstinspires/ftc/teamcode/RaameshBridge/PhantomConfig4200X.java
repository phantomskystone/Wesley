package org.firstinspires.ftc.teamcode.RaameshBridge;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public final class PhantomConfig4200X {
    public DcMotor frontLeft  = null;
    public DcMotor backLeft   = null;
    public DcMotor frontRight = null;
    public DcMotor backRight  = null;
    public BNO055IMU imu      = null;
    public Servo armServo=null;
    public Servo capServo=null;
    public Servo stoneServo=null;
    public Servo skyServo=null;
    public ColorSensor colorSense=null;
    public DistanceSensor colorDist=null;
    public Rev2mDistanceSensor laserDist=null;
    HardwareMap hwMap = null;
    public Orientation lastAngles = new Orientation();
    public double                  globalAngle;
    public double offSetAngle;
    public boolean isInit = false;





    public PhantomConfig4200X() {}

    public BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();



    public void startShip(HardwareMap ahwMap) {
        hwMap = ahwMap;

        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        //parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        //parameters.loggingEnabled      = true;
        //parameters.loggingTag          = "IMU";
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        frontLeft = hwMap.get(DcMotor.class, "front_left_motor");
        backLeft = hwMap.get(DcMotor.class, "back_left_motor");
        frontRight = hwMap.get(DcMotor.class, "front_right_motor");
        backRight = hwMap.get(DcMotor.class, "back_right_motor");

        frontRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //backRight = hwMap.get(DcMotor.class, "back_right_motor");
        //try {hwMap.get(Servo.class, "newC"){
        //}
        //catch(IllegalArgumentException exce)
        armServo= hwMap.get(Servo.class,"armServo");
        armServo.setDirection(Servo.Direction.REVERSE);

       /* capServo= hwMap.get(Servo.class,"capServo");
        capServo.setDirection(Servo.Direction.REVERSE);

        stoneServo= hwMap.get(Servo.class,"stoneServo");
        stoneServo.setDirection(Servo.Direction.REVERSE);

        skyServo= hwMap.get(Servo.class,"skyServo");
        skyServo.setDirection(Servo.Direction.REVERSE);

        colorSense = hwMap.get(ColorSensor.class, "scd");

        // get a reference to the distance sensor that shares the same name.
        colorDist = hwMap.get(DistanceSensor.class, "scd");
        laserDist = hwMap.get(Rev2mDistanceSensor.class, "DistanceSensor");


*/
        isInit = true;
    }
}
