package org.firstinspires.ftc.teamcode.RunnableProgramsAndMore;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RaameshBridge.IMURedux;
import org.firstinspires.ftc.teamcode.RaameshBridge.PhantomConfig4200X;
import org.firstinspires.ftc.teamcode.RaameshBridge.proto.BlueAuto;


@Autonomous(name="BlueFoundationSide (Ends Away From Wall)",group="autonomous")
public final class BridgeIMUBlue2XRaameshPosB extends BlueAuto {

    PhantomConfig4200X robot = new PhantomConfig4200X();

    IMURedux MecanumInstance = new IMURedux(this,robot,0.025f);
    @Override
    public void runOpMode() throws InterruptedException {

        super.d();

        //Init Robot

        robot.startShip(hardwareMap);

        // Calibrate Imu

        MecanumInstance.calibrate();

        // Make sure the foundation servo is in the right position before start.

        robot.armServo.setPosition(1);

        //Wait until Start is pressed

        waitForStart();

        //Raise Foundation arm

        robot.armServo.setPosition(0.7);

        // Drive Forward to clear the wall

        MecanumInstance.imuDrive(0,0.2f,0.3f);

        // Strafe to align to the foundation

        MecanumInstance.imuDrive(-90,1.2f,0.5f);

        // Move to foundation

        MecanumInstance.imuDrive(0,1.8f,0.5f);
        MecanumInstance.imuDrive(0,0.533333333f,0.3f);

        //Grab the foundation

        robot.armServo.setPosition(1);
        sleep(500);

        //Back up

        MecanumInstance.imuDrive(0,1.4f,-0.5f);
        sleep(500);

        //Turn the foundation and the robot

        MecanumInstance.imuTurn(90,1.8);

        sleep(3000);

        //Let go of the foundation

        robot.armServo.setPosition(0.7);
        sleep(500);

        //Push the foundation forwards

        MecanumInstance.imuDrive(0,0.35f,0.7f);

        //Back up

        MecanumInstance.imuDrive(180,1f,0.4f);

        sleep(1000);

        //Get under the alliance bridge

        MecanumInstance.imuDrive(180,2f,0.4f);

    }
}
