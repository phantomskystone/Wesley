package org.firstinspires.ftc.teamcode.RaameshBridge;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RaameshBridge.PhantomConfig4200X;
import org.firstinspires.ftc.teamcode.RaameshBridge.proto.RedAuto;

//Commented by Raamesh Balabharapatruni

@SuppressWarnings("MagicConstant")
@Autonomous(name="RED FoundationSide (Ends Away From Wall)",group="autonomous")
public final class BridgeIMU2XRaameshPosB extends RedAuto {

    PhantomConfig4200X robot = new PhantomConfig4200X();

    IMURedux MecanumInstance = new IMURedux(this,robot,0.025f);




    @Override
    public void runOpMode() {

        super.d();

        robot.startShip(hardwareMap);
        //robot.doIMU();
        //robot.reverse();
        //robot.doNewC();
        MecanumInstance.calibrate();
        //robot.skyServo.setPosition(0);
        robot.armServo.setPosition(1);
        waitForStart();
        robot.armServo.setPosition(0.7);
        MecanumInstance.imuDrive(0,1/3.333333333333333*10/7,0.7);
        MecanumInstance.imuDrive(-90,(0.6*10/7)+0.3,0.7);
        MecanumInstance.imuDrive(0,(0.9*10/7)-0.15,0.7);
        MecanumInstance.imuDrive(0,0.01,1*0.7);
        //sleep(3000);
        telemetry.addData("STATUS:", "PAUSING");
        telemetry.update();
        robot.armServo.setPosition(1);
        sleep(3000);
        MecanumInstance.imuDrive(0,(1.225*10/7)+0.1,-1*0.7);
        sleep(500);

        MecanumInstance.imuTurn(-90,1.8);

        robot.armServo.setPosition(0.7);
        sleep(3000);

        MecanumInstance.imuDrive(0,0.245*10/7,1*0.7);

        MecanumInstance.imuDrive(180,0.4*10/7,1*0.7);

        MecanumInstance.imuDrive(90,((0.625*7/10)*10/7)+0.875,0.7);

        //   MecanumInstance.imuDrive(90,3,0.5f);
        MecanumInstance.imuDrive(180,(1.6*10/7*2/3)-0.32,0.7);

    }
}
