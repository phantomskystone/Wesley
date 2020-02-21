package org.firstinspires.ftc.teamcode.RaameshBridge;

/**
 * Originally written by Raamesh on 17 Jan. 2020
 * Ian it ported to Wesley's SDK on 17 Feb. 2020
 */


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;




@Autonomous(name="Just Parking")
public class JustPark extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {


        PhantomConfig4200X robot = new PhantomConfig4200X();

        IMURedux MecanumInstance = new IMURedux(this,robot,0.025f);

        robot.startShip(hardwareMap);

        MecanumInstance.calibrate();

        waitForStart();



        sleep(20000);

        MecanumInstance.imuDrive(180,1f,0.4f);
    }
}