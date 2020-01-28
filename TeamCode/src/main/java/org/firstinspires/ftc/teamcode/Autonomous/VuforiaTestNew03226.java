package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.AutoSide.RedAuto;
import org.firstinspires.ftc.teamcode.Robot.Drive;
import org.firstinspires.ftc.teamcode.Robot.Path;
import org.firstinspires.ftc.teamcode.Robot.Runner;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;


@Autonomous(name="toast")
@Disabled
public class VuforiaTestNew03226 extends RedAuto{

    Drive robot = new Drive(this);

    ElapsedTime runtime = new ElapsedTime();

    Runner runner = new Runner(robot);

    boolean notFound;

    double stoneX;
    double stonePos;

    OpenGLMatrix lastLocation = null;

    VuforiaLocalizer vuforia;

    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AahhGez/////AAABmSIunC5QeUbDmqSuN/JVvK5jcN03J60dszlHmZqGkfuUkUVQsH4/YxQk1A+Rg/DlMP1LwU5U/g4EW5+j6271bFI0ZWGQScblmtv/MVEAgzJqZdZkHl/ZS8qgIDkPDzKJLCJwz9qtN6fKZJ5FC0uxuQ9vHblm5dyr0iIAUM78c6Wc2fzqDmTm3WuOFbBthCKJRJNpfhUn9CyA1Wdev+LzIbotSs++L3a9O4ekv1/NKi5Khujw3L2i1IxY8M2hZlTYfeCn57W4bCxnfSWChH0yNv7G5gk3jyMP5d8pR1XtqDu4kqx+dQ2/Fq+M9Fdun8nfE5peCIVk592Ul+sjhC5Nr8FQZIFfxoAJIAWu3o4QizOb\n";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");

        targetsSkyStone.activate();
        waitForStart();

        while(!isStopRequested()) {
            if (((VuforiaTrackableDefaultListener)stoneTarget.getListener()).isVisible()) {
                telemetry.addData("Visible Target", stoneTarget.getName());

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)stoneTarget.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    stoneX = -robotLocationTransform.getTranslation().get(1);
                }
                telemetry.addData("Stone Position", stoneX);
            }
            telemetry.update();
        }

        if (notFound) {
            stonePos = 1;
            stoneX = -12;
        } else if (stoneX > 0) {
            stonePos = 2;
            stoneX = -4;
        } else if (stoneX < 0) {
            stonePos = 3;
            stoneX = 4;
        }

        Path drive2 = new Path();
        drive2.addPoint(stoneX + 8, 22, -45);
        drive2.addPoint(stoneX, 40, -45);

        runner.setPath(drive2, 0.5, 1);
        runner.start();

        while (runner.currentPoint() == 0) {
            idle();
        }

        sleep(500);

        robot.intake1.setPower(0.5);
        robot.intake2.setPower(-0.5);

        while (runner.isAlive()) {
            sleep(10);
            idle();
        }

        robot.intake1.setPower(0);
        robot.intake2.setPower(0);

        Path drive3 = new Path();
        drive3.addPoint(12, 24, -90);
        drive3.addPoint(60, 24, -90);

    }
}
