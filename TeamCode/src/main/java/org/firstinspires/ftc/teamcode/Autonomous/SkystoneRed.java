/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot.Drive;
import org.firstinspires.ftc.teamcode.Robot.Path;
import org.firstinspires.ftc.teamcode.Robot.Runner;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous(name="Skystone Red")
public class SkystoneRed extends LinearOpMode
{
    Drive robot = new Drive(this);
    Runner runner = new Runner(robot);

    OpenCvCamera webCam;

    private ElapsedTime runtime = new ElapsedTime();

    Point StonePos = new Point(0,0);
    enum SkyStone {
        LEFT,
        CENTER,
        RIGHT;
    }
    SkyStone SkyStone;

    @Override
    public void runOpMode()
    {
        robot.init(hardwareMap);
        SkyStone = SkyStone.CENTER;

        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using the phone's internal camera. We pass it a
         * CameraDirection enum indicating whether to use the front or back facing
         * camera, as well as the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //webCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

        /*
         * Open the connection to the camera device
         */
        webCam.openCameraDevice();

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        webCam.setPipeline(new SamplePipeline());

        /*
         * Tell the camera to start streaming images to us! Note that you must make sure
         * the resolution you specify is supported by the camera. If it is not, an exception
         * will be thrown.
         *
         * Also, we specify the rotation that the camera is used in. This is so that the image
         * from the camera sensor can be rotated such that it is always displayed with the image upright.
         * For a front facing camera, rotation is defined assuming the user is looking at the screen.
         * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
         * away from the user.
         */
        webCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);

        /*
         * Wait for the user to press start on the Driver Station
         */



        waitForStart();
        robot.motorsOn();
        robot.startPositionTracking();


        setArm(0.19, false);

        runtime.reset();

        while (opModeIsActive())
        {
            /*
             * Send some stats to the telemetry
             */

            if (StonePos.y > 180) {
                SkyStone = SkyStone.LEFT;
            }
            if (StonePos.y < 180 && StonePos.y > 130) {
                SkyStone = SkyStone.CENTER;
            }
            if (StonePos.y < 130) {
                SkyStone = SkyStone.RIGHT;
            }

            telemetry.addData("Stone Position X", StonePos.x);
            telemetry.addData("Stone Position Y", StonePos.y);
            telemetry.addData("Position", SkyStone);
            telemetry.update();

            /*
             * NOTE: stopping the stream from the camera early (before the end of the OpMode
             * when it will be automatically stopped for you) *IS* supported. The "if" statement
             * below will stop streaming from the camera when the "A" button on gamepad 1 is pressed.
             */
            if(runtime.seconds() > 3)
            {
                break;
            }

            /*
             * For the purposes of this sample, throttle ourselves to 10Hz loop to avoid burning
             * excess CPU cycles for no reason. (By default, telemetry is only sent to the DS at 4Hz
             * anyway). Of course in a real OpMode you will likely not want to do this.
             */
            sleep(100);


        }

        Path drive1 = new Path();
        drive1.addPoint(0,8);
        if (SkyStone == SkyStone.LEFT) {
            drive1.addPoint(10,26, -45);
            drive1.addPoint(6, 38, -45);
            drive1.addPoint(-2, 54, -45);
        }
        if (SkyStone == SkyStone.CENTER) {
            drive1.addPoint(18,26, -45);
            drive1.addPoint(14, 38, -45);
            drive1.addPoint(6, 54, -45);

        }
        if (SkyStone == SkyStone.RIGHT) {
            drive1.addPoint(24,26, -45);
            drive1.addPoint(22, 34, -45);
            drive1.addPoint(14, 54, -45);

        }

        drive1.addPoint(14, 26, -90);
        drive1.addPoint(72, 26, -90);
        drive1.addPoint(90, 30, -180);
        drive1.addPoint(90, 42, -180);

        runner.setPath(drive1, 1, 1);
        runner.start();
        while(runner.currentPoint() < 2 && opModeIsActive()) {
            idle();
        }
        robot.intake1.setPower(-0.7);
        robot.intake2.setPower(-0.7);
        while(runner.currentPoint() < 5 && opModeIsActive()) {
            idle();
        }
        robot.intake1.setPower(0);
        robot.intake2.setPower(0);

        setArm(0.24, true);

        sleep(500);

        setArm(0.24, false);

        while(runner.currentPoint() < 6 && opModeIsActive()) {
            idle();
        }

        robot.foundationServo.setPosition(0.3);

        while(runner.currentPoint() < 7 && opModeIsActive()) {
            idle();
        }

        setArm(0.24, true);
        sleep(500);
        setArm(0.8, true);

        robot.foundationServo.setPosition(0);

        sleep(1000);
        setArm(0.8, false);
        sleep(200);
        setArm(0.24, false);
        sleep(500);

        runner.stopPath();
/*
        Path drive2 = new Path();
        drive2.addPoint(82, 12);
        robot.runPath(drive2, 0.55, 1);
        robot.turnTo(-90);
        runtime.reset();
        while (runtime.seconds() < 1 && !isStopRequested()) {
            robot.pointDrive(96, 12, 0.5);
        }

 */

        while(robot.y() > 12 && opModeIsActive()) {
            robot.frontLeft.setPower(0.7);
            robot.frontRight.setPower(0.5);
            robot.backLeft.setPower(0.7);
            robot.backRight.setPower(0.5);
        }

        while(robot.getAngle() < -90 && opModeIsActive()) {
            robot.frontLeft.setPower(0.7);
            robot.frontRight.setPower(-0.3);
            robot.backLeft.setPower(0.7);
            robot.backRight.setPower(-0.3);
        }
        runtime.reset();
        while(runtime.seconds() < 0.5 && opModeIsActive()) {
            robot.frontLeft.setPower(-0.5);
            robot.frontRight.setPower(-0.5);
            robot.backLeft.setPower(-0.5);
            robot.backRight.setPower(-0.5);
        }
        robot.stop();
        robot.foundationServo.setPosition(0.3);
        sleep(500);

        robot.setAngle(-90);

        Path drive3 = new Path();
        drive3.addPoint(72, 30);
        drive3.addPoint(40, 28);
        robot.runPath(drive3, 0.55, 1);
    }

    /*
     * An example image processing pipeline to be run upon receipt of each frame from the camera.
     * Note that the processFrame() method is called serially from the frame worker thread -
     * that is, a new camera frame will not come in while you're still processing a previous one.
     * In other words, the processFrame() method will never be called multiple times simultaneously.
     *
     * However, the rendering of your processed image to the viewport is done in parallel to the
     * frame worker thread. That is, the amount of time it takes to render the image to the
     * viewport does NOT impact the amount of frames per second that your pipeline can process.
     *
     * IMPORTANT NOTE: this pipeline is NOT invoked on your OpMode thread. It is invoked on the
     * frame worker thread. This should not be a problem in the vast majority of cases. However,
     * if you're doing something weird where you do need it synchronized with your OpMode thread,
     * then you will need to account for that accordingly.
     */
    class SamplePipeline extends OpenCvPipeline
    {
        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */

        private void drawRotatedRect(Mat image, RotatedRect rotatedRect, Scalar color, int thickness)
        {
            Point[] vertices = new Point[4];
            rotatedRect.points(vertices);
            MatOfPoint points = new MatOfPoint(vertices);
            Imgproc.drawContours(image, Arrays.asList(points), -1, color, thickness);
        }

        @Override
        public Mat processFrame(Mat input)
        {
            /*
             * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
             * will only dereference to the same image for the duration of this particular
             * invocation of this method. That is, if for some reason you'd like to save a copy
             * of this particular frame for later use, you will need to either clone it or copy
             * it to another Mat.
             */

            /*
             * Draw a simple box around the middle 1/2 of the entire frame
             */
            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new  Size(20,20));
            Mat kernel2 = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new  Size(5,5));

            Mat yellowFilter = new Mat();
            Imgproc.blur(input, yellowFilter, new Size(10,10));
            Imgproc.cvtColor(yellowFilter, yellowFilter, Imgproc.COLOR_BGR2HSV);
            Scalar minYellow = new Scalar(90, 140, 100);
            Scalar maxYellow = new Scalar(125, 255, 255);
            Core.inRange(yellowFilter, minYellow, maxYellow, yellowFilter);
            List<MatOfPoint> contoursYellow = new ArrayList<>();
            Mat hierarchyYellow = new Mat();
            Imgproc.findContours(yellowFilter, contoursYellow, hierarchyYellow, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);

            if (hierarchyYellow.size().height > 0 && hierarchyYellow.size().width > 0)
            {
                // for each contour, display it in blue
                for (int idx = 0; idx >= 0; idx = (int) hierarchyYellow.get(0, idx)[0])
                {
                    if (Imgproc.contourArea(contoursYellow.get(idx)) > 400) {
                        Imgproc.drawContours(input, contoursYellow, idx, new Scalar(0, 255, 0));
                    }
                }
            }

            Mat blackFilter = new Mat();
            Imgproc.medianBlur(input, blackFilter, 15);
            Imgproc.cvtColor(blackFilter, blackFilter, Imgproc.COLOR_BGR2HSV);
            Scalar minBlack = new Scalar(0, 0, 0);
            Scalar maxBlack = new Scalar(255, 255, 25);
            Core.inRange(blackFilter, minBlack, maxBlack, blackFilter);
            Imgproc.dilate(blackFilter, blackFilter, kernel);
            Imgproc.erode(blackFilter, blackFilter, kernel2);
            List<MatOfPoint> contoursBlack = new ArrayList<>();
            Mat hierarchyBlack = new Mat();
            Imgproc.findContours(blackFilter, contoursBlack, hierarchyBlack, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);

            if (hierarchyBlack.size().height > 0 && hierarchyBlack.size().width > 0)
            {
                // for each contour, display it in blue
                for (int idx = 0; idx >= 0; idx = (int) hierarchyBlack.get(0, idx)[0])
                {
                    if (Imgproc.contourArea(contoursBlack.get(idx)) > 400) {
                        Imgproc.drawContours(input, contoursBlack, idx, new Scalar(255, 0, 0));
                    }
                }
            }


            double area = 0;
            List<Rect> Stones = new ArrayList<>();

            for (MatOfPoint contour: contoursYellow) {
                if (Imgproc.contourArea(contour) > 400) {
                    Rect rect = Imgproc.boundingRect(contour);
                    if (rect.area() > area) {
                        area = rect.area();
                        Stones.add(rect);
                    }
                    Imgproc.rectangle(input, rect, new Scalar(0,255,0), 2);
                }
            }

            for (MatOfPoint contour: contoursBlack) {
                if (Imgproc.contourArea(contour) > 400) {
                    RotatedRect rotatedRect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
                    for(Rect Stone : Stones) {
                        if (rotatedRect.center.inside(Stone) || true) {
                            drawRotatedRect(input, rotatedRect, new Scalar(255, 0, 0), 2);
                            StonePos = rotatedRect.center;
                        }
                    }
                }
            }

            /*Imgproc.rectangle(
                    input,
                    new Point(
                            input.cols()/4,
                            input.rows()/4),
                    new Point(
                            input.cols()*(3f/4f),
                            input.rows()*(3f/4f)),
                    new Scalar(0, 255, 0), 4);

            /**
             * NOTE: to see how to get data from your pipeline to your OpMode as well as how
             * to change which stage of the pipeline is rendered to the viewport when it is
             * tapped, please see {@link PipelineStageSwitchingExample}
             */

            return input;
        }
    }
    public void setArm(double position, boolean grip) {
        robot.leftBase.setPosition(-(Math.abs(position-0.24)+0.24) + 1);
        robot.rightBase.setPosition((Math.abs(position-0.24)+0.24));
        robot.leftStabilization.setPosition((Math.abs(position-0.24)+0.24)*1.13 - 0.07);
        robot.rightStabilization.setPosition(-(Math.abs(position-0.24)+0.24)*1.13 + 1.07);
        if (grip) {
            robot.leftGrip.setPosition(0.55);
            robot.rightGrip.setPosition(0.45);
        }
        else {
            robot.leftGrip.setPosition(0.87);
            robot.rightGrip.setPosition(0.07);
        }
    }
}
