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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot.Drive;
import org.firstinspires.ftc.teamcode.Robot.Path;
import org.firstinspires.ftc.teamcode.Robot.Point.AngleType;
import org.firstinspires.ftc.teamcode.Robot.Runner;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static java.lang.Math.abs;
import static java.lang.Math.atan;
import static java.lang.Math.sin;
import static java.lang.Math.tan;
import static java.lang.Math.toRadians;

@Autonomous(name="cv skystone blue")
@Disabled
public class stoneBlue extends LinearOpMode
{
    Drive robot = new Drive(this);
    Runner runner = new Runner(robot);

    OpenCvCamera webCam;

    private ElapsedTime runtime = new ElapsedTime();

    Rect Skystone = new Rect();
    Point StonePos = new Point(0,0);

    double focalLength = 420;

    double cameraHeight = 13;
    double cameraAngle = 60;


    double distance;
    double translation;
    double rotation;

    @Override
    public void runOpMode()
    {
        robot.init(hardwareMap);

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
        //webCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

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

        scanSkystone(1);

        Path drive1 = new Path();
        drive1.addPoint(0,8);
        drive1.addPoint(translation+8, distance);
        drive1.addPoint(translation-8, distance+24, 0, AngleType.DYNAMIC, 0.35);
        drive1.addPoint(translation, 24, 180, AngleType.DYNAMIC, 0.75);
        drive1.addPoint(-72,24, 180, AngleType.DYNAMIC, 1);
        drive1.addPoint(-84, 24, 180, AngleType.DYNAMIC, 0.7);
        drive1.addPoint(-84, 42);

        runner.runPath(drive1,0.65,10);

        runner.waitForPoint(2);

        robot.intake1.setPower(-0.7);
        robot.intake2.setPower(-0.7);

        runner.waitForPoint(4);

        robot.intake1.setPower(0);
        robot.intake2.setPower(0);

        robot.foundationServo.setPosition(0.3);

        runner.waitForPoint(7);

        setArm(0.24, true);
        robot.foundationServo.setPosition(0);

        sleep(400);

        runner.stopPath();

        Path drive2 = new Path();
        drive2.addPoint(-78, 20, 0, AngleType.DYNAMIC, 1.0);
        drive2.addPoint(-60,20);
        runner.runPath(drive2, 1.0, 10);


        setArm(0.8, true);

        sleep(800);

        setArm(0.8, false);
        sleep(100);
        setArm(0.19, false);

        runner.waitForStop();

        robot.foundationServo.setPosition(0.3);

        Path drive3 = new Path();
        drive3.addPoint(-60, 24);
        drive3.addPoint(-18,24);
        drive3.addPoint(translation, 24, 55, AngleType.DIRECT, 0.65);
        robot.runPath(drive3, 0.8, 10);

        scanSkystone(0.1);

        double xTravel = robot.x() + sin(toRadians(robot.getAngle()))*distance+8;

        Path drive4 = new Path();

        if (xTravel > 28) {
            drive4.addPoint(xTravel-8, 24, 90, AngleType.DIRECT, 0.5);
            drive4.addPoint(xTravel-18, 47, 90, AngleType.DIRECT, 0.5);
            drive4.addPoint(32, 47, 90, AngleType.DIRECT, 0.5);
            drive4.addPoint(0, 24, 180, AngleType.DYNAMIC, 1.0);
            drive4.addPoint(-82, 24);
        } else {
            drive4.addPoint(xTravel, 24, 0, AngleType.DYNAMIC, 0.5);
            drive4.addPoint(xTravel, 60);
            drive4.addPoint(28, 60);
            drive4.addPoint(0, 24, 180, AngleType.DYNAMIC, 1.0);
            drive4.addPoint(-82, 24);
        }


        runner.runPath(drive4,0.5,10);

        robot.intake1.setPower(-0.7);
        robot.intake2.setPower(-0.7);

        runner.waitForPoint(4);

        robot.intake1.setPower(0);
        robot.intake2.setPower(0);

        sleep(400);

        setArm(0.24, false);

        sleep(400);

        setArm(0.24, true);
        sleep(500);
        setArm(0.8, true);

        sleep(800);

        setArm(0.8, false);
        sleep(200);
        setArm(0.24, false);
        sleep(200);

        runner.stopPath();

        Path drive5 = new Path();
        drive5.addPoint(-40, 24, 0, AngleType.DYNAMIC, 0.5);
        robot.runPath(drive5, 0.5, 15);

    }

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
            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new  Size(5,5));

            Mat yellowFilter = new Mat();
            Imgproc.blur(input, yellowFilter, new Size(10,10));
            Imgproc.cvtColor(yellowFilter, yellowFilter, Imgproc.COLOR_BGR2HSV);
            Scalar minYellow = new Scalar(93, 60, 80);
            Scalar maxYellow = new Scalar(117, 255, 255);
            Core.inRange(yellowFilter, minYellow, maxYellow, yellowFilter);
            Imgproc.dilate(yellowFilter, yellowFilter, kernel);
            Imgproc.dilate(yellowFilter, yellowFilter, kernel);
            Imgproc.dilate(yellowFilter, yellowFilter, kernel);
            Imgproc.dilate(yellowFilter, yellowFilter, kernel);
            Imgproc.dilate(yellowFilter, yellowFilter, kernel);
            Imgproc.dilate(yellowFilter, yellowFilter, kernel);
            Imgproc.dilate(yellowFilter, yellowFilter, kernel);
            Imgproc.dilate(yellowFilter, yellowFilter, kernel);
            Imgproc.dilate(yellowFilter, yellowFilter, kernel);
            Imgproc.dilate(yellowFilter, yellowFilter, kernel);
            Imgproc.erode(yellowFilter, yellowFilter, kernel);
            Imgproc.erode(yellowFilter, yellowFilter, kernel);
            Imgproc.erode(yellowFilter, yellowFilter, kernel);
            Imgproc.erode(yellowFilter, yellowFilter, kernel);
            Imgproc.erode(yellowFilter, yellowFilter, kernel);
            Imgproc.erode(yellowFilter, yellowFilter, kernel);
            Imgproc.erode(yellowFilter, yellowFilter, kernel);
            Imgproc.erode(yellowFilter, yellowFilter, kernel);
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
            Scalar maxBlack = new Scalar(255, 255, 35);
            Core.inRange(blackFilter, minBlack, maxBlack, blackFilter);
            Imgproc.dilate(blackFilter, blackFilter, kernel);
            Imgproc.dilate(blackFilter, blackFilter, kernel);
            Imgproc.dilate(blackFilter, blackFilter, kernel);
            Imgproc.dilate(blackFilter, blackFilter, kernel);
            Imgproc.dilate(blackFilter, blackFilter, kernel);
            Imgproc.dilate(blackFilter, blackFilter, kernel);
            Imgproc.dilate(blackFilter, blackFilter, kernel);
            Imgproc.erode(blackFilter, blackFilter, kernel);
            Imgproc.erode(blackFilter, blackFilter, kernel);
            Imgproc.erode(blackFilter, blackFilter, kernel);
            Imgproc.erode(blackFilter, blackFilter, kernel);
            Imgproc.erode(blackFilter, blackFilter, kernel);
            Imgproc.erode(blackFilter, blackFilter, kernel);
            List<MatOfPoint> contoursBlack = new ArrayList<>();
            Mat hierarchyBlack = new Mat();
            Imgproc.findContours(blackFilter, contoursBlack, hierarchyBlack, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);

            List<MatOfPoint> hullList = new ArrayList<>();
            for (MatOfPoint contour : contoursBlack) {
                MatOfInt hull = new MatOfInt();
                Imgproc.convexHull(contour, hull);
                Point[] contourArray = contour.toArray();
                Point[] hullPoints = new Point[hull.rows()];
                List<Integer> hullContourIdxList = hull.toList();
                for (int i = 0; i < hullContourIdxList.size(); i++) {
                    hullPoints[i] = contourArray[hullContourIdxList.get(i)];
                }
                hullList.add(new MatOfPoint(hullPoints));
            }



            if (hierarchyBlack.size().height > 0 && hierarchyBlack.size().width > 0)
            {
                // for each contour, display it in blue
                for (int idx = 0; idx >= 0; idx = (int) hierarchyBlack.get(0, idx)[0])
                {
                    if (Imgproc.contourArea(contoursBlack.get(idx)) > 400) {
                        Imgproc.drawContours(input, hullList, idx, new Scalar(255, 0, 0));
                    }
                }
            }


            double area = 0;
            List<Rect> Skystones = new ArrayList<>();
            List<Rect> Stones = new ArrayList<>();

            for (MatOfPoint contour: contoursYellow) {
                if (Imgproc.contourArea(contour) > 400) {
                    Rect rect = Imgproc.boundingRect(contour);
                    Stones.add(rect);
                    //Imgproc.rectangle(input, rect, new Scalar(0,255,0), 1);
                }
            }

            for (MatOfPoint contour: contoursBlack) {
                if (Imgproc.contourArea(contour) > 400) {
                    //RotatedRect rotatedRect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
                    Rect rect = Imgproc.boundingRect(contour);
                    for(Rect Stone : Stones) {
                        if (new Point(rect.x + rect.width/2, rect.y + rect.height/2).inside(Stone)) {
                            //drawRotatedRect(input, rotatedRect, new Scalar(255, 0, 0), 2);
                            Imgproc.rectangle(input, rect, new Scalar(255,0,0), 2);
                            if (rect.area() > area) {
                                Skystone = rect;
                                StonePos = new Point(rect.x + rect.width / 2, rect.y + rect.height / 2);
                                area = rect.area();
                            }
                        }
                    }
                }
            }

/*

            Mat edges = new Mat();
            Mat Lines = new Mat();

            Imgproc.cvtColor(input, edges, Imgproc.COLOR_BGR2GRAY);
            Imgproc.blur(edges, edges, new Size(4,4));
            Imgproc.Canny(edges, edges, 30, 20*3);

            Imgproc.HoughLines(edges, Lines, 2, PI/180, 150);
            for (int x = 0; x < Lines.rows(); x++) {
                double rho = Lines.get(x, 0)[0],
                        theta = Lines.get(x, 0)[1];
                double a = cos(theta), b = sin(theta);
                double x0 = a*rho, y0 = b*rho;
                Point pt1 = new Point(round(x0 + 1000*(-b)), round(y0 + 1000*(a)));
                Point pt2 = new Point(round(x0 - 1000*(-b)), round(y0 - 1000*(a)));
                Imgproc.line(input, pt1, pt2, new Scalar(0, 0, 255), 3, Imgproc.LINE_AA, 0);
            }
*/
            return input;
        }
    }
    public void setArm(double position, boolean grip) {
        robot.leftBase.setPosition(-(abs(position-0.24)+0.24) + 1);
        robot.rightBase.setPosition((abs(position-0.24)+0.24));
        robot.leftStabilization.setPosition((abs(position-0.24)+0.24)*1.13 - 0.07);
        robot.rightStabilization.setPosition(-(abs(position-0.24)+0.24)*1.13 + 1.07);
        if (grip) {
            robot.leftGrip.setPosition(0.55);
            robot.rightGrip.setPosition(0.45);
        }
        else {
            robot.leftGrip.setPosition(0.87);
            robot.rightGrip.setPosition(0.07);
        }
    }

    public void scanSkystone(double time) {

        runtime.reset();

        while (opModeIsActive()) {
            /*
             * Send some stats to the telemetry
             */

            distance = tan(atan((120 - Skystone.x - Skystone.width) / focalLength) + toRadians(cameraAngle)) * cameraHeight;
            translation = ((160 - StonePos.y) / focalLength) * distance * 1.1 + 1;


            telemetry.addData("Stone Position X", StonePos.x);
            telemetry.addData("Stone Position Y", StonePos.y);
            telemetry.addData("distance", distance);
            telemetry.addData("distancereal", distance);
            telemetry.addData("translation", translation);
            telemetry.addData("rotation", rotation);
            telemetry.addData("width", Skystone.height);
            telemetry.update();


            if (runtime.seconds() > time) {
                break;
            }


        }
    }
}
