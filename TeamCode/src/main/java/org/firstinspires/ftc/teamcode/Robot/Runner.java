package org.firstinspires.ftc.teamcode.Robot;

public class Runner extends Thread {

    Drive robot;
    Path path;
    double power;
    double drift;

    public Runner(Drive drive) {
        robot = drive;
    }

    public void setPath(Path path, double power, double drift) {
        this.path = path;
        this.power = power;
        this.drift = drift;
    }

    public int currentPoint() {
        return robot.currentPoint;
    }

    public void stopPath() {
        this.interrupt();
    }

    @Override
    public void run() {
        try {
            robot.runPath(path, power, drift);
        } catch (Exception e) {
            robot.opMode.telemetry.addData("exception thrown", e);
        }
    }
}
