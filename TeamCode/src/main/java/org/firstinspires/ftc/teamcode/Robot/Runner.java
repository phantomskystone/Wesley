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

    public void runPath(Path path, double power, double drift) {
        this.path = path;
        this.power = power;
        this.drift = drift;
        this.start();
    }

    public void waitForPoint(int point) {
        while(currentPoint() < point) {
            robot.opMode.idle();
            if(!this.isAlive()) {
                break;
            }
        }
    }

    public void waitForStop() {
        while(this.isAlive()) {
            robot.opMode.idle();
        }
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
