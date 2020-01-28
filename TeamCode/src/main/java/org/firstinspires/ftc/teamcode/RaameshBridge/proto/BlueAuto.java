package org.firstinspires.ftc.teamcode.RaameshBridge.proto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class BlueAuto extends LinearOpMode implements IAuto{
    //public String sideA="red";
    public void d(){

        if(DebugMotion.debug(this)){
            telemetry.addData("Status","Debug confirmed!");
        }
    }
    //TODO
    @Override
    public String getSide() {
        return "blue";
    }
}
