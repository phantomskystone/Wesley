package org.firstinspires.ftc.teamcode.RaameshBridge.proto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class RedAuto extends LinearOpMode implements IAuto{
    public void d(){

        if(DebugMotion.debug(this)){
           telemetry.addData("Status","Debug confirmed!");
        }
    }

    //public String sideA="red";
    //Right now these are useless, but more may be required in the future.
    //TODO
    @Override
    public String getSide() {
        return "red";
    }
}
