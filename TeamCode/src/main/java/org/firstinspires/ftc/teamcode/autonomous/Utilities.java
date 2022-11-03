package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.hardware.RigatoniHardware;

//Horizontal Claw Utilities
public class Utilities
{

    private RigatoniHardware hardware;
    Utilities(RigatoniHardware hardware)
    {
        this.hardware = hardware;
    }
    public void openClaw()
    {
        hardware.grabServo.setPosition(0.2);
        hardware.grabServo.setPosition(0.0);
    }
    //    public void wait(int waitTime, Telemetry telemetry)
//    {
//        ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
//        time.reset();
//        while (time.time() < waitTime)
//        {
//            telemetry.addData("Status", "Waiting");
//            telemetry.addData("Wait Time", waitTime / 1000);
//            telemetry.addData("Time Left", (waitTime - time.time()) / 1000);
//            telemetry.update();
//        }
//    }
    public void rotateClaw(int position)
    {
        hardware.rotServo.setPosition (position);
    }
    public void liftArm(String pos)
    {
        switch(pos)
        {
            case "floor":
                //hardwareMap.liftArm.
                break;
            case "low":
                break;
            case "medium":
                break;
            case "high":

        }

    }
}
