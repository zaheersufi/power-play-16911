package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.RigatoniHardware;

//Horizontal Claw Utilities
public class Utilities
{

    private RigatoniHardware hardware;
    Utilities(RigatoniHardware hardware)
    {
        this.hardware = hardware;
    }
    public void openClaw(boolean shouldOpen)
    {
        if(!shouldOpen)
            hardware.grabServo.setPosition(1.0);
        else
            hardware.grabServo.setPosition(0.0);
    }
    public void wait(int waitTime, Telemetry telemetry)
    {
        ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        time.reset();
        while (time.time() < waitTime)
        {
            telemetry.addData("Status", "Waiting");
            telemetry.addData("Wait Time", waitTime / 1000);
            telemetry.addData("Time Left", (waitTime - time.time()) / 1000);
            telemetry.update();
        }
    }
    public void rotateClaw(int position)
    {
//        hardware.rotServo.setPosition(position);
    }

    public void dropCone (double power, int time, Telemetry telemetry)
    {
        hardware.liftArm.setPower(power);
        wait(time, telemetry);
        hardware.liftArm.setPower(0);
        openClaw(true);
        hardware.liftArm.setPower(-power);
        wait(time, telemetry);
        hardware.liftArm.setPower(0);
    }

    public void groundJunction (Telemetry telemetry)
    {
        dropCone(.4,100, telemetry);
    }

    public void lowJunction (Telemetry telemetry)
    {
        dropCone(.8,400, telemetry);
    }

    public void midJunction (Telemetry telemetry)
    {
        dropCone(.8,550, telemetry);
    }

    public void highJunction (Telemetry telemetry)
    {
        dropCone(.8, 12000, telemetry);
    }

//        public void dropCone ()
//    {
//        hardware.grabServo.setPosition(.66);
//        hardware.grabServo.setPosition(.33);
//    }
//    public void lowJunction (Telemetry telemetry)
//    {
//        hardware.liftArm.setPower(.8);
//        wait(4, telemetry);
//        hardware.liftArm.setPower(0);
//        hardware.grabServo.setPosition(.66);
//        hardware.grabServo.setPosition(.33);
//        hardware.liftArm.setPower(-.8);
//        wait(4, telemetry);
//        hardware.liftArm.setPower(0);
//    }
//    public void midJunction (Telemetry telemetry)
//    {
//        hardware.liftArm.setPower(.8);
//        wait(7, telemetry);
//        hardware.liftArm.setPower(0);
//        hardware.grabServo.setPosition(.66);
//        hardware.grabServo.setPosition(.33);
//        hardware.liftArm.setPower(-.8);
//        wait(7, telemetry);
//        hardware.liftArm.setPower(0);
//    }
//    public void highJunction (Telemetry telemetry)
//    {
//        hardware.liftArm.setPower(.8);
//        wait(10, telemetry);
//        hardware.liftArm.setPower(0);
//        hardware.grabServo.setPosition(.66);
//        hardware.grabServo.setPosition(.33);
//        hardware.liftArm.setPower(-.8);
//        wait(10, telemetry);
//        hardware.liftArm.setPower(0);
//    }
}
