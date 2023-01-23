package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.OldRigatoniHardware;

public class Utilities
{

    private OldRigatoniHardware hardware;
    SampleMecanumDrive drive;



    public Utilities(OldRigatoniHardware hardware)
    {
        this.hardware = hardware;
    }


    /**
     * Method that can be used to wait for a specific action, in our case used
     * to create 0.2 seconds of time for the camera to scan the signal sleeve.
     *
     * @param   waitTime    Amount of time to wait
     * @param   telemetry   Telemetry to show data/updates
     */
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


    /**
     * Open and closes claw to predetermined positions based on the boolean
     * input of a button on the gamepad.
     *
     * @param   shouldOpen  True -> opens, False -> closes
     */
    public void openClaw(boolean shouldOpen)
    {
        if(!shouldOpen)
            hardware.grabServo.setPosition(1.0);
        else
            hardware.grabServo.setPosition(0.0);
    }


    /**
     * Method that uses encoders to move the arm to a certain height/position,
     * with a set power of 1 (max).
     *
     * @param   pos         Target height/position
     */
    public void liftArmPosition(int pos)
    {
        hardware.liftArm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        hardware.liftArm.setTargetPosition(hardware.liftArm.getCurrentPosition() + pos);
        hardware.liftArm.setTargetPositionTolerance(10);
        hardware.liftArm.setPower(1);
        hardware.liftArm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }


    /**
     *  Method that lifts the arm based on a power for a certain amount of
     *  time that is given.
     *  We stopped using it, we now use position instead.
     *
     * @param   power       Lift power [0,1]
     * @param   time        Time to lift
     * @param   telemetry   Telemetry to show data/updates
     */
    public void liftArm(double power, int time, Telemetry telemetry)
    {
        hardware.liftArm.setPower(power);
        wait(time, telemetry);
        hardware.liftArm.setPower(0);
        wait(1000, telemetry);
    }


    /**
     *  Method that lowers the arm based on a given time using a set power.
     *  We stopped using it, we now use position instead.
     *
     * @param   power       Lift power [0,1]
     * @param   time        Time to lift
     * @param   telemetry   Telemetry to show data/updates
     */
    public void lowerArm(double power, int time, Telemetry telemetry)
    {
        hardware.liftArm.setPower(-power);
        wait(time, telemetry);
        hardware.liftArm.setPower(0);
    }
}
