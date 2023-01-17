package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.RigatoniHardware;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

//Horizontal Claw Utilities
public class Utilities
{

    private RigatoniHardware hardware;
    SampleMecanumDrive drive;
    public Utilities(RigatoniHardware hardware)
    {
        this.hardware = hardware;
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
    public void openClaw(boolean shouldOpen)
    {
        if(!shouldOpen)
            hardware.grabServo.setPosition(1.0);
        else
            hardware.grabServo.setPosition(0.0);
    }
    public void liftArm(double power, int time, Telemetry telemetry)
    {
        hardware.liftArm.setPower(power);
        wait(time, telemetry);
        hardware.liftArm.setPower(0);
        wait(1000, telemetry);
    }
    public void lowerArm(double power, int time, Telemetry telemetry)
    {
        hardware.liftArm.setPower(-power);
        wait(time, telemetry);
        hardware.liftArm.setPower(0);
    }
}
