package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.NewRigatoniHardware;



public class NewUtilities
{

    private NewRigatoniHardware hardware;
    SampleMecanumDrive drive;



    public NewUtilities(NewRigatoniHardware hardware)
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
        hardware.liftArm1.setPower(power*0.95);
        hardware.liftArm2.setPower(power*0.95);

        wait(time, telemetry);

        hardware.liftArm1.setPower(0);
        hardware.liftArm2.setPower(0);

        wait(500, telemetry);
    }



    public void liftArmPosition(int pos)
    {
        hardware.liftArm1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        hardware.liftArm1.setTargetPosition(hardware.liftArm1.getCurrentPosition() + pos);
        hardware.liftArm1.setTargetPositionTolerance(10);
        hardware.liftArm1.setPower(0.95);
        hardware.liftArm1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        hardware.liftArm2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        hardware.liftArm2.setTargetPosition(hardware.liftArm2.getCurrentPosition() + pos);
        hardware.liftArm2.setTargetPositionTolerance(10);
        hardware.liftArm2.setPower(0.95);
        hardware.liftArm2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }



    public void lowerArm(double power, int time, Telemetry telemetry)
    {
        hardware.liftArm1.setPower(-power*0.95);
        hardware.liftArm2.setPower(-power*0.95);

        wait(time, telemetry);

        hardware.liftArm1.setPower(0);
        hardware.liftArm2.setPower(0);

    }
}
