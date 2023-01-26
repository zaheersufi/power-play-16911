package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.NewRigatoniHardware;



public class NewUtilities
{

    private NewRigatoniHardware hardware;

    private double RAISE_POWER = 0.95;
    private double LOWER_POWER = 0.3;



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
            hardware.grabServo.setPosition(0.6);
        else
            hardware.grabServo.setPosition(0.3);
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



    public void liftArmDisplacementPosition(int pos)
    {
        double power = RAISE_POWER;
        if (pos < 0) power = LOWER_POWER;

        hardware.liftArm1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        hardware.liftArm1.setTargetPosition(hardware.liftArm1.getCurrentPosition() + pos);
        hardware.liftArm1.setTargetPositionTolerance(10);
        hardware.liftArm1.setPower(power);
        hardware.liftArm1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        hardware.liftArm2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        hardware.liftArm2.setTargetPosition(hardware.liftArm2.getCurrentPosition() + pos);
        hardware.liftArm2.setTargetPositionTolerance(10);
        hardware.liftArm2.setPower(power);
        hardware.liftArm2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    public void liftArmAbsolutePosition (int pos)
    {
        double power = RAISE_POWER;
        if (pos < hardware.liftArm1.getCurrentPosition()) power = LOWER_POWER;

        hardware.liftArm1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        hardware.liftArm1.setTargetPosition(pos);
        hardware.liftArm1.setTargetPositionTolerance(10);
        hardware.liftArm1.setPower(power);
        hardware.liftArm1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        hardware.liftArm2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        hardware.liftArm2.setTargetPosition(pos);
        hardware.liftArm2.setTargetPositionTolerance(10);
        hardware.liftArm2.setPower(power);
        hardware.liftArm2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }


    public void lowerArm(double power, int time, Telemetry telemetry)
    {
        hardware.liftArm1.setPower(-power* RAISE_POWER);
        hardware.liftArm2.setPower(-power* RAISE_POWER);

        wait(time, telemetry);

        hardware.liftArm1.setPower(0);
        hardware.liftArm2.setPower(0);

    }
    public void turnOnEncoders()
    {
        hardware.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.liftArm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.liftArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.liftArm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.liftArm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
