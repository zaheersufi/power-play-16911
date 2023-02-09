package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.NewRigatoniHardware;



public class NewUtilities
{

    private NewRigatoniHardware hardware;

    private final double FULL_POWER = 0.7;
    private final double SLOW_POWER = 0.4;



    public NewUtilities(NewRigatoniHardware hardware)
    {
        this.hardware = hardware;
    }


    /**
     * Wait time method that allows for a buffer between commands
     * which communicates with the phone and a predetermined variable
     * wait time input
     */
    public void wait(int waitTime, Telemetry telemetry)
    {
        ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        time.reset();

        Telemetry.Item status = telemetry.addData("Status:", "waiting");
        telemetry.update();

        while (time.time() < waitTime){}

        telemetry.removeItem(status);
        telemetry.update();

    }


    /**
     * Open claw method that opens or closes the claw
     * to set positions based on the value of the
     * boolean variable
     */
    public void openClaw(boolean shouldOpen)
    {
        if(!shouldOpen)
            hardware.grabServo.setPosition(0.6);
        else
            hardware.grabServo.setPosition(0.3);
    }


    /**
     * Method that raises the arm to a height
     * based on a power and time which is
     * passed in as a parameter
     */
    public void liftArm(double power, int time, Telemetry telemetry)
    {
        hardware.liftArm1.setPower(power * FULL_POWER);
        hardware.liftArm2.setPower(power * FULL_POWER);

        wait(time, telemetry);

        hardware.liftArm1.setPower(0);
        hardware.liftArm2.setPower(0);

        wait(100, telemetry);
    }


    /**
     * Method that moves the arm down to a height
     * based on a power and time passed in as a
     * parameter
     */
    public void lowerArm(double power, int time, Telemetry telemetry)
    {
        hardware.liftArm1.setPower(-power * SLOW_POWER);
        hardware.liftArm2.setPower(-power * SLOW_POWER);

        wait(time, telemetry);

        hardware.liftArm1.setPower(0);
        hardware.liftArm2.setPower(0);

        wait(100, telemetry);
    }


    /**
     * This method uses encoders to move the arm to
     * a relative displacement based on its previous
     * position
     */
    public void liftArmDisplacementPosition(int pos)
    {
        double power = FULL_POWER;
        if (pos < 0) power = SLOW_POWER;

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


    /**
     * This method moves the lift to a specific
     * position through the use of encoders
     * regardless of previous position
     */
    public void liftArmAbsolutePosition (int pos)
    {
        double power = FULL_POWER;
        if (pos < hardware.liftArm1.getCurrentPosition()) power = SLOW_POWER;

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

}
