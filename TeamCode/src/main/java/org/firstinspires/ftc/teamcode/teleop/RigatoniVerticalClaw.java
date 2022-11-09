package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.internal.system.Assert;
import org.firstinspires.ftc.teamcode.hardware.RigatoniHardware;

@TeleOp(name="RigatoniVerticalClaw")
public class RigatoniVerticalClaw extends OpMode
{
    RigatoniHardware hardware;
    final double FAST_SPEED = .8;
    final double SLOW_SPEED = .5;
    double slowConstant = FAST_SPEED;

    ElapsedTime buttonTime = null;
    @Override
    public void init()
    {
        hardware = new RigatoniHardware();
        Assert.assertNotNull(hardwareMap);
        hardware.initializePrimaryMotors(hardwareMap);
        hardware.initializeClawServos(hardwareMap);
        hardware.initializeSupplementaryMotors(hardwareMap);
        hardware.initializeVerticalClaw(hardwareMap);
    }

    @Override
    public void loop() {
        drive();
        moveArm();
        rotateClaw();
    }
    public void drive() {
        // Mecanum drivecode
        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        double leftFrontPower = y + x + rx;
        double leftRearPower = y - x + rx;
        double rightFrontPower = y - x - rx;
        double rightRearPower = y + x - rx;


        if (Math.abs(leftFrontPower) > 1 || Math.abs(leftRearPower) > 1 ||
                Math.abs(rightFrontPower) > 1 || Math.abs(rightRearPower) > 1 )
        {
            // Find the largest power
            double max;
            max = Math.max(Math.abs(leftFrontPower), Math.abs(leftRearPower));
            max = Math.max(Math.abs(rightFrontPower), max);
            max = Math.max(Math.abs(rightRearPower), max);

            // Divide everything by max (it's positive so we don't need to worry
            // about signs)
            leftFrontPower /= max;
            leftRearPower /= max;
            rightFrontPower /= max;
            rightRearPower /= max;
        }

        if (gamepad1.dpad_up || gamepad1.dpad_right)
        {
            leftFrontPower = -1;
            rightRearPower = -1;
            rightFrontPower = 1;
            leftRearPower = 1;
        }
        else if (gamepad1.dpad_down || gamepad1.dpad_left)
        {
            leftFrontPower = 1;
            rightRearPower = 1;
            rightFrontPower = -1;
            leftRearPower = -1;
        }

        if (gamepad1.square && slowConstant == FAST_SPEED && buttonTime.time() >= 500)
        {
            slowConstant = SLOW_SPEED;
            buttonTime.reset();
        }
        else if (gamepad1.square && slowConstant == SLOW_SPEED && buttonTime.time() >= 500)
        {
            slowConstant = FAST_SPEED;
            buttonTime.reset();
        }

        hardware.leftFront.setPower(leftFrontPower * slowConstant);
        hardware.leftRear.setPower(leftRearPower * slowConstant);
        hardware.rightFront.setPower(rightFrontPower * slowConstant);
        hardware.rightRear.setPower(rightRearPower * slowConstant);
    }
    public void moveArm()
    {
        //triggers = linear pulley
        hardware.liftArm.setPower((gamepad2.right_trigger - gamepad2.left_trigger)*.8);
    }
    public void rotateClaw()
    {
        //[0.0, 1.0] for range; 0 = 0 deg; 1.0 = 270 deg
//        double rotServoPosition = hardware.rotServo.getPosition(); //CIRCLE = 0 DEG; CROSS = 90 DEG; SQUARE = 180
//        if(gamepad2.cross && Math.abs(rotServoPosition - 0.3333) > 0.018) //10 degree range of error
//        {
//            hardware.rotServo.setPosition(0.3333);
//        }
//        if(gamepad2.circle && Math.abs(rotServoPosition - 0) > 0.018)
//        {
//            hardware.rotServo.setPosition(0);
//        }
//        if(gamepad2.square && Math.abs(rotServoPosition - 0.6666) > 0.018)
//        {
//            hardware.rotServo.setPosition(0.6666);
//        }
//        int rightBumper = gamepad2.right_bumper ? 1 : 0;
//        int leftBumper = gamepad2.left_bumper ? 1 : 0;
//        hardware.grabServo.setPosition((rightBumper - leftBumper) * slowConstant);

        if (gamepad2.triangle)
            hardware.grabServo.setPosition(.70);
        if (gamepad2.cross)
            hardware.grabServo.setPosition(.07);
        if(gamepad2.left_bumper)
            hardware.rotServo.setPosition(.5);
        if(gamepad2.right_bumper)
            hardware.rotServo.setPosition(1);
    }
}
