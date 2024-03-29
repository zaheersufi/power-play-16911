package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Assert;
import org.firstinspires.ftc.teamcode.hardware.OldRigatoniHardware;


@Disabled
@TeleOp(name="OldRigatoni")
public class OldRigatoni extends OpMode
{
    private OldRigatoniHardware hardware;
    final double FAST_SPEED = .8;
    final double SLOW_SPEED = .5;
    final double SUPER_SLOW_SPEED = .3;
    double speed = FAST_SPEED;

    ElapsedTime buttonTime = null;



    @Override
    public void init()
    {
        buttonTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        hardware = new OldRigatoniHardware();
        Assert.assertNotNull(hardwareMap);

        hardware.initializePrimaryMotors(hardwareMap);
        hardware.initializeClawServos(hardwareMap);
        hardware.initializeSupplementaryMotors(hardwareMap);

        hardware.liftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.liftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }



    @Override
    public void loop()
    {
        drive();
        moveArm();
        rotateClaw();

    }



    public void drive()
    {
        // Mecanum drivecode
        double y = -gamepad1.left_stick_y;  // Remember, this is reversed!
        double x = gamepad1.left_stick_x;   // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        double leftFrontPower = y + x + rx;
        double leftRearPower = y - x + rx;
        double rightFrontPower = y - x - rx;
        double rightRearPower = y + x - rx;


        // Joystick Power
        if (Math.abs(leftFrontPower) > 1 || Math.abs(leftRearPower) > 1 || Math.abs(rightFrontPower) > 1 || Math.abs(rightRearPower) > 1 )
        {
            // Find the largest power
            double max;
            max = Math.max(Math.abs(leftFrontPower), Math.abs(leftRearPower));
            max = Math.max(Math.abs(rightFrontPower), max);
            max = Math.max(Math.abs(rightRearPower), max);

            // Divide everything by max
            leftFrontPower /= max;
            leftRearPower /= max;
            rightFrontPower /= max;
            rightRearPower /= max;
        }


        // D-PAD Power
        if(gamepad1.dpad_up)
        {
            leftFrontPower = 1;
            rightRearPower = 1;
            leftRearPower = 1;
            rightFrontPower = 1;
        }
        else if (gamepad1.dpad_right)
        {
            leftFrontPower = 1;
            rightRearPower = 1;
            rightFrontPower = -1;
            leftRearPower = -1;
        }
        else if (gamepad1.dpad_left)
        {
            leftFrontPower = -1;
            rightRearPower = -1;
            rightFrontPower = 1;
            leftRearPower = 1;
        }
        else if (gamepad1.dpad_down)
        {
            leftFrontPower = -1;
            rightRearPower = -1;
            rightFrontPower = -1;
            leftRearPower = -1;
        }


        changeSpeed();


        hardware.leftFront.setPower(leftFrontPower * speed);
        hardware.leftRear.setPower(leftRearPower * speed);
        hardware.rightFront.setPower(rightFrontPower * speed);
        hardware.rightRear.setPower(rightRearPower * speed);

    }



    public void changeSpeed()
    {
        // Slow Speed
        if (gamepad1.square && speed == FAST_SPEED && buttonTime.time() >= 500)
        {
            speed = SLOW_SPEED;
            buttonTime.reset();
        }
        else if (gamepad1.square && speed == SLOW_SPEED && buttonTime.time() >= 500)
        {
            speed = FAST_SPEED;
            buttonTime.reset();
        }
        else if(gamepad1.square && speed == SUPER_SLOW_SPEED && buttonTime.time() >= 500)
        {
            speed = SUPER_SLOW_SPEED;
            buttonTime.reset();
        }


        // SuperSLow Speed
        if (gamepad1.circle && speed == FAST_SPEED && buttonTime.time() >= 500)
        {
            speed = SUPER_SLOW_SPEED;
            buttonTime.reset();
        }
        else if (gamepad1.circle && speed == SLOW_SPEED && buttonTime.time() >= 500)
        {
            speed = FAST_SPEED;
            buttonTime.reset();
        }
        else if(gamepad1.circle && speed == SUPER_SLOW_SPEED && buttonTime.time() >= 500)
        {
            speed = FAST_SPEED;
            buttonTime.reset();
        }

    }



    public void moveArm()
    {
        telemetry.addData("Position: ", hardware.liftArm.getCurrentPosition());
        telemetry.update();

        hardware.liftArm.setPower((gamepad2.right_trigger - gamepad2.left_trigger) * 1);

    }



    public void rotateClaw()
    {
        if(gamepad2.square)
            hardware.grabServo.setPosition(1);
        if(gamepad2.circle)
            hardware.grabServo.setPosition(0);

    }


}
