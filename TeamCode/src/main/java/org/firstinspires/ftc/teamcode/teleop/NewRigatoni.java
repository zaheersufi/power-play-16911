package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad.RumbleEffect;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Assert;
import org.firstinspires.ftc.teamcode.autonomous.NewUtilities;
import org.firstinspires.ftc.teamcode.hardware.NewRigatoniHardware;



@TeleOp(name="NewRigatoni")
public class NewRigatoni extends LinearOpMode
{
    private NewRigatoniHardware hardware;
    private NewUtilities utilities;

    // With two motors it's best not to run at full capacity
    final double RAISE_POWER = 0.95;
    final double LOWER_POWER = 0.95;
    final double LOWER_POWER_SLOW = .4;

    final double FAST_SPEED = .8;
    final double SLOW_SPEED = .5;
    final double SUPER_SLOW_SPEED = .3;
    double speed = FAST_SPEED;

    final double ALMOST_END_GAME = 75;        // Wait this many seconds before rumble-alert.
    final double END_GAME = 90;               // Wait this many seconds before rumble-alert.


    ElapsedTime buttonTime = null;
    ElapsedTime timer = null;


    RumbleEffect customRumbleEffect = new RumbleEffect.Builder()
        .addStep(0.0, 1.0, 250)
        .addStep(1.0, 0.0, 250)
        .addStep(0.0, 0.0, 100)
        .addStep(1.0, 1.0, 500)
        .build();



    /**
     * This method initializes the Primary and supplementary motors
     * which consist of the wheels and lift motors, and the claw
     * motors which is a singular servo.
     */
    @Override
    public void runOpMode()
    {
        buttonTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        hardware = new NewRigatoniHardware();
        Assert.assertNotNull(hardwareMap);

        hardware.initializePrimaryMotors(hardwareMap);
        hardware.initializeClawServos(hardwareMap);
        hardware.initializeSupplementaryMotors(hardwareMap);

        utilities = new NewUtilities(hardware);
        utilities.turnOnEncoders();

        waitForStart();
        timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);


        try {
            run();
        } catch (Throwable t) {
            hardware.robotStopAllMotion();
            sleep(100);


            // Expected due to "Stop" button pressed.
            if (t instanceof NewRigatoniHardware.StopImmediatelyException) {
                telemetry.addData("Stop Requested", "");
                telemetry.update();
                return;
            }

            telemetry.addData("Exception caught!", t);
            telemetry.update();

            if (t instanceof RuntimeException) {
                throw (RuntimeException) t;
            }

            throw new RuntimeException(t);
        }

    }



    /**
     * A method that calls the drive, moveArm, and rotateClaw
     * methods, known as helper methods.
     *
     * Loop/actions for Driver-Controlled
     */
    public void run()
    {
        boolean almostEndgame = false;
        boolean endgame = false;

        runRumble(customRumbleEffect);

        while (opModeIsActive()) {
            drive();
            moveArm();
            rotateClaw();

            if ((timer.seconds() > ALMOST_END_GAME) && !almostEndgame)  {
                runRumble(customRumbleEffect);
                almostEndgame = true;
            }

            if ((timer.seconds() > END_GAME) && !endgame)  {
                runRumble(customRumbleEffect);
                endgame = true;
            }


        }

    }



    /**
     * Initializes variables fundamental to driving. Then sets power
     * assigned to each variable in series of if-statements.
     *
     * Set the driver motors' power
     */
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

        //D-Pad Junction heights
        if (gamepad2.dpad_up) //Ground Junction
        {
            utilities.liftArmAbsolutePosition(100);
        }
        else if (gamepad2.dpad_left)
        {
            utilities.liftArmAbsolutePosition(600);
        }
        else if (gamepad2.dpad_down)
        {
            utilities.liftArmAbsolutePosition(1400);
        }
        else if (gamepad2.dpad_right)
        {
            utilities.liftArmAbsolutePosition(2200);
        }



        changeSpeed();


        hardware.leftFront.setPower(leftFrontPower * speed);
        hardware.leftRear.setPower(leftRearPower * speed);
        hardware.rightFront.setPower(rightFrontPower * speed);
        hardware.rightRear.setPower(rightRearPower * speed);

    }



    /**
     * Creates the slow and super slow speeds through if-statements that change the
     * drive speed based on the input of the square (slow) and circle (super slow) button.
     *
     * Change the motors' speed mode:
     * - Fast Speed
     * - Slow Speed
     * - SuperSlow Speed
     */
    public void changeSpeed()
    {
        // Slow Speed
        if (gamepad1.square && speed == SLOW_SPEED && buttonTime.time() >= 500)
        {
            speed = FAST_SPEED;
            buttonTime.reset();
        }
        else if (gamepad1.square && buttonTime.time() >= 500)
        {
            speed = SLOW_SPEED;
            buttonTime.reset();
        }


        // SuperSlow Speed
        if (gamepad1.circle && speed == SUPER_SLOW_SPEED && buttonTime.time() >= 500)
        {
            speed = FAST_SPEED;
            buttonTime.reset();
        }
        else if (gamepad1.circle && buttonTime.time() >= 500)
        {
            speed = SUPER_SLOW_SPEED;
            buttonTime.reset();
        }

    }



    /**
     * This method moves the arm up and down based on the input of the right
     * and left triggers, which moves the pulley up and down respectively.
     *
     * Move the lift up or down
     * - Move up at RAISE_POWER
     * - Move down at LOWER_POWER
     */
    public void moveArm()
    {
        // Raising the lift
        if ( (gamepad2.right_trigger > 0) && (gamepad2.right_trigger > gamepad2.left_trigger) ) {
            hardware.liftArm1.setPower(gamepad2.right_trigger * RAISE_POWER);
            hardware.liftArm2.setPower(gamepad2.right_trigger * RAISE_POWER);
        }

        // Lowering the lift
        else if ( (gamepad2.left_trigger > 0) && (gamepad2.left_trigger > gamepad2.right_trigger) ) {
            hardware.liftArm1.setPower(-gamepad2.left_trigger * LOWER_POWER);
            hardware.liftArm2.setPower(-gamepad2.left_trigger * LOWER_POWER);
        }

        //Lowering the lift low speed
        else if(gamepad2.left_bumper) {
            hardware.liftArm1.setPower(-LOWER_POWER_SLOW);
            hardware.liftArm2.setPower(-LOWER_POWER_SLOW);
        }

        // Nothing happens
        else {
            hardware.liftArm1.setPower(0);
            hardware.liftArm2.setPower(0);
        }

        telemetry.addData("Position1: ", hardware.liftArm1.getCurrentPosition());
        telemetry.addData("Position2: ", hardware.liftArm2.getCurrentPosition());
        telemetry.update();

    }



    /**
     * The circle and square buttons allow the claw to open and close its prongs.
     *
     * Open and Close the grabber/claw
     */
    public void rotateClaw()
    {
        if(gamepad2.square)
            hardware.grabServo.setPosition(.6);
        if(gamepad2.circle)
            hardware.grabServo.setPosition(0.3);

    }



    /**
     * Make controllers vibrate
     */
    public void runRumble(RumbleEffect e) {
        gamepad1.runRumbleEffect(e);
        gamepad2.runRumbleEffect(e);

    }


}
