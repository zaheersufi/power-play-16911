package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Disabled
public class OldRigatoniHardware
{
    // Primary Motors
    public DcMotorEx leftFront = null;
    public DcMotorEx leftRear = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx rightRear = null;

    public DcMotorEx[] motors;

    // Supplementary Motor
    public DcMotorEx liftArm = null;

    // CLaw Servo
    public Servo grabServo = null;



    /**
     * Initializes the drive motors of the robot and
     * sets them to run without encoder.
     *
     * @param hardwareMap   robot's components map
     */
    public void initializePrimaryMotors(HardwareMap hardwareMap)
    {
        // Primary Motors
        leftFront = hardwareMap.get(DcMotorEx.class, RigatoniIds.LEFT_FRONT_MOTOR);
        leftRear = hardwareMap.get(DcMotorEx.class, RigatoniIds.LEFT_REAR_MOTOR);
        rightFront = hardwareMap.get(DcMotorEx.class, RigatoniIds.RIGHT_FRONT_MOTOR);
        rightRear = hardwareMap.get(DcMotorEx.class, RigatoniIds.RIGHT_REAR_MOTOR);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);


        // Set Zero Power Behavior and Initialize Motors
        leftRear.setPower(0);
        leftRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setPower(0);
        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightFront.setPower(0);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightRear.setPower(0);
        rightRear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }



    /**
     * Initializes the claw servo, sets the position to closed,
     * and sets the default movement direction.
     *
     * @param hardwareMap   robot's components map
     */
    public void initializeClawServos(HardwareMap hardwareMap)
    {
        // Claw Servos
        grabServo = hardwareMap.get(Servo.class, RigatoniIds.GRAB_SERVO);

        grabServo.setDirection(Servo.Direction.FORWARD);
        grabServo.setPosition(1);
    }



    /**
     * Initializes the lift motors.
     *
     * @param hardwareMap   robot's components map
     */
    public void initializeSupplementaryMotors(HardwareMap hardwareMap)
    {
        liftArm = hardwareMap.get(DcMotorEx.class, RigatoniIds.LIFT_ARM_MOTOR);

        liftArm.setDirection(DcMotorSimple.Direction.FORWARD);

        liftArm.setPower(0);
        liftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
