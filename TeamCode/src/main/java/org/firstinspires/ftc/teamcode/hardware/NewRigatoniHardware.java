package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;



public class NewRigatoniHardware
{
    // Primary Motors
    public DcMotorEx leftFront = null;
    public DcMotorEx leftRear = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx rightRear = null;

    public DcMotorEx[] motors;

    // Lift Motors
    public DcMotorEx liftArm1 = null;
    public DcMotorEx liftArm2 = null;

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
        // Claw Servo
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
        // LiftArm1
        liftArm1 = hardwareMap.get(DcMotorEx.class, RigatoniIds.LIFT_ARM1);

        liftArm1.setDirection(DcMotorSimple.Direction.FORWARD);

        liftArm1.setPower(0);
        liftArm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftArm1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // LiftArm2
        liftArm2 = hardwareMap.get(DcMotorEx.class, RigatoniIds.LIFT_ARM2);

        liftArm2.setDirection(DcMotorSimple.Direction.FORWARD);

        liftArm2.setPower(0);
        liftArm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftArm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
}
