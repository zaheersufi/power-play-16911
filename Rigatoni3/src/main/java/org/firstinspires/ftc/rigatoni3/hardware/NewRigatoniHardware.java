package org.firstinspires.ftc.rigatoni3.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;



public class NewRigatoniHardware
{
    // Primary Motors initialization
    public DcMotorEx leftFront = null;
    public DcMotorEx leftRear = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx rightRear = null;

    public DcMotorEx[] motors;

    // Lift Motors initialization
    public DcMotorEx liftArm1 = null;
    public DcMotorEx liftArm2 = null;

    // Claw Servo initialization
    public Servo grabServo = null;
    public Servo rotServo = null; //Servo that rotates like a wrist
    public Servo passServo1 = null;
    public Servo passServo2 = null;


    //private PIDFCoefficients PIDF = new PIDFCoefficients(10,0.5,0,45);

    //Constants for initialization position or preset buttons
    public static final double GRAB_CLOSED = 1; //.6
    public static final double GRAB_OPENED = .6; //.38
    public static final double ROTATE_OPENED = 0;
    public static final double PASS_INIT = 0; //initialize position of passover servo
    public static final double PASS_MIDDLE = .5;
    public static final double PASS_ALMOST_FINAL = 0.75;
    public static final double PASS_FINAL = 1.0;
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

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);


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
     * Sets drive motors to run with encoder.
     */
//    public void turnOnDriveEncoders()
//    {
//        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//    }



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
        grabServo.setPosition(GRAB_CLOSED);

        rotServo = hardwareMap.get(Servo.class, RigatoniIds.ROT_SERVO);
        rotServo.setDirection(Servo.Direction.FORWARD);
        rotServo.setPosition(ROTATE_OPENED);

        passServo1 = hardwareMap.get(Servo.class, RigatoniIds.PASS_SERVO1);
        passServo2 = hardwareMap.get(Servo.class, RigatoniIds.PASS_SERVO2);
        passServo1.setDirection(Servo.Direction.FORWARD);
        passServo2.setDirection(Servo.Direction.FORWARD);
        passServo1.setPosition(PASS_INIT);
        passServo2.setPosition(PASS_INIT);
    }


    public void initializeClawServosTeleOp(HardwareMap hardwareMap)
    {
        // Claw Servo
        grabServo = hardwareMap.get(Servo.class, RigatoniIds.GRAB_SERVO);
        grabServo.setDirection(Servo.Direction.FORWARD);
        grabServo.setPosition(GRAB_OPENED);

        rotServo = hardwareMap.get(Servo.class, RigatoniIds.ROT_SERVO);
        rotServo.setDirection(Servo.Direction.FORWARD);
        rotServo.setPosition(ROTATE_OPENED);

        passServo1 = hardwareMap.get(Servo.class, RigatoniIds.PASS_SERVO1);
        passServo2 = hardwareMap.get(Servo.class, RigatoniIds.PASS_SERVO2);
        passServo1.setDirection(Servo.Direction.FORWARD);
        passServo2.setDirection(Servo.Direction.FORWARD);
        passServo1.setPosition(PASS_INIT);
        passServo2.setPosition(PASS_INIT);
    }



    /**
     * Initializes the lift motors.
     *
     * @param hardwareMap   robot's components map
     */
    public void initializeSupplementaryMotors(HardwareMap hardwareMap)
    {
        // LiftArm2
        liftArm2 = hardwareMap.get(DcMotorEx.class, RigatoniIds.LIFT_ARM2);

        liftArm2.setDirection(DcMotorSimple.Direction.FORWARD);

        liftArm2.setPower(0);
        liftArm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftArm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftArm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // LiftArm1
        liftArm1 = hardwareMap.get(DcMotorEx.class, RigatoniIds.LIFT_ARM1);

        liftArm1.setDirection(DcMotorSimple.Direction.FORWARD);

        liftArm1.setPower(0);
        liftArm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftArm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // liftArm1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF);
        // liftArm2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF);
    }



    /**
     * Turns off all motors.
     */
    public void robotStopAllMotion() {
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);

        liftArm1.setPower(0);
        liftArm2.setPower(0);

    }
}
