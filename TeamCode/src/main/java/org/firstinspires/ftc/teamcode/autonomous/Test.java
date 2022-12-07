package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Assert;
import org.firstinspires.ftc.teamcode.hardware.RigatoniHardware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="Test")
public class Test extends LinearOpMode
{
    private Utilities utilities;
    RigatoniHardware hardware;
    OpenCvInternalCamera webcam;
    HsvMaskPipeline pipeline;
    private ElapsedTime runtime = new ElapsedTime();

    private int initialWaitTime = 0;
    private final double POWER = 0.5;


    @Override
    public void runOpMode()
    {
        hardware = new RigatoniHardware(); //Horizontal Claw
        Assert.assertNotNull(hardwareMap);
        hardware.initializePrimaryMotors(hardwareMap);
        hardware.initializeClawServos(hardwareMap);
        hardware.initializeSupplementaryMotors(hardwareMap);

        turnOnEncoders(hardware);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new HsvMaskPipeline(telemetry);
        webcam.setPipeline(pipeline);

        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }
            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error", "* Camera could not be opened *");
                telemetry.update();
            }
        });


        int identifier = pipeline.getDestination();
        utilities = new Utilities(hardware);


        waitForStart();
        if(!opModeIsActive()) {return;}

        utilities.openClaw(false);
        utilities.wait(initialWaitTime, telemetry);

        runtime.reset();

//        goToJunction(hardware, runtime);
//        highJunction(telemetry, runtime);
        park(identifier, runtime);

    }


    private void goToJunction(RigatoniHardware hardware, ElapsedTime runtime)
    {
        return;


    }



    public void highJunction (Telemetry telemetry, ElapsedTime runtime)
    {
        utilities.liftArm(.8, 5400, telemetry);
        utilities.openClaw(true);
        utilities.lowerArm(.8, 5400, telemetry);
    }



    private void park(int identifier, ElapsedTime runtime)
    {
        if (identifier == 0) {
            left(hardware, runtime, 3);
            forward(hardware, runtime, 3);
        }
        else if (identifier == 1) {
            forward(hardware, runtime, 3);
        }
        else {
            right(hardware, runtime, 3);
            forward(hardware, runtime, 3);
        }
    }



    private void turnOnEncoders(RigatoniHardware hardware)
    {
        hardware.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hardware.liftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    private void forward(RigatoniHardware hardware, ElapsedTime runtime, double time)
    {
        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() <= time)) {
            hardware.leftFront.setPower(POWER);
            hardware.leftRear.setPower(POWER);
            hardware.rightFront.setPower(POWER);
            hardware.rightRear.setPower(POWER);
        }
    }


    private void backward(RigatoniHardware hardware, ElapsedTime runtime, double time)
    {
        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() <= time)) {
            hardware.leftFront.setPower(-POWER);
            hardware.leftRear.setPower(-POWER);
            hardware.rightFront.setPower(-POWER);
            hardware.rightRear.setPower(-POWER);
        }
    }


    private void left(RigatoniHardware hardware, ElapsedTime runtime, double time)
    {
        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() <= time)) {
            hardware.leftFront.setPower(POWER);
            hardware.leftRear.setPower(POWER);
            hardware.rightFront.setPower(-POWER);
            hardware.rightRear.setPower(-POWER);
        }
    }


    private void right(RigatoniHardware hardware, ElapsedTime runtime, double time)
    {
        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() <= time)) {
            hardware.leftFront.setPower(-POWER);
            hardware.leftRear.setPower(-POWER);
            hardware.rightFront.setPower(POWER);
            hardware.rightRear.setPower(POWER);
        }
    }

}
