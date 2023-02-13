package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Assert;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.NewRigatoniHardware;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;


@Config
@Autonomous(group = "drive")
public class PositionVsTime extends LinearOpMode{
    public static double VX = 0;
    public static double VY = 0;
    public static double W = 0;

    public static double RUNTIME = 2.0;

    private ElapsedTime timer;
    private double maxVelocity = 0.0;

    private VoltageSensor batteryVoltageSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Please ensure you have enough space cleared.");
        telemetry.addLine("");
        telemetry.addLine("Press start when ready.");
        telemetry.update();

        Datalog datalog = new Datalog("DataLog_"+"VX"+VX+"_VY"+VY+"_W"+W);

        waitForStart();

        telemetry.clearAll();
        telemetry.update();

        drive.setDrivePower(new Pose2d(Math.min(VX,1), Math.min(VY,1), Math.min(W,1)));
        timer = new ElapsedTime();

        int i=0;
        while (!isStopRequested() && timer.seconds() < RUNTIME) {
            drive.updatePoseEstimate();

            Pose2d pose = Objects.requireNonNull(drive.getPoseEstimate(), "poseEstimate() must not be null. Ensure that the getWheelVelocities() method has been overridden in your localizer.");
            Pose2d poseVelo = Objects.requireNonNull(drive.getPoseVelocity(), "poseVelocity() must not be null. Ensure that the getWheelVelocities() method has been overridden in your localizer.");
            maxVelocity = Math.max(poseVelo.vec().norm(), maxVelocity);

            // Robot Position
            telemetry.addData("X Position", pose.getX());
            telemetry.addData("Y Position", pose.getY());
            telemetry.addData("Heading", pose.getHeading());

            // Robot Velocity
            telemetry.addData("X Velocity", poseVelo.getX());
            telemetry.addData("Y Velocity", poseVelo.getY());
            telemetry.addData("Angular Velocity", poseVelo.getHeading());
            telemetry.addData("Max Velocity", maxVelocity);

            // Wheel Velocities
            List<Double> wheelVelocities = drive.getWheelVelocities();
            telemetry.addData("Wheel 0", wheelVelocities.get(0));
            telemetry.addData("Wheel 1", wheelVelocities.get(1));
            telemetry.addData("Wheel 2", wheelVelocities.get(2));
            telemetry.addData("Wheel 3", wheelVelocities.get(3));

            telemetry.update();

            datalog.loopCounter.set(i);
            datalog.time.set(timer.milliseconds());
            datalog.xPos.set(pose.getX());
            datalog.yPos.set(pose.getY());
            datalog.Heading.set(pose.getHeading());
            datalog.xVel.set(poseVelo.getX());
            datalog.yVel.set(poseVelo.getY());
            datalog.angVel.set(poseVelo.getHeading());
            datalog.wheel0.set(wheelVelocities.get(0));
            datalog.wheel1.set(wheelVelocities.get(1));
            datalog.wheel2.set(wheelVelocities.get(2));
            datalog.wheel3.set(wheelVelocities.get(3));

            datalog.writeLine();

            i++;
        }


        while (!isStopRequested() && opModeIsActive()) idle();
    }


    /*
     * This class encapsulates all the fields that will go into the datalog.
     */
    public static class Datalog
    {
        // The underlying datalogger object - it cares only about an array of loggable fields
        private final Datalogger datalogger;

        // These are all of the fields that we want in the datalog.
        // Note that order here is NOT important. The order is important in the setFields() call below
        public Datalogger.GenericField loopCounter   = new Datalogger.GenericField("Loop Counter");
        public Datalogger.GenericField time          = new Datalogger.GenericField("Time");
        public Datalogger.GenericField xPos          = new Datalogger.GenericField("X Position");
        public Datalogger.GenericField yPos          = new Datalogger.GenericField("Y Position");
        public Datalogger.GenericField Heading       = new Datalogger.GenericField("Heading");
        public Datalogger.GenericField xVel          = new Datalogger.GenericField("X Velocity");
        public Datalogger.GenericField yVel          = new Datalogger.GenericField("Y Velocity");
        public Datalogger.GenericField angVel        = new Datalogger.GenericField("Angular Velocity");
        public Datalogger.GenericField wheel0        = new Datalogger.GenericField("Wheel 0 Velocity");
        public Datalogger.GenericField wheel1        = new Datalogger.GenericField("Wheel 1 Velocity");
        public Datalogger.GenericField wheel2        = new Datalogger.GenericField("Wheel 2 Velocity");
        public Datalogger.GenericField wheel3        = new Datalogger.GenericField("Wheel 3 Velocity");



        public Datalog(String name)
        {
            // Build the underlying datalog object
            datalogger = new Datalogger.Builder()

                    // Pass through the filename
                    .setFilename(name)

                    // Request an automatic timestamp field
                    .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)

                    // Tell it about the fields we care to log.
                    // Note that order *IS* important here! The order in which we list
                    // the fields is the order in which they will appear in the log.
                    .setFields(
                            loopCounter,
                            time,
                            xPos,
                            yPos,
                            Heading,
                            xVel,
                            yVel,
                            angVel,
                            wheel0,
                            wheel1,
                            wheel2,
                            wheel3
                    )
                    .build();
        }

        // Tell the datalogger to gather the values of the fields
        // and write a new line in the log.
        public void writeLine()
        {
            datalogger.writeLine();
        }
    }
}

