package org.firstinspires.ftc.teamcode.autonomous.state;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.genericAuton;
import org.firstinspires.ftc.teamcode.autonomous.pipelines.BlueStackPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous (name="blueLeftFour")
public class blueLeftFour extends genericAuton
{
    private BlueStackPipeline stackPipeline;

    //toMid1
    private final Pose2d blueLeftHome = new Pose2d(36.0, 62.5, Math.toRadians(-90.0));
    private final Pose2d atMid1 = new Pose2d(31, 31, Math.toRadians(225));

    //toStack1
    private final Pose2d atStack1Inbetween = new Pose2d(39, 18, Math.toRadians(-75));
    private final Pose2d atStack1Final = new Pose2d(58, 12, Math.toRadians(0));

    //toMidRepeat
    private final Pose2d atMid2Inbetween = new Pose2d(41, 12, Math.toRadians(110));
    private final Pose2d atMid2Final = new Pose2d(29, 19, Math.toRadians(135));

    //toStackRepeat
    private final Pose2d toStackRepeat1 = new Pose2d(31, 18, Math.toRadians(135));
    private final Pose2d toStackRepeat2 = new Pose2d(38, 14, Math.toRadians(20));
    private final Pose2d toStackRepeat3 = new Pose2d(58, 12, Math.toRadians(0));

    //toParking1 (serves as Parking 1)
    private final Pose2d toParking1Inbetween1 = new Pose2d(31, 18, Math.toRadians(135));
    private final Pose2d toParking1Inbetween2 = new Pose2d(38, 14, Math.toRadians(20));
    private final Pose2d toParking1Inbetween3 = new Pose2d(63, 12, Math.toRadians(0));

    //toParking 3
    private final Pose2d toParking3OTW1 = new Pose2d(30,12,Math.toRadians(0));
    private final Pose2d toParking3OTW2 = new Pose2d(12,12,Math.toRadians(0));

    private Trajectory toMid1;
    private Trajectory toStack1;
    private Trajectory toMidRepeat;
    private Trajectory toStackRepeat;
    private Trajectory toParking1;
    private Trajectory toParking2;
    private Trajectory toParking3;
    private TrajectorySequence stackCorrection;
    private TrajectorySequence forward1;
    private TrajectorySequence forward2;

    @Override
    public void run()
    {
        drive.setPoseEstimate(blueLeftHome);
        stackPipeline = new BlueStackPipeline(telemetry, 13, 0);
        webcam.setPipeline(stackPipeline);

        //Go to mid junction 1
        utilities.liftArmAbsolutePosition(290);
        utilities.tiltClaw(false);
        drive.followTrajectory(toMid1);
        utilities.openClaw(true);

        //Go to stack 1
        utilities.liftArmAbsolutePosition(80);
        drive.followTrajectory(toStack1);

            //call in stack alignment

        utilities.wait(300, telemetry);
        double displacement = stackPipeline.getDisplacement();
        telemetry.addData("Robot Displacement", displacement);
        telemetry.update();


        if (displacement > 0) {
            stackCorrection = drive.trajectorySequenceBuilder(toStack1.end())
                    .strafeRight(Math.abs(displacement)).build();
            drive.followTrajectorySequence(stackCorrection);
        } else {
            stackCorrection = drive.trajectorySequenceBuilder(toStack1.end())
                    .strafeLeft(Math.abs(displacement)).build();
            drive.followTrajectorySequence(stackCorrection);
        }

        drive.followTrajectorySequence(forward1);
        utilities.openClaw(false);
        utilities.wait(500, telemetry);
        utilities.liftArmAbsolutePosition(290);
        utilities.wait(500, telemetry);

        //Go to Mid Repeat 1
        utilities.tiltClaw(true);
        drive.followTrajectory(toMidRepeat);
        utilities.openClaw(true);

        //Go to stack repeat 1
        utilities.liftArmAbsolutePosition(65);
        utilities.tiltClaw(false);
        drive.followTrajectory(toStackRepeat);

            //stack realignment

        utilities.wait(300, telemetry);
        displacement = stackPipeline.getDisplacement();
        telemetry.addData("Robot Displacement", displacement);
        telemetry.update();


        if (displacement > 0.1) {
            stackCorrection = drive.trajectorySequenceBuilder(toStackRepeat.end())
                    .strafeRight(Math.abs(displacement)).build();
            drive.followTrajectorySequence(stackCorrection);
        } else if (displacement < -.1){
            stackCorrection = drive.trajectorySequenceBuilder(toStackRepeat.end())
                    .strafeLeft(Math.abs(displacement)).build();
            drive.followTrajectorySequence(stackCorrection);
        }

        drive.followTrajectorySequence(forward2);
        utilities.openClaw(false);
        utilities.wait(500, telemetry);
        utilities.liftArmAbsolutePosition(290);
        utilities.wait(500, telemetry);

        //Go to Mid Repeat 2
        utilities.tiltClaw(true);
        drive.followTrajectory(toMidRepeat);
        utilities.openClaw(true);

        //Go to Stack Repeat 2
        utilities.liftArmAbsolutePosition(65);
        utilities.tiltClaw(false);
        drive.followTrajectory(toStackRepeat);

            //stack realignment

        utilities.wait(300, telemetry);
        displacement = stackPipeline.getDisplacement();
        telemetry.addData("Robot Displacement", displacement);
        telemetry.update();


        if (displacement > 0) {
            stackCorrection = drive.trajectorySequenceBuilder(toStackRepeat.end())
                    .strafeRight(Math.abs(displacement)).build();
            drive.followTrajectorySequence(stackCorrection);
        } else {
            stackCorrection = drive.trajectorySequenceBuilder(toStackRepeat.end())
                    .strafeLeft(Math.abs(displacement)).build();
            drive.followTrajectorySequence(stackCorrection);
        }

        drive.followTrajectorySequence(forward2);
        utilities.openClaw(false);
        utilities.wait(500, telemetry);
        utilities.liftArmAbsolutePosition(290);
        utilities.wait(500, telemetry);

        //Go to Mid Repeat 3
        utilities.tiltClaw(true);
        drive.followTrajectory(toMidRepeat);
        utilities.openClaw(true);

        //Go to Stack Repeat 3
        utilities.liftArmAbsolutePosition(65);
        utilities.tiltClaw(false);
        drive.followTrajectory(toStackRepeat);

            //stack realignment

        utilities.wait(300, telemetry);
        displacement = stackPipeline.getDisplacement();
        telemetry.addData("Robot Displacement", displacement);
        telemetry.update();


        if (displacement > 0) {
            stackCorrection = drive.trajectorySequenceBuilder(toStackRepeat.end())
                    .strafeRight(Math.abs(displacement)).build();
            drive.followTrajectorySequence(stackCorrection);
        } else {
            stackCorrection = drive.trajectorySequenceBuilder(toStackRepeat.end())
                    .strafeLeft(Math.abs(displacement)).build();
            drive.followTrajectorySequence(stackCorrection);
        }

        drive.followTrajectorySequence(forward2);
        utilities.openClaw(false);
        utilities.wait(500, telemetry);
        utilities.liftArmAbsolutePosition(290);
        utilities.wait(500, telemetry);

        //Parking
        if(identifier == 1)
            drive.followTrajectory(toParking1);
        else if (identifier == 2)
            drive.followTrajectory(toParking2);
        else if (identifier == 3)
            drive.followTrajectory(toParking3);
    }

    @Override
    public void buildTrajectories()
    {
        toMid1 = drive.trajectoryBuilder(blueLeftHome, Math.toRadians(-90))
                .splineToLinearHeading(atMid1, Math.toRadians(225))
                .build();
        toStack1 = drive.trajectoryBuilder(toMid1.end(), Math.toRadians(0))
                .splineToSplineHeading(atStack1Inbetween, Math.toRadians(-45))
                .splineToSplineHeading(atStack1Final, Math.toRadians(-6))
                .build();
        forward1 = drive.trajectorySequenceBuilder(toStack1.end())
                .forward(14)
                .build();
        toMidRepeat = drive.trajectoryBuilder(toStack1.end(), Math.toRadians(180))
                .splineToSplineHeading(atMid2Inbetween, Math.toRadians(-180))
                .splineToSplineHeading(atMid2Final, Math.toRadians(100))
                .build();
        toStackRepeat = drive.trajectoryBuilder(toMidRepeat.end(), Math.toRadians(-30))
                .splineToSplineHeading(toStackRepeat1, Math.toRadians(-30))
                .splineToSplineHeading(toStackRepeat2, Math.toRadians(-15))
                .splineToSplineHeading(toStackRepeat3, Math.toRadians(0))
                .build();
        forward2 = drive.trajectorySequenceBuilder(toStackRepeat.end())
                .forward(14)
                .build();
        toParking1 = drive.trajectoryBuilder(toMidRepeat.end(), Math.toRadians(-30))
                .splineToSplineHeading(toParking1Inbetween1, Math.toRadians(-30))
                .splineToSplineHeading(toParking1Inbetween2, Math.toRadians(-15))
                .splineToSplineHeading(toParking1Inbetween3, Math.toRadians(0))
                .build();
        toParking2 = drive.trajectoryBuilder(toMidRepeat.end())
                .back(7)
                .build();
        toParking3 = drive.trajectoryBuilder(toMidRepeat.end(), Math.toRadians(-45))
                .splineToSplineHeading(toParking3OTW1, Math.toRadians(200))
                .splineToSplineHeading(toParking3OTW2, Math.toRadians(190))
                .build();
    }
}
