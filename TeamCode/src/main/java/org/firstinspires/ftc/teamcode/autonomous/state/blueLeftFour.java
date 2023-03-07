package org.firstinspires.ftc.teamcode.autonomous.state;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.autonomous.genericAuton;

public class blueLeftFour extends genericAuton
{
    //toMid1
    private final Pose2d blueLeftHome = new Pose2d(36.0, 62.5, Math.toRadians(-90.0));
    private final Pose2d atMid1 = new Pose2d(29, 29, Math.toRadians(225));

    //toStack1
    private final Pose2d atStack1Inbetween = new Pose2d(39, 18, Math.toRadians(-75));
    private final Pose2d atStack1Final = new Pose2d(63, 12, Math.toRadians(0));

    //toMidRepeat
    private final Pose2d atMid2Inbetween = new Pose2d(41, 12, Math.toRadians(110));
    private final Pose2d atMid2Final = new Pose2d(29, 19, Math.toRadians(135));

    //toStackRepeat (serves as Parking 1)
    private final Pose2d atStack2Inbetween1 = new Pose2d(31, 18, Math.toRadians(135));
    private final Pose2d atStack2Inbetween2 = new Pose2d(38, 14, Math.toRadians(20));
    private final Pose2d atStack2Inbetween3 = new Pose2d(63, 12, Math.toRadians(0));

    //toParking 3
    private final Pose2d toParking3OTW1 = new Pose2d(29,19,Math.toRadians(135));
    private final Pose2d toParking3OTW2 = new Pose2d(30,12,Math.toRadians(0));
    private final Pose2d toParking3OTW3 = new Pose2d(12,12,Math.toRadians(0));

    private Trajectory toMid1;
    private Trajectory toStack1;
    private Trajectory toMidRepeat;
    private Trajectory toStackRepeat;
    private Trajectory toParking1;
    private Trajectory toParking2;
    private Trajectory toParking3;

    //******Change to stack and to mid repeat splines to be 14.8 inches in front of the stack and change paths

    @Override
    public void run()
    {
        drive.setPoseEstimate(blueLeftHome);

        //Go to mid junction 1
        utilities.liftArmAbsolutePosition(290);
        utilities.tiltClaw(false);
        drive.followTrajectory(toMid1);
        utilities.openClaw(true);

        //Go to stack 1
        utilities.liftArmAbsolutePosition(80);
        utilities.tiltClaw(true);
        drive.followTrajectory(toStack1); //needs to be readjusted for stack alignment (move back 14)
        utilities.tiltClaw(false);
        //call in stack alignment
        //forward trajectory of 14/15 to go to stack
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
        drive.followTrajectory(toStackRepeat); //needs to be readjusted for stack alignment (move back 14)
        //stack realignment
        //forward trajectory into stack
        utilities.openClaw(false);
        utilities.wait(500, telemetry);
        utilities.liftArmAbsolutePosition(290);
        utilities.wait(500, telemetry);

        //Go to Mid Repeat 2
        //utilities.tilty
        drive.followTrajectory(toMidRepeat);
        utilities.openClaw(true);

        //Go to Stack Repeat 2
        utilities.liftArmAbsolutePosition(65);
        //utilities.tilty
        drive.followTrajectory(toStackRepeat); //needs to be readjusted for stack alignment (move back 14)
        //stack realignment
        //forward trajectory into stack
        utilities.openClaw(false);
        utilities.wait(500, telemetry);
        utilities.liftArmAbsolutePosition(290);
        utilities.wait(500, telemetry);

        //Go to Mid Repeat 3
        //utilities.tilty
        drive.followTrajectory(toMidRepeat);
        utilities.openClaw(true);

        //Go to Stack Repeat 3
        utilities.liftArmAbsolutePosition(65);
        //utilities.tilty
        drive.followTrajectory(toStackRepeat); //needs to be readjusted for stack alignment (move back 14)
        //stack realignment
        //forward trajectory into stack
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
        toMidRepeat = drive.trajectoryBuilder(toStack1.end(), Math.toRadians(180))
                .splineToSplineHeading(atMid2Inbetween, Math.toRadians(-180))
                .splineToSplineHeading(atMid2Final, Math.toRadians(100))
                .build();
        toStackRepeat = drive.trajectoryBuilder(toMidRepeat.end(), Math.toRadians(-30))
                .splineToSplineHeading(atStack2Inbetween1, Math.toRadians(-30))
                .splineToSplineHeading(atStack2Inbetween2, Math.toRadians(-15))
                .splineToSplineHeading(atStack2Inbetween3, Math.toRadians(0))
                .build();
        toParking1 = drive.trajectoryBuilder(toMidRepeat.end(), Math.toRadians(-30))
                .splineToSplineHeading(atStack2Inbetween1, Math.toRadians(-30))
                .splineToSplineHeading(atStack2Inbetween2, Math.toRadians(-15))
                .splineToSplineHeading(atStack2Inbetween3, Math.toRadians(0))
                .build();
        toParking2 = drive.trajectoryBuilder(toMidRepeat.end())
                .back(7)
                .build();
        toParking3 = drive.trajectoryBuilder(toMidRepeat.end(), Math.toRadians(-45))
                .splineToSplineHeading(toParking3OTW2, Math.toRadians(200))
                .splineToSplineHeading(toParking3OTW3, Math.toRadians(190))
                .build();
    }
}
