package org.firstinspires.ftc.teamcode.autonomous.current;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


//@Disabled
@Autonomous(name="rightAltLinear")
public class rightAltLinear extends genericAuton
{
    private final Pose2d home = new Pose2d(-36.0, -63.0, Math.toRadians(90.0));
    private final Pose2d middleToLow = new Pose2d(-55, -12, Math.toRadians(180));
    private final Pose2d endToLow = new Pose2d(-47, -19.5, Math.toRadians(270));
    private final Pose2d endToStack = new Pose2d(-60, -12, Math.toRadians(180));



    private TrajectorySequence toMid;
    private TrajectorySequence toStack1;
    private TrajectorySequence backMid;
    private TrajectorySequence toLow;
    private TrajectorySequence lowBack;
    private TrajectorySequence toStack2;
    private TrajectorySequence toHigh;
    private TrajectorySequence highBack;
    private Trajectory toLowSpline;
    private Trajectory splineToStack2;
    private Trajectory toLowLinear;
    private TrajectorySequence toParking1;
    private TrajectorySequence toParking2;
    private TrajectorySequence toParking3;

    /**
     * Reads the parking position, scores a cone in the
     * mid junction then grabs cone then scores on low
     * junction then grabs another cone and scores on
     * high junction and parks in the space determined
     * by the custom sleeve.
     */
    @Override
    public void run()
    {
        drive.setPoseEstimate(home);

        //wait time to scan camera
        utilities.wait(250, telemetry);

        //Mid junction
        utilities.liftArmAbsolutePosition(270);
        drive.followTrajectorySequence(toMid);
        utilities.liftArmDisplacementPosition(-30);
        utilities.wait(500, telemetry);
        utilities.openClaw(true);
        drive.followTrajectorySequence(backMid);

        //To stack
        utilities.liftArmAbsolutePosition(80);
        drive.followTrajectorySequence(toStack1);
        utilities.wait(100,telemetry);
        utilities.openClaw(false);
        utilities.liftArmAbsolutePosition(170);
        utilities.wait(750,telemetry);

        //To low
        //utilities.liftArmAbsolutePosition(170);
        //drive.followTrajectorySequence(toLow);
//        drive.followTrajectory(toLowSpline);
        drive.followTrajectorySequence(toLow);
        utilities.liftArmDisplacementPosition(-30);
        utilities.wait(500, telemetry);
        utilities.openClaw(true);


        //To stack
        utilities.liftArmAbsolutePosition(75);
        drive.followTrajectorySequence(lowBack);
//        drive.followTrajectorySequence(toStack2);
//        drive.followTrajectory(splineToStack2);
        utilities.wait(100,telemetry);
        utilities.openClaw(false);
        utilities.liftArmAbsolutePosition(200);
        utilities.wait(750,telemetry);

        //To High
        utilities.liftArmAbsolutePosition(370);
        drive.followTrajectorySequence(toHigh);
        utilities.liftArmAbsolutePosition(325);
        utilities.wait(500, telemetry);
        utilities.openClaw(true);
        drive.followTrajectorySequence(highBack);
        utilities.liftArmAbsolutePosition(100);

        //Parking
        if(identifier == 1)
            drive.followTrajectorySequence(toParking1);
        else if (identifier == 2)
            drive.followTrajectorySequence(toParking2);
        else if (identifier == 3)
            drive.followTrajectorySequence(toParking3);
    }

    /**
     * Defines and builds the different trajectories used.
     */
    public void buildTrajectories()
    {
        toMid = drive.trajectorySequenceBuilder(home)
                .forward(2)
                .strafeLeft(25)
                .forward(39)
                .turn(Math.toRadians(-90))
                .forward(4.75)
                .build();
        backMid = drive.trajectorySequenceBuilder(toMid.end())
                .back(5)
                .build();
        toStack1 = drive.trajectorySequenceBuilder(backMid.end())
                .strafeLeft(12.5)
                .forward(51.00)
                .build();
        toLow = drive.trajectorySequenceBuilder(toStack1.end())
                .back(6.5)
                .turn(Math.toRadians(-115))
                .forward(8.5)
                .build();
//        toLowSpline = drive.trajectoryBuilder(toStack1.end(), Math.toRadians(0))
//                .back(5)
//                .splineToSplineHeading(endToLow, Math.toRadians(280))
//                .build();
        splineToStack2 = drive.trajectoryBuilder(toLow.end(), Math.toRadians(120))
                .splineToSplineHeading(endToStack, Math.toRadians(-120))
                .build();
        lowBack = drive.trajectorySequenceBuilder(toLow.end())
                .back(8.5)
                .turn(Math.toRadians(115))
                .forward(6.5)
                .build();
        toStack2 = drive.trajectorySequenceBuilder(lowBack.end())
                .turn(Math.toRadians(90))
                .strafeLeft(2.0)
                .forward(15.25)
                .build();
        toHigh = drive.trajectorySequenceBuilder(lowBack.end())
                .back(40)
                .strafeRight(3)
                .turn(Math.toRadians(90))
                .forward(4)
                .build();
        highBack = drive.trajectorySequenceBuilder(toHigh.end())
                .back(4)
                .turn(Math.toRadians(-90))
                .build();
        toParking1 = drive.trajectorySequenceBuilder(highBack.end())
                .forward(36)
                .build();
        toParking2 = drive.trajectorySequenceBuilder(highBack.end())
                .forward(12)
                .build();
        toParking3 = drive.trajectorySequenceBuilder(highBack.end())
                .back(12)
                .build();
    }

}
