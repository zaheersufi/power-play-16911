package org.firstinspires.ftc.teamcode.autonomous.current;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


//@Disabled
@Autonomous(name="leftAltLinear")
public class leftAltLinear extends genericAuton
{
    private final Pose2d home = new Pose2d(-36.0, -63.0, Math.toRadians(90.0));

    private TrajectorySequence toMid;
    private TrajectorySequence toStack1;
    private TrajectorySequence backMid;
    private TrajectorySequence toLow;
    private TrajectorySequence lowBack;
    private TrajectorySequence toStack2;
    private TrajectorySequence toHigh;
    private TrajectorySequence highBack;
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
        utilities.liftArmAbsolutePosition(200);
        utilities.wait(750,telemetry);

        //To low
        utilities.liftArmAbsolutePosition(170);
        drive.followTrajectorySequence(toLow);
        utilities.liftArmDisplacementPosition(-30);
        utilities.wait(500, telemetry);
        utilities.openClaw(true);
        drive.followTrajectorySequence(lowBack);

        //To stack
        utilities.liftArmAbsolutePosition(75);
        drive.followTrajectorySequence(toStack2);
        utilities.wait(100,telemetry);
        utilities.openClaw(false);
        utilities.liftArmAbsolutePosition(200);
        utilities.wait(750,telemetry);

        //To High
        utilities.liftArmAbsolutePosition(355);
        drive.followTrajectorySequence(toHigh);
        utilities.liftArmAbsolutePosition(325);
        utilities.wait(500, telemetry);
        utilities.openClaw(true);
        drive.followTrajectorySequence(highBack);
        utilities.liftArmAbsolutePosition(100);

//        //Parking
//        if(identifier == 1)
//            drive.followTrajectorySequence(toParking1);
//        else if (identifier == 2)
//            drive.followTrajectorySequence(toParking2);
//        else if (identifier == 3)
//            drive.followTrajectorySequence(toParking3);
    }

    /**
     * Defines and builds the different trajectories used.
     */
    public void buildTrajectories()
    {
        toMid = drive.trajectorySequenceBuilder(home)
                .forward(2)
                .strafeRight(25)
                .forward(39)
                .turn(Math.toRadians(90))
                .forward(4.75)
                .build();
        backMid = drive.trajectorySequenceBuilder(toMid.end())
                .back(5)
                .build();
        toStack1 = drive.trajectorySequenceBuilder(backMid.end())
                .strafeRight(12.5)
                .forward(51.00)
                .build();
        toLow = drive.trajectorySequenceBuilder(toStack1.end())
                .back(13)
                .strafeLeft(3)
                .turn(Math.toRadians(90))
                .strafeLeft(1)
                .forward(4.5)
                .build();
        lowBack = drive.trajectorySequenceBuilder(toLow.end())
                .back(4.5)
                .build();
        toStack2 = drive.trajectorySequenceBuilder(lowBack.end())
                .turn(Math.toRadians(-90))
                .strafeRight(2.0)
                .forward(12.25)
                .build();
        toHigh = drive.trajectorySequenceBuilder(toStack2.end())
                .back(38)
                .strafeLeft(3)
                .turn(Math.toRadians(-90))
                .forward(4)
                .build();
        highBack = drive.trajectorySequenceBuilder(toHigh.end())
                .back(4)
                .turn(Math.toRadians(90))
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
