package org.firstinspires.ftc.teamcode.autonomous.meet3;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.genericAuton;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


//@Disabled
@Autonomous(name="rightThree")
public class rightThree extends genericAuton
{
    private final Pose2d home = new Pose2d(-36.0, -63.0, Math.toRadians(90.0));


    private TrajectorySequence toMid;
    private TrajectorySequence toStack1;
    private TrajectorySequence backMid;
    private TrajectorySequence toLow;
    private TrajectorySequence lowBack;
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


        //Mid junction
        utilities.liftArmAbsolutePosition(270);
        drive.followTrajectorySequence(toMid);
        utilities.liftArmDisplacementPosition(-30);
        utilities.wait(500, telemetry);
        utilities.openClaw(true);
        drive.followTrajectorySequence(backMid);

        //To stack 1
        utilities.liftArmAbsolutePosition(65);
        drive.followTrajectorySequence(toStack1);
//        utilities.wait(100,telemetry);
        utilities.openClaw(false);
        utilities.wait(200, telemetry);
        utilities.liftArmAbsolutePosition(185);
        utilities.wait(750,telemetry);

        //To low
        drive.followTrajectorySequence(toLow);
        utilities.liftArmDisplacementPosition(-45);
        utilities.wait(500, telemetry);
        utilities.openClaw(true);

        //To stack 2
        utilities.liftArmAbsolutePosition(60);
        drive.followTrajectorySequence(lowBack);
//        utilities.wait(100,telemetry);
        utilities.openClaw(false);
        utilities.wait(200, telemetry);
        utilities.liftArmAbsolutePosition(200);
        utilities.wait(750,telemetry);

        //To High
        utilities.liftArmAbsolutePosition(360);
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
                .strafeLeft(24)
                .forward(35.5)
                .turn(Math.toRadians(-90))
                .forward(4)
                .build();
        backMid = drive.trajectorySequenceBuilder(toMid.end())
                .back(5.1)
                .build();
        toStack1 = drive.trajectorySequenceBuilder(backMid.end())
                .strafeLeft(12.5)
                .forward(51.00)
                .build();
       toLow = drive.trajectorySequenceBuilder(toStack1.end())
                .back(8)
                .turn(Math.toRadians(-130))
                .forward(5.5)
                .build();
        lowBack = drive.trajectorySequenceBuilder(toLow.end())
                .back(5.5)
                .turn(Math.toRadians(130))
                .forward(7)
                .build();
        toHigh = drive.trajectorySequenceBuilder(lowBack.end())
                .back(37)
//                .strafeRight(1)
                .turn(Math.toRadians(90))
                .forward(4.25)
                .build();
        highBack = drive.trajectorySequenceBuilder(toHigh.end())
                .back(5)
                .turn(Math.toRadians(-90))
                .build();
        toParking1 = drive.trajectorySequenceBuilder(highBack.end())
                .back(14)
                .build();
        toParking2 = drive.trajectorySequenceBuilder(highBack.end())
                .forward(9)
                .build();
        toParking3 = drive.trajectorySequenceBuilder(highBack.end())
                .forward(36)
                .build();
    }

}
