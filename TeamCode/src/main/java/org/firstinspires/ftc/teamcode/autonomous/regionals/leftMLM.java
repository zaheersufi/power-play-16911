package org.firstinspires.ftc.teamcode.autonomous.regionals;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.genericAuton;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


//@Disabled
@Autonomous(name="leftMLM")
public class leftMLM extends genericAuton
{
    private final Pose2d home = new Pose2d(-36.0, -63.0, Math.toRadians(90.0));


    private TrajectorySequence toMid;
    private TrajectorySequence toStack1;
    private TrajectorySequence backMid;
    private TrajectorySequence toLow;
    private TrajectorySequence lowBack;
    private TrajectorySequence toMedEnd;
    private TrajectorySequence medBack;
    private TrajectorySequence toParking1;
    private TrajectorySequence toParking2;
    private TrajectorySequence toParking3;
    public int cameraX = 50;
    public int cameraY = 50;




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
        utilities.liftArmAbsolutePosition(80);
        drive.followTrajectorySequence(toStack1);
//        utilities.wait(100,telemetry);
        utilities.openClaw(false);
        utilities.wait(700, telemetry);
        utilities.liftArmAbsolutePosition(185);
        utilities.wait(400,telemetry);

        //To low
        drive.followTrajectorySequence(toLow);
        utilities.liftArmDisplacementPosition(-45);
        utilities.wait(500, telemetry);
        utilities.openClaw(true);

        //To stack 2
        utilities.liftArmAbsolutePosition(65);
        drive.followTrajectorySequence(lowBack);
//        utilities.wait(100,telemetry);
        utilities.openClaw(false);
        utilities.wait(700, telemetry);
        utilities.liftArmAbsolutePosition(200);
        utilities.wait(750,telemetry);

        //To Medium
        utilities.liftArmAbsolutePosition(270);
        drive.followTrajectorySequence(toMedEnd);
        utilities.liftArmAbsolutePosition(240);
        utilities.wait(500, telemetry);
        utilities.openClaw(true);
        drive.followTrajectorySequence(medBack);
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
                .strafeRight(20.5)
                .forward(39)
                .turn(Math.toRadians(90))
                .forward(4)
                .build();
        backMid = drive.trajectorySequenceBuilder(toMid.end())
                .back(5.5)
                .build();
        toStack1 = drive.trajectorySequenceBuilder(backMid.end())
                .strafeRight(12.5)
                .forward(52)
                .build();
        toLow = drive.trajectorySequenceBuilder(toStack1.end())
                .back(7.5)
                .turn(Math.toRadians(115))
                .forward(8.5)
                .build();
        lowBack = drive.trajectorySequenceBuilder(toLow.end())
                .back(8.5)
                .turn(Math.toRadians(-115))
                .forward(7.5)
                .build();
        toMedEnd = drive.trajectorySequenceBuilder(lowBack.end())
                .back(38)
                .strafeLeft(2.5)
                .turn(Math.toRadians(90))
                .forward(6)
                .build();
        medBack = drive.trajectorySequenceBuilder(toMedEnd.end())
                .back(6)
                .turn(Math.toRadians(-90))
                .build();
        toParking1 = drive.trajectorySequenceBuilder(medBack.end())
                .forward(34)
                .build();
        toParking2 = drive.trajectorySequenceBuilder(medBack.end())
                .forward(10)
                .build();
        toParking3 = drive.trajectorySequenceBuilder(medBack.end())
                .back(14)
                .build();
    }


    public int cameraX()
    {
        return 80;
    }

    public int cameraY()
    {
        return 100;
    }

}
