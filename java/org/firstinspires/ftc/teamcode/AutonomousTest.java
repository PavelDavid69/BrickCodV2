/*/package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.Outtake;
import org.firstinspires.ftc.teamcode.Hardware.Intake;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class AutonomousTest extends LinearOpMode {

    enum Randomisation {
        LEFT,
        MIDDLE,
        RIGHT
    }

    //Randomisation randomisation = Randomisation.LEFT;







    Intake intake = new Intake();
    Outtake outtake = new Outtake();

    @Override
    public void runOpMode() {

        intake.init(hardwareMap);
        outtake.init(hardwareMap);
        outtake.Servouri();

        outtake.inchidere();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(54, -150, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(60, -73),Math.toRadians(90))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(60,-100,0),Math.toRadians(90))
                .setReversed(false)
                .splineTo(new Vector2d(164.4, -87), 0)
                .setReversed(true)
                .splineTo(new Vector2d(90, -30), Math.toRadians(180))
                .setReversed(true)
                .setReversed(true)
                .splineTo(new Vector2d(-139 -29), 0)
                .setReversed(true)
                .addDisplacementMarker(() ->{
                    outtake.pixelLevelPreload();
                    outtake.setPid();
                })
                .build();


        waitForStart();

        while(opModeIsActive()) {
            drive.update();

            switch (currentstate)
            {
                case detect:

                    drive.followTrajectorySequence(traj1);
                    currentstate = AutoStages.goToBackdrop;
                    break;

                case goToBackdrop:
                    if(!drive.isBusy())
                    {
                        currentstate = AutoStages.putPreLoad;
                    }
                    break;

                case ridicare:
                    if(!outtake.busy)
                    {
                        currentstate = AutoStages.putPreLoad;
                    }
                case putPreLoad:
                {
                    outtake.punerePanou();
                    outtake.Servouri();
                    outtake.lasare();
                    currentstate = AutoStages.dropPreLoad;
                }
                case dropPreLoad:
                {
                        outtake.lasare();
                        outtake.Servouri();
                        outtake.cob();
                }

            }
        }






        void chooseBackdropPath() {
            if (randomisation == Randomisation.LEFT)
                drive.followTrajectorySequenceAsync(goToBackdropLeft);
            else if (randomisation == Randomisation.MIDDLE)
                drive.followTrajectorySequenceAsync(goToBackdropMid);
            else if(randomisation == Randomisation.RIGHT)
                drive.followTrajectorySequenceAsync(goToBackdropRight);
        }
    }
}
/*/

