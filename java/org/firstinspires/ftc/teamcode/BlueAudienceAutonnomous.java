package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.teamcode.AutonomousBlueBackdrop.AutoStages.parc2;
import static org.firstinspires.ftc.teamcode.AutonomousBlueBackdrop.AutoStages.stop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Hardware.Outtake;
import org.firstinspires.ftc.teamcode.Hardware.Intake;
@Autonomous
public class BlueAudienceAutonnomous  extends LinearOpMode {
    ElapsedTime delay = new ElapsedTime();

    enum AutoStages {
        detect,
        goToBackdrop,
        ridicare,
        putPreLoad,
        dropPreLoad,
        motorInit,
        stackSequence,
        luare,
        back,
        ridica2,
        punere2,
        coboara2,
        park,
        parc2,
        stop
    }

    Pose2d backdrop = new Pose2d(159.4, -87, 0);
    Pose2d stack = new Pose2d(0, 0, 0);

    Pose2d stackint = new Pose2d(-118, -30, 0);

    AutonomousRedBackdrop.AutoStages currentstate = AutonomousRedBackdrop.AutoStages.detect;


    TeamPropRedDetection detection = new TeamPropRedDetection();
    TrajectorySequence traiectorie = null;
    Outtake outtake = new Outtake();
    Intake intake = new Intake();
    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        detection.init(hardwareMap);
        outtake.init(hardwareMap);
        outtake.initPozOut();
        intake.init(hardwareMap);

        delay.startTime();

        waitForStart();
        while (opModeIsActive()) {

            delay.reset();
            Pose2d startPose = new Pose2d(-60, 154, Math.toRadians(270));


            drive.setPoseEstimate(startPose);


            TrajectorySequence Dreapta = drive.trajectorySequenceBuilder(startPose)
                    .splineTo(new Vector2d(-90, 70), Math.toRadians(270))
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(-90, 150, 0), Math.toRadians(270))
                    .setReversed(false)
                    .splineTo(new Vector2d(60, 150), 0)
                    .splineTo(new Vector2d(164.4, 150), 0)
                    .build();

            TrajectorySequence Stanga = drive.trajectorySequenceBuilder(startPose)
                    .splineTo(new Vector2d(-60, 120), Math.toRadians(300))
                    .splineTo(new Vector2d(-30, 70),Math.toRadians(300))
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(-60, 150, 0), Math.toRadians(0))
                    .setReversed(false)
                    .splineTo(new Vector2d(60, 150), 0)
                    .splineTo(new Vector2d(164.4, 150), 0)
                    .build();

            TrajectorySequence Centru = drive.trajectorySequenceBuilder(startPose)
                    .splineTo(new Vector2d(-60, 60), Math.toRadians(270))
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(-60, 150, 0), Math.toRadians(270))
                    .setReversed(false)
                    .splineTo(new Vector2d(60, 150), 0)
                    .splineTo(new Vector2d(163.4, 150), 0)
                    .build();
            TrajectorySequence stackSequence = drive.trajectorySequenceBuilder(backdrop)
                    .back(4)
                    .strafeLeft(30)
                    .build();

            TrajectorySequence backDrop = drive.trajectorySequenceBuilder(stackSequence.end())
                    .splineToConstantHeading(new Vector2d(50, -20), Math.toRadians(0))
                    .splineTo(new Vector2d(164.4, -70), 0)
                    //        .addDisplacementMarker(() ->
                    //       {
                    //         outtake.pixelLevelPreload();
                    //         outtake.setPid();
                    //      })
                    .build();
            TrajectorySequence back = drive.trajectorySequenceBuilder(stackSequence.end())
                    .splineToConstantHeading(new Vector2d(50, -20), Math.toRadians(0))


                    .splineTo(new Vector2d(164.4, -100), 0)
                    .build();
            TrajectorySequence parc = drive.trajectorySequenceBuilder(backdrop)
                    .strafeLeft(40)
                    .build();


            detection.Detection();

            if (detection.poz == 1) {
                traiectorie = Dreapta;
            } else if (detection.poz == 2) {
                traiectorie = Centru;
            } else {
                traiectorie = Stanga;
            }

            switch (currentstate) {
                case detect:

                    drive.followTrajectorySequence(traiectorie);
                    currentstate = AutonomousRedBackdrop.AutoStages.goToBackdrop;
                    break;

                case goToBackdrop:
                    if (!drive.isBusy()) {
                        currentstate = AutonomousRedBackdrop.AutoStages.parc2;

                        backdrop = traiectorie.end();
                    }

                case ridicare: {
                    if (!outtake.asteptare()) {
                        currentstate = AutonomousRedBackdrop.AutoStages.putPreLoad;
                    }
                }

                case putPreLoad: {
                    outtake.punerePanou();
                    outtake.Servouri();
                    outtake.lasare();
                    currentstate = AutonomousRedBackdrop.AutoStages.dropPreLoad;
                }
                case dropPreLoad: {
                    outtake.punerePanou();
                    outtake.Servouri();
                    outtake.lasare();
                    outtake.Servouri();
                    currentstate = AutonomousRedBackdrop.AutoStages.motorInit;
                }
                case motorInit: {
                    sleep(800);
                    outtake.cob();
                    outtake.Servouri();
                    sleep(200);
                    outtake.setPid();
                    outtake.cob();
                    outtake.Servouri();
                    sleep(800);
                    if (!outtake.asteptare())
                        currentstate = AutonomousRedBackdrop.AutoStages.parc2;
                }
                case parc2: {
                    drive.setPoseEstimate(backdrop);
                    drive.followTrajectorySequence(stackSequence);
                    currentstate = AutonomousRedBackdrop.AutoStages.stop;
                }
                /*/case stackSequence:
                    drive.setPoseEstimate(backdrop);
                    drive.followTrajectorySequence(stackSequence);
                    currentstate = AutoStages.luare;/*/
                /*/case luare:
                if(!drive.isBusy()) {
                        stack = stackSequence.end();
                        intake.turnOnStack();
                        sleep(1300);
                        outtake.inchidere();
                        outtake.Servouri();
                        sleep(250);
                        intake.turnOnReverse();
                        sleep(200);
                        currentstate = AutoStages.back;
                    }
                    break;
                case back:
                    intake.turnOff();
                    drive.setPoseEstimate(stackint);
                    drive.followTrajectorySequence(back);
                    if(!drive.isBusy()) {
                        outtake.rid();
                        outtake.pixelLevelPreload();
                        outtake.setPid();
                        sleep(200);
                        currentstate = AutoStages.ridica2;
                    }
                break;
                case ridica2: {
                    if (!outtake.asteptare()) {
                        currentstate=AutoStages.punere2;
                    }
                }
                case punere2: {
                    outtake.punerePanou();
                    outtake.Servouri();
                    sleep(600);
                    outtake.lasare();
                    outtake.Servouri();
                    sleep(300);
                    currentstate=AutoStages.coboara2;
                }
                case coboara2:{

                    outtake.cob();
                    outtake.Servouri();
                    sleep(200);
                    outtake.setPid();
                    outtake.cob();
                    outtake.Servouri();
                    sleep(800);
                    if(!outtake.asteptare())
                        currentstate = AutoStages.park;
            }
                case park:
                    break;/*/
                case stop:
                    if (!drive.isBusy()) {
                        stop();
                    }

            }


            outtake.updt(telemetry);
            //telemetry.addData("Caz",currentstate);
            //telemetry.update();

        }
    }



}
