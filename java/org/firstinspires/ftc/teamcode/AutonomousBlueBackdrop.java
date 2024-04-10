package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.AutonomousRedBackdrop.AutoStages.park;
import static org.firstinspires.ftc.teamcode.AutonomousRedBackdrop.AutoStages.punere2;
import static org.firstinspires.ftc.teamcode.AutonomousRedBackdrop.AutoStages.ridica2;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Intake;
import org.firstinspires.ftc.teamcode.Hardware.Lansator;
import org.firstinspires.ftc.teamcode.Hardware.Outtake;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Autonomous
public class AutonomousBlueBackdrop extends LinearOpMode {
    ElapsedTime delay = new ElapsedTime();
    enum AutoStages {
        detect,
        goToBackdrop,
        ridicare,
        putPreLoad,
        dropPreLoad,
        motorInit,
        coborare,
        stackSequence,
        stack,
        back,
        ridica2,
        punere2,
        coboara2,
        luare,
        park,
        parc2,
        stop
    }
    Pose2d backdrop = new Pose2d(159.4, -87,0);
    Pose2d stack = new Pose2d(0,0,0);

    Pose2d stackint = new Pose2d(-118, -30, 0);

    AutoStages currentstate = AutoStages.detect;


    TeamPropBlueDetection detection = new TeamPropBlueDetection();
    TrajectorySequence traiectorie = null;
    Outtake outtake = new Outtake();
    Intake intake = new Intake();
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        detection.init(hardwareMap);
        outtake.init(hardwareMap);
        intake.init(hardwareMap);
        outtake.initPozOut();

        delay.startTime();

        waitForStart();
        while (opModeIsActive()){

        delay.reset();
        Pose2d startPose = new Pose2d(54, 150, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        TrajectorySequence Centru = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(54, 75),Math.toRadians(270))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(54,90,Math.toRadians(0)),Math.toRadians(0))
                .setReversed(false)
                .splineTo(new Vector2d(162, 80), 0)
                .addDisplacementMarker(() ->
                {
                    outtake.pixelLevelPreload();
                    outtake.setPid();
                })
                //.setReversed(true)
                /*/
                .splineTo(new Vector2d(0 , 20),Math.toRadians((180)))

                 */
                .build();

        TrajectorySequence Dreapta = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(54,120),Math.toRadians(270))
                .splineTo(new Vector2d(27,90), Math.toRadians(240))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(88,85,0),Math.toRadians(90))
                .setReversed(false)
                .splineTo(new Vector2d(164.4, 70), 0)
                .addDisplacementMarker(() ->
                {
                    outtake.pixelLevelPreload();
                    outtake.setPid();
                })
                /*/
                .setReversed(true)
                .splineTo(new Vector2d(0 , 25),Math.toRadians((180)))

                 */
                .build();

        TrajectorySequence Stanga = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(80, 90),Math.toRadians(270))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(77,110,0),Math.toRadians(300))
                .setReversed(false)
                .splineTo(new Vector2d(164.4, 100), 0)
                .addDisplacementMarker(() ->
                {
                    outtake.pixelLevelPreload();
                    outtake.setPid();
                })

                /*/
                .setReversed(true)
                .splineTo(new Vector2d(100, 20), Math.toRadians(180))
                .setReversed(true)
                .splineTo(new Vector2d(0, 30), Math.toRadians(180))
                .setReversed(true)
                /*.setReversed(true)
                .splineTo(new Vector2d(-139 -29), 0)
                .setReversed(true)*/
                /*.addDisplacementMarker(() ->{
                    outtake.pixelLevelPreload();
                    outtake.setPid();
                })*/
                .build();
                /*
            TrajectorySequence stackSequence = drive.trajectorySequenceBuilder(backdrop)
                    .splineToConstantHeading(new Vector2d( 50, 40),Math.toRadians(0))
                    .setReversed(true)
                    .splineTo(new Vector2d(-118,30),Math.toRadians(180))
                    .build();
            TrajectorySequence backDrop = drive.trajectorySequenceBuilder(stackSequence.end())
                    .splineToConstantHeading(new Vector2d( 50, 20),Math.toRadians(0))
                    .splineTo(new Vector2d(164.4, 110),0)
                    //        .addDisplacementMarker(() ->
                    //       {
                    //         outtake.pixelLevelPreload();
                    //         outtake.setPid();
                    //      })
                    .build();
            TrajectorySequence back = drive.trajectorySequenceBuilder(stackSequence.end())
                    .splineToConstantHeading(new Vector2d( 50, 20),Math.toRadians(0))
                    .splineTo(new Vector2d(164.4,100),0)
                    .build();

                 */
            TrajectorySequence parc2 = drive.trajectorySequenceBuilder(backdrop)
                            .strafeLeft(30)
                            .build();
            detection.Detection();

            if (detection.poz == 1) {
                traiectorie = Dreapta;
            }
            else if (detection.poz == 2) {
                traiectorie = Centru;
            } else {
                traiectorie = Stanga;
            }

            switch (currentstate) {
                case detect:

                    drive.followTrajectorySequence(traiectorie);
                    currentstate = AutoStages.goToBackdrop;
                    break;

                case goToBackdrop:
                    if (!drive.isBusy()) {
                        currentstate = AutoStages.ridicare;
                        backdrop = traiectorie.end();
                    }

                case ridicare: {
                    if (!outtake.asteptare()) {


                        currentstate = AutoStages.coborare;
                        outtake.pixelLevelPreload();
                        outtake.setPid();
                    }
                }
                case coborare:
                    if (!outtake.asteptare()) {
                        currentstate = AutoStages.putPreLoad;
                    }

                case putPreLoad: {
                    outtake.punerePanou();
                    outtake.Servouri();
                    outtake.lasare();
                    currentstate = AutoStages.dropPreLoad;
                }
                case dropPreLoad: {
                    outtake.punerePanou();
                    outtake.Servouri();
                    outtake.lasare();
                    outtake.Servouri();
                    currentstate = AutoStages.motorInit;
                }
                case motorInit:
                {
                    sleep(1000);
                    outtake.cob();
                    outtake.Servouri();
                    sleep(200);
                    outtake.setPid();
                    outtake.cob();
                    outtake.Servouri();
                    sleep(1000);
                  //  if(!outtake.asteptare())
                        currentstate = AutoStages.stop;
                }
                case parc2:
                    drive.setPoseEstimate(backdrop);
                    drive.followTrajectorySequence(parc2);
                    if(!drive.isBusy())
                    currentstate = AutoStages.stop;
                    break;
             /*   case stackSequence:
                    drive.setPoseEstimate(backdrop);
                    drive.followTrajectorySequence(stackSequence);
                    currentstate = AutoStages.luare;
                    break;
                case luare:
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
                        currentstate= AutoStages.punere2;
                    }
                }
                case punere2: {
                    outtake.punerePanou();
                    outtake.Servouri();
                    sleep(600);
                    outtake.lasare();
                    outtake.Servouri();
                    sleep(300);
                    currentstate= AutoStages.coboara2;
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

              */
                case park:

                case stop:
                    if(!drive.isBusy())
                    {
                        stop();
                    }

            }


            outtake.updt(telemetry);
            telemetry.addData("Caz",currentstate);
            telemetry.update();

        }



            telemetry.addData("Pozitie",detection.poz);
            telemetry.update();

        }


    }



