package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.Hardware.FieldCentric;
import org.firstinspires.ftc.teamcode.Hardware.Intake;
import org.firstinspires.ftc.teamcode.Hardware.Lansator;
import org.firstinspires.ftc.teamcode.Hardware.Outtake;

@TeleOp
public class Tele_Op extends LinearOpMode {

    FieldCentric dt = new FieldCentric();

    Intake intake = new Intake();
    Outtake outtake = new Outtake();

    Lansator lansator = new Lansator();


    @Override
    public void runOpMode()
    {
        dt.init(hardwareMap);
        intake.init(hardwareMap);
        outtake.init(hardwareMap);
        lansator.init(hardwareMap);

        intake.initPozInt();
        outtake.initPozOut();



        waitForStart();

        while(opModeIsActive()){

            dt.Mecanum(gamepad1);
            if(gamepad1.left_trigger > 0 && gamepad1.left_bumper) {
                intake.turnOnReverse();
            }
            else if (gamepad1.left_trigger > 0)
            {
              intake.turnOn();
            }
            else {
                intake.turnOff();
            }
            outtake.verificare();
            outtake.Servouri();
            if(gamepad2.dpad_up) {
                outtake.rid();
                outtake.pixelLevelIncrement();
            }
            if(gamepad2.dpad_down) {

                outtake.pixelLevelDecrement();
            }
            if (gamepad2.triangle) {
                outtake.cob();
            }

            outtake.setPid();
            if(gamepad2.right_bumper) {
                outtake.punerePanou();
            }
            if(gamepad2.cross) {
                outtake.lasare();
            }
            if(gamepad1.cross)
            {
                outtake.inchidere();
            }
            outtake.updt(telemetry);
            lansator.lansare(gamepad1);

        }
    }
    }