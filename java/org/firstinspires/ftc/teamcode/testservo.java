package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class testservo extends LinearOpMode {

    Servo servo1;
    Servo servo2;
    Servo servo3;

    @Override
    public void runOpMode() {

        servo1 = hardwareMap.get(Servo.class,"servo1");
        servo2 = hardwareMap.get(Servo.class,"servo2");
        servo3 = hardwareMap.get(Servo.class,"servo3");
        double poz1 = 0.00;
        double poz2 = 0.00;
        waitForStart();
        servo1.setPosition(0.23);
        servo2.setPosition(0.77);
        while (opModeIsActive())
        {

            if(gamepad1.dpad_left)
            {
                poz1 += 0.001;
            }
            if(gamepad1.dpad_right)
                poz1 -= 0.001;
            if(gamepad1.cross)
            {
                servo1.setPosition(0.23);
                servo2.setPosition(0.77);
            }
            if(gamepad1.circle)
            {
                servo1.setPosition(0.60);
                servo2.setPosition(0.39);
            }
            if(gamepad1.triangle)
            {
                servo1.setPosition(0.88);
                servo2.setPosition(0.12);
            }
            servo3.setPosition(0.5 + poz1);
            telemetry.addData("Pozitie servo 1",servo1.getPosition());
            telemetry.addData("Pozitie servo 2",servo2.getPosition());
            telemetry.addData("Pozitie servo 3",servo3.getPosition());
            telemetry.update();
        }
    }
}
