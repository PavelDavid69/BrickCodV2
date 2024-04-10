package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@TeleOp
public class TeleOpDedectie extends LinearOpMode {

    int poz = 0;
    public enum ElementPosition
    {
        LEFT,
        MIDDLE,
        RIGHT,
        NOT_FOUND
    }
    private HuskyLens huskyLens;

    ElementPosition position = ElementPosition.NOT_FOUND;
    private final int  READ_PERIOD = 1;

    @Override
    public void runOpMode(){

        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();

        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);


        waitForStart();
        while (opModeIsActive())
        {
            if (!rateLimit.hasExpired()) {
                continue;
            }
            rateLimit.reset();

            HuskyLens.Block[] blocks = huskyLens.blocks();
            int x = -1;
            if (blocks.length == 0) {
                this.position = ElementPosition.LEFT;
            } else {
                {
                    for (int i = 0; i < blocks.length; i++) {
                        if(blocks[i].id == 1){
                            x = blocks[i].x;
                        }
                    }
                }

            }
            if(x > 200)
                this.position = ElementPosition.RIGHT;
            if(x < 200 && x > 120)
                this.position = ElementPosition.MIDDLE;

            if(position == ElementPosition.RIGHT)
                poz = 1;

            else if (position == ElementPosition.MIDDLE)
                poz = 2;
            else if(position == ElementPosition.LEFT)
                poz = 3;

              telemetry.addData("Pozitie", poz);
              telemetry.update();

            }
        }

    }

