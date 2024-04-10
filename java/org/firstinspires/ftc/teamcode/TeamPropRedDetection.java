package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class TeamPropRedDetection {


    private HuskyLens huskyLens;

    public enum ElementPosition
    {
        LEFT,
        MIDDLE,
        RIGHT,
        NOT_FOUND

    }
    public int poz = 0;
     ElementPosition position;

    public void init(HardwareMap hwMap) {
        huskyLens = hwMap.get(HuskyLens.class, "huskylens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
    }

    public void Detection() {
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
    }

    }
