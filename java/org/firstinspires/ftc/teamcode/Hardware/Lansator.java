package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware_Optimisations.OptimisedServo;

public class Lansator {

    Servo Lansator;

    OptimisedServo Lans = new OptimisedServo(Lansator);

    public void init(HardwareMap hw)
    {
        Lans.setName("Lansator",hw);
    }
    public void lansare(Gamepad gamepad)
    {
        if(gamepad.triangle)
            Lans.setPosition(0.5);
    }
}
