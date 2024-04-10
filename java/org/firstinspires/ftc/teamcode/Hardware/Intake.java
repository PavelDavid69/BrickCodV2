package org.firstinspires.ftc.teamcode.Hardware;

import org.firstinspires.ftc.teamcode.Hardware_Optimisations.OptimisedMotor;
import org.firstinspires.ftc.teamcode.Hardware_Optimisations.OptimisedServo;
import org.firstinspires.ftc.teamcode.Constant;
import org.firstinspires.ftc.teamcode.Hardware.Outtake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Intake {

    DcMotorEx intakeM;
    public OptimisedMotor spinner = new OptimisedMotor(intakeM);

    Servo leftservo;
    OptimisedServo leftangleintake = new OptimisedServo(leftservo);

    public void init(HardwareMap hwMap) {
        spinner.setName("Spinner", hwMap);
        spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinner.setDirection(DcMotorSimple.Direction.REVERSE);

        leftangleintake.setName("leftangle", hwMap);




    }

    public void initPozInt()
    {
        leftangleintake.setPosition(Constant.pixelStack);
    }
    public void turnOn() {
            leftangleintake.setPosition(Constant.angleSol);
            spinner.setPower(1);
        }
        public void turnOnReverse()
        {
            leftangleintake.setPosition(Constant.angleSol);
            spinner.setPower(-1);
        }
        public void turnOff()
        {
            leftangleintake.setPosition(Constant.pixelStack);
            spinner.setPower(0);
        }
        public void turnOnStack()
        {
            leftangleintake.setPosition(Constant.pixelStack);
            spinner.setPower(1);
        }
    }





