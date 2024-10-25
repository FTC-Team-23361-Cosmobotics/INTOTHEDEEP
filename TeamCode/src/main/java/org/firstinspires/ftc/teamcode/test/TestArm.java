package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
@Config
@TeleOp(group = "Testing")
public class TestArm extends OpMode {
    private DcMotorEx armMotor;

    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0;


    // TODO: Maybe set beginning to zero
    //Safe, Intaking, Parallels (.5, 1st, 1.5, 2nd, 2.5, 3rd, 3.5), Hang
    public int[] armPositions = {0, 3000, 300, 450, 600, 750, 900, 1050, 1200, 1500};

    public static int state = 0;
    @Override
    public void init() {
        armMotor = hardwareMap.get(DcMotorEx.class, "ArmMotor");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(kP, kI, kD, kF));
    }

    @Override
    public void loop() {
        armMotor.setPower(1);
//        telemetry.addLine("Directions: Press each button to move slide to its respective position");
//        if (gamepad1.dpad_down) state = 0; // Safe
//        if (gamepad1.dpad_left) state = 1; // Intaking
//        if (gamepad1.dpad_up) state = 2; // .5
//        if (gamepad1.dpad_right) state = 3; // 1st
//        if (gamepad1.a) state = 4; // 1.5
//        if (gamepad1.x) state = 5; // 2nd
//        if (gamepad1.y) state = 6; // 2.5
//        if (gamepad1.b) state = 7; // 3rd
//        if (gamepad1.left_bumper) state = 8; //3.5
//        if (gamepad1.right_bumper) state = 9; //Hang

        armMotor.setTargetPosition(state);
        telemetry.addData("target-pos: ", state);
        telemetry.addData("pos: ", armMotor.getCurrentPosition());
    }
}

