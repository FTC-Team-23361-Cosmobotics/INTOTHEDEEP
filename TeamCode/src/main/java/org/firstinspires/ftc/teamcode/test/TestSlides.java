package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
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
public class TestSlides extends OpMode {
    private DcMotorEx slidesMotor;

    public static PIDController slides_controller;
    public static double slides_p = 0, slides_i = 0, slides_d = 0;
    public static double slides_f = 0;
    public static int slidesTarget = 0;
    public static final double slides_ticks_in_degrees = 537.7;
    public static double slidesPos;


    // TODO: Maybe set beginning to zero
    //Safe, Close, Med, Far
    public static final int[] slidesPositions = {0, 1000, 2000, 3000};

    public static int state = 0;
    @Override
    public void init() {
        slidesMotor = hardwareMap.get(DcMotorEx.class, "slidesMotor");
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesMotor.setTargetPosition(state);
        slidesMotor.setPower(1);
        slidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slidesPos = slidesMotor.getCurrentPosition();
    }

    @Override
    public void loop() {
        slides_controller = new PIDController(slides_p, slides_i, slides_d);
        slides_controller.setPID(slides_p, slides_i, slides_d);
        double slides_pid = slides_controller.calculate(slidesPos, slidesTarget);
        double slides_ff = Math.cos(Math.toRadians(slidesTarget / slides_ticks_in_degrees)) * slides_f;
        double slidesPower = slides_pid + slides_ff;
        slidesMotor.setPower(slidesPower);
    }
}


