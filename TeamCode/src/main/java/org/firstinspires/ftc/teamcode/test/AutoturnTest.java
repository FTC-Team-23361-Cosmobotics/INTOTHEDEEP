//package org.firstinspires.ftc.teamcode.test;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.teamcode.teleop.drive.Drive;
//import org.firstinspires.ftc.teamcode.teleop.misc.Misc;
//import org.firstinspires.ftc.teamcode.teleop.transport.Transport;
//import org.firstinspires.ftc.teamcode.teleop.utils.Controls;
//
//import java.util.ArrayList;
//import java.util.Arrays;
//import java.util.List;
//@Config
//@TeleOp(group = "Testing")
//public class AutoturnTest extends OpMode {
//    public static double kP;
//    public static double kI;
//    public static double kD;
//
//    private Drive drive;
//    private Transport transport;
//    private Misc misc;
//    private Controls controls;
//    @Override
//    public void init() {
//        drive = new Drive(hardwareMap);
//        transport = new Transport(hardwareMap);
//        misc = new Misc(hardwareMap);
//        controls = new Controls(gamepad1, gamepad2, drive, transport, misc);
//    }
//
//    @Override
//    public void loop() {
//        Drive.kP = kP;
//        Drive.kI = kI;
//        Drive.kD = kD;
//        drive.update(telemetry);
//        controls.update(telemetry);
//    }
//}