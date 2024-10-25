//TODO: Turn into Autoturn PIDF Tuner
//package org.firstinspires.ftc.teamcode.test;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.teleop.Robot;
//
//
//@Config
//@TeleOp
//public class PIDFTuner extends LinearOpMode {
//    @Override
//    public void runOpMode() {
//        Robot robot;
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        waitForStart();
//        if (isStopRequested()) return;
//        while (opModeIsActive()) {
//            robot = new Robot(this);
//            double target = robot.armTarget;
//            telemetry.addData("pos ", robot.armPos);
//            telemetry.addData("target ", target);
//        }
//
//    }
//}
