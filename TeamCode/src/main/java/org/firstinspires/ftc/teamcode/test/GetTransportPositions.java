package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
@Config
@TeleOp(group = "Testing")
public class GetTransportPositions extends OpMode {
    public static DcMotorEx armMotor;
    public static DcMotorEx slidesMotor;

    public static ServoImplEx rightIntake;

    public static ServoImplEx leftIntake;
    public static ServoImplEx intakeRotation;;
    public static double rightIntakePos = 1;
    public static double leftIntakePos = 1;
    public static double intakeRotationPos = 1;
    @Override
    public void init() {
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        armMotor.setTargetPosition(0);
//        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slidesMotor = hardwareMap.get(DcMotorEx.class, "slidesMotor");
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        slidesMotor.setTargetPosition(0);
//        slidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intakeRotation = hardwareMap.get(ServoImplEx.class, "intakeRotation");
        rightIntake = hardwareMap.get(ServoImplEx.class, "rightIntake");
        leftIntake = hardwareMap.get(ServoImplEx.class, "leftIntake");

        intakeRotation.setDirection(ServoImplEx.Direction.FORWARD);
        rightIntake.setDirection(ServoImplEx.Direction.FORWARD);
        leftIntake.setDirection(ServoImplEx.Direction.REVERSE);
    }

    @Override
    public void loop() {
        intakeRotation.setPosition(intakeRotationPos);
        leftIntake.setPosition(leftIntakePos);
        rightIntake.setPosition(rightIntakePos);
        telemetry.addData("ArmPos: ", armMotor.getCurrentPosition());
        telemetry.addData("SlidesPos: ", slidesMotor.getCurrentPosition());
        telemetry.addData("LeftIntakePos: ", leftIntakePos);
        telemetry.addData("RightIntakePos: ", rightIntakePos);
        telemetry.addData("IntakeRotationPos: ", intakeRotationPos);
    }
}

