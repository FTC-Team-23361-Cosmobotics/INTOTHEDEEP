package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.teleop.drive.Drive;

@Config
@TeleOp
public class PIDFTransport extends OpMode {
    private Drive drive;
    private DcMotorEx extendo, out;
    private PIDController extendoController, outController;
    public static double extendop = .02, extendoi = 0.0001, extendod = 0.0001;
    public static int extendoTarget = 0;
    public static double outp = 0.007, outi = 0, outd = 0.0004;
    public static int outTarget = 0;

    private ServoImplEx outClaw, rot, outArm;
    private CRServoImplEx leftRot, rightRot;

    public static double rotPos;

    public static double outClawPos;

    public static double leftRotPower;
    public static double rightRotPower;

    public static double outArmPos;
    /*
    Notes:
    OutArm
    - Home - .1
    - Score - .7
    Claw //TODO: TUNE
    - Open -
    - Closed -
    Right Intake //TODO: TUNE
    - Negative is
    - Positive is
    Left Intake //TODO: TUNE
    - Negative is
    - Positive is

    Extendo
    - P: .01
    - I: .0001
    - D: .0001
    - -100 Min
    - 2300 Max

    Slides
    - P:
    - I:
    - D:
    - -100 Min
    - 2300 Max

     */

    @Override
    public void init() {
        drive = new Drive(hardwareMap);
        leftRot = hardwareMap.get(CRServoImplEx.class, "leftRot");
        rightRot = hardwareMap.get(CRServoImplEx.class, "rightRot");

        rot = hardwareMap.get(ServoImplEx.class, "rot");

        outClaw = hardwareMap.get(ServoImplEx.class, "outClaw");
        outArm = hardwareMap.get(ServoImplEx.class, "outArm");

        extendoController = new PIDController(extendop, extendoi, extendod);
        extendo = hardwareMap.get(DcMotorEx.class, "extendo");
        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        outController = new PIDController(outp, outi, outd);
        out = hardwareMap.get(DcMotorEx.class, "out");
        out.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        out.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        out.setDirection(DcMotorSimple.Direction.REVERSE);


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        drive.update(gamepad1);
        rot.setPosition(rotPos);
        outClaw.setPosition(outClawPos);
        leftRot.setPower(leftRotPower);
        rightRot.setPower(rightRotPower);
        outArm.setPosition(outArmPos);

        outController.setPID(outp, outi, outd);
        int outPos = out.getCurrentPosition();
        double outpid = outController.calculate(outPos, outTarget);
        out.setPower(outpid);

        extendoController.setPID(extendop, extendoi, extendod);
        int extendoPos = extendo.getCurrentPosition();
        double extendopid = extendoController.calculate(extendoPos, extendoTarget);
        extendo.setPower(extendopid);

//        if (!gamepad1.b) {
//            slidesMotor.setPower(slidespower);
//            armMotor.setPower(armPower);
//        }

        telemetry.addData("Extendo Pos:", extendoPos);
        telemetry.addData("Extendo Target:", extendoTarget);
        telemetry.addData("Out Pos:", outPos);
        telemetry.addData("Out Target:", outTarget);
        telemetry.update();
    }
}