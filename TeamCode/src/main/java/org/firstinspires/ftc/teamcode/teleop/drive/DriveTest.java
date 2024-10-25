package org.firstinspires.ftc.teamcode.teleop.drive;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.teleop.drive.Drive;

@Config
@TeleOp
public class DriveTest extends OpMode {
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    public Gamepad gamepad1;

    @Override
    public void init() {
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        gamepad1 = new Gamepad();

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            frontLeft.setPower(1);
        }
        if (gamepad1.b) {
            frontRight.setPower(1);
        }
        if (gamepad1.x) {
            backLeft.setPower(1);
        }
        if (gamepad1.y) {
            backRight.setPower(1);
        }
        telemetry.addData("frontLeft:", frontLeft.getPower());
        telemetry.addData("frontRight:", frontRight.getPower());
        telemetry.addData("backLeft:", backLeft.getPower());
        telemetry.addData("backRight:", backRight.getPower());
        telemetry.update();
    }
}
