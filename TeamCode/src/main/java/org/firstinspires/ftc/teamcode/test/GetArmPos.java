package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class GetArmPos extends LinearOpMode{
    @Override
    public void runOpMode() {
        DcMotor armMotor = hardwareMap.dcMotor.get("ArmMotor");
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setPower(1);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("ArmMotor Pos:", armMotor.getCurrentPosition());
        }
    }
}
