package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import static org.firstinspires.ftc.teamcode.teleop.AllianceStorage.isRed;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.teleop.utils.Toggle;

@Config
@TeleOp
public class TestDrive extends OpMode {
    public static DcMotorEx frontRight;
    public static DcMotorEx frontLeft;
    public static DcMotorEx backRight;
    public static DcMotorEx backLeft;

    public static IMU imu;
    public double botHeading, targetHeading;
    public static double turnP = 0, turnI = 0, turnD = 0;

    private PIDFController turnController = new PIDFController(new PIDCoefficients(turnP, turnI, turnD));

    public double IMUOffset;
    public double RedOffset = Math.toRadians(90);
    public double BlueOffset = Math.toRadians(270);

    public boolean RobotCentric = true;

    public Toggle slowmode;

    @Override
    public void init() {
        slowmode = new Toggle(false);
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        imu.resetYaw();

        botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        targetHeading = botHeading;

        if (isRed) {
            IMUOffset = RedOffset;
        } else {
            IMUOffset = BlueOffset;
        }
    }

    @Override
    public void loop() {
        if (RobotCentric == false) {
            botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            //Field Centric Drive:
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            if (rx == 0){
                rx = calcRotBasedOnIdeal(botHeading, targetHeading);
            }
            else {
                targetHeading = botHeading;
            }
            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX = rotX * 1.1;  // Counteract imperfect strafing
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

            slowmode.update(gamepad1.start);
            if (slowmode.value() == true) {
                denominator *= 2;
            }

            //Autoturn Logic:
            if (gamepad1.right_bumper) {
                rx = calcRotBasedOnIdeal(botHeading, Math.toRadians(270));
            }
            if (gamepad1.left_bumper) {
                rx = calcRotBasedOnIdeal(botHeading, Math.toRadians(325));
            }

            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);
            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            //Reset IMU:
            if (gamepad1.back) {
                resetImu();
            }
        } else {
            botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            if (rx == 0){
                rx = calcRotBasedOnIdeal(botHeading, targetHeading);
            }
            else {
                rx = gamepad1.right_stick_x;
                targetHeading = botHeading;
            }

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            slowmode.update(gamepad1.start);
            if (slowmode.value() == true) {
                denominator *= 2;
            }

            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);
        }
    }

    private double calcRotBasedOnIdeal(double heading, double idealHeading) {
        // Error in rotations (should always be between (-0.5,0.5))
        double err = angleWrap(idealHeading - heading);
        turnController.setTargetPosition(0);
        double correction = turnController.update(err);
        return correction;
    }
    public double angleWrap(double angle) {
        angle = Math.toRadians(angle);
        // Changes any angle between [-179,180] degrees
        // If rotation is greater than half a full rotation, it would be more efficient to turn the other way
        while (Math.abs(angle) > Math.PI) {
            angle -= 2 * Math.PI * (angle > 0 ? 1 : -1); // if angle > 0 * 1, < 0 * -1
        }
        return Math.toDegrees(angle);
    }
//    private double normalizeAngle(double angle) {
//        // Normalizes angle in [0,360] range
//        while (angle > 2 * Math.PI) angle -= 2 * Math.PI;
//        while (angle < 0) angle += 2 * Math.PI;
//        return angle;
//    }

    public void resetImu () {
        IMUOffset = 0;
        imu.resetYaw();
    }
}
