package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class TeleOp2 extends LinearOpMode {
    int target = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        //Declaring more motors this one is the claw/rotational arm movement motor

        //REVERSE MOTORS:
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        imu.resetYaw();

        // SLIDES MOTOR SET:
        DcMotor slidesMotor = hardwareMap.dcMotor.get("slidesMotor");
        slidesMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slidesMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setTargetPosition(0);
        slidesMotor.setPower(1);
        slidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //ROTATIONAL ARM MOTOR SET:
        DcMotor armMotor = hardwareMap.dcMotor.get("ArmMotor");
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setPower(1);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //DRONE LAUNCHER SERVO SET:
        Servo droneLauncherMotor = hardwareMap.servo.get("droneLauncherMotor");

        //LEFT & RIGHT INTAKE SET:
        Servo leftIntakeMotor = hardwareMap.servo.get("leftIntakeMotor");
        Servo rightIntakeMotor = hardwareMap.servo.get("rightIntakeMotor");
        leftIntakeMotor.setDirection(Servo.Direction.REVERSE);

        //ROTATIONAL INTAKE MOTOR SET:
        Servo rotationalIntake = hardwareMap.servo.get("rotationalIntake");
        double rotationalIntakePositionStart = 0.31;
        double rotationalIntakePositionEnd = .92;

        //ELAPSED TIME SET:
        ElapsedTime gamepadXTime = new ElapsedTime();
        ElapsedTime gamepadYTime = new ElapsedTime();
        ElapsedTime gamepadATime = new ElapsedTime();
        ElapsedTime gamepadBTime = new ElapsedTime();

        //SERVO POSITION VARIABLES:
        int intakePos = 0;
        int intakePos1 = 0;
        int outtakePos = 1;
        int outtakePos1 = 1;
        int movementIntakePos = 0;

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            //field centric
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            //this version is robot centric
            /*double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;*/

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.back) {
                imu.resetYaw();
            }
            //field centric ***/))
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing



            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (-rotY + rotX - rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            //robot centric (there is different logic)
            /*double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y + x - rx) / denominator;
            double backRightPower = (y - x - rx) / denominator;
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            */





            //set drone launcher's button:
            boolean droneButton = gamepad2.start;

            //set rotating claw button:
            boolean movementIntake = gamepad1.y;

            //slides motor initializations:
            target += gamepad1.right_trigger * 25  - gamepad1.left_trigger * 15;

            if (target < 0) {
                target = 0;
            } else if (target > 3200){
                target = 3200;
            }

            slidesMotor.setTargetPosition(target);

            if (movementIntake && gamepadYTime.milliseconds() > 500){
                if(movementIntakePos == 0){
                    rotationalIntake.setPosition(rotationalIntakePositionEnd);
                    movementIntakePos = 1;
                } else if (movementIntakePos == 1) {
                    rotationalIntake.setPosition(rotationalIntakePositionStart);
                    movementIntakePos = 0;
                }
                gamepadYTime.reset();
            }

            if (droneButton && gamepadXTime.milliseconds() > 500) {
                droneLauncherMotor.setPosition(1);
                gamepadXTime.reset();
            }

            //initiate slides motors output mech
            slidesMotor.setPower(1);
            armMotor.setPower(1);
            //slidesRightMotor.setPower(Power);
            //intake/claw
            boolean rightIntake = gamepad1.right_bumper;

            boolean leftIntake = gamepad1.left_bumper;

            if (rightIntake && gamepadATime.milliseconds() > 500){
                if(intakePos == 0){
                    rightIntakeMotor.setPosition(-1);
                    intakePos = 1;
                } else if (intakePos == outtakePos) {
                    rightIntakeMotor.setPosition(1);
                    intakePos = 0;
                }
                gamepadATime.reset();
            }
            if (leftIntake && gamepadBTime.milliseconds() > 500){
                if(intakePos1 == 0){
                    leftIntakeMotor.setPosition(1);
                    intakePos1 = 1;
                } else if (intakePos1 == outtakePos1) {
                    leftIntakeMotor.setPosition(-1);
                    intakePos1 = 0;
                }
                gamepadBTime.reset();
            }

            boolean setLineOne = gamepad2.a;
            boolean setLineTwo = gamepad2.b;
            boolean setLineThree = gamepad2.y;

            //setLineFunctions:
            if (setLineOne && gamepadATime.milliseconds() > 500){
                rotationalIntake.setPosition(rotationalIntakePositionStart);
                movementIntakePos = 0;
                if(armMotor.getTargetPosition() != 1400) {
                    armMotor.setTargetPosition(1400);
                }
                if(slidesMotor.getTargetPosition() != 1250) {
                    slidesMotor.setTargetPosition(1250);
                }
                sleep(1000);
                leftIntakeMotor.setPosition(0);
                rightIntakeMotor.setPosition(0);
                sleep(400);
                leftIntakeMotor.setPosition(1);
                rightIntakeMotor.setPosition(1);
                slidesMotor.setTargetPosition(0);
                armMotor.setTargetPosition(250);
                sleep(250);
                armMotor.setTargetPosition(0);
                gamepadATime.reset();
            }
            if (setLineTwo && gamepadBTime.milliseconds() > 500) {
                rotationalIntake.setPosition(rotationalIntakePositionStart);
                movementIntakePos = 0;
                if(armMotor.getTargetPosition() != 1400) {
                    armMotor.setTargetPosition(1400);
                }
                if(slidesMotor.getTargetPosition() != 2225){
                    slidesMotor.setTargetPosition(2225);
                }
                sleep(1000);
                leftIntakeMotor.setPosition(0);
                rightIntakeMotor.setPosition(0);
                sleep(400);
                leftIntakeMotor.setPosition(1);
                rightIntakeMotor.setPosition(1);
                slidesMotor.setTargetPosition(0);
                armMotor.setTargetPosition(250);
                sleep(250);
                armMotor.setTargetPosition(0);
                gamepadBTime.reset();
            }
            if (setLineThree && gamepadYTime.milliseconds() > 500) {
                rotationalIntake.setPosition(rotationalIntakePositionStart);
                movementIntakePos = 0;
                if(armMotor.getTargetPosition() != 1400) {
                    armMotor.setTargetPosition(1400);
                }
                if(slidesMotor.getTargetPosition() != 3200){
                    slidesMotor.setTargetPosition(3200);
                }
                sleep(1250);
                leftIntakeMotor.setPosition(0);
                rightIntakeMotor.setPosition(0);
                sleep(400);
                leftIntakeMotor.setPosition(1);
                rightIntakeMotor.setPosition(1);
                slidesMotor.setTargetPosition(0);
                armMotor.setTargetPosition(250);
                sleep(250);
                armMotor.setTargetPosition(0);
                gamepadYTime.reset();
            }


            /*double setPos1 = 1.105555;
            double setPos2 = 0.31;
            if (movementIntake){
                if(intakeMovementPos == 0){
                    rightintakeMovement.setPosition(-setPos1);
                    leftintakeMovement.setPosition(setPos1);
                    intakeMovementPos = 1;
                } else if (intakeMovementPos == 1) {
                    rightintakeMovement.setPosition(-setPos2);
                    leftintakeMovement.setPosition(setPos2);
                    intakeMovementPos = 0;
                }
            }*/
        }
        telemetry.addData("LiftPos", slidesMotor.getCurrentPosition());
        telemetry.update();
    }
}