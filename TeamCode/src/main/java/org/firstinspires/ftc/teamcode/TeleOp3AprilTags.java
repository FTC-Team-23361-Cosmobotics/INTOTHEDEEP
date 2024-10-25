package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@TeleOp
public class TeleOp3AprilTags extends LinearOpMode {
    int target = 0;
    int armTarget = 0;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 821.993;
    double fy = 821.993;
    double cx = 330.489;
    double cy = 248.997;

    // UNITS ARE METERS
    double tagsize = 0.0508;
    double largetagsize = .127;

    int none = 0; // Tag ID 0  from the 36h11 family
    int blueLeft = 1; // Tag ID 1  from the 36h11 family
    int blueCenter = 2; // Tag ID 2  from the 36h11 family
    int blueRight = 3; // Tag ID 3  from the 36h11 family
    int blueBack = 10; // Tag ID 10  from the 36h11 family
    int redLeft = 4; // Tag ID 4  from the 36h11 family
    int redCenter = 5; // Tag ID 5  from the 36h11 family
    int redRight = 6; // Tag ID 6  from the 36h11 family
    int redBack = 7; // Tag ID 7  from the 36h11 family
    int ID_TAG_OF_INTEREST = none;

    int setLineOne = 1;
    int setLineTwo = 2;
    int setLineThree = 3;
    int POS_OF_INTEREST = setLineThree;

    AprilTagDetection tagOfInterest = null;




    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);


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
        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        //Declaring more motors this one is the claw/rotational arm movement motor
        DcMotor rotationalArmMotor = hardwareMap.dcMotor.get("ArmMotor");
        rotationalArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Servo rotationIntakeMotor = hardwareMap.servo.get("rotationalIntake");//edit this later on continuos servo?

        rotationalArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotationalArmMotor.setTargetPosition(0);
        rotationalArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        imu.resetYaw();

        // SLIDES MOTOR SET:
        DcMotor slidesMotor = hardwareMap.dcMotor.get("slidesMotor");
        slidesMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setTargetPosition(0);
        slidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Drone launcher motor set:
        Servo droneLauncherMotor = hardwareMap.servo.get("droneLauncherMotor");

        int intakeMovementPos = 0;
        //leftintakeMovement.setDirection(Servo.Direction.REVERSE);

        Servo leftIntakeMotor = hardwareMap.servo.get("leftIntakeMotor");
        Servo rightIntakeMotor = hardwareMap.servo.get("rightIntakeMotor");
        leftIntakeMotor.setDirection(Servo.Direction.REVERSE);
        rightIntakeMotor.setDirection(Servo.Direction.REVERSE);
        boolean intakeMotorButton = false;
        int intakePos = 0;
        int intakePos1 = 0;
        int outtakePos = 1;
        int outtakePos1 = 1;

        int movementIntakePos = 0;

        leftIntakeMotor.setPosition(0);
        rightIntakeMotor.setPosition(0);

        ElapsedTime gamepadXTime = new ElapsedTime();
        ElapsedTime gamepadYTime = new ElapsedTime();
        ElapsedTime gamepadATime = new ElapsedTime();
        ElapsedTime gamepadBTime = new ElapsedTime();
        waitForStart();


        if (isStopRequested()) return;

        while (opModeIsActive()) {
            //April Tag Detection Code:


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


            //set slides motors:
            double leftTrig2 = -gamepad1.left_trigger;
            double rightTrig2 = gamepad1.right_trigger;

            //set drone launcher's button:
            boolean droneButton = gamepad2.start;

            //set rotating claw button:
            boolean movementIntake = gamepad1.a;

            //set tagOfInterest Buttons:
            boolean setLeftBlueTag = gamepad1.dpad_left;
            boolean setCenterBlueTag = gamepad1.dpad_up;
            boolean setRightBlueTag = gamepad1.dpad_right;
            boolean setLeftRedTag = gamepad1.x;
            boolean setCenterRedTag = gamepad1.y;
            boolean setRightRedTag = gamepad1.b;

            if (setLeftBlueTag) {
                ID_TAG_OF_INTEREST = blueLeft;
            }
            if (setCenterBlueTag) {
                ID_TAG_OF_INTEREST = blueCenter;
            }
            if (setRightBlueTag) {
                ID_TAG_OF_INTEREST = blueRight;
            }
            if (setLeftRedTag) {
                ID_TAG_OF_INTEREST = redLeft;
            }
            if (setCenterRedTag) {
                ID_TAG_OF_INTEREST = redCenter;
            }
            if (setRightRedTag) {
                ID_TAG_OF_INTEREST = redRight;
            }

            // set setLinesOfInterest Buttons:
            boolean setLineOneInterest = gamepad2.a;
            boolean setLineTwoInterest = gamepad2.b;
            boolean setLineThreeInterest = gamepad2.y;

            if (setLineOneInterest) {
                POS_OF_INTEREST = setLineOne;
            }
            if (setLineTwoInterest) {
                POS_OF_INTEREST = setLineTwo;
            }
            if (setLineThreeInterest) {
                POS_OF_INTEREST = setLineThree;
            }

            target += gamepad2.right_trigger * 25 - gamepad2.left_trigger * 15;

            if (target < 0) {
                target = 0;
            } else if (target > 1600) {
                target = 1600;
            }

            if (armTarget < 0) {
                armTarget = 0;
            } else if (armTarget > 1250) {
                armTarget = 1250;
            }

            slidesMotor.setTargetPosition(target);
            rotationalArmMotor.setTargetPosition(armTarget);

            if (movementIntake && gamepadYTime.milliseconds() > 500) {
                if (movementIntakePos == 0) {
                    rotationIntakeMotor.setPosition(.92);
                    movementIntakePos = 1;
                } else if (movementIntakePos == 1) {
                    rotationIntakeMotor.setPosition(.2);
                    movementIntakePos = 0;
                }
                gamepadYTime.reset();
            }
            if (droneButton) {
                droneLauncherMotor.setPosition(1);
            }

            //initiate slides motors output mech
            slidesMotor.setPower(1);
            rotationalArmMotor.setPower(1);
            //slidesRightMotor.setPower(Power);
            //intake/claw
            boolean rightIntake = gamepad1.right_bumper;
            boolean leftIntake = gamepad1.left_bumper;

            if (rightIntake && gamepadATime.milliseconds() > 500) {
                if (intakePos == 0) {
                    rightIntakeMotor.setPosition(-1);
                    intakePos = 1;
                } else if (intakePos == outtakePos) {
                    rightIntakeMotor.setPosition(1);
                    intakePos = 0;
                }
                gamepadATime.reset();
            }
            if (leftIntake && gamepadBTime.milliseconds() > 500) {
                if (intakePos1 == 0) {
                    leftIntakeMotor.setPosition(1);
                    intakePos1 = 1;
                } else if (intakePos1 == outtakePos1) {
                    leftIntakeMotor.setPosition(-1);
                    intakePos1 = 0;
                }
                gamepadBTime.reset();
            }
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == ID_TAG_OF_INTEREST) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);

            if(tagOfInterest != null)
            {
                telemetry.addLine("Tag snapshot:\n");
                tagToTelemetry(tagOfInterest);
                telemetry.update();
            }
            else
            {
                telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
                telemetry.update();
            }

            /* Actually do something useful */
            if(tagOfInterest != null)
            {
                if(tagOfInterest.pose.x >= 30 && tagOfInterest.pose.x <= 100 && tagOfInterest.pose.z <= 20)
                {
                    //brake
                    frontLeftMotor.setPower(0);
                    frontRightMotor.setPower(0);
                    backLeftMotor.setPower(0);
                    backRightMotor.setPower(0);
                    //setLinePositions
                    if (POS_OF_INTEREST == setLineOne){
                        if (movementIntakePos != 1) {
                            rotationIntakeMotor.setPosition(.2);
                            movementIntakePos = 1;
                        }
                        if(rotationalArmMotor.getTargetPosition() != 1250) {
                            rotationalArmMotor.setTargetPosition(1250);
                        }
                        if(slidesMotor.getTargetPosition() != 540) {
                            slidesMotor.setTargetPosition(540);
                        }
                        //Place and Reset Sequence
                        leftIntakeMotor.setPosition(1);
                        rightIntakeMotor.setPosition(1);
                        sleep(500);
                        leftIntakeMotor.setPosition(0);
                        rightIntakeMotor.setPosition(0);
                        slidesMotor.setTargetPosition(0);
                        rotationalArmMotor.setTargetPosition(0);
                        frontLeftMotor.setPower(-.25);
                        frontRightMotor.setPower(-.25);
                        backLeftMotor.setPower(-.25);
                        backRightMotor.setPower(-.25);
                        sleep(200);
                        frontLeftMotor.setPower(frontLeftPower);
                        frontRightMotor.setPower(frontRightPower);
                        backLeftMotor.setPower(backLeftPower);
                        backRightMotor.setPower(backRightPower);
                    } else if (POS_OF_INTEREST == setLineTwo) {

                        if (movementIntakePos != 1) {
                            rotationIntakeMotor.setPosition(.2);
                            movementIntakePos = 1;
                        }
                        if(rotationalArmMotor.getTargetPosition() != 1250) {
                            rotationalArmMotor.setTargetPosition(1250);
                        }
                        if(slidesMotor.getTargetPosition() != 1080){
                            slidesMotor.setTargetPosition(1080);
                        }
                        //Place and Reset Sequence
                        leftIntakeMotor.setPosition(1);
                        rightIntakeMotor.setPosition(1);
                        sleep(500);
                        leftIntakeMotor.setPosition(0);
                        rightIntakeMotor.setPosition(0);
                        slidesMotor.setTargetPosition(0);
                        rotationalArmMotor.setTargetPosition(0);
                        frontLeftMotor.setPower(-.25);
                        frontRightMotor.setPower(-.25);
                        backLeftMotor.setPower(-.25);
                        backRightMotor.setPower(-.25);
                        sleep(200);
                        frontLeftMotor.setPower(frontLeftPower);
                        frontRightMotor.setPower(frontRightPower);
                        backLeftMotor.setPower(backLeftPower);
                        backRightMotor.setPower(backRightPower);
                    } else {
                        if (movementIntakePos != 1) {
                            rotationIntakeMotor.setPosition(.2);
                            movementIntakePos = 1;
                        }
                        if(rotationalArmMotor.getTargetPosition() != 1250) {
                            rotationalArmMotor.setTargetPosition(1250);
                        }
                        if(slidesMotor.getTargetPosition() != 1600){
                            slidesMotor.setTargetPosition(1600);
                        }
                        //Place and Reset Sequence
                        leftIntakeMotor.setPosition(1);
                        rightIntakeMotor.setPosition(1);
                        sleep(500);
                        leftIntakeMotor.setPosition(0);
                        rightIntakeMotor.setPosition(0);
                        slidesMotor.setTargetPosition(0);
                        rotationalArmMotor.setTargetPosition(0);
                        frontLeftMotor.setPower(-.25);
                        frontRightMotor.setPower(-.25);
                        backLeftMotor.setPower(-.25);
                        backRightMotor.setPower(-.25);
                        sleep(200);
                        frontLeftMotor.setPower(frontLeftPower);
                        frontRightMotor.setPower(frontRightPower);
                        backLeftMotor.setPower(backLeftPower);
                        backRightMotor.setPower(backRightPower);
                    }
                }
            }
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
            telemetry.addData("LiftPos", slidesMotor.getCurrentPosition());
            telemetry.update();
        }

    void tagToTelemetry(AprilTagDetection detection)
    {
        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));
    }
}
