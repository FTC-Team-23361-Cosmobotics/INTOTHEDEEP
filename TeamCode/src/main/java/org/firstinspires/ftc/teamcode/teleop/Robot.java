//package org.firstinspires.ftc.teamcode.teleop;
//
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
//
//import com.arcrobotics.ftclib.controller.PIDController;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.ServoImplEx;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//
//import java.util.List;
//
//
//public class Robot {
//
//
//    public static PIDController arm_controller;
//    public static double arm_p = 0, arm_i = 0, arm_d = 0;
//    public static double arm_f = 0;
//    public static int armTarget = 0;
//    public static final double arm_ticks_in_degrees = 1425.1;
//    public static PIDController slides_controller;
//
//    public static double slides_p = 0, slides_i = 0, slides_d = 0;
//    public static double slides_f = 0;
//    public static int slidesTarget = 0;
//    public static final double slides_ticks_in_degrees = 537.7;
//
//    //    public final double arm_max_acceleration = 0;
////    public final double arm_max_velocity = 0;
////
////    public double arm_distance = 0;
////    public ElapsedTime arm_elapsed_time = new ElapsedTime();
////    public final double slides_max_acceleration = 0;
////    public final double slides_max_velocity = 0;
////    public double slides_distance = 0;
////    public ElapsedTime slides_elapsed_time = new ElapsedTime();
//
//
//
//    //Bulk Read:
//    public double leftIntakePos = leftIntake.getPosition();
//    public double rightIntakePos = rightIntake.getPosition();
//    public double intakeRotationPos = intakeRotation.getPosition();
//    public int armPos = armMotor.getCurrentPosition();
//    public int slidesPos = slidesMotor.getCurrentPosition();
//
//    public double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//
//    //Source: CtrlAltFTC
////    public double motion_profile(double max_acceleration, double max_velocity, double distance, ElapsedTime elapsed_time) {
////        elapsed_time.reset(); //May be bad implementation :skull:
////        // calculate the time it takes to accelerate to max velocity
////        double acceleration_dt = max_velocity / max_acceleration;
////
////        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
////        double halfway_distance = distance / 2;
////        double acceleration_distance = Math.pow(0.5 * max_acceleration * acceleration_dt, 2);
////
////        if (acceleration_distance > halfway_distance)
////            acceleration_dt = Math.sqrt(halfway_distance / (0.5 * max_acceleration));
////
////        acceleration_distance = Math.pow(0.5 * max_acceleration * acceleration_dt, 2);
////
////        // recalculate max velocity based on the time we have to accelerate and decelerate
////        max_velocity = max_acceleration * acceleration_dt;
////
////        // we decelerate at the same rate as we accelerate
////        double deceleration_dt = acceleration_dt;
////
////        // calculate the time that we're at max velocity
////        double cruise_distance = distance - 2 * acceleration_distance;
////        double cruise_dt = cruise_distance / max_velocity;
////        double deceleration_time = acceleration_dt + cruise_dt;
////
////        // check if we're still in the motion profile
////        double entire_dt = acceleration_dt + cruise_dt + deceleration_dt;
////        if (elapsed_time.milliseconds() > entire_dt)
////            return distance;
////
////        // if we're accelerating
////        if (elapsed_time.milliseconds() < acceleration_dt) {
////            // use the kinematic equation for acceleration
////            return Math.pow(0.5 * max_acceleration * elapsed_time.milliseconds(), 2);
////        } else if (elapsed_time.milliseconds() < deceleration_time) {
////            // if we're cruising
////            acceleration_distance = Math.pow(0.5 * max_acceleration * acceleration_dt, 2);
////            double cruise_current_dt = elapsed_time.milliseconds() - acceleration_dt;
////
////            // use the kinematic equation for constant velocity
////            return acceleration_distance + max_velocity * cruise_current_dt;
////        }
////
////        // if we're decelerating
////        else {
////            acceleration_distance = Math.pow(0.5 * max_acceleration * acceleration_dt, 2);
////            cruise_distance = max_velocity * cruise_dt;
////            deceleration_time = elapsed_time.milliseconds() - deceleration_time;
////
////            // use the kinematic equations to calculate the instantaneous desired position
////            return Math.pow(acceleration_distance + cruise_distance + max_velocity * deceleration_time - 0.5 * max_acceleration * deceleration_time, 2);
////        }
////    }
////   oh
//    public Robot(OpMode opmode){
//        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
//
//        for (LynxModule hub : allHubs) {
//            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
//        }
//
//        armMotor = opmode.hardwareMap.get(DcMotorEx.class, "armMotor");
//        slidesMotor = opmode.hardwareMap.get(DcMotorEx.class, "slidesMotor");
//        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        slidesMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        //Arm PID + Motion Profiling:
//        arm_controller = new PIDController(arm_p, arm_i, arm_d);
//        arm_controller.setPID(arm_p, arm_i, arm_d);
//        double arm_pid = arm_controller.calculate(armPos, armTarget);
//        double arm_ff = Math.cos(Math.toRadians(armTarget / arm_ticks_in_degrees)) * arm_f;
//        double armPower = arm_pid + arm_ff;
//        armMotor.setPower(armPower);
//        //Slides PID + Motion Profiling:
//        slides_controller = new PIDController(slides_p, slides_i, slides_d);
//        slides_controller.setPID(slides_p, slides_i, slides_d);
//        double slides_pid = slides_controller.calculate(slidesPos, slidesTarget);
//        double slides_ff = Math.cos(Math.toRadians(slidesTarget / slides_ticks_in_degrees)) * slides_f;
//        double slidesPower = slides_pid + slides_ff;
//        slidesMotor.setPower(slidesPower);
//
//        intakeRotation = opmode.hardwareMap.get(ServoImplEx.class, "intakeRotation");
//        rightIntake = opmode.hardwareMap.get(ServoImplEx.class, "rightIntake");
//        leftIntake = opmode.hardwareMap.get(ServoImplEx.class, "leftIntake");
//        droneServo = opmode.hardwareMap.get(ServoImplEx.class, "droneLauncher");
//        intakeRotation.setDirection(ServoImplEx.Direction.FORWARD);
//        rightIntake.setDirection(ServoImplEx.Direction.FORWARD);
//        leftIntake.setDirection(ServoImplEx.Direction.REVERSE);
//        droneServo.setDirection(ServoImplEx.Direction.REVERSE);
//
//        imu = opmode.hardwareMap.get(IMU.class, "imu");
//        // Adjust the orientation parameters to match your robot
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
//                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
//        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
//        imu.initialize(parameters);
//        imu.resetYaw();
//    }
//    public Robot(LinearOpMode linearOpMode){
//        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
//
//        for (LynxModule hub : allHubs) {
//            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
//        }
//
//
//
//        armMotor = linearOpMode.hardwareMap.get(DcMotorEx.class, "armMotor");
//        slidesMotor = linearOpMode.hardwareMap.get(DcMotorEx.class, "slidesMotor");
//        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        slidesMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        slidesMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        //Arm PID:
//        arm_controller = new PIDController(arm_p, arm_i, arm_d);
//        arm_controller.setPID(arm_p, arm_i, arm_d);
//        double arm_pid = arm_controller.calculate(armPos, armTarget);
//        double arm_ff = Math.cos(Math.toRadians(armTarget / arm_ticks_in_degrees)) * arm_f;
//        double armPower = arm_pid + arm_ff;
//        armMotor.setPower(armPower);
//        //Slides PID:
//        slides_controller = new PIDController(slides_p, slides_i, slides_d);
//        slides_controller.setPID(slides_p, slides_i, slides_d);
//        double slides_pid = slides_controller.calculate(slidesPos, slidesTarget);
//        double slides_ff = Math.cos(Math.toRadians(slidesTarget / slides_ticks_in_degrees)) * slides_f;
//        double slidesPower = slides_pid + slides_ff;
//        armMotor.setPower(slidesPower);
//
//        intakeRotation = linearOpMode.hardwareMap.get(ServoImplEx.class, "intakeRotation");
//        rightIntake = linearOpMode.hardwareMap.get(ServoImplEx.class, "rightIntake");
//        leftIntake = linearOpMode.hardwareMap.get(ServoImplEx.class, "leftIntake");
//        droneServo = linearOpMode.hardwareMap.get(ServoImplEx.class, "droneLauncher");
//        intakeRotation.setDirection(ServoImplEx.Direction.FORWARD);
//        rightIntake.setDirection(ServoImplEx.Direction.FORWARD);
//        leftIntake.setDirection(ServoImplEx.Direction.REVERSE);
//        droneServo.setDirection(ServoImplEx.Direction.REVERSE);
//
//        imu = linearOpMode.hardwareMap.get(IMU.class, "imu");
//        // Adjust the orientation parameters to match your robot
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
//                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
//        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
//        imu.initialize(parameters);
//        imu.resetYaw();
//    }
//
//    public void resetImu(){
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
//                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
//        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
//        imu.initialize(parameters);
//        imu.resetYaw();
//    }
//}
//
