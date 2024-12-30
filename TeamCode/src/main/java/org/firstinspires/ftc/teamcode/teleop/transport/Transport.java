//package org.firstinspires.ftc.teamcode.teleop.transport;
//
//import static org.firstinspires.ftc.teamcode.teleop.AllianceStorage.isRed;
//import static org.firstinspires.ftc.teamcode.test.GetTransportPositions.armMotor;
//
//import com.arcrobotics.ftclib.controller.PIDController;
//import com.qualcomm.robotcore.hardware.CRServoImpl;
//import com.qualcomm.robotcore.hardware.CRServoImplEx;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.ServoImplEx;
//import com.qualcomm.robotcore.hardware.TouchSensor;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.teleop.utils.Toggle;
//
//public class Transport {
//    public Toggle clawToggle, rotToggle; //TODO: REMOVE TOGGLES AND HARD CODE
////    private ServoImplEx outClaw, rot, outArm;
//    private CRServoImplEx leftRot, rightRot;
//    //private ServoImplEx specClaw, rot, bucketYaw, bucketPitch;
//    //private DcMotorEx extendo, out, intake;
//
//    private DcMotorEx extendo, out;
//    private PIDController extendoController;
//
//    public double extendoPos, outPos;
//
//    public static double extendop = .02, extendoi = .0001, extendod = 0.0001;
//    public static int extendoTarget = 0;
//    private PIDController outController;
//
//    public static double outp = 0.01, outi = .0001, outd = .0001;
//    public static int outTarget = 0;
//
//    public static final double clawOpen = 0; //TODO: TUNE THESE
//    public static final double clawClosed = 1;
//
//
//    public static final double outArmHome = 0.05;
//    public static final double outArmScoreBucket = .7;
//    public static final double outArmScoreSpec = 1;
//
//
//    public static final double rotIntake = 1;
//    public static final double rotHome = 0;
//
//
//    public static double outClawPos, leftRotPower, rightRotPower, outArmPos, rotPos;
//
//    public static final int highBucket = 2500; //TODO: TUNE VVV
//    public static final int lowBucket = 1500;
//    public static final int highBar = 1600;
//
//    public static TransportState transportState = TransportState.START;
//    public static ElapsedTime transportTime = new ElapsedTime();
//
//    public enum TransportState {
//        START,
//        INTAKING,
//        TRANSFER_SAMPLE,
//        DEPOSIT_SPEC_SAMPLE,
//
//        LOW_BUCKET,
//        SCORE_LOW_BUCKET,
//
//        HIGH_BUCKET,
//        SCORE_HIGH_BUCKET,
//
//        SCORE_SPEC,
//        MANUAL
//    }
//
//    public Transport(HardwareMap hardwareMap) {
//        clawToggle = new Toggle(false);
//        rotToggle = new Toggle(false);
//
//        extendoController = new PIDController(extendop, extendoi, extendod);
//        extendo = hardwareMap.get(DcMotorEx.class, "extendo");
//        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //TODO: Possibly Reset is Not Needed
//        extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        extendoController.setPID(extendop, extendoi, extendod);
//        extendoPos = extendo.getCurrentPosition();
//
//        outController = new PIDController(outp, outi, outd);
//        out = hardwareMap.get(DcMotorEx.class, "out");
//        out.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //TODO: Possibly Reset is Not Needed
//        out.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        out.setDirection(DcMotorSimple.Direction.REVERSE);
//        outController.setPID(outp, outi, outd);
//        outPos = out.getCurrentPosition();
//
//        leftRot = hardwareMap.get(CRServoImplEx.class, "leftRot");
//        rightRot = hardwareMap.get(CRServoImplEx.class, "rightRot");
//        rot = hardwareMap.get(ServoImplEx.class, "rot");
//        outClaw = hardwareMap.get(ServoImplEx.class, "outClaw");
//        outArm = hardwareMap.get(ServoImplEx.class, "outArm");
//
//        leftRot.setPower(0);
//        rightRot.setPower(0);
//        rot.setPosition(rotHome);
//        outClaw.setPosition(clawClosed);
//        outArm.setPosition(outArmHome);
//    }
//
//    public void update() {
//        outController.setPID(outp, outi, outd);
//        outPos = out.getCurrentPosition();
//        double outpid = outController.calculate(outPos, outTarget);
//        out.setPower(outpid);
//
//        extendoController.setPID(extendop, extendoi, extendod);
//        extendoPos = extendo.getCurrentPosition();
//        double extendopid = extendoController.calculate(extendoPos, extendoTarget);
//        extendo.setPower(extendopid);
//
//        leftRot.setPower(leftRotPower);
//        rightRot.setPower(rightRotPower);
//        rot.setPosition(rotPos);
//        outClaw.setPosition(outClawPos);
//        outArm.setPosition(outArmPos);
//    }
//
//    public void update(Gamepad gamepad1, Gamepad gamepad2) {
//        outController.setPID(outp, outi, outd);
//        int outPos = out.getCurrentPosition();
//        double outpid = outController.calculate(outPos, outTarget);
//        out.setPower(outpid);
//
//        extendoController.setPID(extendop, extendoi, extendod);
//        int extendoPos = extendo.getCurrentPosition();
//        double extendopid = extendoController.calculate(extendoPos, extendoTarget);
//        extendo.setPower(extendopid);
//
//        leftRot.setPower(leftRotPower);
//        rightRot.setPower(rightRotPower);
//        rot.setPosition(rotPos);
//        outClaw.setPosition(outClawPos);
//        outArm.setPosition(outArmPos);
//
//        clawToggle.update(gamepad1.y);
//        rotToggle.update(gamepad1.a);
//
//        if (clawToggle.value() == false || (extendoTarget > 500 && extendoTarget < 600)) {
//            outClawPos = clawOpen;
//        } else {
//            outClawPos = clawClosed;
//        }
//
//        if (gamepad2.b) {
//            outtake(1);
//        } else if (gamepad1.x) {
//            resetIntake();
//        } else if (rotToggle.value() == true || extendoTarget > 1500) {
//            rotPos = rotIntake;
//            intake(1);
//        } else {
//            rotPos = rotHome;
//            intake(0);
//        }
//
//        if (gamepad1.b) {
//            if (outArmPos == outArmScoreBucket) {
//                resetOut();
//            }   else if (outArmPos == outArmScoreSpec) {
//                outTarget = -200;
//                if (outPos < 1350) {
//                    resetOut();
//                }
//            }
//        }
//
//        if (gamepad1.back) {
//            outTarget = 300;
//        }
//
//        if (gamepad1.left_bumper) {
//            highBucket();
//        }
//
//        if (gamepad1.right_bumper) {
//            highBar();
//        }
//
//        if (gamepad2.y) {
//            lowBucket();
//        }
//
//        if (gamepad1.left_trigger > 0 && extendoTarget > -1000) {
//            extendoTarget -= 8;
//        }
//
//        if (gamepad1.right_trigger > 0 && extendoTarget < 1000) {
//            extendoTarget += 8;
//        }
//
//        if (gamepad2.left_bumper) {
//            extendoTarget -= 8;
//        }
//
//        if (gamepad2.right_bumper) {
//            extendoTarget += 8;
//        }
//
//        if (gamepad2.left_trigger > 0) {
//            outTarget -= 10;
//        }
//
//        if (gamepad2.right_trigger > 0) {
//            outTarget += 15;
//        }
//
////        if (gamepad2.left_bumper) {
////            extendoTarget -= 15;
////        }
////        if (gamepad2.right_bumper) {
////            extendoTarget += 1500;
////        }
//        if (gamepad2.x) {
//            out.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            out.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        }
//
//        if (gamepad2.a) {
//            extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        }
//    }
//
//    //Extendo is Dynamic: Gamepad 1 Triggers
//    //Outtake Slides are Automatic: Transfer, then - low bucket - **gamepad2.y**, high bucket - left bumper, high rung - right bumper
//    //Reset Intake - Gamepad1.y
//    //Score/Reset Out is One Button: Gamepad1.a
//    //gamepad1.x flip down and start intaking, unless gamepad1.b which ejects
//    //transfer does not transfer unless slides are reset
//    //
//    // Gamepad2 is backups, that can be triggered by gamepad1 (if gamepad1 dcs gamepad2 auto becomes gamepad1)
//    public void resetIntake() {
//        extendoTarget = -200;
//        rotPos = rotHome;
//        intake(0);
//    }
//
//    public void resetOut() {
//        outTarget = -200;
//        outArmPos = outArmHome;
//        resetIntake();
//    }
//    public void lowBucket() {
//        extendoTarget = 150;
//        outArmPos = outArmScoreBucket;
//        outTarget = lowBucket;
//    }
//
//    public void highBucket() {
//        extendoTarget = 150;
//        outArmPos = outArmScoreBucket;
//        outTarget = highBucket;
//    }
//
//    public void highBar() {
//        extendoTarget = 150;
//        outArmPos = outArmScoreSpec;
//        outTarget = highBar;
//    }
//
////    public void setLeftClaw(double val) {
////        leftClawPos = val;
////    }
////
////    public void setRightClaw(double val) {
////        rightClawPos = val;
////    }
//
//    public void setClawPos(double val) {
//        outClawPos = val;
//    }
//    public void setRot(double val) {
//        rotPos = val;
//    }
//
//    public void setOutArm(double val) {
//        outArmPos = val;
//    }
//
//    public void intake(double mag) {
//        leftRotPower = 1*mag;
//        rightRotPower = -1*mag;
//    }
//
//    public void outtake(double mag) {
//        leftRotPower = -1*mag;
//        rightRotPower = 1*mag;
//    }
//
//    public void setExtendoTarget(int val) {
//        extendoTarget = val;
//    }
//
//    public void setOutTarget(int val) {
//        outTarget = val;
//    }
//
//    public void reverseExtendoDir() {
//        out.setDirection(DcMotorSimple.Direction.REVERSE);
//    }
//    public void forwardExtendoDir() {
//        out.setDirection(DcMotorSimple.Direction.FORWARD);
//    }
//}
