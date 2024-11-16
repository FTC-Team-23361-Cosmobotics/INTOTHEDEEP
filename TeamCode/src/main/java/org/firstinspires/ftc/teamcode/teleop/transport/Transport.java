package org.firstinspires.ftc.teamcode.teleop.transport;

import static org.firstinspires.ftc.teamcode.teleop.AllianceStorage.isRed;
import static org.firstinspires.ftc.teamcode.test.GetTransportPositions.armMotor;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleop.utils.Toggle;

public class Transport {
    private Toggle clawToggle, outtakeToggle, rotToggle, driv2;
    private ServoImplEx outClaw, rot, outArm;
    private CRServoImplEx leftRot, rightRot;

    private DcMotorEx extendo, out;
    private PIDController extendoController;

    public double extendoPos, outPos;

    public static double extendop = .01, extendoi = .0001, extendod = 0.0001;
    public static int extendoTarget = 0;
    private PIDController outController;

    public static double outp = 0.01, outi = .0001, outd = .0001;
    public static int outTarget = 0;

//    public int mode = 0;

//    //Safe, Intaking Ground, Intaking Med, Intaking Top, Parallels (.5, 1st, 1.5, 2nd, 2.5, 3rd, 3.5), Hang
//    public static final int[] armPositions = {0, 3100, 2975, 2900, 450, 500, 600, 775, 775, 850, 900, 1500};
//
//    //Safe, Extended, Mid-Way
//    public static final int[] slidesPositions = {0, 1500, 3000, 1500, 1750, 2700, 3000};
//    //Safe, Deploy, Intaking, Intaking Off-Ground, Parallels (.5, 1st, 1.5, 2 - 3.5), Hang
//    public static final double[] intakeRotPositions = {0, .9, .45, .42, .95, 1, .6};

    public static final double clawOpen = 1; //TODO: TUNE THESE
    public static final double clawClosed = 0;


    public static final double outArmHome = .15;
    public static final double outArmScore = .6; //TODO: TUNE


    public static final double rotHome = .95;
    public static final double rotIntake = .2;


    public static double outClawPos, leftRotPower, rightRotPower, outArmPos, rotPos;

    public static final int highBucket = 3500; //TODO: TUNE VVV
    public static final int lowBucket = 1500;
    public static final int highBar = 2000;

//    public boolean armInRange;
//    public boolean slidesInRange;
//    public boolean slidesAtZero;

    //Neutral, In-taking, Out-taking
//    public enum TPos {
//        //Reset:
//        RESET("RESET", armPositions[0], intakeRotPositions[0], slidesPositions[0]),
//
//        //Deploy:
//        DEPLOY("DEPLOY", armPositions[1], intakeRotPositions[2], slidesPositions[0]),
//
//        //Intaking Positions:
//        INTAKING_MED_GROUND("INTAKING_CLOSE_GROUND", armPositions[1], intakeRotPositions[2], slidesPositions[1]),
//        INTAKING_FAR_GROUND("INTAKING_FAR_GROUND", armPositions[1], intakeRotPositions[2], slidesPositions[6]),
//        INTAKING_CLOSE_MEDSTACK("INTAKING_CLOSE_MEDSTACK", armPositions[2], intakeRotPositions[6], slidesPositions[0]),
//        //TODO: TUNE ^^^
//        INTAKING_FAR_MEDSTACK("INTAKING_FAR_MEDSTACK", armPositions[2], intakeRotPositions[6], slidesPositions[1]),
//        //TODO: TUNE ^^^
//        INTAKING_CLOSE_TOPSTACK("INTAKING_CLOSE_TOPSTACK", armPositions[3], intakeRotPositions[6], slidesPositions[0]),
//        INTAKING_FAR_TOPSTACK("INTAKING_FAR_TOPSTACK", armPositions[3], intakeRotPositions[6], slidesPositions[1]),
//        //TODO: TUNE ^^^
//        //Outtaking Positions:
//        OUTTAKING_1("OUTTAKING_1", armPositions[4], intakeRotPositions[4], slidesPositions[1]),
//
//        OUTTAKING_2("OUTTAKING_2", armPositions[5], intakeRotPositions[4], slidesPositions[1]),
//
//        OUTTAKING_3("OUTTAKING_3", armPositions[6], intakeRotPositions[4], slidesPositions[1]),
//
//        OUTTAKING_4("OUTTAKING_4", armPositions[7], intakeRotPositions[5], slidesPositions[1]),
//        OUTTAKING_5("OUTTAKING_5", armPositions[7], intakeRotPositions[5], slidesPositions[5]),
//        OUTTAKING_6("OUTTAKING_6", armPositions[9], intakeRotPositions[5], slidesPositions[5]),
//        OUTTAKING_7("OUTTAKING_7", armPositions[9], intakeRotPositions[5], slidesPositions[2]),
//
//        HANG("HANG", armPositions[11], intakeRotPositions[6], slidesPositions[2]);
//
//        private final String debug;
//        private final int armPosition;
//        private final double intakeRotPosition;
//
//        private final int slidesPosition;
//
//        TPos(String debug, int armPosition, double intakeRotPosition, int slidesPosition) {
//            this.debug = debug;
//            this.armPosition = armPosition;
//            this.intakeRotPosition = intakeRotPosition;
//            this.slidesPosition = slidesPosition;
//        }
//
//        public String toString() {
//            return debug;
//        }
//
//        public int armPos() {
//            return armPosition;
//        }
//
//        public double intakeRotPos() {
//            return intakeRotPosition;
//        }
//
//        public int slidesPos() {
//            return slidesPosition;
//        }
//    }
//
//    public TPos transportPos = TPos.RESET;

    public Transport(HardwareMap hardwareMap) {
        clawToggle = new Toggle(false);
        outtakeToggle = new Toggle(false);
        rotToggle = new Toggle(false);
        driv2 = new Toggle(false);

        extendoController = new PIDController(extendop, extendoi, extendod);
        extendo = hardwareMap.get(DcMotorEx.class, "extendo");
        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //TODO: Possibly Reset is Not Needed
        extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendoController.setPID(extendop, extendoi, extendod);
        extendoPos = extendo.getCurrentPosition();

        outController = new PIDController(outp, outi, outd);
        out = hardwareMap.get(DcMotorEx.class, "out");
        out.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //TODO: Possibly Reset is Not Needed
        out.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        out.setDirection(DcMotorSimple.Direction.REVERSE);
        outController.setPID(outp, outi, outd);
        outPos = out.getCurrentPosition();

        leftRot = hardwareMap.get(CRServoImplEx.class, "leftRot");
        rightRot = hardwareMap.get(CRServoImplEx.class, "rightRot");
        rot = hardwareMap.get(ServoImplEx.class, "rot");
        outClaw = hardwareMap.get(ServoImplEx.class, "outClaw");
        outArm = hardwareMap.get(ServoImplEx.class, "outArm");

        leftRot.setPower(0);
        rightRot.setPower(0);
        rot.setPosition(rotIntake);
        outClaw.setPosition(clawClosed);
        outArm.setPosition(outArmHome);
    }


//    public void setTPos() {
//        armInRange = Math.abs(transportPos.armPos() - armMotor.getCurrentPosition()) < 50;
//        slidesInRange = Math.abs(transportPos.slidesPos() - slidesMotor.getCurrentPosition()) < 15;
//        slidesAtZero = slidesMotor.getCurrentPosition() < 50;
//        if (!slidesAtZero && !armInRange) {
//            slidesTarget = 0;
//        }
//        if (slidesAtZero && !armInRange) {
//            armTarget = transportPos.armPos();
//        }
//        if (!slidesInRange && armInRange) {
//            slidesTarget = transportPos.slidesPos();
//        }
//
//        if (mode == 1 && slidesAtZero && armInRange) {
//            leftIntake.setPosition(clawPositions[2] - .12);
//            rightIntake.setPosition(clawPositions[2]);
//        } else if (mode == 0 && transportPos.debug != "AUTO_DEPLOY" || slidesAtZero && transportPos.debug != "AUTO_DEPLOY") {
//            leftIntake.setPosition(clawPositions[0] - .12);
//            rightIntake.setPosition(clawPositions[0]);
//        } else {
//            leftIntake.setPosition(clawPositions[leftClawPos] - .12);
//            rightIntake.setPosition(clawPositions[rightClawPos]);
//        }
//
//        if (mode != 2) {
//            intakeRotation.setPosition(transportPos.intakeRotPos());
//        }
//        if (mode == 2 && !armInRange) {
//            intakeRotation.setPosition(0.35);
//        }
//        if (mode == 2 && armInRange) {
//            intakeRotation.setPosition(transportPos.intakeRotPos());
//        }
//    }

    public void update() {
        outController.setPID(outp, outi, outd);
        outPos = out.getCurrentPosition();
        double outpid = outController.calculate(outPos, outTarget);
        out.setPower(outpid);

        extendoController.setPID(extendop, extendoi, extendod);
        extendoPos = extendo.getCurrentPosition();
        double extendopid = extendoController.calculate(extendoPos, extendoTarget);
        extendo.setPower(extendopid);

        leftRot.setPower(leftRotPower);
        rightRot.setPower(rightRotPower);
        rot.setPosition(rotPos);
        outClaw.setPosition(outClawPos);
        outArm.setPosition(outArmPos);
    }

    public void update(Gamepad gamepad1, Gamepad gamepad2) {
        clawToggle.update(gamepad2.y);
        outtakeToggle.update(gamepad2.x);
        rotToggle.update(gamepad2.a);
        driv2.update(gamepad1.back);

        outController.setPID(outp, outi, outd);
        int outPos = out.getCurrentPosition();
        double outpid = outController.calculate(outPos, outTarget);
        out.setPower(outpid);

        extendoController.setPID(extendop, extendoi, extendod);
        int extendoPos = extendo.getCurrentPosition();
        double extendopid = extendoController.calculate(extendoPos, extendoTarget);
        extendo.setPower(extendopid);

        leftRot.setPower(leftRotPower);
        rightRot.setPower(rightRotPower);
        rot.setPosition(rotPos);
        outClaw.setPosition(outClawPos);
        outArm.setPosition(outArmPos);

        if (outtakeToggle.value() == true) {
            outArmPos = outArmScore;
        } else {
            outArmPos = outArmHome;
        }

        if (rotToggle.value() == true) { // || extendoPos > 2000
            rotPos = rotIntake;
        } else {
            rotPos = rotHome;
        }

        if (gamepad1.b) {
            intake(1);
        }

        if (gamepad1.x) {
            outtake(1);
        }

        if (gamepad1.a) {
            intake(0);
        }

        if (gamepad1.y) {
            transfer();
        }

        if (gamepad1.left_bumper) {
            highBucket();
        }

        if (gamepad1.right_bumper) {
            highBar();
        }

        if (gamepad2.y) {
            lowBucket();
        }

        if (clawToggle.value() == true) {
            outClawPos = clawOpen;
        } else {
            outClawPos = clawClosed;
        }

        if (gamepad1.left_trigger > 0 && extendoTarget > -30) {
            extendoTarget -= 15;
        }

        if (gamepad1.right_trigger > 0 && extendoTarget < 2015) {
            extendoTarget += 15;
        }

        if (gamepad2.left_trigger > 0) {
            outTarget -= 10;
        }

        if (gamepad2.right_trigger > 0) {
            outTarget += 15;
        }
       //FULL DRIVER2 CONTROL
        if (driv2.value() == true) {
            //TODO: FILL WITH SAME CONTROLS AS DRIVER 1 BUT FOR DRIVER 2
        }

//        if (gamepad2.left_bumper) {
//            extendoTarget -= 15;
//        }
//        if (gamepad2.right_bumper) {
//            extendoTarget += 1500;
//        }
        if (gamepad2.b) {
            out.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            out.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    //Extendo is Dynamic: Gamepad 1 Triggers
    //Outtake Slides are Automatic: Transfer, then - low bucket - **gamepad2.y**, high bucket - left bumper, high rung - right bumper
    //Reset Intake - Gamepad1.y
    //Score/Reset Out is One Button: Gamepad1.a
    //gamepad1.x flip down and start intaking, unless gamepad1.b which ejects
    //transfer does not transfer unless slides are reset
    //
    // Gamepad2 is backups, that can be triggered by gamepad1 (if gamepad1 dcs gamepad2 auto becomes gamepad1)
    public void resetIntake() {
        extendoTarget = 0;
        rotPos = rotHome;
        intake(0);
    }

    public void resetOut() {
        outPos = -200;
        outArmPos = outArmHome;
        outClawPos = clawOpen;
    }
    public void lowBucket() {
        transfer();
        outArmPos = outArmScore;
        outTarget = lowBucket;
    }

    public void highBucket() {
        transfer();
        outArmPos = outArmScore;
        outTarget = highBucket;
    }

    public void highBar() {
        transfer();
        outArmPos = outArmScore;
        outTarget = highBar;
    }

//    public void setLeftClaw(double val) {
//        leftClawPos = val;
//    }
//
//    public void setRightClaw(double val) {
//        rightClawPos = val;
//    }

    public void transfer() {
        //outtake(.35);
        outClawPos = clawClosed;
        rotPos = rotIntake;
    }

    public void reset() {
        leftRotPower = 0;
        rightRotPower = 0;
        rotPos = rotIntake;
        outClawPos = clawOpen;
    }
    public void setClawPos(double val) {
        outClawPos = val;
    }
    public void setRot(double val) {
        rotPos = val;
    }

    public void setOutArm(double val) {
        outArmPos = val;
    }

    public void intake(double mag) {
        leftRotPower = -1*mag;
        rightRotPower = 1*mag;
    }

    public void outtake(double mag) {
        leftRotPower = 1*mag;
        rightRotPower = -1*mag;
    }

    public void setExtendoTarget(int val) {
        extendoTarget = val;
    }

    public void setOutTarget(int val) {
        outTarget = val;
    }

    public void reverseExtendoDir() {
        out.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void forwardExtendoDir() {
        out.setDirection(DcMotorSimple.Direction.FORWARD);
    }
}
