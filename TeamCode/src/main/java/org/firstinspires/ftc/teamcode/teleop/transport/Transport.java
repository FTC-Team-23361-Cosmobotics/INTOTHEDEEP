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
    private Toggle clawToggle, bucketToggle, rotToggle;
    private ServoImplEx leftClaw, rightClaw, rot, bucket;
    private CRServoImplEx intake;

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

    public static final double leftClawOpen = .65;
    public static final double leftClawSizing = .2;
    public static final double leftClawClosed = .4;

    public static final double rightClawOpen = .35;
    public static final double rightClawSizing = .4;
    public static final double rightClawClosed = .05;


    public static final double bucketHome = .1; //TODO: WILL CHANGE
    public static final double bucketScore = .7;


    public static final double rotHome = .95; //TODO: WILL CHANGE
    public static final double rotIntake = .2;

    public static final double intaking = -1;
    public static final double dormant = 0;
    public static final double outtaking = 1;
    public static final double transfer = .15;


    public static double intakePower, leftClawPos, rightClawPos, bucketPos, rotPos;

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
        bucketToggle = new Toggle(false);
        rotToggle = new Toggle(false);

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

        leftClaw = hardwareMap.get(ServoImplEx.class, "leftClaw");
        rightClaw = hardwareMap.get(ServoImplEx.class, "rightClaw");
        rot = hardwareMap.get(ServoImplEx.class, "rot");
        intake = hardwareMap.get(CRServoImplEx.class, "intake");
        bucket = hardwareMap.get(ServoImplEx.class, "bucket");

        rightClaw.setDirection(Servo.Direction.REVERSE);

        leftClaw.setPosition(leftClawSizing);
        rightClaw.setPosition(rightClawSizing);
        rot.setPosition(rotIntake);
        intake.setPower(dormant);
        bucket.setPosition(bucketHome);
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

        rot.setPosition(rotPos);
        intake.setPower(intakePower);
        leftClaw.setPosition(leftClawPos);
        rightClaw.setPosition(rightClawPos);
        bucket.setPosition(bucketPos);
    }

    public void update(Gamepad gamepad1, Gamepad gamepad2) {
        clawToggle.update(gamepad1.y);
        bucketToggle.update(gamepad1.x);
        rotToggle.update(gamepad1.a);

        outController.setPID(outp, outi, outd);
        int outPos = out.getCurrentPosition();
        double outpid = outController.calculate(outPos, outTarget);
        out.setPower(outpid);

        extendoController.setPID(extendop, extendoi, extendod);
        int extendoPos = extendo.getCurrentPosition();
        double extendopid = extendoController.calculate(extendoPos, extendoTarget);
        extendo.setPower(extendopid);

        rot.setPosition(rotPos);
        intake.setPower(intakePower);
        leftClaw.setPosition(leftClawPos);
        rightClaw.setPosition(rightClawPos);
        bucket.setPosition(bucketPos);

        if (bucketToggle.value() == true) {
            bucketPos = bucketScore;
        } else {
            bucketPos = bucketHome;
        }

        if (rotToggle.value() == true || extendoPos > 2000) {
            rotPos = rotIntake;
        } else {
            rotPos = rotHome;
        }

        if (gamepad1.b) {
            if (extendoPos <= 50) {
                intakePower = transfer;
            } else {
                intakePower = outtaking;
            }
        } else if (rotPos == rotIntake) {
            intakePower = intaking;
        } else {
            intakePower = dormant;
        }

        if (clawToggle.value() == true) {
            leftClawPos = leftClawClosed;
            rightClawPos = rightClawClosed;
        } else {
            leftClawPos = leftClawOpen;
            rightClawPos = rightClawOpen;
        }

        if (gamepad1.left_trigger > 0 && extendoTarget > -30) {
            extendoTarget -= 15;
        }

        if (gamepad1.right_trigger > 0 && extendoTarget < 2315) {
            extendoTarget += 15;
        }

        if (gamepad2.left_trigger > 0 && extendoTarget > -100) {
            outTarget -= 10;
        }

        if (gamepad2.right_trigger > 0 && extendoTarget > 3500) {
            outTarget += 15;
        }
    }

    public void setRot(double val) {
        rotPos = val;
    }

    public void setBucket(double val) {
        bucketPos = val;
    }

    public void setLeftClaw(double val) {
        leftClawPos = val;
    }

    public void setRightClaw(double val) {
        rightClawPos = val;
    }

    public void setIntakePower(double val) {
        intakePower = val;
    }
}
