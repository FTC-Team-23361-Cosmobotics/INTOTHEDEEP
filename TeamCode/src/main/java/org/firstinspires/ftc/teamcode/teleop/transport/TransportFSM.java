package org.firstinspires.ftc.teamcode.teleop.transport;

import static org.firstinspires.ftc.teamcode.teleop.AllianceStorage.isRed;

import android.graphics.Color;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.teleop.utils.Toggle;

public class TransportFSM {
    public boolean test = false;
    public TouchSensor zeroLimit;

    public Toggle intakeToggle;
    private ServoImplEx rot, flap, bucketPitch;
//    private ServoImplEx bucketYaw;
    //private ServoImplEx specClawRoll, specClaw;
    public DcMotorEx extendo, out, arm, intake;

    private PIDController extendoController, outController, intakeController;
//    private PIDController armController;
//    private ColorSensor sampleColor = new ColorRangeSensor() {
//        @Override
//        public int red() {
//            return 0;
//        }
//
//        @Override
//        public int green() {
//            return 0;
//        }
//
//        @Override
//        public int blue() {
//            return 0;
//        }
//
//        @Override
//        public int alpha() {
//            return 0;
//        }
//
//        @Override
//        public int argb() {
//            return 0;
//        }
//
//        @Override
//        public void enableLed(boolean enable) {
//
//        }
//
//        @Override
//        public void setI2cAddress(I2cAddr newAddress) {
//
//        }
//
//        @Override
//        public I2cAddr getI2cAddress() {
//            return null;
//        }
//
//        @Override
//        public double getDistance(DistanceUnit unit) {
//            return 0;
//        }
//
//        @Override
//        public double getLightDetected() {
//            return 0;
//        }
//
//        @Override
//        public double getRawLightDetected() {
//            return 0;
//        }
//
//        @Override
//        public double getRawLightDetectedMax() {
//            return 0;
//        }
//
//        @Override
//        public String status() {
//            return "";
//        }
//
//        @Override
//        public NormalizedRGBA getNormalizedColors() {
//            return null;
//        }
//
//        @Override
//        public float getGain() {
//            return 0;
//        }
//
//        @Override
//        public void setGain(float newGain) {
//
//        }
//
//        @Override
//        public Manufacturer getManufacturer() {
//            return null;
//        }
//
//        @Override
//        public String getDeviceName() {
//            return "";
//        }
//
//        @Override
//        public String getConnectionInfo() {
//            return "";
//        }
//
//        @Override
//        public int getVersion() {
//            return 0;
//        }
//
//        @Override
//        public void resetDeviceConfigurationForOpMode() {
//
//        }
//
//        @Override
//        public void close() {
//
//        }
//    };

    //COLOR LOGIC:
//    public final int invalid = 0;
//    public final int valid = 1;
//    public final int empty = 2;
    //PID Values:

    public final double extendop = .02, extendoi = .0001, extendod = 0.0001, outp = 0.01, outi = .0001, outd = .0001;
//    public final double armp = 0, armi = 0, armd = 0, armf = 0;
    public double extendopid, outpid, armpid;
//    public double armff;
//    public final double arm_ticks_in_degrees = 1425.1 / 360;
//    public final double zeroOffset = 84;

    //SERVO POSITIONS
    public double rotPos, flapPos, bucketPitchPos;
    public double specClawRollPos, specClawPos, bucketYawPos;

    //MOTOR POSITIONS

    public int extendoPos, outPos, armPos;
    //MOTOR POWER
    public double intakePower;

    //MOTOR TARGETS
    public int extendoTarget, outTarget, armTarget;

    //SERVO VALUES
//TODO: TUNE
    public static double flapOpenRotIntake = .5;
    public static double flapClosedrotHome = .35;
    public static double flapClosedrotIntake = 1;

    public static double bucketYawHome = 0.15;
    public static double bucketYawSpit = .5;

    public static double rotIntake = .9;
    public static double rotPrep = .6;
    public static double rotHome = .05;

    public static double bucketPitchHome = 1;
    public static double bucketPitchPrep = .75;
    public static double bucketPitchScore = .55;

    public static double specClawRollIntake = 0;
    public static double specClawRollOuttake = 1;


    public static double specClawOpen = 0;
    public static double specClawClosed = 1;

    //MOTOR POSITIONS
    //TODO: TUNE
    public static int autoExtendoUpper = 600;
    public static int extendoUpper = 750;
    //TODO: set bck to mx lter
    public static int extendoLower = 0;
    public static int extended = 300;
    public static int transferTrigger = 0;
    public static int increment = 20;
    public static int outHome = -5;
    public static int outDump = 700;
//    public final int dumpTrigger = 600;
    public static int outLowBucket = 600;
//    public final int lowFlipTrigger = 300;
    public static int outHighBucket = 2250;
//    public final int highFlipTrigger = 2300;
//    public final int armHome = 0;
//    public final int armLowBar = 700;
//    public final int armHighBar = 1500;
    public static double intaking = 1;
    public static double transferring = -.5;
    public static double maintaining = .2;
    public static double dormant = 0;



    //ElapsedTimes:
    ElapsedTime sampleWait;
    ElapsedTime specimenWait;

    //Wait Values:
    public static int shortTransferWait = 1;
    public static double longTransferWait = .5;
    public static double dumpWait = .75;

    //Get Sample Color
//    public String getColor() {
//        if (sampleColor.red() < 128 && sampleColor.blue() > 128) {
//            return "Blue";
//        } else if (sampleColor.red() > 128 && sampleColor.blue() < 128) {
//            return "Red";
//        } else if (sampleColor.red() > 128 && sampleColor.blue() > 128) {
//            return "Yellow";
//        }
//        return "Null";
//    }
//
//    public int isValid() {
//        if (getColor() == "Blue" && isRed) {
//            return invalid;
//        } else if (getColor() == "Red" && !isRed) {
//            return invalid;
//        } else if (getColor() == "Null") {
//            return empty;
//        } else {
//            return valid;
//        }
//    }

    //FSMS:
    public enum SampleTransport {
        SAMPLE_HOME,
        INTAKE,
        EXTENDED,
        RETRACTING,
        OUTTAKE,
        EMERGENCY_OUTTAKE,
        TRANSFER,
        PREP_BUCKET,
        LOW_BUCKET,
        HIGH_BUCKET,
        DUMP

    }

    public SampleTransport sampleTransport = SampleTransport.SAMPLE_HOME;

//    public enum SpecimenTransport {
//        SPECIMEN_HOME,
//        INTAKE_SPEC,
//        LOW_BAR,
//        HIGH_BAR,
//        SPECIMEN_SCORE
//    }
//
//    SpecimenTransport specimenTransport = SpecimenTransport.SPECIMEN_HOME;

    //Set States
    public int sampleState, specState;
    public static final int sampleHome = 0, sampleIntake = 1, sampleOuttake = 2, sampleTransfer = 3, sampleLowBucket = 4, sampleHighBucket = 5, sampleDump = 6, specHome = 0, specIntake = 1, specLowBar = 2, specHighBar = 3, specScore = 4;

    public boolean extendSlides, retractSlides;


    public TransportFSM(HardwareMap hardwareMap) {
        zeroLimit = hardwareMap.get(TouchSensor.class, "zeroLimit");
        intakeToggle = new Toggle(false);

        sampleWait = new ElapsedTime();
        specimenWait = new ElapsedTime();
        sampleWait.reset();
        specimenWait.reset();

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

//        armController = new PIDController(armp, armi, armd);
//        arm = hardwareMap.get(DcMotorEx.class, "arm");
//        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //TODO: Possibly Reset is Not Needed
//        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        armController.setPID(armp, armi, armd);
//        armPos = arm.getCurrentPosition();
//
//        intakeController = new PIDController(intakep, intakei, intaked);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
//        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        intakeController.setPID(intakep, intakei, intaked);
//        intakePower = intake.getVelocity();

//        specClawRoll = hardwareMap.get(ServoImplEx.class, "specClawRoll");
//        specClaw = hardwareMap.get(ServoImplEx.class, "specClaw");
        rot = hardwareMap.get(ServoImplEx.class, "rot");
        rot.setDirection(Servo.Direction.REVERSE);
        flap = hardwareMap.get(ServoImplEx.class, "flap");
        flap.setDirection(Servo.Direction.REVERSE);
        bucketPitch = hardwareMap.get(ServoImplEx.class, "bucketPitch");
        bucketPitch.setDirection(Servo.Direction.REVERSE);
//        bucketYaw = hardwareMap.get(ServoImplEx.class, "bucketYaw");

        intake.setPower(dormant);
//        specClawRoll.setPosition(specClawRollIntake);
//        specClaw.setPosition(specClawOpen);
        rot.setPosition(rotHome);
        flap.setPosition(flapClosedrotHome);
        bucketPitch.setPosition(bucketPitchHome);
//        bucketYaw.setPosition(bucketYawHome);

        //INIT LED COLOR: purpleRGB!
    }

    public void setSampleState(int sampleState) {
        this.sampleState = sampleState;
    }

    public void setSpecState(int specState) {
        this.specState = specState;
    }
    //TODO: MAKE IT SO THE FSM CAN RUN DURING AUTO
    public void update() {
        outPos = out.getCurrentPosition();
        outpid = outController.calculate(outPos, outTarget);
        out.setPower(outpid);

        extendoPos = extendo.getCurrentPosition();
        extendopid = extendoController.calculate(extendoPos, extendoTarget);
        extendo.setPower(extendopid);

//        armPos = arm.getCurrentPosition();
//        armpid = outController.calculate(outPos, outTarget);
//        armff = Math.sin(Math.toRadians(armPos / arm_ticks_in_degrees + zeroOffset)) * armf;
//        arm.setPower(armpid + armff);

        intake.setPower(intakePower);

//        specClawRoll.setPosition(specClawRollPos);
//        specClaw.setPosition(specClawPos);
        rot.setPosition(rotPos);
        flap.setPosition(flapPos);
        bucketPitch.setPosition(bucketPitchPos);
//        bucketYaw.setPosition(bucketYawPos);
        switch (sampleTransport) {
            case SAMPLE_HOME:
                intakePower = dormant;
                rotPos = rotHome;
                bucketPitchPos = bucketPitchHome;
                bucketYawPos = bucketYawHome;
                outTarget = outHome;
                flapPos = flapClosedrotHome;
                break;
            case INTAKE:
                intakePower = intaking;
                rotPos = rotIntake;
                flapPos = flapClosedrotIntake;
                break;
            case EXTENDED:
                intakePower = intaking;
                rotPos = rotHome;
                flapPos = flapClosedrotHome;
                break;
            case OUTTAKE:
                intakePower = transferring;
                break;
            case EMERGENCY_OUTTAKE:
                intakePower = transferring;
                break;
            case TRANSFER:
                extendoTarget = 10;
                intakePower = transferring;
                break;
            case LOW_BUCKET:
                outTarget = outLowBucket;
                break;
            case HIGH_BUCKET:
                extendoTarget = 75;
                intakePower = dormant;
                outTarget = outHighBucket;
                break;
            case DUMP:
                bucketPitchPos = bucketPitchScore;
                bucketYawPos = bucketYawSpit;
                break;
            default:
                sampleTransport = sampleTransport.SAMPLE_HOME;
        }

        if (zeroLimit.isPressed()) {
            extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (out.getCurrent(CurrentUnit.AMPS) > 5) {
            out.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            out.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }


    public void setExtendoTarget(int val) {
        extendoTarget = val;
    }
    public void setOutTarget(int val) {
        outTarget = val;
    }

    public void setIntakePower(double val) {
        intakePower = val;
    }

    public void setRotPos(double val) {
        rotPos = val;
    }
    public void setBucketPitchPos(double val) {
        bucketPitchPos = val;
    }


    public void update(Gamepad gamepad1, Gamepad gamepad2) {
        intakeToggle.update(gamepad1.a);

        outPos = out.getCurrentPosition();
        outpid = outController.calculate(outPos, outTarget);
        out.setPower(outpid);

        extendoPos = extendo.getCurrentPosition();
        extendopid = extendoController.calculate(extendoPos, extendoTarget);
        extendo.setPower(extendopid);

//        armPos = arm.getCurrentPosition();
//        armpid = outController.calculate(outPos, outTarget);
//        armff = Math.sin(Math.toRadians(armPos / arm_ticks_in_degrees + zeroOffset)) * armf;
//        arm.setPower(armpid + armff);

//        intakePower = intake.getVelocity();
//        intakepid = intakeController.calculate(intakePower, intakeTarget);
        intake.setPower(intakePower);

//        specClawRoll.setPosition(specClawRollPos);
//        specClaw.setPosition(specClawPos);
        rot.setPosition(rotPos);
        flap.setPosition(flapPos);
        bucketPitch.setPosition(bucketPitchPos);
//        bucketYaw.setPosition(bucketYawPos);

        switch (sampleTransport) {
            case SAMPLE_HOME:
                intakePower = dormant;
                rotPos = rotHome;
                bucketPitchPos = bucketPitchHome;
                bucketYawPos = bucketYawHome;
                outTarget = outHome;
                flapPos = flapClosedrotHome;
                if ((intakeToggle.value() == false) && extendoPos > extended) {
                    sampleTransport = SampleTransport.EXTENDED;
                }
                if ((intakeToggle.value() == true) && extendoPos >= extended) {
                    sampleTransport = SampleTransport.INTAKE;
                }
                if ((extendoPos <= extended) && gamepad1.left_bumper) {
                    sampleWait.reset();
                    sampleTransport = SampleTransport.TRANSFER;
                }
                if (gamepad2.a) {
                    sampleTransport = SampleTransport.LOW_BUCKET;
                }
                if (gamepad1.right_bumper) {
                    sampleTransport = SampleTransport.HIGH_BUCKET;
                }
                if (gamepad2.x) {
                    sampleTransport = SampleTransport.DUMP;
                }
                break;
            case EXTENDED:
                intakePower = maintaining;
                //chnge lter
                rotPos = rotPrep;
                if (gamepad1.left_bumper) {
                    sampleWait.reset();
                    sampleTransport = SampleTransport.RETRACTING;
                }
                if (extendoPos < extended) {
                    sampleTransport = SampleTransport.SAMPLE_HOME;
                }
                if (intakeToggle.value() == true) {
                    sampleTransport = SampleTransport.INTAKE;
                }
                break;
            case INTAKE:
                intakePower = intaking;
                rotPos = rotIntake;
//                flapPos = flapClosedrotIntake;
                if (intakeToggle.value() == false) {
                    sampleTransport = SampleTransport.EXTENDED;
                }
                if (gamepad1.b) {
                    sampleWait.reset();
                    sampleTransport = SampleTransport.OUTTAKE;
                }
                break;
            case OUTTAKE:
                intakePower = transferring;
                if (sampleWait.seconds() >= longTransferWait) {
                    sampleTransport = SampleTransport.INTAKE;
                }
                break;
            case EMERGENCY_OUTTAKE:
                intakePower = transferring;
                if (sampleWait.seconds() >= longTransferWait) {
                    sampleTransport = SampleTransport.INTAKE;
                }
                break;
            case RETRACTING:
                rotPos = rotHome;
                intakePower = maintaining;
                //TODO: SWITCH BCK TO MINTINING LTER, HRDWRE BROKEN
                if (sampleWait.seconds() >= longTransferWait); {
                    extendoTarget = -10;
                }
                if (intakeToggle.value()) {
                    sampleTransport = SampleTransport.INTAKE;
                }
                if (extendoPos < 100 || gamepad1.right_bumper) {
                    sampleWait.reset();
                    sampleTransport = SampleTransport.TRANSFER;
                }
                break;
            case TRANSFER:
                rotPos = rotHome;
                intakePower = transferring;
                extendoTarget = -3;
                if (sampleWait.seconds() > shortTransferWait) {
                    sampleTransport = SampleTransport.HIGH_BUCKET;
                }
                break;
            case LOW_BUCKET:
                outTarget = outLowBucket;
                bucketPitchPos = bucketPitchPrep;
                if (gamepad1.y) {
                    sampleWait.reset();
                    sampleTransport = SampleTransport.DUMP;
                }
                break;
            case HIGH_BUCKET:
                extendoTarget = extended;
                intakePower = dormant;
                outTarget = outHighBucket;
                bucketPitchPos = bucketPitchPrep;
                if (gamepad1.y) {
                    sampleTransport = SampleTransport.DUMP;
                }
                break;
            case DUMP:
                bucketPitchPos = bucketPitchScore;
                bucketYawPos = bucketYawSpit;
                if (extendoPos > extended + 25) {
                    sampleTransport = SampleTransport.SAMPLE_HOME;
                }
                break;
            default:
                sampleTransport = sampleTransport.SAMPLE_HOME;
        }
        if (gamepad1.left_trigger > 0 && extendoTarget > extendoLower) {
            extendoTarget -= increment;
        }

        if (gamepad1.right_trigger > 0 && extendoTarget <= extendoUpper) {
            extendoTarget += increment;
        }

        if (gamepad1.x && sampleTransport != SampleTransport.SAMPLE_HOME) {
//            extendoTarget = extendoLower;
            sampleTransport = SampleTransport.SAMPLE_HOME;
//            specimenTransport = SpecimenTransport.SPECIMEN_HOME;
        }

        if (zeroLimit.isPressed() || gamepad1.dpad_up) {
            extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if ((out.getCurrent(CurrentUnit.AMPS) > 4.5 || gamepad2.y) && outTarget == outHome) {
            out.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            out.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        /*
        update():
        if intaking:
        if color is red: LED Red
        if blue blue
        if yellow yellow
        if color is white/Null: LED White
        else if scoring:
        color is green
        else:
        color is purpleRGB
         */



    }


//    public void updateFSM() {
//        switch (sampleTransport) {
//            case SAMPLE_HOME:
//                intakePower = dormant;
//                rotPos = rotHome;
//                bucketPitchPos = bucketPitchHome;
//                bucketYawPos = bucketYawHome;
//                outTarget = outHome;
//                flapPos = flapClosed;
//                if (intakeToggle.value() == true) {
//                    sampleTransport = SampleTransport.INTAKE;
//                }
//                if ( (extendoPos <= extended) && gamepad1.y ) {
//                    sampleWait.reset();
//                    sampleTransport = SampleTransport.TRANSFER;
//                }
//                if (sampleState == sampleLowBucket) {
//                    sampleTransport = SampleTransport.LOW_BUCKET;
//                }
//                if (sampleState == sampleHighBucket) {
//                    sampleTransport = SampleTransport.HIGH_BUCKET;
//                }
//                if (sampleState == sampleDump) {
//                    sampleTransport = SampleTransport.DUMP;
//                }
//                break;
//            case INTAKE:
//                intakePower = intaking;
//                rotPos = rotIntake;
//                flapPos = flapClosed;
//                if (intakeToggle.value() == false || sampleState == sampleHome) {
//                    sampleTransport = SampleTransport.SAMPLE_HOME;
//                }
//                if (sampleState == sampleOuttake || (isValid() == invalid)) {
//                    sampleWait.reset();
//                    sampleTransport = SampleTransport.OUTTAKE;
//                }
//                break;
//            case OUTTAKE:
//                //intakePower = outtaking;
//                flapPos = flapOpen;
//                if (isValid() != invalid && sampleWait.seconds() >= shortTransferWait) {
//                    sampleTransport = SampleTransport.INTAKE;
//                }
//                if (isValid() == invalid && sampleWait.seconds() >= longTransferWait) {
//                    sampleWait.reset();
//                    sampleTransport = SampleTransport.EMERGENCY_OUTTAKE;
//                }
//                break;
//            case EMERGENCY_OUTTAKE:
//                intakePower = transferring;
//                if (sampleWait.seconds() >= longTransferWait) {
//                    sampleTransport = SampleTransport.INTAKE;
//                }
//                break;
//            case TRANSFER:
//                intakePower = transferring;
//                if (sampleWait.seconds() > shortTransferWait) {
//                    sampleTransport = SampleTransport.SAMPLE_HOME;
//                }
//                break;
//            case LOW_BUCKET:
//                outTarget = outLowBucket;
//                if (outPos >= lowFlipTrigger || sampleState == sampleDump) {
//                    sampleWait.reset();
//                    sampleTransport = SampleTransport.DUMP;
//                }
//                break;
//            case HIGH_BUCKET:
//                outTarget = outHighBucket;
//                if (outPos >= highFlipTrigger || sampleState == sampleDump) {
//                    sampleWait.reset();
//                    sampleTransport = SampleTransport.DUMP;
//                }
//                break;
//            case DUMP:
//                outTarget = outDump;
//                if (outPos >= dumpTrigger) {
//                    bucketPitchPos = bucketPitchScore;
//                    bucketYawPos = bucketYawSpit;
//                }
//                if (sampleWait.seconds() > dumpWait) {
//                    sampleTransport = SampleTransport.SAMPLE_HOME;
//                }
//                break;
//            default:
//                sampleTransport = sampleTransport.SAMPLE_HOME;
//        }
//
//
//        /*
//        update():
//        if intaking:
//        if color is red: LED Red
//        if blue blue
//        if yellow yellow
//        if color is white/Null: LED White
//        else if scoring:
//        color is green
//        else:
//        color is purpleRGB
//         */
//
//
//        if (retractSlides && extendoTarget >= extendoLower) {
//            extendoTarget -= increment;
//        }
//
//        if (extendSlides && extendoTarget <= extendoUpper) {
//            extendoTarget += increment;
//        }
//
//        if (sampleState == sampleHome && sampleTransport != SampleTransport.SAMPLE_HOME) {
//            extendoTarget = extendoLower;
//            sampleTransport = SampleTransport.SAMPLE_HOME;
//            specimenTransport = SpecimenTransport.SPECIMEN_HOME;
//        }
//
//        switch (specimenTransport) {
//            case SPECIMEN_HOME:
//                specClawRollPos = specClawRollIntake;
//                specClawPos = specClawOpen;
//                armTarget = armHome;
//                if (specState == specIntake) {
//                    specimenWait.reset();
//                    specimenTransport = SpecimenTransport.INTAKE_SPEC;
//                }
//                break;
//            case INTAKE_SPEC:
//                specClawPos = specClawClosed;
//                if (specState == specHighBar) {
//                    specimenTransport = SpecimenTransport.HIGH_BAR;
//                }
//                if (specState == specLowBar) {
//                    specimenTransport = SpecimenTransport.LOW_BAR;
//                }
//                break;
//            case LOW_BAR:
//                specClawRollPos = specClawRollOuttake;
//                armTarget = armLowBar;
//                if (specState == specScore) {
//                    specimenTransport = SpecimenTransport.SPECIMEN_SCORE;
//                }
//                break;
//            case HIGH_BAR:
//                specClawRollPos = specClawRollOuttake;
//                armTarget = armHighBar;
//                if (specState == specScore) {
//                    specimenTransport = SpecimenTransport.SPECIMEN_SCORE;
//                }
//                break;
//            case SPECIMEN_SCORE:
//                specClawPos = specClawOpen;
//            default:
//                specimenTransport = specimenTransport.SPECIMEN_HOME;
//        }
//    }
}
