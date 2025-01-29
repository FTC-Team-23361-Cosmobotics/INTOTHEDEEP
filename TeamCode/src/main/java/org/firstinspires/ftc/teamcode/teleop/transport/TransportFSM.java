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
import org.firstinspires.ftc.teamcode.teleop.transport.EncoderStorage;



import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.teleop.utils.Toggle;
import android.graphics.Color;

public class TransportFSM {
    public boolean test = false;
    public TouchSensor zeroLimit, outLimit;

    public Toggle intakeToggle, isSpecToggle;
    private ServoImplEx rot, bucketPitch, specClaw, specRot, specArm, flicker;
    private CRServoImplEx outWheel;
    //    private ServoImplEx flap;
//    private ServoImplEx bucketYaw;
    //private ServoImplEx specClawRoll, specClaw;
    public static DcMotorEx extendo, out;
    public DcMotorEx intake;

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
    public boolean isHighBucket = true;

    public final double extendop = .02, extendoi = .0001, extendod = 0.0001, outp = 0.01, outi = .0001, outd = .0001;
    //    public final double armp = 0, armi = 0, armd = 0, armf = 0;
    public double extendopid, outpid, armpid;
//    public double armff;
//    public final double arm_ticks_in_degrees = 1425.1 / 360;
//    public final double zeroOffset = 84;

    //SERVO POSITIONS
    public double rotPos, outWheelPower, bucketPitchPos, flickerPos, specArmPos, specClawPos, specRotPos;

    //MOTOR POSITIONS

    public int extendoPos, outPos, armPos;
    //MOTOR POWER
    public double intakePower;

    //MOTOR TARGETS
    public int extendoTarget, outTarget, armTarget;

    //SERVO VALUES
    public static double flapOpenRotIntake = .5;
    public static double flapClosedrotHome = .35;
    public static double flapClosedrotIntake = 1;

    public static double bucketYawHome = 0.15;
    public static double bucketYawSpit = .5;

    public static double rotIntake = .9;
    public static double rotPrep = .6;
    public static double rotHome = .25;
    public static double rotOuttake = .25;

    public static double bucketPitchHome = .815;
    public static double bucketPitchPrep = .55;
    public static double bucketPitchScore = .3;

    public static double specClawRollIntake = .15;
    public static double specClawRollOuttake = .86;

    public static double specClawOpen = .6;
    public static double specClawClosed = 0;
    public static double specArmHome = .05;
    public static double specRotPrep = .2;
    public static double specArmScore = .53;
    public static double specArmClear = .25;
    //MOTOR POSITIONS
    public static double flickerRetracted = .6;
    public static double flickerOut = 0.1;
    public static int autoExtendoUpper = 675;
    public static int extendoUpper = 765;
    public static int extendoLower = 0;
    public static int extended = 115;
    public static int transferTrigger = 0;
    public static int increment = 20;
    public static int outHome = 0;
    public static int outDump = 700;
    //    public final int dumpTrigger = 600;
    public static int outLowBucket = 600;
    //    public final int lowFlipTrigger = 300;
    public static int outHighBucket = 2200;
    //    public final int highFlipTrigger = 2300;
//    public final int armHome = 0;
//    public final int armLowBar = 700;
//    public final int armHighBar = 1500;
    public static double intaking = 1;
    public static double transferring = -1;
    public static double maintaining = 0;
    public static double dormant = 0;
    public static double outtaking = -1;
//    public static double flicking = -.35;
    ColorSensor colorSensor;
    public static double hue;
    public static int validSample = 0;
    public static double blueSample = 222;
    public static double redSample = 23;
    public static double yellowSample = 84;
    public static double hueError = 35;
    public float hsvValues[] = {0F, 0F, 0F};


    //ElapsedTimes:
    ElapsedTime sampleWait;
    ElapsedTime specimenWait;

    //Wait Values:
    public static double shortshortTransferWait = .5;
    public static double longlongTransferWait = 1.75;
    public static double shortTransferWait = .25;
    public static double longTransferWait = .5;
    public static double emergencyWit = .75;
    public static double dumpWait = .75;
    public static double specRotWait = .2;
    public static double specScoreWait = .85;
    public static double specRetractWait = .65;
    public static boolean isSpec = true;

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
        HIGH_BUCKET,
        DUMP

    }

    public SampleTransport sampleTransport = SampleTransport.SAMPLE_HOME;

    public enum SpecimenTransport {
        SPECIMEN_HOME,
        PREP,
        INTAKE_SPEC,
        SCORE,
        OPEN,
        PREP_HOME,
        EMERGENCY_PREP
    }

    public SpecimenTransport specimenTransport = SpecimenTransport.SPECIMEN_HOME;

    //Set States
    public int sampleState, specState;
    public static final int sampleHome = 0, sampleIntake = 1, sampleOuttake = 2, sampleTransfer = 3, sampleLowBucket = 4, sampleHighBucket = 5, sampleDump = 6, specHome = 0, specIntake = 1, specLowBar = 2, specHighBar = 3, specScore = 4;

    public boolean extendSlides, retractSlides;


    public TransportFSM(HardwareMap hardwareMap) {
        zeroLimit = hardwareMap.get(TouchSensor.class, "zeroLimit");
        outLimit = hardwareMap.get(TouchSensor.class, "outLimit");
        intakeToggle = new Toggle(false);
        isSpecToggle = new Toggle(true);
//        if (isRed) {
//            isRedToggle = new Toggle(true);
//        } else {
//            isRedToggle = new Toggle(false);
//        }

        sampleWait = new ElapsedTime();
        specimenWait = new ElapsedTime();
        sampleWait.reset();
        specimenWait.reset();

        //TODO: SAVE POSITION FROM AUTO (OR AUTO RESET) FOR MOTORS

        extendoController = new PIDController(extendop, extendoi, extendod);
        extendo = hardwareMap.get(DcMotorEx.class, "extendo");
        extendoController.setPID(extendop, extendoi, extendod);
        if (EncoderStorage.isAuto) {
            extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            extendoPos = extendo.getCurrentPosition();
        }

        outController = new PIDController(outp, outi, outd);
        out = hardwareMap.get(DcMotorEx.class, "out");
        out.setDirection(DcMotorSimple.Direction.REVERSE);
        outController.setPID(outp, outi, outd);
        if (EncoderStorage.isAuto) {
            out.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            out.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            outPos = out.getCurrentPosition();
        }


//        armController = new PIDController(armp, armi, armd);
//        arm = hardwareMap.get(DcMotorEx.class, "arm");
//        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        armController.setPID(armp, armi, armd);
//        armPos = arm.getCurrentPosition();
//
//        intakeController = new PIDController(intakep, intakei, intaked);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
//        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        intakeController.setPID(intakep, intakei, intaked);
//        intakePower = intake.getVelocity();

//        specClawRoll = hardwareMap.get(ServoImplEx.class, "specClawRoll");
//        specClaw = hardwareMap.get(ServoImplEx.class, "specClaw");
        rot = hardwareMap.get(ServoImplEx.class, "rot");
        rot.setDirection(Servo.Direction.REVERSE);
//        flap = hardwareMap.get(ServoImplEx.class, "flap");
//        flap.setDirection(Servo.Direction.REVERSE);
        bucketPitch = hardwareMap.get(ServoImplEx.class, "bucketPitch");
        specClaw = hardwareMap.get(ServoImplEx.class, "specClaw");
        specRot = hardwareMap.get(ServoImplEx.class, "specRot");
        specArm = hardwareMap.get(ServoImplEx.class, "specArm");
        flicker = hardwareMap.get(ServoImplEx.class, "flicker");
        outWheel = hardwareMap.get(CRServoImplEx.class, "outWheel");
        outWheel.setDirection(DcMotorSimple.Direction.REVERSE);
//        bucketYaw = hardwareMap.get(ServoImplEx.class, "bucketYaw");

        intake.setPower(dormant);
        outWheel.setPower(dormant);
//        specClawRoll.setPosition(specClawRollIntake);
//        specClaw.setPosition(specClawOpen);
        rot.setPosition(rotHome);
//        flap.setPosition(flapClosedrotHome);
        bucketPitch.setPosition(bucketPitchHome);
        specClaw.setPosition(specClawClosed);
        specRot.setPosition(specClawRollIntake);
        specArm.setPosition(specArmHome);
        flicker.setPosition(flickerRetracted);
//        bucketYaw.setPosition(bucketYawHome);
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        colorSensor.enableLed(true);
        //INIT LED COLOR: purpleRGB!
    }

    public void setSampleState(int sampleState) {
        this.sampleState = sampleState;
    }

    public void setSpecState(int specState) {
        this.specState = specState;
    }

    public void update() {
        validSample = updateColor();
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
//        flap.setPosition(flapPos);
        bucketPitch.setPosition(bucketPitchPos);
//        bucketYaw.setPosition(bucketYawPos);
        specArm.setPosition(specArmPos);
        specClaw.setPosition(specClawPos);
        specRot.setPosition(specRotPos);
        flicker.setPosition(flickerPos);
        outWheel.setPower(outWheelPower);

        validSample = updateColor();

        //TODO: FSM UPDTE FOR UTO
        switch (sampleTransport) {
            case SAMPLE_HOME:
                intakePower = dormant;
                rotPos = rotHome;
                bucketPitchPos = bucketPitchHome;
//                bucketYawPos = bucketYawHome;
                outTarget = outHome;
//                flapPos = flapClosedrotHome;
                extendoTarget = extendoLower;
                outWheelPower = dormant;
                break;
            case EXTENDED:
                outTarget = outHome;
                bucketPitchPos = bucketPitchHome;
                intakePower = maintaining;
                //chnge lter
                rotPos = rotPrep;
                extendoTarget = extendoUpper;
                outWheelPower = dormant;
                break;
            case INTAKE:
                intakePower = intaking;
                rotPos = rotIntake;
                outWheelPower = dormant;
//                flapPos = flapClosedrotIntake;
                break;
            case EMERGENCY_OUTTAKE:
                outWheelPower = dormant;
                intakePower = outtaking;
                rotPos = rotOuttake;
                break;
            case OUTTAKE:
                outWheelPower = outtaking;
                break;
            case RETRACTING:
                rotPos = rotHome;
                intakePower = maintaining;
                specimenTransport = SpecimenTransport.SPECIMEN_HOME;
                break;
            case TRANSFER:
                rotPos = rotHome;
                intakePower = transferring;
                extendoTarget = extendoLower;
                if (sampleWait.seconds() > shortTransferWait) {
                    if (isHighBucket && (specimenTransport == SpecimenTransport.SPECIMEN_HOME || specimenTransport == SpecimenTransport.INTAKE_SPEC)) {
                        sampleTransport = SampleTransport.HIGH_BUCKET;
                    } else if (!isHighBucket && (specimenTransport == SpecimenTransport.SPECIMEN_HOME || specimenTransport == SpecimenTransport.INTAKE_SPEC)){
                        sampleTransport = SampleTransport.DUMP;
                    } else {
                        sampleTransport = SampleTransport.SAMPLE_HOME;
                    }
                }
                break;
            case HIGH_BUCKET:
                extendoTarget = extended;
                intakePower = dormant;
                outTarget = outHighBucket;
                bucketPitchPos = bucketPitchPrep;
                break;
            default:
                sampleTransport = sampleTransport.SAMPLE_HOME;
        }

        if (zeroLimit.isPressed()) {
            extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (outLimit.isPressed()) {
            out.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            out.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        switch (specimenTransport) {
            case SPECIMEN_HOME:
                specArmPos = specArmHome;
                specRotPos = specClawRollIntake;
                specClawPos = specClawOpen;
                break;
            case INTAKE_SPEC:
                specArmPos = specArmHome;
                specRotPos = specClawRollIntake;
                specClawPos = specClawClosed;
                break;
            case PREP:
                specArmPos = specArmScore;
                specRotPos = specClawRollIntake;
                break;
            case SCORE:
                specRotPos = specClawRollOuttake;
                break;
            case OPEN:
                specClawPos = specClawOpen;
                specArmPos = specArmClear;
                break;
            case PREP_HOME:
                specRotPos = specClawRollIntake;
                break;
            case EMERGENCY_PREP:
                specRotPos = specClawRollIntake;
                break;
        }
    }


    public void setExtendoTarget(int val) {
        extendoTarget = val;
    }

    public void flickerOut(boolean active) {
        if (active) {
            flickerPos = flickerOut;
        } else {
            flickerPos = flickerRetracted;
        }
    }


    public void update(Gamepad gamepad1, Gamepad gamepad2) {
        validSample = updateColor();
        intakeToggle.update(gamepad1.a);
        isSpecToggle.update(gamepad1.back);
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
//        flap.setPosition(flapPos);
        bucketPitch.setPosition(bucketPitchPos);
//        bucketYaw.setPosition(bucketYawPos);
        specClaw.setPosition(specClawPos);
        specArm.setPosition(specArmPos);
        specRot.setPosition(specRotPos);
        flicker.setPosition(flickerPos);
        outWheel.setPower(outWheelPower);

        switch (sampleTransport) {
            case SAMPLE_HOME:
                intakePower = dormant;
                rotPos = rotHome;
                bucketPitchPos = bucketPitchHome;
//                bucketYawPos = bucketYawHome;
                outTarget = outHome;
//                flapPos = flapClosedrotHome;
                extendoTarget = extendoLower;
                outWheelPower = dormant;
                if (gamepad1.right_trigger > 0) {
                    sampleTransport = SampleTransport.EXTENDED;
                }
                if ((extendoPos <= extended) && gamepad1.left_bumper && (specimenTransport == SpecimenTransport.SPECIMEN_HOME || specimenTransport == SpecimenTransport.INTAKE_SPEC)) {
                    sampleWait.reset();
                    sampleTransport = SampleTransport.TRANSFER;
                }
                if (gamepad1.y && (specimenTransport == SpecimenTransport.SPECIMEN_HOME || specimenTransport == SpecimenTransport.INTAKE_SPEC)) {
                    sampleTransport = SampleTransport.DUMP;
                }
                if (gamepad1.right_bumper && (specimenTransport == SpecimenTransport.SPECIMEN_HOME || specimenTransport == SpecimenTransport.INTAKE_SPEC)) {
                    sampleTransport = SampleTransport.HIGH_BUCKET;
                }
                break;
            case EXTENDED:
                outTarget = outHome;
                bucketPitchPos = bucketPitchHome;
                intakePower = maintaining;
                //chnge lter
                rotPos = rotPrep;
                extendoTarget = extendoUpper;
                outWheelPower = dormant;
                if (gamepad1.left_bumper) {
                    intakeToggle.value = false;
                    sampleWait.reset();
                    isHighBucket = true;
                    sampleTransport = SampleTransport.RETRACTING;
                }
                if (gamepad1.right_bumper) {
                    intakeToggle.value = false;
                    sampleWait.reset();
                    isHighBucket = false;
                    sampleTransport = SampleTransport.RETRACTING;
                }
                if (gamepad1.left_trigger > 0) {
                    sampleTransport = SampleTransport.SAMPLE_HOME;
                }
//                if (extendoPos < extended) {
//                    sampleTransport = SampleTransport.SAMPLE_HOME;
//                }
                if (intakeToggle.value() == true) {
                    sampleTransport = SampleTransport.INTAKE;
                }
                if (gamepad1.y) {
                    sampleWait.reset();
                    sampleTransport = SampleTransport.EMERGENCY_OUTTAKE;
                }
                if (validSample == 0) {
                    sampleWait.reset();
                    sampleTransport = SampleTransport.OUTTAKE;
                }
                break;
            case INTAKE:
                intakePower = intaking;
                rotPos = rotIntake;
                outWheelPower = dormant;
//                flapPos = flapClosedrotIntake;
                if (intakeToggle.value() == false) {
                    sampleTransport = SampleTransport.EXTENDED;
                }
                if (gamepad1.left_bumper || (validSample == 1 && !isSpec)) {
                    intakeToggle.value = false;
                    sampleWait.reset();
                    isHighBucket = true;
                    sampleTransport = SampleTransport.RETRACTING;
                }
                if (gamepad1.right_bumper || (validSample == 1 && isSpec)) {
                    intakeToggle.value = false;
                    sampleWait.reset();
                    isHighBucket = false;
                    sampleTransport = SampleTransport.RETRACTING;
                }
                if (gamepad1.y) {
                    sampleWait.reset();
                    sampleTransport = SampleTransport.EMERGENCY_OUTTAKE;
                }
                if (validSample == 0) {
                    sampleWait.reset();
                    sampleTransport = SampleTransport.OUTTAKE;
                }
                break;
            case EMERGENCY_OUTTAKE:
                outWheelPower = intaking;
                intakePower = outtaking;
                rotPos = rotIntake;
                if (sampleWait.seconds() > emergencyWit) {
                    sampleTransport = SampleTransport.EXTENDED;
                }
                break;
            case OUTTAKE:
                outWheelPower = outtaking;
                if ((validSample != 0)) {
                    sampleTransport = SampleTransport.INTAKE;
                }
                if (gamepad1.y) {
                    sampleWait.reset();
                    sampleTransport = SampleTransport.EMERGENCY_OUTTAKE;
                }
//                if ((sampleWait.seconds() >= longlongTransferWait) && (validSample == 0)) {
//                    sampleWait.reset();
//                    sampleTransport = SampleTransport.EMERGENCY_OUTTAKE;
//                }
                break;
            case RETRACTING:
                rotPos = rotHome;
                outWheelPower = intaking;
                specimenTransport = SpecimenTransport.SPECIMEN_HOME;
                if (sampleWait.seconds() > emergencyWit) {
                    intakePower = dormant;
                    extendoTarget = -5;
                }
                if (gamepad1.left_bumper) {
                    isHighBucket = true;
                }
                if (gamepad1.right_bumper) {
                    isHighBucket = false;
                }
                if (gamepad1.right_trigger > 0) {
                    sampleTransport = SampleTransport.EXTENDED;
                }
                if (isHighBucket && (specimenTransport == SpecimenTransport.SPECIMEN_HOME || specimenTransport == SpecimenTransport.INTAKE_SPEC)) {
                    if (extendoPos < 50 || gamepad1.y) {
                        sampleWait.reset();
                        sampleTransport = SampleTransport.TRANSFER;
                    }
                } else {
                    if (extendoPos < 50 || gamepad1.y) {
                        sampleWait.reset();
                        sampleTransport = SampleTransport.TRANSFER;
                    }
                }
                break;
            case TRANSFER:
                rotPos = rotHome;
                intakePower = transferring;
                outWheelPower = intaking;
                extendoTarget = extendoLower;
                if (sampleWait.seconds() > shortTransferWait) {
                    if (isHighBucket && (specimenTransport == SpecimenTransport.SPECIMEN_HOME || specimenTransport == SpecimenTransport.INTAKE_SPEC)) {
                        sampleTransport = SampleTransport.HIGH_BUCKET;
                    } else if (!isHighBucket && (specimenTransport == SpecimenTransport.SPECIMEN_HOME || specimenTransport == SpecimenTransport.INTAKE_SPEC)){
                        sampleTransport = SampleTransport.SAMPLE_HOME;
                    } else {
                        sampleTransport = SampleTransport.SAMPLE_HOME;
                    }
                }
                break;
            case HIGH_BUCKET:
                outWheelPower = dormant;
                extendoTarget = extended;
                intakePower = dormant;
                outTarget = outHighBucket;
                bucketPitchPos = bucketPitchPrep;
                if (gamepad1.y) {
                    sampleTransport = SampleTransport.DUMP;
                }
                if (gamepad1.x) {
                    sampleTransport = SampleTransport.SAMPLE_HOME;
                }
                break;
            case DUMP:
                outWheelPower = dormant;
                extendoTarget = extended;
                bucketPitchPos = bucketPitchScore;
//                bucketYawPos = bucketYawSpit;
                intakePower = dormant;
                if (gamepad1.left_trigger > 0) {
                    sampleTransport = SampleTransport.SAMPLE_HOME;
                }
                if (gamepad1.right_trigger > 0) {
                    sampleTransport = SampleTransport.EXTENDED;
                }
                break;
            default:
                sampleTransport = sampleTransport.SAMPLE_HOME;
        }

        if (gamepad1.dpad_right && sampleTransport != SampleTransport.SAMPLE_HOME) {
//            extendoTarget = extendoLower;
            sampleTransport = SampleTransport.SAMPLE_HOME;
//            specimenTransport = SpecimenTransport.SPECIMEN_HOME;
        }

        if (zeroLimit.isPressed() || gamepad1.dpad_up) {
            extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (outLimit.isPressed()) {
            out.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            out.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (gamepad1.dpad_left) {
            flickerOut(true);
        } else {
            flickerOut(false);
        }

        if (isSpecToggle.value() == true) {
            isSpec = true;
        } else {
            isSpec = false;
        }

        switch (specimenTransport) {
            case SPECIMEN_HOME:
                specArmPos = specArmHome;
                specRotPos = specClawRollIntake;
                specClawPos = specClawOpen;
                if (gamepad1.b) {
                    specimenTransport = SpecimenTransport.INTAKE_SPEC;
                }
                break;
            case INTAKE_SPEC:
                specArmPos = specArmHome;
                specRotPos = specClawRollIntake;
                specClawPos = specClawClosed;
                if (gamepad1.x && (sampleTransport != SampleTransport.HIGH_BUCKET && sampleTransport != SampleTransport.DUMP)) {
                    specimenWait.reset();
                    specimenTransport = SpecimenTransport.PREP;
                }
                break;
            case PREP:
                specArmPos = specArmScore;
                specRotPos = specClawRollIntake;
                if (specimenWait.seconds() > specRotWait) {
                    specimenWait.reset();
                    specimenTransport = SpecimenTransport.SCORE;
                }
                break;
            case SCORE:
                specRotPos = specClawRollOuttake;
                if (gamepad1.b) {
                    specimenWait.reset();
                    specimenTransport = SpecimenTransport.EMERGENCY_PREP;
                }
                if (gamepad1.x) {
                    specimenWait.reset();
                    specimenTransport = SpecimenTransport.OPEN;
                }
                break;
            case OPEN:
                specClawPos = specClawOpen;
                if (specimenWait.seconds() > specRetractWait || gamepad1.b) {
                    specimenWait.reset();
                    specimenTransport = SpecimenTransport.PREP_HOME;
                }
                break;
            case PREP_HOME:
                specRotPos = specRotPrep;
                specClawPos = specClawClosed;
                specArmPos = specArmClear;
                if (specimenWait.seconds() > specScoreWait) {
                    specimenTransport = SpecimenTransport.SPECIMEN_HOME;
                }
                break;
            case EMERGENCY_PREP:
                specRotPos = specClawRollIntake;
                if (specimenWait.seconds() > specScoreWait) {
                    specimenTransport = SpecimenTransport.INTAKE_SPEC;
                }
                break;
        }

        if (gamepad1.dpad_down && specimenTransport != SpecimenTransport.SPECIMEN_HOME) {
            specimenTransport = SpecimenTransport.SPECIMEN_HOME;
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

    public int updateColor() {
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
        hue = hsvValues[0];
        if (Math.abs(hue - yellowSample) < hueError) {
            if (isSpec) {
                return 0;
            } else {
                return 1;
            }
        }
        if (Math.abs(hue - redSample) < hueError) {
            if (isRed) {
                return 1;
            } else {
                return 0;
            }
        }
        if (Math.abs(hue - blueSample) < hueError) {
            if (isRed) {
                return 0;
            } else {
                return 1;
            }
        }
        return 2;
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
