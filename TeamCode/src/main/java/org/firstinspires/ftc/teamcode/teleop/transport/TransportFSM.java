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
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.teleop.utils.Toggle;

public class TransportFSM {

    public Toggle intakeToggle;
    private ServoImplEx specClawRoll, specClaw, rot, flap, bucketPitch, bucketYaw;
    private DcMotorEx extendo, out, arm, intake;

    private PIDController extendoController, outController, armController, intakeController;
    private ColorSensor sampleColor = new ColorRangeSensor() {
        @Override
        public int red() {
            return 0;
        }

        @Override
        public int green() {
            return 0;
        }

        @Override
        public int blue() {
            return 0;
        }

        @Override
        public int alpha() {
            return 0;
        }

        @Override
        public int argb() {
            return 0;
        }

        @Override
        public void enableLed(boolean enable) {

        }

        @Override
        public void setI2cAddress(I2cAddr newAddress) {

        }

        @Override
        public I2cAddr getI2cAddress() {
            return null;
        }

        @Override
        public double getDistance(DistanceUnit unit) {
            return 0;
        }

        @Override
        public double getLightDetected() {
            return 0;
        }

        @Override
        public double getRawLightDetected() {
            return 0;
        }

        @Override
        public double getRawLightDetectedMax() {
            return 0;
        }

        @Override
        public String status() {
            return "";
        }

        @Override
        public NormalizedRGBA getNormalizedColors() {
            return null;
        }

        @Override
        public float getGain() {
            return 0;
        }

        @Override
        public void setGain(float newGain) {

        }

        @Override
        public Manufacturer getManufacturer() {
            return null;
        }

        @Override
        public String getDeviceName() {
            return "";
        }

        @Override
        public String getConnectionInfo() {
            return "";
        }

        @Override
        public int getVersion() {
            return 0;
        }

        @Override
        public void resetDeviceConfigurationForOpMode() {

        }

        @Override
        public void close() {

        }
    };

    //COLOR LOGIC:
    public final int invalid = 0;
    public final int valid = 1;
    public final int empty = 2;
    //PID Values:

    public final double extendop = .02, extendoi = .0001, extendod = 0.0001, outp = 0.01, outi = .0001, outd = .0001, armp = 0, armi = 0, armd = 0, armf = 0, intakep = 0, intakei = 0, intaked = 0;
    public double extendopid, outpid, armpid, intakepid;
    public double armff;
    public final double arm_ticks_in_degrees = 1425.1 / 360;
    public final double zeroOffset = 84;

    //SERVO POSITIONS
    public double specClawRollPos, specClawPos, rotPos, flapPos, bucketPitchPos, bucketYawPos;


    //MOTOR POSITIONS

    public int extendoPos, outPos, armPos;
    //MOTOR POWER
    public double intakePower;

    //MOTOR TARGETS
    public int extendoTarget, outTarget, armTarget, intakeTarget;

    //SERVO VALUES
//TODO: TUNE
    public final double flapOpen = -1;
    public final double flapClosed = 1;

    public final double bucketYawHome = 0;
    public final double bucketYawSpit = .5;

    public final double rotIntake = 0;
    public final double rotHome = 1;

    public final double bucketPitchHome = 0;
    public final double bucketPitchScore = .75;

    public final double specClawRollIntake = 0;
    public final double specClawRollOuttake = 1;


    public final double specClawOpen = 0;
    public final double specClawClosed = 1;

    //MOTOR POSITIONS
    //TODO: TUNE
    public final int extendoHome = 0;
    public final int extendoLower = -200;
    public final int extendoUpper = 800;
    public final int extended = 200;
    public final int increment = 15;
    public final int outHome = 0;
    public final int outDump = 700;
    public final int dumpTrigger = 600;
    public final int outLowBucket = 600;
    public final int lowFlipTrigger = 300;
    public final int outHighBucket = 2500;
    public final int highFlipTrigger = 2300;
    public final int armHome = 0;
    public final int armLowBar = 700;
    public final int armHighBar = 1500;
    public final int intaking = 1000;
    public final int transferring = -1000;
    public final int dormant = 0;



    //ElapsedTimes:
    ElapsedTime sampleWait;
    ElapsedTime specimenWait;

    //Wait Values:
    public final int shortTransferWait = 1;
    public final int longTransferWait = 2;
    public final double dumpWait = .75;

    //Get Sample Color
    public String getColor() {
        if (sampleColor.red() < 128 && sampleColor.blue() > 128) {
            return "Blue";
        } else if (sampleColor.red() > 128 && sampleColor.blue() < 128) {
            return "Red";
        } else if (sampleColor.red() > 128 && sampleColor.blue() > 128) {
            return "Yellow";
        }
        return "Null";
    }

    public int isValid() {
        if (getColor() == "Blue" && isRed) {
            return invalid;
        } else if (getColor() == "Red" && !isRed) {
            return invalid;
        } else if (getColor() == "Null") {
            return empty;
        } else {
            return valid;
        }
    }

    //FSMS:
    public enum SampleTransport {
        SAMPLE_HOME,
        INTAKE,
        OUTTAKE,
        EMERGENCY_OUTTAKE,
        TRANSFER,
        LOW_BUCKET,
        HIGH_BUCKET,
        DUMP

    }

    SampleTransport sampleTransport = SampleTransport.SAMPLE_HOME;

    public enum SpecimenTransport {
        SPECIMEN_HOME,
        INTAKE_SPEC,
        LOW_BAR,
        HIGH_BAR,
        SPECIMEN_SCORE
    }

    SpecimenTransport specimenTransport = SpecimenTransport.SPECIMEN_HOME;

    //Set States
    public int sampleState, specState;
    public static final int sampleHome = 0, sampleIntake = 1, sampleOuttake = 2, sampleTransfer = 3, sampleLowBucket = 4, sampleHighBucket = 5, sampleDump = 6, specHome = 0, specIntake = 1, specLowBar = 2, specHighBar = 3, specScore = 4;

    public boolean extendSlides, retractSlides;


    public TransportFSM(HardwareMap hardwareMap) {
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

        armController = new PIDController(armp, armi, armd);
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //TODO: Possibly Reset is Not Needed
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armController.setPID(armp, armi, armd);
        armPos = arm.getCurrentPosition();

        intakeController = new PIDController(intakep, intakei, intaked);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeController.setPID(intakep, intakei, intaked);
        intakePower = intake.getVelocity();

        specClawRoll = hardwareMap.get(ServoImplEx.class, "specClawRoll");
        specClaw = hardwareMap.get(ServoImplEx.class, "specClaw");
        rot = hardwareMap.get(ServoImplEx.class, "rot");
        flap = hardwareMap.get(ServoImplEx.class, "flap");
        bucketPitch = hardwareMap.get(ServoImplEx.class, "bucketPitch");
        bucketYaw = hardwareMap.get(ServoImplEx.class, "bucketYaw");

        intake.setVelocity(0);
        specClawRoll.setPosition(specClawRollIntake);
        specClaw.setPosition(specClawOpen);
        rot.setPosition(rotHome);
        flap.setPosition(flapClosed);
        bucketPitch.setPosition(bucketPitchHome);
        bucketYaw.setPosition(bucketYawHome);

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

        armPos = arm.getCurrentPosition();
        armpid = outController.calculate(outPos, outTarget);
        armff = Math.sin(Math.toRadians(armPos / arm_ticks_in_degrees + zeroOffset)) * armf;
        arm.setPower(armpid + armff);

        intakePower = intake.getVelocity();
        intakepid = intakeController.calculate(intakePower, intakeTarget);
        intake.setPower(intakePower);

        specClawRoll.setPosition(specClawRollPos);
        specClaw.setPosition(specClawPos);
        rot.setPosition(rotPos);
        flap.setPosition(flapPos);
        bucketPitch.setPosition(bucketPitchPos);
        bucketYaw.setPosition(bucketYawPos);

        updateFSM();
    }

    public void update(Gamepad gamepad1, Gamepad gamepad2) {
        outPos = out.getCurrentPosition();
        outpid = outController.calculate(outPos, outTarget);
        out.setPower(outpid);

        extendoPos = extendo.getCurrentPosition();
        extendopid = extendoController.calculate(extendoPos, extendoTarget);
        extendo.setPower(extendopid);

        armPos = arm.getCurrentPosition();
        armpid = outController.calculate(outPos, outTarget);
        armff = Math.sin(Math.toRadians(armPos / arm_ticks_in_degrees + zeroOffset)) * armf;
        arm.setPower(armpid + armff);

        intakePower = intake.getVelocity();
        intakepid = intakeController.calculate(intakePower, intakeTarget);
        intake.setPower(intakePower);

        specClawRoll.setPosition(specClawRollPos);
        specClaw.setPosition(specClawPos);
        rot.setPosition(rotPos);
        flap.setPosition(flapPos);
        bucketPitch.setPosition(bucketPitchPos);
        bucketYaw.setPosition(bucketYawPos);

        if (gamepad1.x) {
            sampleState = sampleHome;
        }
        if (gamepad1.y) {
            sampleState = sampleDump;
        }
        //GAMEPAD1.A IS TAKEN

        if (gamepad1.b) {
            specState = specIntake;
        }
        if (gamepad1.left_bumper) {
            specState = specHighBar;
        }
        if (gamepad1.right_bumper) {
            sampleState = sampleHighBucket;
        }

        if (gamepad1.left_trigger > 0) {
            retractSlides = true;
        } else {
            retractSlides = false;
        }

        if (gamepad1.right_trigger > 0) {
            extendSlides = true;
        } else {
            extendSlides = false;
        }


        if (gamepad2.y) {
            sampleState = sampleDump;
        }
        if (gamepad2.a) {
            sampleState = sampleTransfer;
        }
        if (gamepad2.b) {
            sampleState = sampleLowBucket;
        }

        if (gamepad2.left_bumper) {
            specState = specLowBar;
        }
        if (gamepad2.right_bumper) {
            specState = specScore;
        }

        if (gamepad2.left_trigger > 0) {

        }
        if (gamepad2.right_trigger > 0) {

        }

        updateFSM();
    }

    public void updateFSM() {
        switch (sampleTransport) {
            case SAMPLE_HOME:
                intakePower = dormant;
                rotPos = rotHome;
                bucketPitchPos = bucketPitchHome;
                bucketYawPos = bucketYawHome;
                outTarget = outHome;
                flapPos = flapClosed;
                if (intakeToggle.value() == true || sampleState == sampleIntake) {
                    sampleTransport = SampleTransport.INTAKE;
                }
                if ( (extendoPos <= extended) && (sampleState == sampleTransfer || isValid() == valid) ) {
                    sampleWait.reset();
                    sampleTransport = SampleTransport.TRANSFER;
                }
                if (sampleState == sampleLowBucket) {
                    sampleTransport = SampleTransport.LOW_BUCKET;
                }
                if (sampleState == sampleHighBucket) {
                    sampleTransport = SampleTransport.HIGH_BUCKET;
                }
                if (sampleState == sampleDump) {
                    sampleTransport = SampleTransport.DUMP;
                }
                break;
            case INTAKE:
                intakePower = intaking;
                rotPos = rotIntake;
                flapPos = flapClosed;
                if (intakeToggle.value() == false || sampleState == sampleHome) {
                    sampleTransport = SampleTransport.SAMPLE_HOME;
                }
                if (sampleState == sampleOuttake || (isValid() == invalid)) {
                    sampleWait.reset();
                    sampleTransport = SampleTransport.OUTTAKE;
                }
                break;
            case OUTTAKE:
                //intakePower = outtaking;
                flapPos = flapOpen;
                if (isValid() != invalid && sampleWait.seconds() >= shortTransferWait) {
                    sampleTransport = SampleTransport.INTAKE;
                }
                if (isValid() == invalid && sampleWait.seconds() >= longTransferWait) {
                    sampleWait.reset();
                    sampleTransport = SampleTransport.EMERGENCY_OUTTAKE;
                }
                break;
            case EMERGENCY_OUTTAKE:
                intakePower = transferring;
                if (sampleWait.seconds() >= longTransferWait) {
                    sampleTransport = SampleTransport.INTAKE;
                }
                break;
            case TRANSFER:
                intakePower = transferring;
                if (sampleWait.seconds() > shortTransferWait) {
                    sampleTransport = SampleTransport.SAMPLE_HOME;
                }
                break;
            case LOW_BUCKET:
                outTarget = outLowBucket;
                if (outPos >= lowFlipTrigger || sampleState == sampleDump) {
                    sampleWait.reset();
                    sampleTransport = SampleTransport.DUMP;
                }
                break;
            case HIGH_BUCKET:
                outTarget = outHighBucket;
                if (outPos >= highFlipTrigger || sampleState == sampleDump) {
                    sampleWait.reset();
                    sampleTransport = SampleTransport.DUMP;
                }
                break;
            case DUMP:
                outTarget = outDump;
                if (outPos >= dumpTrigger) {
                    bucketPitchPos = bucketPitchScore;
                    bucketYawPos = bucketYawSpit;
                }
                if (sampleWait.seconds() > dumpWait) {
                    sampleTransport = SampleTransport.SAMPLE_HOME;
                }
                break;
            default:
                sampleTransport = sampleTransport.SAMPLE_HOME;
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


        if (retractSlides && extendoTarget >= extendoLower) {
            extendoTarget -= increment;
        }

        if (extendSlides && extendoTarget <= extendoUpper) {
            extendoTarget += increment;
        }

        if (sampleState == sampleHome && sampleTransport != SampleTransport.SAMPLE_HOME) {
            extendoTarget = extendoLower;
            sampleTransport = SampleTransport.SAMPLE_HOME;
            specimenTransport = SpecimenTransport.SPECIMEN_HOME;
        }

        switch (specimenTransport) {
            case SPECIMEN_HOME:
                specClawRollPos = specClawRollIntake;
                specClawPos = specClawOpen;
                armTarget = armHome;
                if (specState == specIntake) {
                    specimenWait.reset();
                    specimenTransport = SpecimenTransport.INTAKE_SPEC;
                }
                break;
            case INTAKE_SPEC:
                specClawPos = specClawClosed;
                if (specState == specHighBar) {
                    specimenTransport = SpecimenTransport.HIGH_BAR;
                }
                if (specState == specLowBar) {
                    specimenTransport = SpecimenTransport.LOW_BAR;
                }
                break;
            case LOW_BAR:
                specClawRollPos = specClawRollOuttake;
                armTarget = armLowBar;
                if (specState == specScore) {
                    specimenTransport = SpecimenTransport.SPECIMEN_SCORE;
                }
                break;
            case HIGH_BAR:
                specClawRollPos = specClawRollOuttake;
                armTarget = armHighBar;
                if (specState == specScore) {
                    specimenTransport = SpecimenTransport.SPECIMEN_SCORE;
                }
                break;
            case SPECIMEN_SCORE:
                specClawPos = specClawOpen;
            default:
                specimenTransport = specimenTransport.SPECIMEN_HOME;
        }
    }
}
