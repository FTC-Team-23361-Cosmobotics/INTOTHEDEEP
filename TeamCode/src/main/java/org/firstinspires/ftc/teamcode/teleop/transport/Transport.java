package org.firstinspires.ftc.teamcode.teleop.transport;

import static org.firstinspires.ftc.teamcode.teleop.AllianceStorage.isRed;
import static org.firstinspires.ftc.teamcode.test.GetTransportPositions.armMotor;
import static org.firstinspires.ftc.teamcode.test.GetTransportPositions.slidesMotor;

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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teleop.utils.Toggle;

public class Transport {
    private Toggle rotToggle, driv2;
    private ServoImplEx outClaw, rot, outArm, flap;
    private CRServoImplEx leftRot, rightRot;
    private DcMotorEx extendo, out, intake;
    private PIDController extendoController;

//    public ElapsedTime wait;
    public double extendoPos, outPos;
    public static double extendop = .01, extendoi = .0001, extendod = 0.0001;
    public static int extendoTarget = 0;
    public static final int extendoHome = -200;
    public static final int extendoMax = 800;
    private PIDController outController;
    public static double outp = 0.01, outi = .0001, outd = .0001;
    public static int outTarget = 0;
    public static final double flapClosedHome = 0.4;
    public static final double flapClosedIntake = 1;
    public static final double flapOpenIntake = 0.5;
    public static final double clawOpen = -1;
    public static final double clawClosed = 1;
    public static final double outArmHome = 1;
    public static final double outArmScoreBucket = .2;
    public static final double outArmScoreSpec = 1;
    public static final double rotHome = 0.1;
    public static boolean intakeInUse = false;
    public static final int intakeIn = 1;
    public static final int intakeOut = -1;
    public static final double rotIntake = 0.9;
    public static double outClawPos, leftRotPower, rightRotPower, outArmPos, rotPos;
    public static final int liftHome = -100;
    public static final int highBucket = 2300;
    public static final int lowBucket = 1500;
    public static final int highBar = 1600;
    public static final double transferWait = 1;
    public static final double scoreWait = 2;
    public enum SampleState {
        NEUTRAL,
        INTAKE_RETRACT,
        INTAKE_TRANSFER,
        BUCKET_LIFT,
        BUCKET_SCORE,
        LIFT_RETURN

    };
    SampleState sampleState;
    public long startTime;
    public long totalTime;

    public Transport(HardwareMap hardwareMap) {
//        wait = new ElapsedTime();

        rotToggle = new Toggle(false);
        driv2 = new Toggle(false);

        extendoController = new PIDController(extendop, extendoi, extendod);
        extendo = hardwareMap.get(DcMotorEx.class, "extendo");
        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Reset is only needed if you don't run auto
        extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendoController.setPID(extendop, extendoi, extendod);
        extendoPos = extendo.getCurrentPosition();

        outController = new PIDController(outp, outi, outd);
        out = hardwareMap.get(DcMotorEx.class, "out");
        out.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        out.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        out.setDirection(DcMotorSimple.Direction.REVERSE);
        outController.setPID(outp, outi, outd);
        outPos = out.getCurrentPosition();

        intake = hardwareMap.get(DcMotorEx.class, "intake");

        rot = hardwareMap.get(ServoImplEx.class, "rot");
        outArm = hardwareMap.get(ServoImplEx.class, "outArm");
        flap = hardwareMap.get(ServoImplEx.class, "flap");

        leftRot.setPower(0);
        rightRot.setPower(0);
        rot.setPosition(rotHome);
        outClaw.setPosition(clawOpen);
        outArm.setPosition(outArmHome);
        flap.setPosition(flapClosedHome);

        sampleState = SampleState.NEUTRAL;
    }

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
        rotToggle.update(gamepad1.a);
        driv2.update(gamepad2.back);

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


        switch (sampleState) {
            case NEUTRAL:
                if (gamepad1.x) {
                    extendoTarget = extendoHome;
                    rotPos = rotHome;
                    intakeInUse = true;
                    flap.setPosition(flapClosedHome);
                    sampleState = SampleState.INTAKE_RETRACT;
                } else if (gamepad1.right_bumper) {
                    outTarget = highBucket;
                    sampleState = SampleState.BUCKET_LIFT;
                }
                break;
            case INTAKE_RETRACT:
                if (extendo.getCurrentPosition() < 10) {
                    sampleState = SampleState.INTAKE_TRANSFER;
                    startTime = System.currentTimeMillis();
                    intake.setPower(intakeOut);
                }
                break;
            case INTAKE_TRANSFER:
                totalTime = System.currentTimeMillis() - startTime; // milliseconds
                if (totalTime > transferWait) {
                    intake.setPower(0);
                    intakeInUse = false;
                    sampleState = SampleState.NEUTRAL;
                }
                break;
            case BUCKET_LIFT:
                if (out.getCurrentPosition() > highBucket - 10 && gamepad1.y) {
                    outArmPos = outArmScoreBucket;
                    sampleState = SampleState.BUCKET_SCORE;
                    startTime = System.currentTimeMillis();
                }
                break;
            case BUCKET_SCORE:
                totalTime = System.currentTimeMillis() - startTime; // milliseconds
                if (totalTime > scoreWait) {
                    outTarget = liftHome;
                    outArmPos = outArmHome;
                    sampleState = SampleState.LIFT_RETURN;
                }
                break;
            case LIFT_RETURN:
                if (out.getCurrentPosition() < 10) {
                    sampleState = SampleState.NEUTRAL;
                }
                break;
        }

        if (!intakeInUse) {
            if (gamepad1.right_trigger > 0) {
                extendoTarget += 15;
                extendoTarget = Math.min(extendoMax, extendoTarget);
            } else if (gamepad1.left_trigger > 0) {
                extendoTarget -= 15;
                extendoTarget = Math.min(extendoHome, extendoTarget);
            }
            if (extendoTarget > 1500 || rotToggle.value()) { // rotToggle is the a button
                rotPos = rotIntake;
                intake.setPower(intakeIn);
                flap.setPosition(flapClosedIntake);
            } else {
                rotPos = rotHome;
                intake.setPower(0);
                flap.setPosition(flapClosedHome);
            }
        }


        if (gamepad2.left_trigger > 0) {
            outTarget -= 15;
        }

        if (gamepad2.right_trigger > 0) {
            outTarget += 15;
        }

        if (gamepad2.left_trigger > 0) {
            extendoTarget -= 15;
        }

        if (gamepad2.right_trigger > 0) {
            extendoTarget += 15;
        }


       //FULL DRIVER2 CONTROL


        if (gamepad2.b) {
            out.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            out.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (gamepad2.y) {
            extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extendo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }


    }
    public void resetIntake() {
        extendoTarget = 0;
        rotPos = rotHome;
        intake(0);
    }

    public void resetOut() {
//        wait.reset();
        outClawPos = clawOpen;
//        if (wait.seconds() > .5) {
            outPos = -200;
            outArmPos = outArmHome;
//        }
    }
    public void lowBucket() {
//        wait.reset();
        outClawPos = clawClosed;
        extendoTarget = 500;
        if (extendo.getCurrentPosition() > 300) {
            outArmPos = outArmScoreBucket;
            outTarget = lowBucket;
        }
    }

    public void highBucket() {
//        wait.reset();
        outClawPos = clawClosed;
//        if (wait.seconds() > .25) {
            extendoTarget = 500;
//        }
        if (extendo.getCurrentPosition() > 300) {
            outArmPos = outArmScoreBucket;
            outTarget = highBucket;
        }
    }

    public void highBar() {
//        wait.reset();
        outClawPos = clawClosed;
//        if (wait.seconds() > .25) {
            extendoTarget = 500;
//        }
        if (extendo.getCurrentPosition() > 300) {
            outArmPos = outArmScoreSpec;
            outTarget = highBar;
        }
    }

    //AUTON HARD CODES:
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
        leftRotPower = 1*mag;
        rightRotPower = -1*mag;
    }

    public void outtake(double mag) {
        leftRotPower = -1*mag;
        rightRotPower = 1*mag;
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
