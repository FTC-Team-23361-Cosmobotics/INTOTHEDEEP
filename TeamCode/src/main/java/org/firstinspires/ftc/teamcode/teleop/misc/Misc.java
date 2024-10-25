package org.firstinspires.ftc.teamcode.teleop.misc;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.teleop.utils.Toggle;

import java.util.List;

public class Misc {
    public Toggle readyDrone;
    public Toggle fireDrone;
    public static ServoImplEx droneServo;
    public static ServoImplEx droneRot;
    public List<LynxModule> allHubs;
    public LynxModule CtrlHub;

    public LynxModule ExpHub;

    //Yellow, White, Green, Purple
    public int[] Colors = {Integer.parseInt("FFFF00", 16), Integer.parseInt("FFFFFF", 16), Integer.parseInt("00FF00", 16), Integer.parseInt("800080", 16)};

    public Misc (HardwareMap hardwareMap) {
        allHubs = hardwareMap.getAll(LynxModule.class);
        CtrlHub = allHubs.get(0);
        ExpHub = allHubs.get(1);
        readyDrone = new Toggle(false);
        fireDrone = new Toggle(false);
        droneServo = hardwareMap.get(ServoImplEx.class, "droneLauncher");
        droneRot = hardwareMap.get(ServoImplEx.class, "droneRotation");

        droneServo.setDirection(Servo.Direction.FORWARD);
        droneRot.setDirection(Servo.Direction.REVERSE);

    }
    public void update(Gamepad gamepad2) {
        double y = -gamepad2.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad2.left_stick_x;
        double rx = gamepad2.right_stick_x;
        double ry = -gamepad2.right_stick_y;

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        readyDrone.update(gamepad2.left_stick_button);
        fireDrone.update(gamepad2.right_stick_button);
        if (readyDrone.value() == true) {
            droneRot.setPosition(.35);
            if (fireDrone.value() == true) {
                droneServo.setPosition(.3);
            }
        } else {
            droneRot.setPosition(0);
            droneServo.setPosition(0);
        }

        if (y > .9) {
            ExpHub.setConstant(Colors[0]);
        }
        if (x > .9) {
            ExpHub.setConstant(Colors[1]);
        }
        if (y < -.9) {
            ExpHub.setConstant(Colors[2]);
        }
        if (x < -.9) {
            ExpHub.setConstant(Colors[3]);
        }
        if (ry > .9) {
            CtrlHub.setConstant(Colors[0]);
        }
        if (rx > .9) {
            CtrlHub.setConstant(Colors[1]);
        }
        if (ry < -.9) {
            CtrlHub.setConstant(Colors[2]);
        }
        if (rx < -.9) {
            CtrlHub.setConstant(Colors[3]);
        }
    }
}