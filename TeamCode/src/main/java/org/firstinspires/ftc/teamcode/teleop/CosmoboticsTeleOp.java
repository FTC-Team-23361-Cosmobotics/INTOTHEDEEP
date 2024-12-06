package org.firstinspires.ftc.teamcode.teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.teleop.drive.Drive;
import org.firstinspires.ftc.teamcode.teleop.misc.Misc;
import org.firstinspires.ftc.teamcode.teleop.transport.Transport;

import java.util.List;

@Config
@TeleOp
public class CosmoboticsTeleOp extends OpMode {
    Drive drive;
    Transport transport;
    public List<LynxModule> allHubs;
    public LynxModule CtrlHub;

    public LynxModule ExpHub;

    @Override
    public void init() {
        drive = new Drive(hardwareMap);
        transport = new Transport(hardwareMap);

        allHubs = hardwareMap.getAll(LynxModule.class);
        CtrlHub = allHubs.get(0);
        ExpHub = allHubs.get(1);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        drive.update(gamepad1);
        transport.update(gamepad1, gamepad2);

        telemetry.addData("Extendo Pos:", transport.extendoPos);
        telemetry.addData("Extendo Target:", transport.extendoTarget);
        telemetry.addData("Out Pos:", transport.outPos);
        telemetry.addData("Out Target:", transport.outTarget);
        telemetry.addData("Rotation Pos:", transport.rotPos);
        telemetry.addData("Intake Power:", transport.leftRotPower);
        telemetry.addData("Intake Toggle:", transport.rotToggle.value());
        telemetry.addData("Bucket Pos:", transport.outArmPos);
        telemetry.addData("Heading", drive.botHeading);
        telemetry.addData("Slowmode:", drive.slowmode.value());
    }
}
