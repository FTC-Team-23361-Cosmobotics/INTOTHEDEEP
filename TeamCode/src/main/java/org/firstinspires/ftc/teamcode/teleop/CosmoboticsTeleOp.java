package org.firstinspires.ftc.teamcode.teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.teleop.drive.Drive;
import org.firstinspires.ftc.teamcode.teleop.misc.Misc;
import org.firstinspires.ftc.teamcode.teleop.transport.TransportFSM;

import java.util.List;

@Config
@TeleOp
public class CosmoboticsTeleOp extends OpMode {
    Drive drive;
    TransportFSM transportFSM;
    public List<LynxModule> allHubs;
    public LynxModule CtrlHub;

    public LynxModule ExpHub;


    @Override
    public void init() {
        drive = new Drive(hardwareMap);
        transportFSM = new TransportFSM(hardwareMap);

        allHubs = hardwareMap.getAll(LynxModule.class);
        CtrlHub = allHubs.get(0);
        ExpHub = allHubs.get(1);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("test", transportFSM.test);
    }

    @Override
    public void loop() {
        drive.update(gamepad1);
        transportFSM.update(gamepad1, gamepad2);

        telemetry.addData("Extendo Pos:", transportFSM.extendoPos);
        telemetry.addData("Extendo Target:", transportFSM.extendoTarget);
        telemetry.addData("Out Pos:", transportFSM.outPos);
        telemetry.addData("Out Target:", transportFSM.outTarget);
        telemetry.addData("Rotation Pos:", transportFSM.rotPos);
        telemetry.addData("Intake Toggle:", transportFSM.intakeToggle.value());
        telemetry.addData("Bucket Pos:", transportFSM.bucketPitchPos);
        telemetry.addData("Flp Pos:", transportFSM.flapPos);
        telemetry.addData("Gmepdnot2 true", gamepad1.left_trigger > 0);
        telemetry.addData("gmpdnot2 rkght", gamepad1.right_trigger > 0);
        telemetry.addData("extendolesthnuper ", transportFSM.extendoTarget <= transportFSM.extendoUpper);
        telemetry.addData("extendo voltge", transportFSM.extendo.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("out voltge", transportFSM.out.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("ero limit", transportFSM.zeroLimit.isPressed());
        telemetry.addData("test", transportFSM.test);
        telemetry.addData("Heading", drive.botHeading);
        telemetry.addData("Slowmode:", drive.slowmode.value());
    }
}
