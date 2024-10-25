package org.firstinspires.ftc.teamcode.teleop;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.teleop.drive.Drive;
import org.firstinspires.ftc.teamcode.teleop.misc.Misc;
import org.firstinspires.ftc.teamcode.teleop.transport.Transport;

@Config
@TeleOp
public class CosmoboticsTeleOp extends OpMode {
    Drive drive;
    Transport transport;
//    Misc misc;
    @Override
    public void init() {
        drive = new Drive(hardwareMap);
        transport = new Transport(hardwareMap);
//        misc = new Misc(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        drive.update(gamepad1);
        transport.update(gamepad1, gamepad2);
//        misc.update(gamepad2);
        telemetry.addData("Extendo Pos:", transport.extendoPos);
        telemetry.addData("Extendo Target:", Transport.extendoTarget);
        telemetry.addData("Out Pos:", transport.outPos);
        telemetry.addData("Out Target:", Transport.outTarget);
        telemetry.addData("Rotation Pos:", Transport.rotPos);
        telemetry.update();
    }
}
