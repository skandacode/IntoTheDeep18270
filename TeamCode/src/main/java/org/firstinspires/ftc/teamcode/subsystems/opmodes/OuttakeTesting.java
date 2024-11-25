package org.firstinspires.ftc.teamcode.subsystems.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Outtake;

@Config
@TeleOp
public class OuttakeTesting extends LinearOpMode {
    Outtake outtake;
    public static int targetPos = 0;
    public static double wristPos=0.5;
    public static double armPos=0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        outtake = new Outtake(hardwareMap, telemetry);
        outtake.resetEncoder();
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.a){
                outtake.openClaw();
            }
            if (gamepad1.b){
                outtake.closeClaw();
            }
            outtake.setFlipPos(armPos);
            outtake.setWristPos(wristPos);
            outtake.setTargetPos(targetPos);
            telemetry.addData("outtake pos", outtake.getLiftPos());
            telemetry.update();
            outtake.update();
        }
    }
}
