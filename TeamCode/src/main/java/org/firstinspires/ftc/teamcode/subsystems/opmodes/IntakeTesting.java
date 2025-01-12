package org.firstinspires.ftc.teamcode.subsystems.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp
@Config
public class IntakeTesting extends LinearOpMode {
    Intake intake;
    public static double intakePower=0;
    public static double extendoPower=0;

    public static double flip=0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        intake = new Intake(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            intake.setExtendo(extendoPower);
            intake.setPower(intakePower);
            intake.setFlip(flip);
            telemetry.addData("Extendo position", intake.getExtendoMotorPos());
            telemetry.addData("intake distance", intake.getDistance());
            telemetry.addData("intake color", intake.getColor());

            telemetry.update();
        }
    }
}
