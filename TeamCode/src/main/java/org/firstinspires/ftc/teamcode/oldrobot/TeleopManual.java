/*package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;


@TeleOp
@Config
public class TeleopManual extends LinearOpMode {
    Intake intake;
    Outtake outtake;
    MecanumDrivetrain drive;
    public static double liftFF=0.08;
    @Override
    public void runOpMode() throws InterruptedException {
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap, telemetry);
        outtake.resetEncoder();
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new MecanumDrivetrain(hardwareMap, telemetry, FtcDashboard.getInstance());

        waitForStart();
        intake.retract();
        outtake.transferPos();
        outtake.resetEncoder();


        while (opModeIsActive()){
            if (gamepad1.right_bumper){
                outtake.openClaw();
            }
            if (gamepad1.left_bumper){
                outtake.closeClaw();
            }
            if (gamepad1.y){
                intake.intakePosition();
            }
            if (gamepad1.x){
                intake.retract();
                outtake.transferPos();
            }
            if (gamepad1.b){
                outtake.transferPos();
            }
            if (gamepad1.a){
                outtake.scorePos();
            }
            if (gamepad1.dpad_down){
                outtake.specimenScorePos();
            }
            outtake.setPower(gamepad1.right_trigger-gamepad1.left_trigger+liftFF);
            if (!gamepad1.left_bumper){
                drive.setWeightedPowers(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            }else{
                drive.setWeightedPowers(-gamepad1.left_stick_y/3, -gamepad1.left_stick_x/3, -gamepad1.right_stick_x/3);
            }
            intake.update();
            telemetry.addData("Intake extend Pos", intake.getExtendoMotorPos());
            telemetry.addData("Outtake extend Pos", outtake.getLiftPos());
            telemetry.addData("intake color", intake.getRawSensorValues());

            telemetry.update();
        }
    }

}*/
