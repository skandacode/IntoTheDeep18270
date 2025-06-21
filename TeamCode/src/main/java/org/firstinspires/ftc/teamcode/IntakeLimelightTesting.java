package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Config
public class IntakeLimelightTesting extends LinearOpMode {
    IntakeCopy intake;
    public static double turretPos=0;
    public static double flipPos=0;
    public static double wristPos=0;
    public static boolean clawClosed=false;
    public static int slidesTarget=0;


    @Override
    public void runOpMode() throws InterruptedException {
        intake = new IntakeCopy(hardwareMap, telemetry);
        waitForStart();
        while (opModeIsActive()){
            intake.setTargetPos(slidesTarget);
            intake.setTurretPos(turretPos);
            intake.setArmPos(flipPos);
            intake.setWristPos(wristPos);
            if (clawClosed){
                intake.closeClaw();
            }else{
                intake.openClaw();
            }
            intake.update();
            telemetry.addData("Intake pos", intake.getLiftPos());
            telemetry.update();
        }
    }
}
