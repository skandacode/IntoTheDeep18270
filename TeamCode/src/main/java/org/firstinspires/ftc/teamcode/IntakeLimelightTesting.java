package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Config
public class IntakeLimelightTesting extends LinearOpMode {
    IntakeCopy intake;
    LimelightIVK limelightIVK;
    public static double flipPos=0.1;
    public static double wristPos=0;
    public static boolean clawClosed=false;
    public static int slidesTarget=0;


    @Override
    public void runOpMode() throws InterruptedException {
        intake = new IntakeCopy(hardwareMap, telemetry);
        limelightIVK= new LimelightIVK(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            Position sampPos = limelightIVK.getPosition();
            intake.setArmPos(flipPos);
            //intake.setDistance(slidesTarget);
            intake.setWristPos(wristPos);
            if (clawClosed){
                intake.closeClaw();
            }else{
                intake.openClaw();
            }
            intake.update();
            telemetry.addData("Intake pos", intake.getIntakePos());
            if (sampPos != null) {
                sampPos.y-=4;
                intake.goToPosition(sampPos);
                telemetry.addData("Samp Pos x", sampPos.x);
                telemetry.addData("Samp Pos y", sampPos.y);
            }
            telemetry.addData("Intake Target", intake.getTargetPos());
            telemetry.update();
        }
    }
}
