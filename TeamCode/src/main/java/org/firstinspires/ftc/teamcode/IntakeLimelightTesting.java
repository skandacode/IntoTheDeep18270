package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

@TeleOp
@Config
public class IntakeLimelightTesting extends LinearOpMode {
    IntakeCopyCopy intake;
    LimelightIVK limelightIVK;
    public static double flipPos=0.2;
    public static double wristPos=0;
    public static boolean clawClosed=false;
    public static int slidesTarget=0;
    StateMachine IntakeMachine;
    public enum IntakeStates{
        IDLE, ARMDROP, CLOSECLAW, ARMUP, RETRACT, OPENCLAW
    }
    Position sampPos;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        intake = new IntakeCopyCopy(hardwareMap, telemetry);
        limelightIVK=new LimelightIVK(hardwareMap);
        StateMachine IntakeMachine = new StateMachineBuilder()
                .state(IntakeStates.IDLE)
                .onEnter(() -> {
                    sampPos = limelightIVK.getPosition();
                    telemetry.addData("Intake pos", intake.getIntakePos());
                    if (sampPos != null) {
                        sampPos.y -= 5.5;
                        intake.goToPosition(sampPos);
                        telemetry.addData("Samp Pos x", sampPos.x);
                        telemetry.addData("Samp Pos y", sampPos.y);
                        System.out.println(sampPos.x+" "+ sampPos.y);
                    }
                    telemetry.addData("Intake Target", intake.getTargetPos());
                })
                .transitionTimed(1)
                .transition(()->sampPos==null, IntakeStates.IDLE)
                .state(IntakeStates.ARMDROP)
                .onEnter(() -> {
                    intake.setArmPos(0.06);
                })
                .transitionTimed(1)
                .state(IntakeStates.CLOSECLAW)
                .onEnter(() -> {
                    intake.closeClaw();
                })
                .transitionTimed(0.6)
                .state(IntakeStates.ARMUP)
                .onEnter(() -> {
                    intake.arm.setTargetPosition(0.5);
                })
                .transitionTimed(0.6)
                .state(IntakeStates.RETRACT)
                .onEnter(() -> {
                    intake.depositPos();
                })
                .transitionTimed(0.6)
                .state(IntakeStates.OPENCLAW)
                .onEnter(() -> {
                    intake.setTargetPos(0);
                    intake.openClaw();
                })
                .transitionTimed(0.6, IntakeStates.IDLE)
                .build();

        intake.setTargetPos(0);
        intake.setArmPos(0.5);
        intake.setTurretPos(0.5);
        while (opModeInInit()){
            intake.update();
            telemetry.update();
        }
        waitForStart();
        IntakeMachine.start();
        while (opModeIsActive()) {
            IntakeMachine.update();
            telemetry.addData("State", IntakeMachine.getState());
            telemetry.addData("Flip target", intake.arm.getTargetPosition());
            telemetry.addData("Flip actual", intake.arm.getCurrentPosition());
            telemetry.addData("Flip error", intake.arm.getError());

            intake.update();
            telemetry.update();
        }
    }
}
