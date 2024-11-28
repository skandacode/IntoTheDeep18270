package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.internal.opengl.models.SavedMeshObject;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

import java.util.List;

@Config
@TeleOp
public class FasterTeleop extends LinearOpMode {
    Outtake outtake;
    Intake intake;
    MecanumDrivetrain drive;

    public enum SampleStates {
        IDLE, EXTEND, DROP, SENSORWAIT, SENSE, RETRACT, OPENCOVER, WAIT, CLOSE, LIFT, WRIST, OPEN, LOWERLIFT, EJECT, LIDOPENEJECT
    }

    public enum SpecimenScoreStates {IDLE, INTAKEPOS, INTAKE, CLOSE_CLAW, HOLD, SCORE, OPENCLAW, RETRACT}

    Intake.SampleColor targetColor = Intake.SampleColor.YELLOW;

    public static int targetLiftPosSample =2900;

    public static Intake.SampleColor allianceColor= Intake.SampleColor.BLUE;
    Intake.SampleColor currentSense= Intake.SampleColor.NONE;

    boolean hanging=false;
    boolean pullingdown=false;



    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        outtake = new Outtake(hardwareMap, telemetry);
        intake = new Intake(hardwareMap);

        drive = new MecanumDrivetrain(hardwareMap, telemetry, FtcDashboard.getInstance());
        StateMachine sampleMachine = new StateMachineBuilder()
                .state(SampleStates.IDLE)
                .onEnter(() -> {
                    intake.retract();
                    intake.setPower(0);
                })
                .transition(() -> gamepad1.right_trigger>0.3, SampleStates.EXTEND)


                .state(SampleStates.EXTEND)
                .onEnter(()->intake.setExtended(true))
                .transition(()->gamepad1.right_bumper, SampleStates.DROP)
                .transition(()->gamepad1.right_trigger<0.3, SampleStates.IDLE)


                .state(SampleStates.DROP)
                .onEnter(() -> intake.intakePosition())
                .loop(()->{
                    if (gamepad1.options){
                        if (intake.getIntakeSpeed() != -1){
                            intake.setPower(-1);
                        }
                    }else{
                        if (intake.getIntakeSpeed()==-1){
                            intake.intakePosition();
                        }
                    }
                })
                .transition(()->intake.getDistance()<4.5, SampleStates.SENSORWAIT)
                .transition(()->gamepad1.right_trigger<0.3, SampleStates.IDLE)


                .state(SampleStates.SENSORWAIT)
                .onEnter(()->intake.intakePosition())
                .transitionTimed(0.1, SampleStates.SENSE)


                .state(SampleStates.SENSE)
                .transition(() -> {
                    currentSense=intake.getColor();
                    return currentSense == targetColor || currentSense== allianceColor;
                }, SampleStates.RETRACT)
                .transition(()->currentSense == Intake.SampleColor.NONE, SampleStates.DROP)
                .transition(() -> currentSense != targetColor && currentSense != allianceColor, SampleStates.EJECT)

                .state(SampleStates.EJECT, true)
                .onEnter(() -> intake.eject())
                .transitionTimed(0.2, SampleStates.LIDOPENEJECT)

                .state(SampleStates.LIDOPENEJECT, true)
                .onEnter(()->intake.setCover(false))
                .transitionTimed(0.7, SampleStates.DROP)

                .state(SampleStates.RETRACT)
                .onEnter(() -> {
                    intake.retract();
                    intake.setCover(true);
                    intake.setPower(0.3);
                    outtake.transferPos();
                })
                .transitionTimed(0.1, SampleStates.OPENCOVER)
                .state(SampleStates.OPENCOVER)
                .onEnter(() -> {
                    intake.setCover(false);
                    intake.setPower(0);
                })
                .transition(() -> intake.isDone() && outtake.getTargetPos()<10, SampleStates.WAIT)

                .state(SampleStates.WAIT)
                .onEnter(() -> intake.setPower(0.2))
                .transitionTimed(0.7, SampleStates.CLOSE)

                .state(SampleStates.CLOSE)
                .onEnter(() -> outtake.closeClaw())
                .transitionTimed(0.2, SampleStates.LIFT)

                .state(SampleStates.LIFT)
                .onEnter(() -> {
                    outtake.setTargetPos(targetLiftPosSample);
                })
                .loop(() -> {
                    if (gamepad1.left_trigger > 0.3) {
                        outtake.setTargetPos(500);
                    }
                })
                .transitionTimed(0.2, SampleStates.WRIST)

                .state(SampleStates.WRIST).onEnter(() -> {
                    outtake.scorePos();
                    intake.setPower(0);
                    intake.setPower(-0.5);
                }).loop(() -> {
                    if (gamepad1.left_trigger > 0.3) {
                        outtake.setTargetPos(500);
                    }
                }).transition(() -> gamepad1.right_bumper)

                .state(SampleStates.OPEN)
                .onEnter(() -> outtake.openClaw())
                .transition(()->gamepad1.right_trigger>0.3 || gamepad1.left_stick_y<-0.1)
                .onExit(() -> {
                    outtake.setTargetPos(0);
                    outtake.transferPos();
                })
                .state(SampleStates.LOWERLIFT)
                .onEnter(() -> {
                    outtake.setTargetPos(0);
                    outtake.transferPos();
                })
                .transition(() -> outtake.getTargetPos()<10 || gamepad1.right_trigger>0.3 || gamepad1.left_stick_y<-0.1, SampleStates.IDLE)
                .build();

        StateMachine specimenScorer = new StateMachineBuilder()
                .state(SpecimenScoreStates.IDLE)
                .transition(() -> gamepad1.left_trigger>0.5 && sampleMachine.getState()==SampleStates.IDLE)
                .state(SpecimenScoreStates.INTAKEPOS)
                .onEnter(() -> {
                    outtake.scorePos();
                    outtake.openClawWide();
                    outtake.setTargetPos(200);
                })
                .transitionTimed(0.7)
                .state(SpecimenScoreStates.INTAKE)
                .onEnter(() -> outtake.setTargetPos(0))
                .transition(() -> gamepad1.right_bumper)
                .state(SpecimenScoreStates.CLOSE_CLAW)
                .onEnter(() -> outtake.closeClaw())
                .transitionTimed(0.3)
                .state(SpecimenScoreStates.HOLD)
                .onEnter(() -> outtake.specimenHoldPos())
                .transition(() -> (outtake.atTarget() && gamepad1.right_bumper))
                .state(SpecimenScoreStates.SCORE)
                .onEnter(() -> outtake.specimenScorePos())
                .transitionTimed(0.3)
                .state(SpecimenScoreStates.OPENCLAW)
                .onEnter(() -> outtake.openClaw())
                .transitionTimed(0.2)
                .state(SpecimenScoreStates.RETRACT)
                .onEnter(() -> outtake.transferPos())
                .transition(() -> (outtake.atTarget() || gamepad1.dpad_up), SpecimenScoreStates.IDLE)
                .build();

        while (opModeInInit()){
            if (gamepad1.a){
                allianceColor = Intake.SampleColor.BLUE;
                gamepad1.setLedColor(0, 0, 1, 1000);
            }
            if (gamepad1.b){
                allianceColor = Intake.SampleColor.RED;
                gamepad1.setLedColor(1, 0, 0, 1000);
            }
            telemetry.addData("Alliance Color", allianceColor.toString());
            telemetry.update();
        }

        waitForStart();
        do{
            outtake.setPower(-1);
            sleep(100);
        }while(opModeIsActive() && Math.abs(outtake.getCurrent())<5);
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
        outtake.resetEncoder();
        outtake.transferPos();
        sampleMachine.start();
        specimenScorer.start();
        long prevLoop = System.nanoTime();
        while (opModeIsActive()) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
            if (gamepad1.b){
                targetColor=allianceColor;
            }
            if (gamepad1.a){
                targetColor= Intake.SampleColor.YELLOW;
            }
            if (!gamepad1.left_bumper) {
                drive.setWeightedPowers(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            } else {
                drive.setWeightedPowers(-gamepad1.left_stick_y / 5, -gamepad1.left_stick_x / 5, -gamepad1.right_stick_x / 8);
            }
            if (gamepad1.touchpad && sampleMachine.getState()==SampleStates.IDLE && specimenScorer.getState()==SpecimenScoreStates.IDLE){
                intake.setPower(0.5);
                intake.setExtendo(0.3);
                sleep(200);
                outtake.closeClaw();
                sampleMachine.setState(SampleStates.CLOSE);
            }
            if (gamepad1.dpad_left){
                hanging=true;
                intake.retract();
                intake.setPower(0);
                outtake.transferPos();
                outtake.setTargetPos(2900);
            }
            if (gamepad1.dpad_right && hanging){
                pullingdown=true;
            }
            if (pullingdown){
                if (outtake.getLiftPos()>1900){
                    outtake.setPower(-1);
                }else{
                    outtake.setPower(-0.5);
                }
            }
            if (!hanging) {
                sampleMachine.update();
                specimenScorer.update();
            }
            intake.update();
            if (!pullingdown){
                outtake.update();
            }
            telemetry.addData("target color", targetColor.toString());
            telemetry.addData("alliance color", allianceColor.toString());

            telemetry.addData("State sample", sampleMachine.getState());
            telemetry.addData("Specimen sample", specimenScorer.getState());

            //telemetry.addData("Intake color", Arrays.toString(intake.getRawSensorValues()));
            //telemetry.addData("Intake distance", intake.getDistance());
            telemetry.addData("hanging?", hanging);
            telemetry.addData("pulling down?", pullingdown);

            telemetry.addData("Outtake Pos", outtake.getLiftPos());
            telemetry.addData("Extendo Pos", intake.getExtendoMotorPos());
            long currLoop = System.nanoTime();
            telemetry.addData("Ms per loop", (currLoop - prevLoop) / 1000000);
            prevLoop = currLoop;

            telemetry.update();
        }
    }
}
