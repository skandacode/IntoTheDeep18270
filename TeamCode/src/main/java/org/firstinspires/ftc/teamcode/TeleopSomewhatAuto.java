package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

import java.util.List;

@Config
@TeleOp
public class TeleopSomewhatAuto extends LinearOpMode {
    Outtake outtake;
    Intake intake;
    MecanumDrivetrain drive;

    public enum SampleStates {
        IDLE, EXTEND, DROP, SENSORWAIT, SENSE, RETRACT, OPENCOVER, WAIT, CLOSE, LIFT, WRIST, OPEN, LOWERLIFT, EJECT
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
                .transition(() -> gamepad1.y)
                .state(SampleStates.EXTEND)
                .onEnter(()->intake.setExtended(true))
                .transitionTimed(0.025)
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
                .transition(()->intake.getDistance()<5)
                .state(SampleStates.SENSORWAIT)
                .onEnter(()->intake.intakePosition())
                .transitionTimed(0.1)
                .state(SampleStates.SENSE)
                .transition(() -> {
                    currentSense=intake.getColor();
                    return currentSense == targetColor || currentSense== allianceColor;
                }, SampleStates.RETRACT)
                .transition(()->currentSense == Intake.SampleColor.NONE, SampleStates.DROP)
                .transition(() -> currentSense != targetColor && currentSense != allianceColor, SampleStates.EJECT)

                .state(SampleStates.EJECT, true)
                .onEnter(() -> {
                    intake.eject();
                    intake.setCover(false);
                })
                .transitionTimed(0.7, SampleStates.DROP)

                .state(SampleStates.RETRACT)
                .onEnter(() -> {
                    intake.retract();
                    intake.setCover(true);
                    intake.setPower(0.4);
                    outtake.transferPos();
                })
                .transitionTimed(0.1)
                .state(SampleStates.OPENCOVER)
                .onEnter(() -> {
                    intake.setCover(false);
                    intake.setPower(0.1);
                })
                .transition(() -> intake.isDone())

                .state(SampleStates.WAIT)
                .onEnter(() -> intake.setPower(0.4))
                .transitionTimed(0.7)

                .state(SampleStates.CLOSE)
                .onEnter(() -> {
                    outtake.closeClaw();
                    intake.setPower(0.4);
                })
                .transitionTimed(0.2)

                .state(SampleStates.LIFT)
                .onEnter(() -> {
                    outtake.setTargetPos(targetLiftPosSample);
                    intake.setPower(0);
                })
                .loop(() -> {
                    if (gamepad1.left_trigger > 0.3) {
                        outtake.setTargetPos(500);
                    }
                })
                .transitionTimed(0.2)

                .state(SampleStates.WRIST).onEnter(() -> {
                    outtake.scorePos();
                    intake.setPower(-0.5);
                }).loop(() -> {
                    if (gamepad1.left_trigger > 0.3) {
                        outtake.setTargetPos(500);
                    }
                }).transition(() -> gamepad1.left_bumper)

                .state(SampleStates.OPEN)
                .onEnter(() -> outtake.openClaw())
                .transitionTimed(0.5)
                .transition(()->gamepad1.y || gamepad1.left_stick_y<-0.1)
                .onExit(() -> {
                    outtake.setTargetPos(0);
                    outtake.transferPos();
                })
                .state(SampleStates.LOWERLIFT)
                .onEnter(() -> {
                    outtake.setTargetPos(0);
                    outtake.transferPos();
                })
                .transition(()->gamepad1.left_trigger>0.3)
                .transition(() -> (outtake.atTarget() || gamepad1.y), SampleStates.IDLE)
                .build();
        StateMachine specimenScorer = new StateMachineBuilder()
                .state(SpecimenScoreStates.IDLE)
                .transition(() -> gamepad1.dpad_up)
                .state(SpecimenScoreStates.INTAKEPOS)
                .onEnter(() -> {
                    outtake.scorePos();
                    outtake.openClawWide();
                    outtake.setTargetPos(200);
                })
                .transitionTimed(0.7)
                .state(SpecimenScoreStates.INTAKE)
                .onEnter(() -> outtake.setTargetPos(0))
                .transition(() -> gamepad1.dpad_down)
                .state(SpecimenScoreStates.CLOSE_CLAW)
                .onEnter(() -> outtake.closeClaw())
                .transitionTimed(0.3)
                .state(SpecimenScoreStates.HOLD)
                .onEnter(() -> outtake.specimenHoldPos())
                .transition(() -> (outtake.atTarget() && gamepad1.left_bumper))
                .state(SpecimenScoreStates.SCORE)
                .onEnter(() -> outtake.specimenScorePos())
                .transitionTimed(0.3)
                .state(SpecimenScoreStates.OPENCLAW)
                .onEnter(() -> outtake.openClaw())
                .transitionTimed(1)
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
        boolean prevSampleColorToggle = false;
        long prevLoop = System.nanoTime();
        while (opModeIsActive()) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
            boolean currSampleColorToggle = gamepad1.right_trigger>0.5;
            if (currSampleColorToggle && !prevSampleColorToggle) {
                if (targetColor == Intake.SampleColor.YELLOW) {
                    targetColor = allianceColor;
                } else if (targetColor == allianceColor) {
                    targetColor = Intake.SampleColor.YELLOW;
                }
            }
            if (gamepad1.share && outtake.getTargetPos()==0){
                outtake.resetEncoder();
            }
            if (gamepad1.x) {
                if (sampleMachine.getState() == SampleStates.EXTEND || sampleMachine.getState() == SampleStates.DROP) {
                    sampleMachine.setState(SampleStates.IDLE);
                    intake.retract();
                    intake.setPower(0);
                }
            }

            if (!gamepad1.right_bumper) {
                drive.setWeightedPowers(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            } else {
                drive.setWeightedPowers(-gamepad1.left_stick_y / 5, -gamepad1.left_stick_x / 5, -gamepad1.right_stick_x / 8);
            }
            if (gamepad1.touchpad && sampleMachine.getState()==SampleStates.IDLE && specimenScorer.getState()==SpecimenScoreStates.IDLE){
                intake.setPower(0.5);
                intake.setExtendo(0.3);
                sampleMachine.setState(SampleStates.WAIT);
            }
            if (gamepad1.left_stick_button){
                hanging=true;
                intake.retract();
                intake.setPower(0);
                outtake.transferPos();
                outtake.setTargetPos(2900);
            }
            if (gamepad1.right_stick_button && hanging){
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
            if (gamepad1.share){
                telemetry.addData("intake distance", intake.getDistance());
                telemetry.addData("intake color", intake.getColor());
            }
            telemetry.addData("gamepad strafe", gamepad1.left_stick_x);
            telemetry.addData("target color", targetColor.toString());
            telemetry.addData("alliance color", allianceColor.toString());
            telemetry.addData("Intake speed", intake.getIntakeSpeed());

            telemetry.addData("State sample", sampleMachine.getState());
            telemetry.addData("Specimen sample", specimenScorer.getState());

            //telemetry.addData("Intake color", Arrays.toString(intake.getRawSensorValues()));
            //telemetry.addData("Intake distance", intake.getDistance());
            telemetry.addData("hanging?", hanging);
            telemetry.addData("pulling down?", pullingdown);

            telemetry.addData("Outtake Pos", outtake.getLiftPos());
            telemetry.addData("Extendo Pos", intake.getExtendoMotorPos());
            drive.telemetryPower();
            long currLoop = System.nanoTime();
            telemetry.addData("Ms per loop", (currLoop - prevLoop) / 1000000);
            prevLoop = currLoop;

            telemetry.update();
            prevSampleColorToggle = currSampleColorToggle;
        }
    }
}
