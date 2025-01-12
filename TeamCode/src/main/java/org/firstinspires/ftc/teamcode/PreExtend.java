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
public class PreExtend extends LinearOpMode {
    Outtake outtake;
    Intake intake;
    MecanumDrivetrain drive;

    public enum SampleStates {
        IDLE, EXTEND, SENSORWAIT, SENSE, RETRACT, OPENCOVER, WAIT, CLOSE, LIFT, PARTIALFLIP, WRIST, OPEN, LOWERLIFT, EJECTFLIP, EJECTLIDOPEN
    }


    public enum SpecimenScoreStates {IDLE, INTAKEPOS, INTAKE, CLOSE_CLAW, HOLD, SCORE, OPENCLAW, RETRACT}

    Intake.SampleColor targetColor = Intake.SampleColor.YELLOW;

    public static int targetLiftPosSample =2975;

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
                .loop(()->{
                    if (gamepad1.left_trigger>0.3){
                        intake.setExtended(true);
                    }else{
                        intake.setExtended(false);
                    }
                })
                .transition(() -> gamepad1.right_bumper && gamepad1.left_trigger>0.1)

                .state(SampleStates.EXTEND)
                .onEnter(()->intake.intakePosition())
                .loop(()->{
                    if (gamepad1.options){
                        if (intake.getIntakeSpeed() != -1){
                            intake.setPower(-1);
                        }
                        intake.setPower(-1);
                    }else{
                        if (intake.getIntakeSpeed()==-1){
                            intake.intakePosition();
                        }
                        intake.setPower(1);
                    }
                })
                .transition(()->intake.isSampleIntaked())
                .transition(()->!gamepad1.right_bumper, SampleStates.IDLE)
                .transition(()->gamepad1.left_trigger<0.1, SampleStates.IDLE)


                .state(SampleStates.SENSORWAIT)
                .onEnter(()->intake.intakePosition())
                .transitionTimed(0.05)
                .state(SampleStates.SENSE)
                .transition(() -> {
                    currentSense=intake.getColor();
                    return currentSense == targetColor || currentSense== allianceColor;
                }, SampleStates.RETRACT)
                .transition(()->currentSense == Intake.SampleColor.NONE, SampleStates.EXTEND)
                .transition(() -> currentSense != targetColor && currentSense != allianceColor, SampleStates.EJECTFLIP)

                .state(SampleStates.EJECTFLIP, true)
                .onEnter(() -> {
                    intake.eject();
                })
                .transitionTimed(0.2, SampleStates.EJECTLIDOPEN)

                .state(SampleStates.EJECTLIDOPEN, true)
                .onEnter(() -> {
                    intake.setCover(false);
                })
                .transitionTimed(0.4, SampleStates.EXTEND)

                .state(SampleStates.RETRACT)
                .onEnter(() -> {
                    intake.retract();
                    intake.setCover(true);
                    intake.setPower(0.4);
                    outtake.transferPos();
                })
                .transitionTimed(0.25)
                .state(SampleStates.OPENCOVER)
                .onEnter(() -> {
                    intake.setCover(false);
                    intake.setPower(0.1);
                })
                .transition(() -> intake.isDone())

                .state(SampleStates.WAIT)
                .onEnter(() -> intake.setPower(0.4))
                .transitionTimed(0.4)

                .state(SampleStates.CLOSE)
                .onEnter(() -> {
                    outtake.closeClaw();
                    intake.setPower(0.4);
                })
                .transitionTimed(0.15)

                .state(SampleStates.LIFT)
                .onEnter(() -> {
                    outtake.setTargetPos(targetLiftPosSample);
                    intake.setPower(0);
                })
                .loop(() -> {
                    if (gamepad1.left_trigger > 0.3) {
                        outtake.setTargetPos(500);
                        outtake.scorePos();
                    }
                })
                .transitionTimed(0.3)

                .state(SampleStates.PARTIALFLIP)
                .onEnter(()->{
                    outtake.setFlipPos(0.3);
                    outtake.setWristPos(0.5);
                    intake.setPower(-1);
                })
                .loop(() -> {
                    if (gamepad1.left_trigger > 0.3) {
                        outtake.setTargetPos(500);
                        outtake.scorePos();
                    }
                })
                .transition(()->gamepad1.left_bumper && outtake.getTargetPos()==500, SampleStates.OPEN)
                .transition(()->outtake.getLiftPos()>2400)

                .state(SampleStates.WRIST).onEnter(() -> {
                    outtake.scorePosTeleop();
                })
                .loop(() -> {
                    if (gamepad1.left_trigger > 0.3) {
                        outtake.setTargetPos(500);
                        outtake.scorePos();
                    }
                })
                .transition(() -> gamepad1.left_bumper)

                .state(SampleStates.OPEN)
                .onEnter(() -> outtake.openClawWide())
                .transitionTimed(0.5)
                .transition(()->gamepad1.y || (gamepad1.left_stick_y<-0.8 && !gamepad1.right_bumper))
                .onExit(() -> {
                    outtake.setTargetPos(0);
                    outtake.transferPos();
                })
                .state(SampleStates.LOWERLIFT)
                .onEnter(() -> {
                    outtake.setTargetPos(0);
                    outtake.transferPos();
                })
                .transition(()->gamepad1.left_trigger>0.3, SampleStates.IDLE)
                .transition(() -> (outtake.atTarget() || gamepad1.y), SampleStates.IDLE)
                .build();
        StateMachine specimenScorer = new StateMachineBuilder()
                .state(SpecimenScoreStates.IDLE)
                .transition(() -> gamepad1.dpad_up)
                .state(SpecimenScoreStates.INTAKEPOS)
                .onEnter(() -> {
                    outtake.scorePos();
                    outtake.openClawWide();
                    outtake.setTargetPos(300);
                })
                .transitionTimed(0.7)
                .state(SpecimenScoreStates.INTAKE)
                .onEnter(() -> outtake.setTargetPos(100))
                .transition(() -> gamepad1.dpad_down)
                .state(SpecimenScoreStates.CLOSE_CLAW)
                .onEnter(() -> outtake.closeClaw())
                .transitionTimed(0.3)
                .state(SpecimenScoreStates.HOLD)
                .onEnter(() -> outtake.specimenHoldPos())
                .transition(() -> (outtake.atTarget() && gamepad1.left_bumper))
                .state(SpecimenScoreStates.SCORE)
                .onEnter(() -> outtake.specimenScorePos())
                .transitionTimed(0.5)
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
        outtake.transferPos();
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
            if (gamepad1.share && outtake.getTargetPos()==0){
                outtake.resetEncoder();
            }
            if (gamepad1.x) {
                if (sampleMachine.getState() == SampleStates.EXTEND) {
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
            if (gamepad1.touchpad && sampleMachine.getState()== SampleStates.IDLE && specimenScorer.getState()== SpecimenScoreStates.IDLE){
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
                if (outtake.getLiftPos()>1700){
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

            long currLoop = System.nanoTime();
            telemetry.addData("Ms per loop", (currLoop - prevLoop) / 1000000);
            prevLoop = currLoop;

            telemetry.update();
        }
    }
}
