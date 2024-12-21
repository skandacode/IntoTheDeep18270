package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pathing.WayPoint;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

import java.util.List;

@Config
@Autonomous
public class Dumbauto extends LinearOpMode {
    MecanumDrivetrain drive;
    Intake intake;
    Outtake outtake;
    public static boolean closedPressed=false;
    public static boolean scoredPressed=false;
    public static boolean yPressed=false;
    public static boolean lbPressed=false;
    enum autoStates {
        clawclose, depositPosPreload, pauseToDepo, scorePreload,
        intakePreExtend1, intakeExtend1, intakeExtendStart, intakeExtend1Pos, intakeReversePos1, intakeReverse1,
        intakePreExtend2, intakeExtend2, intakeExtend2Pos, intakeReversePos2, intakeReverse2,
        intakePreExtend3, intakeExtend3, intakeExtend3Pos, intakeReversePos3, intakeReverse3,
        intakeRetract1, intakePos1,intakePos1f2, closeClaw1, depositPos1, depositPos1f2, score1,
        preintake2, intakePos2, intakePos2f2, closeClaw2, depositPos2, depositPos2f2, score2,
        preintake3,intakePos3, intakePos3f2, closeClaw3, depositPos3, depositPos3f2, score3,
        preintake4,intakePos4, intakePos4f2, closeClaw4, depositPos4, depositPos4f2, score4,
        backintakePosHuman, intakePosHuman, intakeExtend4, depositPosbucket, drop
    }

    @Override

    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry= new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        drive=new MecanumDrivetrain(hardwareMap, telemetry, dashboard);
        intake= new Intake(hardwareMap);
        outtake= new Outtake(hardwareMap, telemetry);
        WayPoint preintake=new WayPoint(new Pose2D(DistanceUnit.INCH, -5, -38, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH,  3, 3, AngleUnit.DEGREES, 5));
        WayPoint depositPosPreload=new WayPoint(new Pose2D(DistanceUnit.INCH, -5, -34, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH,  1, 2, AngleUnit.DEGREES, 2));
        WayPoint depositPosPreload2=new WayPoint(new Pose2D(DistanceUnit.INCH, -1, -27, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH,  1, 1, AngleUnit.DEGREES, 2));
        WayPoint depositPos1=new WayPoint(new Pose2D(DistanceUnit.INCH, -6, -50, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH,  4, 4, AngleUnit.DEGREES, 3));
        WayPoint depositPos2=new WayPoint(new Pose2D(DistanceUnit.INCH, -6, -24.5, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH,  1, 1, AngleUnit.DEGREES, 2));
        WayPoint depositPos12=new WayPoint(new Pose2D(DistanceUnit.INCH, -8, -50, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH,  4, 4, AngleUnit.DEGREES, 3));
        WayPoint depositPos22=new WayPoint(new Pose2D(DistanceUnit.INCH, -8, -24.5, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH,  1, 1, AngleUnit.DEGREES, 2));
        WayPoint depositPos22Strafe=new WayPoint(new Pose2D(DistanceUnit.INCH, -5, -24.5, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH,  1, 1, AngleUnit.DEGREES, 2));
        WayPoint depositPos13=new WayPoint(new Pose2D(DistanceUnit.INCH, -10, -50, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH,  4, 4, AngleUnit.DEGREES, 3));
        WayPoint depositPos23=new WayPoint(new Pose2D(DistanceUnit.INCH, -10, -24.5, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH,  1, 1, AngleUnit.DEGREES, 2));
        WayPoint depositPos14=new WayPoint(new Pose2D(DistanceUnit.INCH, -13, -50, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH,  4, 4, AngleUnit.DEGREES, 3));
        WayPoint depositPos24=new WayPoint(new Pose2D(DistanceUnit.INCH, -13, -24.5, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH,  1, 1, AngleUnit.DEGREES, 2));
        WayPoint backintakePosHuman=new WayPoint(new Pose2D(DistanceUnit.INCH, -13, -30, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH,  1, 1, AngleUnit.DEGREES, 2));
        WayPoint intakeExtend1Pos=new WayPoint(new Pose2D(DistanceUnit.INCH, 22, -38, AngleUnit.DEGREES, 34),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 1));
        WayPoint intakeReversePos1=new WayPoint(new Pose2D(DistanceUnit.INCH, 20, -42, AngleUnit.DEGREES, -45),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));
        WayPoint intakeExtend2Pos=new WayPoint(new Pose2D(DistanceUnit.INCH, 34, -37, AngleUnit.DEGREES, 34),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));
        WayPoint intakeExtend3Pos=new WayPoint(new Pose2D(DistanceUnit.INCH, 41, -36, AngleUnit.DEGREES, 30),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));
        WayPoint intakeReversePos2=new WayPoint(new Pose2D(DistanceUnit.INCH, 20, -42, AngleUnit.DEGREES, -45),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));
        WayPoint intakePreExtend1=new WayPoint(new Pose2D(DistanceUnit.INCH, 10, -50, AngleUnit.DEGREES, 37),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));
        WayPoint intakePreExtend2=new WayPoint(new Pose2D(DistanceUnit.INCH, 22, -42, AngleUnit.DEGREES, 30),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));
        WayPoint specimenGrab=new WayPoint(new Pose2D(DistanceUnit.INCH, 34.5, -47, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));
        WayPoint specimenGrabForward=new WayPoint(new Pose2D(DistanceUnit.INCH, 35, -53, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));
        WayPoint specimenGrab2=new WayPoint(new Pose2D(DistanceUnit.INCH, 34.5, -48, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));
        WayPoint specimenGrabForward2=new WayPoint(new Pose2D(DistanceUnit.INCH, 35, -53, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));
        WayPoint specimenGrab3=new WayPoint(new Pose2D(DistanceUnit.INCH, 34.5, -48.5, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));
        WayPoint specimenGrabForward3=new WayPoint(new Pose2D(DistanceUnit.INCH, 35, -53, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));
        WayPoint intakePosHuman=new WayPoint(new Pose2D(DistanceUnit.INCH, 12, -52, AngleUnit.DEGREES, -10),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));

        WayPoint depositPosbucket=new WayPoint(new Pose2D(DistanceUnit.INCH, -62, -50, AngleUnit.DEGREES, 30),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));
        WayPoint park=new WayPoint(new Pose2D(DistanceUnit.INCH, 45, -54, AngleUnit.DEGREES, 0),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));
        StateMachine specimenMachine = new StateMachineBuilder()
                .state(TeleopSomewhatAuto.SpecimenScoreStates.INTAKEPOS)
                .onEnter(() -> {
                    outtake.scorePos();
                    outtake.openClawWide();
                    outtake.setTargetPos(0);
                })
                .transitionTimed(0.7)
                .state(TeleopSomewhatAuto.SpecimenScoreStates.INTAKE)
                .onEnter(() -> outtake.setTargetPos(50))
                .transition(() -> closedPressed)
                .state(TeleopSomewhatAuto.SpecimenScoreStates.CLOSE_CLAW)
                .onEnter(() -> {
                    outtake.closeClaw();
                    closedPressed=false;
                })
                .transitionTimed(0.3)
                .state(TeleopSomewhatAuto.SpecimenScoreStates.HOLD)
                .onEnter(() -> outtake.specimenHoldPos())
                .transition(() -> (outtake.atTarget() && scoredPressed))
                .state(TeleopSomewhatAuto.SpecimenScoreStates.SCORE)
                .onEnter(() -> {
                    outtake.specimenScorePos();
                    scoredPressed=false;
                })
                .transitionTimed(0.4)
                .state(TeleopSomewhatAuto.SpecimenScoreStates.OPENCLAW)
                .onEnter(() -> outtake.openClaw())
                .transitionTimed(0.2)
                .state(TeleopSomewhatAuto.SpecimenScoreStates.RETRACT)
                .onEnter(() -> {
                    outtake.setTargetPos(0);
                    outtake.scorePos();
                    outtake.openClawWide();
                })
                .transition(() -> outtake.atTarget(), TeleopSomewhatAuto.SpecimenScoreStates.INTAKEPOS)
                .transitionTimed(3, TeleopSomewhatAuto.SpecimenScoreStates.INTAKEPOS)
                .build();

        StateMachine sampleMachine = new StateMachineBuilder()
                .state(TeleopSomewhatAuto.SampleStates.IDLE)
                .onEnter(() -> {
                    intake.retract();
                    intake.setPower(0);
                })
                .transition(() -> yPressed)
                .state(TeleopSomewhatAuto.SampleStates.EXTEND)
                .onEnter(()->{
                    intake.setExtended(true);
                    yPressed=false;
                })
                .transitionTimed(0.1)
                .state(TeleopSomewhatAuto.SampleStates.DROP)
                .onEnter(() -> {
                    intake.intakePosition();
                    intake.setPower(0.8);
                })
                .transition(()->intake.getDistance()<4)
                .transitionTimed(3)
                .state(TeleopSomewhatAuto.SampleStates.RETRACT)
                .onEnter(() -> {
                    intake.retract();
                    intake.setCover(true);
                    intake.setPower(0.7);
                    outtake.transferPos();
                })
                .transitionTimed(0.2)
                .state(TeleopSomewhatAuto.SampleStates.OPENCOVER)
                .onEnter(() -> intake.setCover(false))
                .transition(() -> intake.isDone())

                .state(TeleopSomewhatAuto.SampleStates.WAIT)
                .onEnter(() -> intake.setPower(0.4))
                .transitionTimed(0.7)

                .state(TeleopSomewhatAuto.SampleStates.CLOSE)
                .onEnter(() -> outtake.closeClaw())
                .transitionTimed(0.2)

                .state(TeleopSomewhatAuto.SampleStates.LIFT)
                .onEnter(() -> outtake.setTargetPos(2950))
                .transitionTimed(0.2)

                .state(TeleopSomewhatAuto.SampleStates.WRIST).onEnter(() -> {
                    outtake.scorePos();
                    intake.setPower(0);
                    intake.setPower(-0.2);
                })
                .transition(() -> lbPressed)

                .state(TeleopSomewhatAuto.SampleStates.OPEN).onEnter(() -> {
                    outtake.openClaw();
                    lbPressed=false;
                }).transitionTimed(2)

                .state(TeleopSomewhatAuto.SampleStates.LOWERLIFT).onEnter(() -> {
                    outtake.setTargetPos(0);
                    outtake.transferPos();
                }).transition(() -> (outtake.atTarget() || yPressed), TeleopSomewhatAuto.SampleStates.IDLE).build();
        StateMachine autoMachine = new StateMachineBuilder()
                .state(autoStates.clawclose)
                .onEnter(()-> outtake.closeClaw())
                .transitionTimed(0.1)
                .state(autoStates.depositPosPreload)
                .onEnter(()->{
                    drive.setTarget(depositPosPreload);
                })
                .transition(()->drive.atTarget())
                .transitionTimed(1.2)
                .state(autoStates.pauseToDepo)
                .onEnter(()->drive.setTarget(depositPosPreload2))
                .transitionTimed(0.3)
                .state(autoStates.scorePreload)
                .onEnter(()->scoredPressed=true)
                .transitionTimed(0.6)
                .state(autoStates.intakePreExtend1)
                .onEnter(()->drive.setTarget(intakePreExtend1))
                .transitionTimed(0.2)
                .state(autoStates.intakeExtend1Pos)
                .onEnter(()->drive.setTarget(intakeExtend1Pos))
                .transitionTimed(0.2)
                .state(autoStates.intakeExtend1)
                .onEnter(()->{
                    intake.intakePosition();
                    intake.setPower(1);
                })
                .transitionTimed(0.1)
                .state(autoStates.intakeExtendStart)
                .onEnter(()->intake.setPower(1))
                .transitionTimed(0.6)

                .state(autoStates.intakeReversePos1)
                .onEnter(()->drive.setTarget(intakeReversePos1))
                .transitionTimed(0.6)
                .state(autoStates.intakeReverse1)
                .onEnter(()->intake.setPower(-1))
                .transitionTimed(0.3)
                .state(autoStates.intakePreExtend2)
                .onEnter(()->drive.setTarget(intakePreExtend2))
                .transitionTimed(0.1)
                .state(autoStates.intakeExtend2)
                .onEnter(()->intake.setPower(1))
                .transitionTimed(0.2)
                .state(autoStates.intakeExtend2Pos)
                .onEnter(()->drive.setTarget(intakeExtend2Pos))
                .transitionTimed(0.6)
                .state(autoStates.intakeReversePos2)
                .onEnter(()->drive.setTarget(intakeReversePos1))
                .transitionTimed(0.7)
                .state(autoStates.intakeReverse2)
                .onEnter(()->intake.setPower(-1))
                .transitionTimed(0.4)
                .state(autoStates.intakePreExtend3)
                .onEnter(()->drive.setTarget(intakePreExtend2))
                .transitionTimed(0.1)
                .state(autoStates.intakeExtend3)
                .onEnter(()->intake.setPower(1))
                .transitionTimed(0.2)
                .state(autoStates.intakeExtend3Pos)
                .onEnter(()->drive.setTarget(intakeExtend3Pos))
                .transitionTimed(0.8)
                .state(autoStates.intakeReversePos3)
                .onEnter(()->drive.setTarget(intakeReversePos1))
                .transitionTimed(0.7)
                .state(autoStates.intakeReverse3)
                .onEnter(()->intake.setPower(-1))
                .transitionTimed(0.5)
                .state((autoStates.intakeRetract1))
                .onEnter(()->intake.retract())
                .transitionTimed(0.1)
                .state(autoStates.intakePos1)
                .onEnter(()->drive.setTarget(specimenGrab))
                .transitionTimed(1.2)
                .state(autoStates.intakePos1f2)
                .onEnter(()->drive.setTarget(specimenGrabForward))
                .transitionTimed(0.5)
                .state(autoStates.closeClaw1)
                .onEnter(()-> closedPressed=true)
                .transitionTimed(0.3)
                .state(autoStates.depositPos1)
                .onEnter(()->drive.setTarget(depositPos1))
                .transition(()->drive.atTarget())
                .transitionTimed(0.8)
                .state(autoStates.depositPos1f2)
                .onEnter(()->drive.setTarget(depositPos2))
                .transitionTimed(0.7)
                .state(autoStates.score1)
                .onEnter(()->scoredPressed=true)
                .transitionTimed(0.7)
                .state(autoStates.preintake2)
                .onEnter(()->drive.setTarget(preintake))
                .transitionTimed(0.2)
                .state(autoStates.intakePos2)
                .onEnter(()->drive.setTarget(specimenGrab3))
                .transitionTimed(1)
                .state(autoStates.intakePos2f2)
                .onEnter(()->drive.setTarget(specimenGrabForward3))
                .transitionTimed(0.5)
                .state(autoStates.closeClaw2)
                .onEnter(()-> closedPressed=true)
                .transitionTimed(0.3)
                .state(autoStates.depositPos2)
                .onEnter(()->drive.setTarget(depositPos12))
                .transition(()->drive.atTarget())
                .transitionTimed(0.7)
                .state(autoStates.depositPos2f2)
                .onEnter(()->drive.setTarget(depositPos22))
                .transitionTimed(0.7)
                .state(autoStates.score2)
                .onEnter(()->scoredPressed=true)
                .transitionTimed(0.7)
                .state(autoStates.preintake3)
                .onEnter(()->drive.setTarget(preintake))
                .transitionTimed(0.2)
                .state(autoStates.intakePos3)
                .onEnter(()->drive.setTarget(specimenGrab3))
                .transitionTimed(1)
                .state(autoStates.intakePos3f2)
                .onEnter(()->drive.setTarget(specimenGrabForward3))
                .transitionTimed(0.5)
                .state(autoStates.closeClaw3)
                .onEnter(()-> closedPressed=true)
                .transitionTimed(0.3)
                .state(autoStates.depositPos3)
                .onEnter(()->drive.setTarget(depositPos13))
                .transition(()->drive.atTarget())
                .transitionTimed(0.7)
                .state(autoStates.depositPos3f2)
                .onEnter(()->drive.setTarget(depositPos23))
                .transitionTimed(0.7)
                .state(autoStates.score3)
                .onEnter(()->scoredPressed=true)
                .transitionTimed(0.7)
                .state(autoStates.preintake4)
                .onEnter(()->drive.setTarget(preintake))
                .transitionTimed(0.2)
                .state(autoStates.intakePos4)
                .onEnter(()->drive.setTarget(specimenGrab2))
                .transitionTimed(1)
                .state(autoStates.intakePos4f2)
                .onEnter(()->drive.setTarget(specimenGrabForward2))
                .transitionTimed(0.5)
                .state(autoStates.closeClaw4)
                .onEnter(()-> closedPressed=true)
                .transitionTimed(0.3)
                .state(autoStates.depositPos4)
                .onEnter(()->drive.setTarget(depositPos14))
                .transition(()->drive.atTarget())
                .transitionTimed(0.6)
                .state(autoStates.depositPos4f2)
                .onEnter(()->drive.setTarget(depositPos24))
                .transitionTimed(0.7)
                .state(autoStates.score4)
                .onEnter(()->lbPressed=true)
                .transitionTimed(0.7)
                .state(autoStates.backintakePosHuman)
                .onEnter(()->drive.setTarget(backintakePosHuman))
                .transitionTimed(0.3)
                .state(autoStates.intakePosHuman)
                .onEnter(()->drive.setTarget(intakePosHuman))
                .transitionTimed(0.7)
                .state(autoStates.intakeExtend4)
                .onEnter(()->yPressed=true)
                .transitionTimed(1)
                .state(autoStates.depositPosbucket)
                .onEnter(()->drive.setTarget(depositPosbucket))
                .build();

        WayPoint startPoint=new WayPoint(new Pose2D(DistanceUnit.INCH, 2, -60.5, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH, 0.5, 0.5, AngleUnit.DEGREES, 0.5));

        drive.setTarget(startPoint);
        drive.setPosition(startPoint.getPosition());

        while (opModeInInit()){
            drive.update();
        }

        waitForStart();


        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
        intake.retract();
        specimenMachine.start();
        sampleMachine.start();

        autoMachine.start();
        sampleMachine.setState(TeleopSomewhatAuto.SpecimenScoreStates.IDLE);
        specimenMachine.setState(TeleopSomewhatAuto.SpecimenScoreStates.CLOSE_CLAW);
        outtake.resetEncoder();
        long prevLoop=System.nanoTime();
        while (opModeIsActive()){
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
            autoMachine.update();
            specimenMachine.update();
            sampleMachine.update();
            drive.update();
            drive.updatePIDS();
            intake.update();
            outtake.update();
            long currLoop = System.nanoTime();
            telemetry.addData("Outtake position", outtake.getLiftPos());
            telemetry.addData("State", autoMachine.getState());
            telemetry.addData("Specimen State", specimenMachine.getState());
            telemetry.addData("At target", outtake.atTarget());

            telemetry.addData("Ms per loop", (currLoop - prevLoop) / 1000000);
            prevLoop = currLoop;
            telemetry.update();
        }
    }
}
