/* package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
@Config
@Autonomous
public class AutoSpecimen extends LinearOpMode {
    MecanumDrivetrain drive;
    Intake intake;
    Outtake outtake;
    public static boolean closedPressed=false;
    public static boolean scoredPressed=false;
    enum autoStates {
        clawclose, depositPosPreload, pauseToDepo, scorePreload,
        intakePreExtend1,intakeExtend1, intakeExtendStart, intakeExtend1Pos, intakeReversePos1, intakeReverse1,
        intakePreExtend2, intakeExtend2, intakeExtend2Pos, intakeReversePos2, intakeReverse2,
        intakeRetract1, intakePos1,intakePos1f2, closeClaw1, depositPos1, depositPos1f2, score1,
        intakePos2, intakePos2f2, closeClaw2, depositPos2, depositPos2f2, score2,
        park
    }

    @Override

    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry= new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        drive=new MecanumDrivetrain(hardwareMap, telemetry, dashboard);
        intake= new Intake(hardwareMap);
        outtake= new Outtake(hardwareMap, telemetry);
        WayPoint depositPosPreload=new WayPoint(new Pose2D(DistanceUnit.INCH, -5, -34, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH,  1, 2, AngleUnit.DEGREES, 2));
        WayPoint depositPosPreload2=new WayPoint(new Pose2D(DistanceUnit.INCH, -5, -28, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH,  1, 1, AngleUnit.DEGREES, 2));
        WayPoint depositPos1=new WayPoint(new Pose2D(DistanceUnit.INCH, -8, -50, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH,  1, 2, AngleUnit.DEGREES, 2));
        WayPoint depositPos2=new WayPoint(new Pose2D(DistanceUnit.INCH, -8, -28, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH,  1, 1, AngleUnit.DEGREES, 2));
        WayPoint depositPos12=new WayPoint(new Pose2D(DistanceUnit.INCH, -12, -50, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH,  1, 2, AngleUnit.DEGREES, 2));
        WayPoint depositPos22=new WayPoint(new Pose2D(DistanceUnit.INCH, -12, -25, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH,  1, 1, AngleUnit.DEGREES, 2));
        WayPoint intakeExtend1Pos=new WayPoint(new Pose2D(DistanceUnit.INCH, 22, -35, AngleUnit.DEGREES, 34),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 1));
        WayPoint intakeReversePos1=new WayPoint(new Pose2D(DistanceUnit.INCH, 20, -42, AngleUnit.DEGREES, -45),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));
        WayPoint intakeExtend2Pos=new WayPoint(new Pose2D(DistanceUnit.INCH, 34, -37, AngleUnit.DEGREES, 34),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));
        WayPoint intakeReversePos2=new WayPoint(new Pose2D(DistanceUnit.INCH, 20, -42, AngleUnit.DEGREES, -45),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));
        WayPoint intakePreExtend1=new WayPoint(new Pose2D(DistanceUnit.INCH, 15, -45, AngleUnit.DEGREES, 37),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));
        WayPoint intakePreExtend2=new WayPoint(new Pose2D(DistanceUnit.INCH, 22, -42, AngleUnit.DEGREES, 30),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));
        WayPoint specimenGrab=new WayPoint(new Pose2D(DistanceUnit.INCH, 30, -51, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));
        WayPoint specimenGrabForward=new WayPoint(new Pose2D(DistanceUnit.INCH, 30, -54, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));
        WayPoint specimenGrab2=new WayPoint(new Pose2D(DistanceUnit.INCH, 29, -47, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));
        WayPoint specimenGrabForward2=new WayPoint(new Pose2D(DistanceUnit.INCH, 29, -51, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));
        WayPoint park=new WayPoint(new Pose2D(DistanceUnit.INCH, 45, -54, AngleUnit.DEGREES, 0),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));
        StateMachine specimenMachine = new StateMachineBuilder()
                .state(TeleopSomewhatAuto.SpecimenScoreStates.INTAKEPOS)
                .onEnter(() -> {
                    outtake.scorePos();
                    outtake.openClawWide();
                    outtake.setTargetPos(200);
                })
                .transitionTimed(0.7)
                .state(TeleopSomewhatAuto.SpecimenScoreStates.INTAKE)
                .onEnter(() -> outtake.setTargetPos(0))
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
                .transitionTimed(0.3)
                .state(TeleopSomewhatAuto.SpecimenScoreStates.OPENCLAW)
                .onEnter(() -> outtake.openClaw())
                .transitionTimed(0.2)
                .state(TeleopSomewhatAuto.SpecimenScoreStates.RETRACT)
                .onEnter(() -> outtake.transferPos())
                .transition(() -> outtake.atTarget(), TeleopSomewhatAuto.SpecimenScoreStates.INTAKEPOS)
                .transitionTimed(3, TeleopSomewhatAuto.SpecimenScoreStates.INTAKEPOS)
                .build();

        StateMachine autoMachine = new StateMachineBuilder()
                .state(autoStates.clawclose)
                .onEnter(()-> outtake.closeClaw())
                .transitionTimed(0.1)
                .state(autoStates.depositPosPreload)
                .onEnter(()->drive.setTarget(depositPosPreload))
                .transition(()->drive.atTarget())
                .transitionTimed(3)
                .state(autoStates.pauseToDepo)
                .onEnter(()->drive.setTarget(depositPosPreload2))
                .transitionTimed(0.5)
                .state(autoStates.scorePreload)
                .onEnter(()->scoredPressed=true)
                .transitionTimed(0.7)
                .state(autoStates.intakePreExtend1)
                .onEnter(()->drive.setTarget(intakePreExtend1))
                .transition(()->drive.atTarget())
                .state(autoStates.intakeExtend1)
                .onEnter(()->intake.intakePosition())
                .transitionTimed(0.4)
                .state(autoStates.intakeExtendStart)
                .onEnter(()->intake.setPower(0.8))
                .transitionTimed(0.4)
                .state(autoStates.intakeExtend1Pos)
                .onEnter(()->drive.setTarget(intakeExtend1Pos))
                .transitionTimed(1)
                .state(autoStates.intakeReversePos1)
                .onEnter(()->drive.setTarget(intakeReversePos1))
                .transition(()->drive.atTarget())
                .state(autoStates.intakeReverse1)
                .onEnter(()->intake.setPower(-0.5))
                .transitionTimed(1, autoStates.intakePreExtend2)
                .state(autoStates.intakePreExtend2)
                .onEnter(()->drive.setTarget(intakePreExtend2))
                .transitionTimed(0.5)
                .state(autoStates.intakeExtend2)
                .onEnter(()->intake.setPower(0.8))
                .transitionTimed(0.8)
                .state(autoStates.intakeExtend2Pos)
                .onEnter(()->drive.setTarget(intakeExtend2Pos))
                .transitionTimed(1)
                .state(autoStates.intakeReversePos2)
                .onEnter(()->drive.setTarget(intakeReversePos1))
                .transition(()->drive.atTarget())
                .state(autoStates.intakeReverse2)
                .onEnter(()->intake.setPower(-1))
                .transitionTimed(0.6)
                .state((autoStates.intakeRetract1))
                .onEnter(()->intake.retract())
                .transitionTimed(0.4)
                .state(autoStates.intakePos1)
                .onEnter(()->drive.setTarget(specimenGrab))
                .transitionTimed(2.5)
                .state(autoStates.intakePos1f2)
                .onEnter(()->drive.setTarget(specimenGrabForward))
                .transitionTimed(1)
                .state(autoStates.closeClaw1)
                .onEnter(()-> closedPressed=true)
                .transitionTimed(0.5)
                .state(autoStates.depositPos1)
                .onEnter(()->drive.setTarget(depositPos1))
                .transition(()->drive.atTarget())
                .transitionTimed(1)
                .state(autoStates.depositPos1f2)
                .onEnter(()->drive.setTarget(depositPos2))
                .transitionTimed(1.5)
                .state(autoStates.score1)
                .onEnter(()->scoredPressed=true)
                .transitionTimed(1)
                .state(autoStates.intakePos2)
                .onEnter(()->drive.setTarget(specimenGrab2))
                .transitionTimed(2.5)
                .state(autoStates.intakePos2f2)
                .onEnter(()->drive.setTarget(specimenGrabForward2))
                .transitionTimed(1)
                .state(autoStates.closeClaw2)
                .onEnter(()-> closedPressed=true)
                .transitionTimed(0.5)
                .state(autoStates.depositPos2)
                .onEnter(()->drive.setTarget(depositPos12))
                .transition(()->drive.atTarget())
                .transitionTimed(1)
                .state(autoStates.depositPos2f2)
                .onEnter(()->drive.setTarget(depositPos22))
                .transitionTimed(1.5)
                .state(autoStates.score2)
                .onEnter(()->scoredPressed=true)
                .transitionTimed(1)
                .state(autoStates.park)
                .onEnter(()->drive.setTarget(park))
                .build();

        WayPoint startPoint=new WayPoint(new Pose2D(DistanceUnit.INCH, 2, -60.5, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH, 0.5, 0.5, AngleUnit.DEGREES, 0.5));

        waitForStart();
        intake.retract();
        drive.setTarget(startPoint);
        drive.setPosition(startPoint.getPosition());
        intake.retract();
        specimenMachine.start();
        autoMachine.start();
        specimenMachine.setState(TeleopSomewhatAuto.SpecimenScoreStates.CLOSE_CLAW);
        outtake.resetEncoder();
        long prevLoop=System.nanoTime();
        while (opModeIsActive()){
            autoMachine.update();
            specimenMachine.update();
            drive.update();
            drive.updatePIDS();
            intake.update();
            outtake.update();
            long currLoop = System.nanoTime();
            telemetry.addData("Outtake position", outtake.getLiftPos());
            telemetry.addData("State", autoMachine.getState());
            telemetry.addData("Specimen State", specimenMachine.getState());

            telemetry.addData("Ms per loop", (currLoop - prevLoop) / 1000000);
            prevLoop = currLoop;
            telemetry.update();
        }
    }
}
*/