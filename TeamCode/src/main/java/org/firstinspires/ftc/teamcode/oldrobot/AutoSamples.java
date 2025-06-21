package org.firstinspires.ftc.teamcode.oldrobot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.IntakeCopy;
import org.firstinspires.ftc.teamcode.oldrobot.pathing.WayPoint;
import org.firstinspires.ftc.teamcode.oldrobot.subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.oldrobot.subsystems.Outtake;

import java.util.List;

@Config
@Autonomous
public class AutoSamples extends LinearOpMode {
    MecanumDrivetrain drive;
    IntakeCopy intake;
    Outtake outtake;
    public static boolean yPressed=false;
    public static boolean lbPressed=false;
    enum autoStates {PREBUCKET1, BUCKET1, SCORE1,
        PRESAMPLE1, EXTEND1, INTAKE1, PREBUCKET2, BUCKET2, SCORE2,
        PRESAMPLE2, EXTEND2, INTAKE2, PREBUCKET3, BUCKET3, SCORE3,
        PRESAMPLE3, EXTEND3, INTAKE3, PREBUCKET4, BUCKET4, SCORE4,
        PREPARK, LIFTOUTTAKE, PARK, TOUCHBAR
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
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        drive = new MecanumDrivetrain(hardwareMap, telemetry, dashboard);
        // intake = new IntakeCopy(hardwareMap);
        outtake = new Outtake(hardwareMap, telemetry);

        WayPoint bucketPos1 = new WayPoint(new Pose2D(DistanceUnit.INCH, -49, -50, AngleUnit.DEGREES, 45),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));
        WayPoint bucketPos2 = new WayPoint(new Pose2D(DistanceUnit.INCH, -53, -54, AngleUnit.DEGREES, 45),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));
        WayPoint prepark = new WayPoint(new Pose2D(DistanceUnit.INCH, -34, -8, AngleUnit.DEGREES, 180),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));
        WayPoint park = new WayPoint(new Pose2D(DistanceUnit.INCH, -21, -10, AngleUnit.DEGREES, 180),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));


        WayPoint presample1 = new WayPoint(new Pose2D(DistanceUnit.INCH, -47, -56.5, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH, 0.5, 0.5, AngleUnit.DEGREES, 1));
        WayPoint sample1 = new WayPoint(new Pose2D(DistanceUnit.INCH, -47, -50, AngleUnit.DEGREES, 89),
                new Pose2D(DistanceUnit.INCH, 0.5, 0.5, AngleUnit.DEGREES, 1));
        WayPoint presample2 = new WayPoint(new Pose2D(DistanceUnit.INCH, -53, -56.5, AngleUnit.DEGREES, 95),
                new Pose2D(DistanceUnit.INCH, 0.5, 0.5, AngleUnit.DEGREES, 1));
        WayPoint sample2 = new WayPoint(new Pose2D(DistanceUnit.INCH, -57, -50, AngleUnit.DEGREES, 95),
                new Pose2D(DistanceUnit.INCH, 0.5, 0.5, AngleUnit.DEGREES, 1));
        WayPoint presample3 = new WayPoint(new Pose2D(DistanceUnit.INCH, -41.5, -39, AngleUnit.DEGREES, 155),
                new Pose2D(DistanceUnit.INCH, 0.5, 0.5, AngleUnit.DEGREES, 1));
        WayPoint sample3 = new WayPoint(new Pose2D(DistanceUnit.INCH, -44, -34, AngleUnit.DEGREES, 160),
                new Pose2D(DistanceUnit.INCH, 0.5, 0.5, AngleUnit.DEGREES, 1));


       /* StateMachine sampleMachine = new StateMachineBuilder()
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
                .transition(()->intake.isSampleIntaked())
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
                    outtake.openClawWide();
                    lbPressed=false;
                }).transitionTimed(2)

                .state(TeleopSomewhatAuto.SampleStates.LOWERLIFT).onEnter(() -> {
                    outtake.setTargetPos(0);
                    outtake.transferPos();
                }).transition(() -> (outtake.atTarget() || yPressed), TeleopSomewhatAuto.SampleStates.IDLE).build();
        StateMachine autoMachine = new StateMachineBuilder()
                .state(autoStates.PREBUCKET1)
                .onEnter(()->drive.setTarget(bucketPos1))
                .transition(()-> drive.atTarget() && outtake.getLiftPos()>2850)
                .state(autoStates.BUCKET1)
                .onEnter(()->drive.setTarget(bucketPos2))
                .transition(()-> drive.atTarget())
                .state(autoStates.SCORE1)
                .onEnter(()->lbPressed=true)
                .transitionTimed(0.7)
                .state(autoStates.PRESAMPLE1)
                .onEnter(()->drive.setTarget(presample1))
                .transition(()->drive.atTarget())
                .state(autoStates.EXTEND1)
                .onEnter(()->yPressed=true)
                .transitionTimed(1.3)
                .state(autoStates.INTAKE1)
                .onEnter(()->drive.setTarget(sample1))
                .transition(()->sampleMachine.getState()!= TeleopSomewhatAuto.SampleStates.DROP && sampleMachine.getState() != TeleopSomewhatAuto.SampleStates.EXTEND)
                .state(autoStates.PREBUCKET2)
                .onEnter(()->drive.setTarget(bucketPos1))
                .transition(()->drive.atTarget() && outtake.getLiftPos()>2850)
                .state(autoStates.BUCKET2)
                .onEnter(()->drive.setTarget(bucketPos2))
                .transition(()->drive.atTarget())
                .state(autoStates.SCORE2)
                .onEnter(()->lbPressed=true)
                .transitionTimed(0.7)

                .state(autoStates.PRESAMPLE2)
                .onEnter(()->drive.setTarget(presample2))
                .transition(()-> drive.atTarget())

                .state(autoStates.EXTEND2)
                .onEnter(()->yPressed=true)
                .transitionTimed(1.3)
                .state(autoStates.INTAKE2)
                .onEnter(()->drive.setTarget(sample2))
                .transition(()->sampleMachine.getState()!= TeleopSomewhatAuto.SampleStates.DROP && sampleMachine.getState() != TeleopSomewhatAuto.SampleStates.EXTEND)
                .state(autoStates.PREBUCKET3)
                .onEnter(()->drive.setTarget(bucketPos1))
                .transition(()->drive.atTarget() && outtake.getLiftPos()>2850)
                .state(autoStates.BUCKET3)
                .onEnter(()->drive.setTarget(bucketPos2))
                .transition(()->drive.atTarget())
                .state(autoStates.SCORE3)
                .onEnter(()->lbPressed=true)
                .transitionTimed(0.7)

                .state(autoStates.PRESAMPLE3)
                .onEnter(()->drive.setTarget(presample3))
                .transition(()-> drive.atTarget())

                .state(autoStates.EXTEND3)
                .onEnter(()->yPressed=true)
                .transitionTimed(1.3)
                .state(autoStates.INTAKE3)
                .onEnter(()->drive.setTarget(sample3))
                .transition(()->sampleMachine.getState()!= TeleopSomewhatAuto.SampleStates.DROP && sampleMachine.getState() != TeleopSomewhatAuto.SampleStates.EXTEND)
                .state(autoStates.PREBUCKET4)
                .onEnter(()->drive.setTarget(bucketPos1))
                .transition(()->drive.atTarget() && outtake.getLiftPos()>2850)
                .state(autoStates.BUCKET4)
                .onEnter(()->drive.setTarget(bucketPos2))
                .transition(()->drive.atTarget())
                .state(autoStates.SCORE4)
                .onEnter(()->lbPressed=true)
                .transitionTimed(0.7)
                .state(autoStates.PREPARK)
                .onEnter(()-> {
                    drive.setTarget(prepark);
                })
                .transitionTimed(0.5)
                .state(autoStates.LIFTOUTTAKE)
                .onEnter(()->outtake.setTargetPos(100))
                .transition(()->drive.atTarget())
                .state(autoStates.PARK)
                .onEnter(()-> {
                    drive.setTarget(park);
                    outtake.setFlipPos(0.28);
                    outtake.setWristPos(0.5);
                    outtake.setTargetPos(200);
                    outtake.closeClaw();
                })
                .transitionTimed(2)
                .state(autoStates.TOUCHBAR)
                .onEnter(()->{
                    outtake.setFlipPos(0.2);
                    drive.setTarget(new WayPoint(drive.position, new Pose2D(DistanceUnit.INCH, 2, 2, AngleUnit.DEGREES, 2)));
                })
                .build();

        WayPoint startPoint=new WayPoint(new Pose2D(DistanceUnit.INCH, -36, -63, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH, 0.5, 0.5, AngleUnit.DEGREES, 0.5));

        waitForStart();
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
        drive.setTarget(startPoint);
        drive.setPosition(startPoint.getPosition());
        sampleMachine.start();
        autoMachine.start();
        sampleMachine.setState(TeleopSomewhatAuto.SampleStates.RETRACT);
        outtake.resetEncoder();
        long prevLoop=System.nanoTime();
        while (opModeIsActive()){
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
            autoMachine.update();
            sampleMachine.update();
            drive.update();
            drive.updatePIDS();
            intake.update();
            outtake.update();
            long currLoop = System.nanoTime();
            telemetry.addData("Outtake position", outtake.getLiftPos());
            telemetry.addData("Ms per loop", (currLoop - prevLoop) / 1000000);
            prevLoop = currLoop;
            telemetry.update();
        }
    } */

    }}