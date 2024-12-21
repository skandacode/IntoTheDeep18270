package org.firstinspires.ftc.teamcode;

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
public class AutoSamplesFaster extends LinearOpMode {
    MecanumDrivetrain drive;
    Intake intake;
    Outtake outtake;
    public static boolean yPressed=false;
    public static boolean lbPressed=false;
    public static Intake.SampleColor allianceColor= Intake.SampleColor.BLUE;
    public static Intake.SampleColor opposingColor= Intake.SampleColor.RED;

    enum autoStates {PREBUCKET1, BUCKET1, SCORE1,
        EXTEND1, INTAKE1, PREBUCKET2, BUCKET2, SCORE2,
        EXTEND2, INTAKE2, PREBUCKET3, BUCKET3, SCORE3,
        EXTEND3, INTAKE3, PREBUCKET4, BUCKET4, SCORE4,
        PRESUB, SUB, INTAKESUB, RETRACTSUB, TRYAGAIN, INTAKESUBSTRAFE, EJECT, PREBUCKETSUB, PREBUCKET5, BUCKET5, SCORE5,
        PREPARK, LIFTOUTTAKE, PARK, TOUCHBAR
    }

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry= new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        drive=new MecanumDrivetrain(hardwareMap, telemetry, dashboard);
        intake= new Intake(hardwareMap);
        outtake= new Outtake(hardwareMap, telemetry);

        WayPoint bucketPos2=new WayPoint(new Pose2D(DistanceUnit.INCH, -53, -55.5, AngleUnit.DEGREES, 45),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));
        WayPoint bucketPos25th=new WayPoint(new Pose2D(DistanceUnit.INCH, -54.5, -55.5, AngleUnit.DEGREES, 45),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));
        WayPoint prepark=new WayPoint(new Pose2D(DistanceUnit.INCH, -34, -8, AngleUnit.DEGREES, 180),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));
        WayPoint park=new WayPoint(new Pose2D(DistanceUnit.INCH, -20, -12, AngleUnit.DEGREES, 180),
                new Pose2D(DistanceUnit.INCH, 3, 3, AngleUnit.DEGREES, 2));
        WayPoint presub=new WayPoint(new Pose2D(DistanceUnit.INCH, -36, -8, AngleUnit.DEGREES, 0),
                new Pose2D(DistanceUnit.INCH, 3, 3, AngleUnit.DEGREES, 2));
        WayPoint sub=new WayPoint(new Pose2D(DistanceUnit.INCH, -15, -12, AngleUnit.DEGREES, 0),
                new Pose2D(DistanceUnit.INCH, 3, 3, AngleUnit.DEGREES, 2));
        WayPoint subStrafe=new WayPoint(new Pose2D(DistanceUnit.INCH, -15, 12, AngleUnit.DEGREES, 0),
                new Pose2D(DistanceUnit.INCH, 3, 3, AngleUnit.DEGREES, 2));


        WayPoint sample1=new WayPoint(new Pose2D(DistanceUnit.INCH, -49, -50, AngleUnit.DEGREES, 89),
                new Pose2D(DistanceUnit.INCH, 0.5, 0.5, AngleUnit.DEGREES, 1));
        WayPoint sample2=new WayPoint(new Pose2D(DistanceUnit.INCH, -54, -49, AngleUnit.DEGREES, 101),
                new Pose2D(DistanceUnit.INCH, 0.5, 0.5, AngleUnit.DEGREES, 1));
        WayPoint sample3=new WayPoint(new Pose2D(DistanceUnit.INCH, -43.5, -35.5, AngleUnit.DEGREES, 160),
                new Pose2D(DistanceUnit.INCH, 0.5, 0.5, AngleUnit.DEGREES, 1));


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
                    intake.intakePosition();
                    intake.setPower(1);
                    yPressed=false;
                })
                .transitionTimed(0.1)
                .state(TeleopSomewhatAuto.SampleStates.DROP)
                .onEnter(() -> {
                    intake.intakePosition();
                    intake.setPower(1);
                })
                .transition(()->intake.getDistance()<5)
                .transitionTimed(3)
                .state(TeleopSomewhatAuto.SampleStates.RETRACT)
                .onEnter(() -> {
                    intake.retract();
                    intake.setCover(true);
                    intake.setPower(0.6);
                    outtake.transferPos();
                })
                .transitionTimed(0.2)

                .state(TeleopSomewhatAuto.SampleStates.OPENCOVER)
                .onEnter(() -> {
                    intake.setCover(false);
                    intake.setPower(0.1);
                })
                .transition(() -> intake.isDone())

                .state(TeleopSomewhatAuto.SampleStates.WAIT)
                .onEnter(() -> intake.setPower(0.4))
                .transitionTimed(0.4)

                .state(TeleopSomewhatAuto.SampleStates.CLOSE)
                .onEnter(() -> outtake.closeClaw())
                .transitionTimed(0.2)


                .state(FasterTeleop.SampleStates.LIFT)
                .onEnter(() -> {
                    outtake.setTargetPos(2950);
                    intake.setPower(0);
                })
                .transitionTimed(0.3)

                .state(FasterTeleop.SampleStates.PARTIALFLIP)
                .onEnter(()->{
                    outtake.setFlipPos(0.3);
                    outtake.setWristPos(0.5);
                })
                .transition(()->outtake.getLiftPos()>2450)

                .state(FasterTeleop.SampleStates.WRIST).onEnter(() -> {
                    outtake.scorePosTeleop();
                })

                .transition(() -> lbPressed)

                .state(TeleopSomewhatAuto.SampleStates.WAITBEFOREOPEN)
                .transitionTimed(0.15)

                .state(TeleopSomewhatAuto.SampleStates.OPEN).onEnter(() -> {
                    outtake.openClaw();
                    lbPressed=false;
                }).transitionTimed(1)

                .state(TeleopSomewhatAuto.SampleStates.LOWERLIFT).onEnter(() -> {
                    outtake.setTargetPos(0);
                    outtake.transferPos();
                }).transition(() -> (outtake.atTarget() || yPressed), TeleopSomewhatAuto.SampleStates.IDLE).build();
        StateMachine autoMachine = new StateMachineBuilder()
                .state(autoStates.PREBUCKET1)
                .onEnter(()->drive.setTarget(bucketPos2))
                .transitionTimed(1.5)
                .state(autoStates.SCORE1)
                .onEnter(()->lbPressed=true)
                .transitionTimed(0.8)


                .state(autoStates.INTAKE1)
                .onEnter(()->drive.setTarget(sample1))
                .transition(()->drive.atTarget())
                .state(autoStates.EXTEND1)
                .onEnter(()->yPressed=true)
                .transition(()->sampleMachine.getState()== TeleopSomewhatAuto.SampleStates.RETRACT)
                .state(autoStates.PREBUCKET2)
                .onEnter(()->drive.setTarget(bucketPos2))
                .transition(()-> drive.atTarget() && outtake.getLiftPos()>2500)
                .state(autoStates.SCORE2)
                .onEnter(()->lbPressed=true)
                .transitionTimed(0.7)


                .state(autoStates.INTAKE2)
                .onEnter(()->drive.setTarget(sample2))
                .transition(()->drive.atTarget())
                .state(autoStates.EXTEND2)
                .onEnter(()->yPressed=true)
                .transition(()->sampleMachine.getState()== TeleopSomewhatAuto.SampleStates.RETRACT)
                .state(autoStates.PREBUCKET3)
                .onEnter(()->drive.setTarget(bucketPos2))
                .transition(()-> drive.atTarget() && outtake.getLiftPos()>2500)
                .state(autoStates.SCORE3)
                .onEnter(()->lbPressed=true)
                .transitionTimed(0.7)


                .state(autoStates.INTAKE3)
                .onEnter(()->drive.setTarget(sample3))
                .transition(()->drive.atTarget())
                .state(autoStates.EXTEND3)
                .onEnter(()->yPressed=true)
                .transition(()->sampleMachine.getState()== TeleopSomewhatAuto.SampleStates.RETRACT)
                .state(autoStates.PREBUCKET4)
                .onEnter(()->drive.setTarget(bucketPos2))
                .transition(()-> drive.atTarget() && outtake.getLiftPos()>2500)
                .state(autoStates.SCORE4)
                .onEnter(()->lbPressed=true)
                .transitionTimed(0.7)
                .state(autoStates.PRESUB)
                .onEnter(()->drive.setTarget(presub))
                .transition(()-> drive.atTarget())
                .state(autoStates.SUB)
                .onEnter(()->drive.setTarget(sub))
                .transitionTimed(0.9)
                .state(autoStates.INTAKESUB)
                .onEnter(()->{
                    intake.setExtended(true);
                    intake.intakePosition();
                })
                .transitionTimed(0.9, autoStates.RETRACTSUB)
                .transition(()->intake.getColor()== Intake.SampleColor.YELLOW || intake.getColor()==allianceColor, autoStates.PREBUCKETSUB)
                .transition(()->intake.getColor()==opposingColor, autoStates.EJECT)

                .state(autoStates.RETRACTSUB)
                .onEnter(()->{
                    intake.setExtended(false);
                    intake.setPower(-0.5);
                    intake.retract();
                })
                .transitionTimed(0.5, autoStates.TRYAGAIN)

                .state(autoStates.TRYAGAIN)
                .onEnter(()->{
                    intake.setExtended(true);
                    intake.setPower(1);
                    intake.intakePosition();
                })
                .transitionTimed(1, autoStates.INTAKESUBSTRAFE)
                .transition(()->intake.getColor()== Intake.SampleColor.YELLOW || intake.getColor()==allianceColor, autoStates.PREBUCKETSUB)
                .transition(()->intake.getColor()==opposingColor, autoStates.EJECT)

                .state(autoStates.INTAKESUBSTRAFE)
                .onEnter(()->drive.setTarget(subStrafe))
                .transitionTimed(2, autoStates.PREPARK)
                .transition(()->intake.getColor()== Intake.SampleColor.YELLOW || intake.getColor()==allianceColor, autoStates.PREBUCKETSUB)
                .transition(()->intake.getColor()==opposingColor, autoStates.EJECT)

                .state(autoStates.EJECT, true)
                .onEnter(()->{
                    intake.eject();
                    intake.setCover(false);
                })
                .transitionTimed(0.7, autoStates.PREPARK)
                .state(autoStates.PREBUCKETSUB)
                .onEnter(()->{
                    sampleMachine.setState(TeleopSomewhatAuto.SampleStates.DROP);
                    intake.setCover(true);
                    drive.setTarget(presub);
                })
                .transitionTimed(1)
                .state(autoStates.PREBUCKET5)
                .onEnter(()->drive.setTarget(bucketPos25th))
                .transition(()->drive.atTarget() && outtake.getLiftPos()>2500)


                .state(autoStates.SCORE5)
                .onEnter(()->lbPressed=true)
                .transitionTimed(0.7)

                .state(autoStates.PREPARK)
                .onEnter(()-> {
                    drive.setTarget(prepark);
                    intake.retract();
                    intake.setPower(0);
                })
                .transitionTimed(0.5)
                .state(autoStates.LIFTOUTTAKE)
                .onEnter(()->outtake.setTargetPos(100))
                .transition(()->drive.atTarget())
                .state(autoStates.PARK)
                .onEnter(()-> {
                    drive.setTarget(park);
                    outtake.setFlipPos(0.3);
                    outtake.setWristPos(0.5);
                    outtake.setTargetPos(200);
                    outtake.closeClaw();
                })
                .transitionTimed(2)
                .state(autoStates.TOUCHBAR)
                .onEnter(()->{
                    outtake.setFlipPos(0.2);
                })
                .build();

        WayPoint startPoint=new WayPoint(new Pose2D(DistanceUnit.INCH, -36, -63, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH, 0.5, 0.5, AngleUnit.DEGREES, 0.5));

        drive.setTarget(startPoint);
        drive.setPosition(startPoint.getPosition());

        while (opModeInInit()){
            drive.update();
            telemetry.update();
            if (gamepad1.touchpad){
                drive.calibrateIMU();
            }
            if (gamepad1.a){
                allianceColor= Intake.SampleColor.BLUE;
                opposingColor= Intake.SampleColor.RED;
                gamepad1.setLedColor(0, 0, 1, 1000);
            }
            if (gamepad1.b){
                allianceColor= Intake.SampleColor.RED;
                opposingColor= Intake.SampleColor.BLUE;
                gamepad1.setLedColor(1, 0, 0, 1000);
            }
        }

        waitForStart();
        long startTime=System.nanoTime();
        intake.retract();
        outtake.specimenScorePos();
        outtake.setTargetPos(2600);
        drive.setTarget(startPoint);
        drive.setPosition(startPoint.getPosition());
        sampleMachine.start();
        autoMachine.start();
        sampleMachine.setState(TeleopSomewhatAuto.SampleStates.RETRACT);
        outtake.resetEncoder();
        long prevLoop=System.nanoTime();
        while (opModeIsActive()){
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
            if (autoMachine.getState()==autoStates.PARK){
                System.out.println(System.nanoTime()-startTime);
            }
        }
    }
}