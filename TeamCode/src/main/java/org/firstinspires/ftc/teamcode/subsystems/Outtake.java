package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.CachedMotorEx;

public class Outtake implements Subsystem{
    private CachedMotorEx leftLift, rightLift;
    private Servo depositFlip1, depositFlip2, wrist, claw;
    private TouchSensor outtakeEnd0;
    private int targetPos=0;

    public enum possibleHeights {HIGHBASKET, LOWBASKET, HIGHSPECIMEN, LOWSPECIMEN, HUMANPLAYER}

    private PIDFController controller;
    Telemetry telemetry;

    public Outtake(HardwareMap hwMap, Telemetry telemetry){
        leftLift= new CachedMotorEx(hwMap, "leftSlideStack");
        rightLift= new CachedMotorEx(hwMap, "rightSlideStack");

        depositFlip1=hwMap.servo.get("depositFlip1");
        depositFlip2=hwMap.servo.get("depositFlip2");

        wrist=hwMap.servo.get("wrist");
        claw=hwMap.servo.get("claw");

        outtakeEnd0=hwMap.get(TouchSensor.class, "outtake_limit_0");


        resetEncoder();

        controller = new PIDFController(0.01, 0, 0, 0);

        controller.setTolerance(30);

        this.telemetry=telemetry;
    }

    public void setPower(double power){
        leftLift.setPower(-power);
        rightLift.setPower(power);
    }

    public int getLiftPos(){
        return -rightLift.getCurrentPosition();
    }

    public void openClaw(){
        claw.setPosition(0.84);
    }
    public void openClawWide(){
        claw.setPosition(0.6);
    }


    public void closeClaw(){
        claw.setPosition(1);
    }

    public void setFlipPos(double pos){
        depositFlip1.setPosition(pos);
        depositFlip2.setPosition(1-pos);
    }

    public void setWristPos(double pos){
        wrist.setPosition(pos);
    }

    public void transferPos(){
        setFlipPos(0.84);
        setWristPos(0.20);
        openClaw();
        setTargetPos(0);
    }
    public void resetEncoder(){
        leftLift.resetEncoder();
        rightLift.resetEncoder();
    }
    public void scorePos(){
        setFlipPos(0.05);
        setWristPos(0.50);
        closeClaw();
    }
    public void scorePosTeleop(){
        setFlipPos(0.2);
        setTargetPos(2600);
        setWristPos(0.65);
    }
    @Override
    public void update() {
        if (outtakeEnd0.isPressed()){
            resetEncoder();
        }
        double controller_output=controller.calculate(getLiftPos());
        telemetry.addData("Outtake applied power", controller_output);
        setPower(controller_output);
    }
    public boolean atTarget(){
        return controller.atSetPoint();
    }

    public void setTargetPos(int targetPos) {
        this.targetPos = targetPos;
        controller.setSetPoint(targetPos);
    }
    public void specimenHoldPos(){
        setFlipPos(0.8);
        closeClaw();
        setWristPos(0.57);
        setTargetPos(1200);
    }
    public void specimenScorePos(){
        setFlipPos(0.6);
        setWristPos(0.6);
        closeClaw();
        setTargetPos(1700);
    }

    public int getTargetPos() {
        return targetPos;
    }
    public double getCurrent(){
        return rightLift.getCurrent();
    }
}
