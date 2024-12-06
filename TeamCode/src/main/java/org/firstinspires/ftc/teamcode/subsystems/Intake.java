package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.CachedMotorEx;

import java.util.Arrays;

public class Intake implements Subsystem {
    CachedMotorEx intakeMotor, extendoMotor;
    ServoImplEx intakeServoLeft, intakeServoRight;
    Servo cover;
    TouchSensor intakeEnd;
    RevColorSensorV3 intakecolor;

    private boolean isExtended=false;
    private boolean isDone = true;

    double intakeSpeed=0;

    public enum SampleColor {RED, BLUE, YELLOW, NONE}

    public Intake(HardwareMap hwMap){
        intakeMotor = new CachedMotorEx(hwMap, "intakeMotor");
        extendoMotor = new CachedMotorEx(hwMap, "extendoMotor");

        intakeServoLeft= (ServoImplEx) hwMap.servo.get("intakeFlip2");
        intakeServoRight= (ServoImplEx) hwMap.servo.get("intakeFlip1");
        cover = hwMap.servo.get("intakecover");

        intakeEnd = hwMap.get(TouchSensor.class, "intakeend0");
        intakecolor = hwMap.get(RevColorSensorV3.class, "intakecolor");

        extendoMotor.resetEncoder();

    }

    @Override
    public void update() {
        //bang bang controller
        if (intakeEnd.isPressed()){
            extendoMotor.resetEncoder();
        }
        if (isExtended){
            if (getExtendoMotorPos()>-450){
                setExtendo(-1);
                isDone=false;
            }else{
                setExtendo(0);
                isDone=true;
            }
        } else{
            if (getExtendoMotorPos()>-10){
                setExtendo(0.2);
                isDone=true;
            }else{
                setExtendo(1);
                isDone=false;
            }
        }
    }
    public void setPower(double power){
        intakeMotor.setPower(-power);
        intakeSpeed=power;
    }
    public void setExtendo(double power){
        extendoMotor.setPower(power);
    }
    public void setFlip(double pos){
        intakeServoLeft.setPosition(pos);
        intakeServoRight.setPosition(1-pos);
    }
    public void setCover(boolean closed){
        if (closed){
            cover.setPosition(0.94);
        }else{
            cover.setPosition(0.08);
        }
    }

    public void enableServos(){
        intakeServoLeft.setPwmEnable();
        intakeServoRight.setPwmEnable();
    }
    public void disableServos(){
        intakeServoLeft.setPwmDisable();
        intakeServoRight.setPwmDisable();
    }
    public int getExtendoMotorPos(){
        return extendoMotor.getCurrentPosition();
    }

    public void setExtended(boolean extended) {
        isExtended = extended;
    }
    public void intakePosition(){
        setExtended(true);
        setCover(true);
        setFlip(0.31);
        setPower(1);
    }
    public void eject(){
        //setCover(false);
        setFlip(0.62);
    }
    public void retract(){
        setExtended(false);
        setPower(0);
        setCover(false);
        setFlip(0.5);
    }

    public boolean isDone() {
        return isDone;
    }
    public int[] getRawSensorValues() {
        return new int[]{intakecolor.red(), intakecolor.green(), intakecolor.blue()}; // Return RGB as an array
    }
    public double getDistance(){
        return intakecolor.getDistance(DistanceUnit.CM);
    }
    public SampleColor getColor(){
        if (getDistance()<5){
            int[] rgbValues = getRawSensorValues();
            System.out.println(Arrays.toString(rgbValues));
            int[] tweakedValues = new int[] {rgbValues[0]-20, rgbValues[1]-25, rgbValues[2]-30};
            if (tweakedValues[0]>tweakedValues[1] && tweakedValues[0]>tweakedValues[2]){
                System.out.println(Arrays.toString(tweakedValues)+" Red");
                return SampleColor.RED;
            }
            if (tweakedValues[1]>tweakedValues[0] && tweakedValues[1]>tweakedValues[2]){
                System.out.println(Arrays.toString(tweakedValues)+" Yellow");
                return SampleColor.YELLOW;
            }
            if (tweakedValues[2]>tweakedValues[1] && tweakedValues[2]>tweakedValues[0]){
                System.out.println(Arrays.toString(tweakedValues)+" Blue");
                return SampleColor.BLUE;
            }
        }else{
            return SampleColor.NONE;
        }
        System.out.println("Possible intake hang");
        return SampleColor.NONE;
    }

    public double getIntakeSpeed() {
        return intakeSpeed;
    }
}
