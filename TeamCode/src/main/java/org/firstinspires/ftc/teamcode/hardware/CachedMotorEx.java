package org.firstinspires.ftc.teamcode.hardware;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class CachedMotorEx extends MotorEx {
    double prevPower=0;

    public CachedMotorEx(@NonNull HardwareMap hMap, String id) {
        super(hMap, id);
    }

    public CachedMotorEx(@NonNull HardwareMap hMap, String id, @NonNull GoBILDA gobildaType) {
        super(hMap, id, gobildaType);
    }

    public CachedMotorEx(@NonNull HardwareMap hMap, String id, double cpr, double rpm) {
        super(hMap, id, cpr, rpm);
    }
    public void setPower(double p){

        if (p==prevPower){
            return;
        }
        if (p==0){
            super.set(0);
            this.prevPower=p;
        }else if (Math.abs(p-prevPower)>0.01){
            super.set(p);
            this.prevPower=p;
        }
    }
    public double getCurrent(){
        return super.motorEx.getCurrent(CurrentUnit.AMPS);
    }
    public double getPrevPower(){
        return prevPower;
    }
}