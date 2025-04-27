package org.firstinspires.ftc.teamcode.Systems;

import static java.lang.Math.abs;

import android.widget.GridLayout;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorSensorSystem {

    LynxI2cColorRangeSensor colorSensor;
    public ColorSensor leftSensor;
    public ColorSensor rightSensor;

    private SpecimenColor currentSpecimentColor;
    boolean isTeamBlue;

    Gamepad gamepad1;

    Gamepad gamepad2;

    Gamepad.RumbleEffect rumbleEffectGood, rumbleEffectBad;

    private SpecimenColor lastSpecimenColor = SpecimenColor.NO_COLOR_DETECTED;

    public enum SpecimenColor{
        RED, BLUE, YELLOW, NO_COLOR_DETECTED;
        public static SpecimenColor getColor (double hue){
            int red = 0, yellow = 40, blue = 200;
            final double epsilon = 19;
            if (hue < red+epsilon) {
                return SpecimenColor.RED;
            }
            else if (blue-epsilon < hue && hue < blue+epsilon) {
                return SpecimenColor.BLUE;
            }
            else if (yellow-epsilon < hue && hue < yellow+epsilon) {
                return SpecimenColor.YELLOW;
            }
            else return NO_COLOR_DETECTED;
        }
    }
    public ColorSensorSystem(OpMode opMode, boolean isBlueTeam){
        colorSensor = opMode.hardwareMap.get(LynxI2cColorRangeSensor.class, "clawSensor");
        rightSensor = opMode.hardwareMap.get(ColorSensor.class, "rightColorSensor");
        leftSensor = opMode.hardwareMap.get(ColorSensor.class, "leftColorSensor");
        isTeamBlue = isBlueTeam;
        gamepad1 = opMode.gamepad1;
        gamepad2 = opMode.gamepad2;
        colorSensor.enableLed(true);
        rightSensor.enableLed(true);
        leftSensor.enableLed(true);
        initializeRamble();
    }

    public void initializeRamble(){
        final int rightColorFreq = 210, badColorFreq = 750;

        if (currentSpecimentColor == SpecimenColor.NO_COLOR_DETECTED){
            gamepad1.stopRumble();
            gamepad2.stopRumble();
            return;
        }

        int freqTime = 0;
        int timeWait = 0;
        double power = 0;
        freqTime = rightColorFreq; //right specimen
        timeWait = 300;
        power = 1;
        rumbleEffectGood = new Gamepad.RumbleEffect.Builder()
                .addStep(power, power, freqTime)  //  Rumble motors 100% for freqTime mSec
                .addStep(0.0, 0.0, timeWait)  //  Pause for freqTime/2 mSec
                .addStep(power, power, freqTime)  //  Rumble motors 100% for freqTime mSec
                .addStep(0.0, 0.0, timeWait)  //  Pause for freqTime/2 mSec
                .addStep(power, power, freqTime)  //  Rumble motors 100% for freqTime mSec
                .addStep(0.0, 0.0, timeWait)  //  Pause for freqTime/2 mSec
                .build();

      //not good specimen
        freqTime = badColorFreq;
        timeWait = 510;
        power = 0.9;
        rumbleEffectBad = new Gamepad.RumbleEffect.Builder()
                .addStep(power, power, freqTime)  //  Rumble motors 100% for freqTime mSec
                .addStep(0.0, 0.0, timeWait)  //  Pause for freqTime/2 mSec
                .addStep(power, power, freqTime)  //  Rumble motors 100% for freqTime mSec
                .addStep(0.0, 0.0, timeWait)  //  Pause for freqTime/2 mSec
                .addStep(power, power, freqTime)  //  Rumble motors 100% for freqTime mSec
                .addStep(0.0, 0.0, timeWait)  //  Pause for freqTime/2 mSec
                .build();



    }

    public boolean isSpecimenIn(){
        double minValue = 5.5; //cm
        return colorSensor.getDistance(DistanceUnit.CM) <= minValue;
    }

    public double getDistance(DistanceUnit unit){
        return  colorSensor.getDistance(unit);
    }
    public double getHue(NormalizedRGBA normalzie) {
        double red = normalzie.red;
        double green = normalzie.green;
        double blue = normalzie.blue;
        double maxColor = Math.max(Math.max(red, green), blue);
        double minColor = Math.min(Math.min(red, green), blue);
        double hue = 0;
        if (maxColor == red){
            hue = (green-blue)/(maxColor-minColor);
        }
        else if (maxColor == green) {
            hue = 2.0 + (blue - green) / (maxColor - minColor);
        }
        else if (maxColor == blue){
            hue = 4.0 + (red-green)/(maxColor-minColor);
        }
        hue *= 60;
        if (hue < 0) hue+=360;

        return hue;
    }
    public SpecimenColor getSpecimenColor(){
        double hue = getHue(colorSensor.getNormalizedColors());
        if (isSpecimenIn())
            return SpecimenColor.getColor(hue);
        else
            return SpecimenColor.NO_COLOR_DETECTED;
    }

    public boolean isOnTape(boolean isRight){
        int eps = 700;
        int redConst = 1420;
        int blueConst = 2960;
        int whiteGreenConst = 6600;

        ColorSensor currSensor = isRight ? rightSensor : leftSensor;

        if (isTeamBlue) {
            return Math.abs(currSensor.blue() - blueConst) <= eps && Math.abs(currSensor.green() - whiteGreenConst) > eps;
        }
        else {
            return Math.abs(currSensor.red() - redConst) <= eps && Math.abs(currSensor.green() - whiteGreenConst) > eps;
        }
    }

    public boolean myTeamSpecimen(SpecimenColor specimenColor){
        return (specimenColor == SpecimenColor.BLUE && isTeamBlue) || (specimenColor == SpecimenColor.RED && !isTeamBlue)
                || (specimenColor == SpecimenColor.YELLOW);
    }
    public void alertToGamePads(){
         //milliseconds
        if(currentSpecimentColor == SpecimenColor.NO_COLOR_DETECTED){
            gamepad1.stopRumble();
            gamepad2.stopRumble();
            return;
        }


        if(myTeamSpecimen(currentSpecimentColor)) {
            gamepad1.runRumbleEffect(rumbleEffectGood);
            gamepad2.runRumbleEffect(rumbleEffectGood);
        }
        else {
            gamepad1.runRumbleEffect(rumbleEffectBad);
            gamepad2.runRumbleEffect(rumbleEffectBad);
        }

//        if(!gamepad1.isRumbling()) {
//            rumbleEffect = new Gamepad.RumbleEffect.Builder()
//                    .addStep(power, power, freqTime)  //  Rumble motors 100% for freqTime mSec
//                    .addStep(0.0, 0.0, timeWait)  //  Pause for freqTime/2 mSec
//                    .addStep(power, power, freqTime)  //  Rumble motors 100% for freqTime mSec
//                    .addStep(0.0, 0.0, timeWait)  //  Pause for freqTime/2 mSec
//                    .addStep(power, power, freqTime)  //  Rumble motors 100% for freqTime mSec
//                    .addStep(0.0, 0.0, timeWait)  //  Pause for freqTime/2 mSec
//                    .build();
//
//            gamepad1.runRumbleEffect(rumbleEffect);
//            gamepad2.runRumbleEffect(rumbleEffect);
//        }
    }

    public boolean updateAlert(){
        currentSpecimentColor = getSpecimenColor();
        if(lastSpecimenColor != currentSpecimentColor || !myTeamSpecimen(currentSpecimentColor)){
            alertToGamePads();
        }

        lastSpecimenColor = currentSpecimentColor;
        return myTeamSpecimen(currentSpecimentColor) || currentSpecimentColor == SpecimenColor.NO_COLOR_DETECTED;
    }
}
