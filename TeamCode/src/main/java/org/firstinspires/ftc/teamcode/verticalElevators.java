package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class verticalElevators{
   private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;

    public verticalElevators(OpMode Op){
        leftMotor = Op.hardwareMap.get(DcMotorEx.class,"leftMotor");
        rightMotor = Op.hardwareMap.get(DcMotorEx.class,"rightMotor");

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    public enum verticalPos{
        MIN(0),
        LOW(2),
        HIGH(4);

        public final int pos;

        verticalPos(int pos){
            this.pos=pos;

        }
    }
    public void setPos(verticalPos destination){
        leftMotor.setTargetPosition(destination.pos);
        rightMotor.setTargetPosition(destination.pos);
    }
    public void setPower(double power){
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }
    public boolean areMotorsInDestination(){
        return  (Math.abs(leftMotor.getCurrentPosition()-leftMotor.getTargetPosition())<leftMotor.getTargetPositionTolerance() &&
                Math.abs(rightMotor.getCurrentPosition()-rightMotor.getTargetPosition())<rightMotor.getTargetPositionTolerance());
    }
    public class verticalAction implements Action{
        private verticalPos destination;
        private boolean isInit;

        public verticalAction(verticalPos pos){
            this.destination = pos;

        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (isInit==false){
                setPos(destination);
                isInit=true;
            }

            return !areMotorsInDestination();
        }
    }
}
