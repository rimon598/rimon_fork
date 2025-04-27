package org.firstinspires.ftc.teamcode.Systems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Systems.Token.Token;
import org.firstinspires.ftc.teamcode.Systems.Token.TokenAction;
import org.firstinspires.ftc.teamcode.Systems.Token.Tokenable;

public class Elevators implements Tokenable {
    final int epsilon = 100;


    DcMotorEx rightVert;
    DcMotorEx leftVert;
    static Servo rightHor;
    static Servo leftHor;

    private static Elevators instance = null;

    int verticalDestination;

    // sets the vertical elevator to the specified position
    public class VerticalElevatorAction extends TokenAction {
        private final int destination;
        Token token;

        public VerticalElevatorAction(int destination) {
            this.destination = destination;
            isDone = Elevators.this::isElevatorInDestination;
        }

        public VerticalElevatorAction(int destination, Token token) {
            this(destination);
            this.token = token;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(token != null && token.checkInterruption())
                return false;

            if (!isInitialized) {
                setVerticalDestination(this.destination);
                isInitialized = true;
            }

            updateVert();
            
            return !isElevatorInDestination(); //|| System.currentTimeMillis() - TEST_START_TIME < TEST_LONGEST_TIME;
        }
    }


    /*
    -------------------------------------------------------------------
    | stepSize and tolerance need to be tuned so it feels good to use |
    -------------------------------------------------------------------
     */
    // moves the horizontal elevators to destination, and is considered finished when they reach the destination
    public class HorizontalElevatorAction extends TokenAction {
        private boolean hasEnoughTimePassed() {
            return System.currentTimeMillis() - startTime >= timeUntilDone;
        }
        private final double desitnation;
        private final double timeUntilDone;
        private double startTime;
        private Token token;

        public HorizontalElevatorAction(HorizontalState state, double timeUntilDone) {
            isDone = HorizontalElevatorAction.this::hasEnoughTimePassed;
            desitnation = state.state;
            this.timeUntilDone = timeUntilDone;
            this.isInitialized = false;
        }

        public HorizontalElevatorAction(HorizontalState state, double timeUntilDone, Token token) {
            this(state, timeUntilDone);
            this.token = token;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(token != null && token.checkInterruption())
                return false;

            if (!isInitialized) {
                startTime = System.currentTimeMillis();
                setHorizontalDestination(desitnation);
                isInitialized = true;
            }

            return !hasEnoughTimePassed();
        }
    }



    // Vertical min is lowest possible, max is highest possible, low and high are terms for the baskets
    public enum VerticalState {
        VERTICAL_MIN(0),
        VERTICAL_PICKUP(0),
        VERTICAL_HURDLE(356),
        VERTICAL_SPECIMEN_PICKUP(775),
        VERTICAL_SPECIMEN_PICKUP_AUTO(557),
        VERTICAL_SPECIMEN_HIGH(1670),
        VERTICAL_LOW(2804),
        VERTICAL_HIGH(3650),
        VERTICAL_OPMODE_HIGH(3570),
        VERTICAL_MAX(4200); //4200/11448

        public final int state;

        VerticalState(int state) {
            this.state = state;
        }
    }

    public enum HorizontalState {
        HORIZONTAL_RETRACTED(0),
        HORIZONTAL_QUARTERWAY(0.16),
        HORIZONTAL_HALFWAY(0.32),
        HORIZONTAL_EXTENDED(0.52);

        public final double state;

        HorizontalState(double state) {
            this.state = state;
        }
    }

    private Elevators(OpMode opMode) {
        rightVert = opMode.hardwareMap.get(DcMotorEx.class, "rightVert");
        leftVert = opMode.hardwareMap.get(DcMotorEx.class, "leftVert");
        rightHor = opMode.hardwareMap.get(Servo.class, "rightHor");
        leftHor = opMode.hardwareMap.get(Servo.class, "leftHor");

        rightVert.setDirection(DcMotorSimple.Direction.REVERSE);
        leftVert.setDirection(DcMotorSimple.Direction.FORWARD);
        setVerticalDestination(VerticalState.VERTICAL_MIN.state);
        rightVert.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftVert.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        rightVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //setVertTolerances();

//        leftHor.setDirection(Servo.Direction.REVERSE);
//        rightHor.setDirection(Servo.Direction.FORWARD);

        setHorizontalCorrectDirection();
        setHorizontalDestination(HorizontalState.HORIZONTAL_RETRACTED);
//        setHoriozontalScales();


//        setHorizontalPosition(HorizontalState.HORIZONTAL_RETRACTED.state);


    }

    public static Elevators getInstance(OpMode opMode) {
//        if (instance == null) {
            instance = new Elevators(opMode);
//        }
//        else {
////            instance.setVertMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            instance.setVertMode(DcMotor.RunMode.RUN_TO_POSITION);
//            instance.resetDirections();
//            instance.setVerticalDestination(instance.getVerticalCurrentPosition());
//            //instance.setVertTolerances();
//
//            instance.setHorizontalCorrectDirection();
//            instance.setHorizontalDestination(HorizontalState.HORIZONTAL_RETRACTED);
////            instance.setHoriozontalScales();
//        }
        return instance;
    }

    public void setVertTolerances() {
        rightVert.setTargetPositionTolerance(4);
        leftVert.setTargetPositionTolerance(4);
    }

    public void setVertVelocities() {
        rightVert.setVelocity(1900);
        leftVert.setVelocity(1900);
    }

    public void setVertMode(DcMotor.RunMode runMode) {
        rightVert.setMode(runMode);
        leftVert.setMode(runMode);
    }

    public void resetDirections() {
        rightVert.setDirection(DcMotorSimple.Direction.REVERSE);
        leftVert.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setHorizontalCorrectDirection() {
        rightHor.setDirection(Servo.Direction.REVERSE);
        leftHor.setDirection(Servo.Direction.FORWARD);
    }
    public void setHoriozontalScales() {
        rightHor.scaleRange(0, 0.84);
        leftHor.scaleRange(0, 0.84);
    }


    public int getVerticalDestination() {
        return verticalDestination;
    }

    public int getVerticalCurrentPosition() {
        return rightVert.getCurrentPosition();
    }

    // sets the destination of the vertical motors to the specified number of ticks
    public void setVerticalDestination(int destination) {
        if (Math.abs(destination-getVerticalCurrentPosition())<=60 && destination == 0) {
            setVerticalPower(0);
        }
        else {
            setVerticalPower(1);
        }
        rightVert.setTargetPosition(destination);
        leftVert.setTargetPosition(destination);
        verticalDestination = destination;
    }

    public int getVertDestination(){
        return verticalDestination;
    }

    public void updateVert(){
        if (Math.abs(verticalDestination-getVerticalCurrentPosition())<=30 && verticalDestination == 0) {
            setVerticalPower(0);
//            double innerEps = 10;
//            if(Math.abs(verticalDestination-getVerticalCurrentPosition())>= innerEps) {
                rightVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }
        }
        else if(Math.abs(verticalDestination-getVerticalCurrentPosition())<=epsilon){
            setVerticalPower(0.8);
        }
        else {
            setVerticalPower(1);
        }
    }

    // checks whether the elevator is close enough (+- epsilon) to it's destination
    public boolean isElevatorInDestination() {
//        if (getVerticalCurrentPosition() < getVerticalDestination()) {
//            return getVerticalCurrentPosition() >= getVerticalDestination() - epsilon;
//        }
        return Math.abs(getVerticalCurrentPosition() - getVerticalDestination()) <= epsilon;
    }

    public void setVerticalPower(double power){
        rightVert.setPower(power);
        leftVert.setPower(power);
    }

    public double getLeftVertPos() {
        return leftVert.getCurrentPosition();
    }

    public double getRightVertPos() {
        return rightVert.getCurrentPosition();
    }

    // Does not work anymore since we have no motor
    @Deprecated
    public double getHorizontalPosition() {
        return (leftHor.getPosition() + rightHor.getPosition())/2.0;
    }

    public double getLeftHorPos() {
        return leftHor.getPosition();
    }

    public double getRightHorPos() {
        return rightHor.getPosition();
    }

    public double getHorizontalDestination() {
        return leftHor.getPosition();
    }

    public boolean motorIsHorizontalInDestination() {
        return Math.abs(getHorizontalPosition() - getHorizontalDestination()) < 20;
    }

    public void setHorizontalDestination(HorizontalState state) {
        setHorizontalDestination(state.state);
    }

    public void setHorizontalDestination(double destination) {
        leftHor.setPosition(destination);
        rightHor.setPosition(destination);
    }

    public VerticalElevatorAction setVerticalElevatorAction(VerticalState targetState) {
        return new VerticalElevatorAction(targetState.state);
    }
    public VerticalElevatorAction setVerticalElevatorAction(VerticalState targetState, Token token) {
        return new VerticalElevatorAction(targetState.state, token);
    }

    public void resetVert(){
        rightVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightVert.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftVert.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftVert.setPower(0);
        rightVert.setPower(0);
    }

    public HorizontalElevatorAction setHorizontalElevatorAction(HorizontalState destinationState) {
        return new HorizontalElevatorAction(destinationState, 0.0);
    }

    public HorizontalElevatorAction setHorizontalElevatorAction(HorizontalState destinationState, double timeUntilDone) {
        return new HorizontalElevatorAction(destinationState, timeUntilDone);
    }
    public void setVertDest(int dest){
        rightVert.setTargetPosition(dest);
        leftVert.setTargetPosition(dest);
        rightVert.setPower(0.65);
        leftVert.setPower(0.65);
    }

    public double getLeftVelocity(){
        return leftVert.getVelocity();
    }

    public double getRightVelocity(){
        return rightVert.getVelocity();
    }

    @Override
    public Boolean invoke() {
        return isElevatorInDestination();
    }
}
