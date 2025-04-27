package org.firstinspires.ftc.teamcode.Systems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Systems.Elevators.VerticalState;
import org.firstinspires.ftc.teamcode.Systems.Token.Token;
import org.firstinspires.ftc.teamcode.Systems.Token.TokenParallelAction;
import org.firstinspires.ftc.teamcode.Systems.Token.TokenSequentialAction;


public class ActionControl {
    Elevators elevators;
    DifferentialClaws claws;

    ColorSensorSystem colorSensorSystem;
    MecanumDrive mecanumDrive;
    GamepadEx gamepad1, gamepad2;

    Sweeper sweeper;

    public ActionControl(Elevators elevators, DifferentialClaws claws, ColorSensorSystem colorSensorSystem,
                         MecanumDrive drive, Sweeper sweeper, GamepadEx gamepad1, GamepadEx gamepad2) {
        this.elevators = elevators;
        this.claws = claws;
        this.colorSensorSystem = colorSensorSystem;
        this.mecanumDrive = drive;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.sweeper = sweeper;
        //TODO: ADD CLAW MOVEMENTS TO THESE
    }

    public Action returnWithDrive(TokenSequentialAction tokenAction, Token stopToken){
        return new ParallelAction(mecanumDrive.getMecanumDriveAction(gamepad1, gamepad2, sweeper, tokenAction, stopToken)
                , tokenAction);
    }

    public Action hangSpecimenHigh() {
        Token stopToken = new Token();
        return returnWithDrive(new TokenSequentialAction(
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 750),
                elevators.setVerticalElevatorAction(VerticalState.VERTICAL_SPECIMEN_HIGH),
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.HANG_SPECIMEN.state, 750),
                elevators.setVerticalElevatorAction(VerticalState.VERTICAL_MIN)
                )
                , stopToken);
    }

    public Action hangHighRaise() {
        Token stopToken = new Token();
        return returnWithDrive(new TokenSequentialAction(
                        claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 750, stopToken),
                        elevators.setVerticalElevatorAction(VerticalState.VERTICAL_SPECIMEN_HIGH, stopToken),
                        claws.clawMovementAction(DifferentialClaws.ClawPositionState.HANG_SPECIMEN.state, 750, stopToken),
                        elevators.setVerticalElevatorAction(Elevators.VerticalState.VERTICAL_SPECIMEN_PICKUP_AUTO, stopToken)
                )
                , stopToken);
    }

    public Action dropHigh() {
        Token stopToken = new Token();
        return returnWithDrive(new TokenSequentialAction(
                new TokenParallelAction(
                    elevators.setHorizontalElevatorAction(Elevators.HorizontalState.HORIZONTAL_RETRACTED),
                    claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 1000, stopToken),
                    elevators.setVerticalElevatorAction(VerticalState.VERTICAL_HIGH, stopToken)
                ),
                    claws.clawMovementAction(DifferentialClaws.ClawPositionState.SPIT_STATE.state, 750, stopToken),
                    claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.SPIT,colorSensorSystem, stopToken),
                    new TokenParallelAction(
                        claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 1000, stopToken),
                        elevators.setVerticalElevatorAction(VerticalState.VERTICAL_MIN, stopToken)
                    )
                )
                , stopToken);
    }

    public Action spitWrong(){
        Token stopToken = new Token();
        return returnWithDrive(
                new TokenSequentialAction(
                        claws.clawMovementAction(DifferentialClaws.ClawPositionState.TAKE_SPECIMEN.state, 750),
                        claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.SPIT_HARD, colorSensorSystem)
                ), stopToken
        );
    }

    public Action dropHighAndToPlace() {
        Token stopToken = new Token();

        return new TokenSequentialAction(
                mecanumDrive.getDriveUntilStopAction(colorSensorSystem, stopToken),
                elevators.setVerticalElevatorAction(VerticalState.VERTICAL_HIGH, stopToken),
                claws.clawMovementAction(DifferentialClaws.ClawPositionState.SPIT_STATE.state, 750, stopToken),
                claws.setClawSampleInteractionAction(DifferentialClaws.ClawPowerState.SPIT,colorSensorSystem, stopToken),
                new TokenParallelAction(
                        claws.clawMovementAction(DifferentialClaws.ClawPositionState.MAX.state, 1000, stopToken),
                        elevators.setVerticalElevatorAction(VerticalState.VERTICAL_MIN, stopToken)
                )
        );
    }

    public Action splineToDropLine(){
        Pose2d start = new Pose2d(-24,-11.7, 0);
        Pose2d midPoint = new Pose2d(-40, -11.2, 0);
        Pose2d end = new Pose2d(-59.2, -11.2, (1.5)*Math.PI);
        mecanumDrive.pose = start;

        return mecanumDrive.actionBuilder(start)
                .setTangent(Math.PI)
                .splineToSplineHeading(midPoint, Math.PI)
                .splineToSplineHeading(end, Math.PI)
                .build();
    }

}
