package org.firstinspires.ftc.teamcode.Systems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class Robot {
    private MecanumDrive drive;
    private DifferentialClaws claws;
    private Elevators elevators;
    private ColorSensorSystem colorSensorSystem;
    private Sweeper sweeper;
    private boolean isTeamBlue;

    private boolean takenEnemySample = false;
    private double targetArmPosition = 0;

    private boolean isPIDFActive = false;

    public Robot(OpMode opMode, boolean isTeamBlue) {
        this.isTeamBlue = isTeamBlue;
        drive = new MecanumDrive(opMode.hardwareMap, new Pose2d(0, 0, 0));
        claws = DifferentialClaws.getInstance(opMode);
        elevators = Elevators.getInstance(opMode);
        colorSensorSystem = new ColorSensorSystem(opMode, isTeamBlue);
        sweeper = new Sweeper(opMode);
    }

    // must be called at the start of every iteration
    public void update() {
        claws.updateLeftClawServoRotation();
        claws.updateRightClawServoRotation();

        takenEnemySample = colorSensorSystem.updateAlert();

        if (isPIDFActive) {
            claws.setArmTargetPosition(targetArmPosition);
            claws.rotateArm(claws.getPIDArmPower());
        }

        elevators.updateVert();
    }


    public Elevators getElevators() {
        return elevators;
    }
    public MecanumDrive getDrive() {
        return drive;
    }
    public DifferentialClaws getClaws() {
        return claws;
    }
    public ColorSensorSystem getColorSensorSystem() {
        return colorSensorSystem;
    }
    public Sweeper getSweeper() {
        return sweeper;
    }
    public boolean isTeamBlue() {
        return isTeamBlue;
    }

    public static double linearToExpo(double input) {
        return (input >= 0) ? input*input : -input*input;
    }

    // leftX and leftY control the robots movement, rightX controls it's rotation, and slowAmount slows everything down (all must be between 0-1)
    public void setOpModeDrivePowers(double leftX, double leftY, double rightX, double slowAmount) {
        //driving
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        linearToExpo(leftY)*(1.0/Math.pow(4.5, slowAmount)),
                        leftX*(1.0/Math.pow(4, slowAmount))
                ),
                rightX*(1.0/Math.pow(5, slowAmount))
        ));

        drive.updatePoseEstimate();
    }

    public void setVerticalElevatorsHeight(int height) {
        elevators.setVerticalDestination((height > Elevators.VerticalState.VERTICAL_MIN.state && height < Elevators.VerticalState.VERTICAL_MAX.state ? height : (height > Elevators.VerticalState.VERTICAL_MIN.state ? Elevators.VerticalState.VERTICAL_MAX.state : Elevators.VerticalState.VERTICAL_MIN.state)));
    }

    public void setVerticalElevatorsHeight(Elevators.VerticalState height) {
        elevators.setVerticalDestination(height.state);
    }

    public void setHorizontalElevatorsPosition(double position) {
        elevators.setHorizontalDestination(position);
    }

    public void setSweeperPosition(Sweeper.SweeperAngle angle) {
        sweeper.setPosition(angle);
    }

    public boolean isSampleIn() {
        return colorSensorSystem.isSpecimenIn();
    }

    public boolean isMySampleIn() {
        return !takenEnemySample;
    }

    // takes in a sample with the given power
    public void startTakingSampleIn(double power) {
        claws.rotateWheels(Math.abs(power));
        isPIDFActive = false;
    }

    public void startTakingSampleIn() {
        claws.rotateWheels(DifferentialClaws.ClawPowerState.TAKE_IN);
        isPIDFActive = false;
    }

    // spits a sample with the given power
    public void startSpittingSample(double power) {
        claws.rotateWheels(-Math.abs(power));
        isPIDFActive = false;
    }

    public void startSpittingSample() {
        claws.rotateWheels(DifferentialClaws.ClawPowerState.SPIT);
        isPIDFActive = false;
    }

    public void stopSampleInteraction() {
        claws.rotateWheels(0);
        isPIDFActive = true;
    }

    public void setClawPosition(double position) {
        if (position < DifferentialClaws.ClawPositionState.MIN.state) {
            position = DifferentialClaws.ClawPositionState.MIN.state;
        }
        else if (position > DifferentialClaws.ClawPositionState.MAX.state) {
            position = DifferentialClaws.ClawPositionState.MAX.state;
        }
        targetArmPosition = position;
        isPIDFActive = true;
    }

    public void setClawPosition(DifferentialClaws.ClawPositionState position) {
        targetArmPosition = position.state;
        isPIDFActive = true;
    }
}
