package org.firstinspires.ftc.teamcode.Systems.Token;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;

import org.firstinspires.ftc.teamcode.Systems.Elevators;

import kotlin.jvm.functions.Function0;

public class TokenSequentialAction implements Action, Tokenable{

    private SequentialAction sequentialAction;
    private int indexFinished = 0;

    private TokenAction[] actions;

    public TokenSequentialAction(TokenAction... actions){
        sequentialAction = new SequentialAction(actions);
        this.actions = actions;
    }

    public boolean isAllDone(){
        assert indexFinished < actions.length;
        if(actions[indexFinished].checkToken())
            indexFinished++;

        if(indexFinished == actions.length){
            indexFinished = 0;
            return true;
        }
        else
            return false;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        return sequentialAction.run(telemetryPacket);
    }

    @Override
    public Boolean invoke() {
        return isAllDone();
    }
}
