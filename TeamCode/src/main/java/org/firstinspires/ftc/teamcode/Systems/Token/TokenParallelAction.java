package org.firstinspires.ftc.teamcode.Systems.Token;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;

public class TokenParallelAction extends TokenAction{

    private ParallelAction parallelAction;

    private TokenAction[] actions;

    public TokenParallelAction(TokenAction... actions){
        parallelAction = new ParallelAction(actions);
        this.actions = actions;
        isDone = this::isAllDone;
    }

    public boolean isAllDone(){
        boolean isAllDone = true;
        for(int i = 0; i< actions.length; i++){
            isAllDone = isAllDone && actions[i].checkToken();
        }
        return isAllDone;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if(!isInitialized)
            parallelAction.run(telemetryPacket);
        isInitialized = true;

        return parallelAction.run(telemetryPacket);
    }
}
