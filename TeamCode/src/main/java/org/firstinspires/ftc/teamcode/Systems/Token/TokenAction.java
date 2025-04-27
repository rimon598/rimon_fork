package org.firstinspires.ftc.teamcode.Systems.Token;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.Action;

public abstract class TokenAction implements Action {
    protected Tokenable isDone;
    protected boolean isInitialized = false;
    public boolean checkToken(){
        return isDone.invoke() && isInitialized;
    }

    @Override
    public void preview(@NonNull Canvas fieldOverlay) {
        Action.super.preview(fieldOverlay);
    }
}
