package org.firstinspires.ftc.teamcode.Systems.Token;

public class Token {
    private boolean isInterrupted = false;

    public boolean checkInterruption(){
        //return true if interapted
        return  isInterrupted;
    }

    public void Interrupt(){
        isInterrupted = true;
    }
}
