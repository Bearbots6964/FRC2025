package frc.robot.motionInputs;

import edu.wpi.first.math.geometry.Rotation2d;
import lombok.Getter;

public class MotionInput {
    private double rotation;

    //number of 'frames' the buffer hold an input
    @Getter
    private int life = MotionInputs.maxBufferLength;

    public MotionInput(Rotation2d rotation) {
        if (rotation.getCos() == 0.0 && rotation.getSin() == 0.0) this.rotation = -1;
        else this.rotation = rotation.getDegrees();
    }

    public int getNumpadNotation() {
        if(rotation >= 202.5 && rotation < 247.5) return 1;
        if(rotation >= 247.5 && rotation < 292.5) return 2;
        if(rotation >= 292.5 && rotation < 337.5) return 3;
        if(rotation >= 337.5 && rotation < 360 || rotation >= 0 && rotation < 22.5) return 6;
        if(rotation >= 22.5 && rotation < 67.5) return 9;
        if(rotation  >= 67.5 && rotation < 112.5) return 8;
        if(rotation >= 112.5 && rotation < 157.5) return 7;
        if(rotation >= 157.5 && rotation < 202.5) return 4;
        return 5;
    }

    public void update() {
        life--;
    }
}
