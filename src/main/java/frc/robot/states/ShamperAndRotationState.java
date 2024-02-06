package frc.robot.states;

public class ShamperAndRotationState {
    private double shamperAngle;
    private double robotAngle;

    public ShamperAndRotationState(double shamperAngle, double robotAngle) {
        this.shamperAngle = shamperAngle;
        this.robotAngle = robotAngle;
    }

    public double getShamperAngle(){
        return shamperAngle;
    }

    public double getRobotAngle(){
        return robotAngle;
    }
}
