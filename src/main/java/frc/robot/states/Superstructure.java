package frc.robot.states;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.shamper.ShamperAngleCommand;
import frc.robot.commands.shamper.ShamperShootCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShamperSubsystem;
import frc.robot.subsystems.ShamperSubsystem.ShamperSpeeds;
import frc.robot.util.AimingCalculator;

public class Superstructure extends SubsystemBase {
  private SuperstructureState state;
  
  private ShamperSubsystem shamper;

  private ClimberSubsystem climber;

  private IndexerSubsystem indexer;

  public  Superstructure(ShamperSubsystem shamper, ClimberSubsystem climber, IndexerSubsystem indexer) {
    this.shamper = shamper;
    this.climber = climber;
    this.indexer = indexer;
  }

  public void setState(SuperstructureState state) {
    this.state = state;
    switch(state){
      case DEFAULT:
        setDefault();
      case SPEAKER_IDLE:
        setSpeakerIdle();
      case SPEAKER_SCORE:
        setSpeakerScoring();
      case AMP_IDLE:
        setAmpIdle();
      case AMP_SCORE:
        setAmpScore();
      case CLIMBING:
        setClimbing();
    }
  }

  public SuperstructureState getState() {
    return state;
  }

  private void setDefault() {
    new ShamperAngleCommand(shamper, Constants.Shamper.Angle.DEFAULT);
    new ShamperShootCommand(shamper, indexer, ShamperSpeeds.SPEAKER_IDLE);
  }

  private void setSpeakerIdle() {
    new ShamperAngleCommand(shamper, Constants.Shamper.Angle.DEFAULT);
    new ShamperShootCommand(shamper, indexer, ShamperSpeeds.SPEAKER_IDLE);
  }
  
  private void setSpeakerScoring() {
    new ShamperAngleCommand(shamper, AimingCalculator.Calculation().getShamperAngle());
    new ShamperShootCommand(shamper, indexer, ShamperSpeeds.SPEAKER_IDLE);
  }  
  
  private void setAmpIdle() {
    new ShamperAngleCommand(shamper, Constants.Shamper.Angle.AMP);
    new ShamperShootCommand(shamper, indexer, ShamperSpeeds.AMP_IDLE);
  }  

  private void setAmpScore() {
    new ShamperAngleCommand(shamper, Constants.Shamper.Angle.AMP);
    new ShamperShootCommand(shamper, indexer, ShamperSpeeds.AMP);
  }

  private void setClimbing() {
    new ShamperAngleCommand(shamper, Constants.Shamper.Angle.CLIMB);
    new ShamperShootCommand(shamper, indexer, ShamperSpeeds.OFF);
  }

  @Override
  public void periodic() {
    if(state.needsRefresh()){
      setState(state);
    }
  }

  public enum SuperstructureState {
    DEFAULT(false),
    SPEAKER_IDLE(false),
    SPEAKER_SCORE(true),
    AMP_IDLE(false),
    AMP_SCORE(false),
    CLIMBING(false);

    private boolean refreshRequired;

    private SuperstructureState(boolean refreshRequired) {
      this.refreshRequired = refreshRequired;
    }

    public boolean needsRefresh() {
      return refreshRequired;
    }
  }
}
