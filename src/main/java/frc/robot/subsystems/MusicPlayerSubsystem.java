package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MusicPlayerSubsystem extends SubsystemBase{

    private final Orchestra musicPlayer;
    private final TalonFX[] instramentList;

    public MusicPlayerSubsystem() {
        musicPlayer = new Orchestra();
        instramentList = Constants.MusicPlayer.INSTRAMENT_TALONFX_PORT_LIST;

        for (int i = 0; i < instramentList.length; i++) {
            musicPlayer.addInstrument(instramentList[i]);
        }
    }

    public void loadMusic(String filepath) {
        musicPlayer.loadMusic(filepath);
    }

    public void playLoadedTrack() {
        musicPlayer.play();
    }

    public void pauseMusic() {
        musicPlayer.pause();
    }

    public void stopMusic() {
        musicPlayer.stop();
    }

    public int getCurrentPlayTime() {
        return musicPlayer.getCurrentTime();
    }

    public boolean isPlayerPLaying() {
        return musicPlayer.isPlaying();
    }
}
