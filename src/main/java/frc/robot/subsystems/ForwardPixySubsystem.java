package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.io.pixy.Pixy2;
import frc.robot.util.io.pixy.Pixy2CCC.Block;
import frc.robot.util.io.pixy.links.SPILink;

public class ForwardPixySubsystem extends SubsystemBase{
	private final Pixy2 pixy;

    public ForwardPixySubsystem(){
        pixy = Pixy2.createInstance(new SPILink()); // Creates a new Pixy2 camera using SPILink
		pixy.init(); // Initializes the camera and prepares to send/receive data
    }

    @Override
    public void periodic() {
        //findBlocks();
    }

    public void findBlocks(){
        pixy.getCCC().getBlocks();
        ArrayList<Block> blocks = pixy.getCCC().getBlockCache();

        System.out.println(blocks.size());

        for (Block block : blocks){
            if (block.getSignature() == 1){
                System.out.println("Note Position x = " + block.getX() + " y = " + block.getY());
            }
            // if (block.getSignature() == 2){
            //     System.out.println("Cube Position x = " + block.getX() + " y = " + block.getY());
            // }
            if (block.getSignature() == 3){
                System.out.println("White Line Position x = " + block.getX() + " y = " + block.getY());
            }
        }
    }

    public Block findCentermostBlock(){
        List<Block> blocks = new ArrayList<Block>();

        try {
            pixy.getCCC().getBlocks();
            blocks = pixy.getCCC().getBlockCache();
        } catch(Exception e) {
            DriverStation.reportError(e.getMessage(), e.getStackTrace());
        }

        Block centerBlock = null;
        for (Block block : blocks){
            if (block.getSignature() == 1) {
                if (centerBlock == null) {
                    centerBlock = block;
                    continue;
                }
                if (Math.abs(xOffsetFromCenter(block)) < 39) {
                    // if (Math.abs(xOffsetFromCenter(block)) < Math.abs(xOffsetFromCenter(centerBlock))) {
                    //     centerBlock = block;
                    // }
                    if (yOffsetFromTop(block) > yOffsetFromTop(centerBlock)) {
                        centerBlock = block;
                    }
                }
            }
        }

        if (centerBlock != null) {
            System.out.println(xOffsetFromCenter(centerBlock));
        }

        return centerBlock;
    }

    public double xOffsetFromCenter(Block block){
        //pixy cam pixel res width is 316, midpoint is 158
        if (block != null) {
            return block.getX() - 158;
        }

        return 0;
    }

    public double yOffsetFromTop(Block block){
        //pixy cam pixel res width is 316, midpoint is 158
        if (block != null) {
            return block.getY();
        }

        return 0;
    }

    public Block getMiddleLine(){
        ArrayList<Block> blocks = pixy.getCCC().getBlockCache();

        Block middleLineBlock = null;
        for (Block block : blocks){
            // Pixy cam signature for the white line is 3.
            if (block.getSignature() == 3){
                if (middleLineBlock == null) {
                    middleLineBlock = block;
                    continue;
                }

                // Get the lowest block with a signature of 3.
                if (block.getY() > middleLineBlock.getY()) {
                    middleLineBlock = block;
                }
            }
        }
        return middleLineBlock;
    }
}
