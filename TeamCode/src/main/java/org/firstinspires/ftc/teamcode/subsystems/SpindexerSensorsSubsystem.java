package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.RobotHardwareMap;
import org.firstinspires.ftc.teamcode.util.rConstants;

public class SpindexerSensorsSubsystem {
    private final RobotHardwareMap robot;

    private final boolean[] slotOccupied = new boolean[3];

    public SpindexerSensorsSubsystem(RobotHardwareMap robotHardwareMap){
        this.robot = robotHardwareMap;
    }

    public void readSensors(){
        int frontSlot = getCurrentFrontSlot();

        double leftReading = robot.leftDistanceSensor.getDistance(DistanceUnit.CM);
        double rightReading = robot.rightDistanceSensor.getDistance(DistanceUnit.CM);

        slotOccupied[frontSlot] = leftReading < rConstants.SensorConstants.distanceSensorOccupiedThreshold
                || rightReading < rConstants.SensorConstants.distanceSensorOccupiedThreshold;

        // Other two slots are not updated — they retain whatever state
        // was set when they were last in front of the distance sensors.
    }

    // Helper Functions
    public boolean spindexerIsFull() { return slotOccupied[0] && slotOccupied[1] && slotOccupied[2]; }
    public boolean spindexerIsEmpty() { return !slotOccupied[0] && !slotOccupied[1] && !slotOccupied[2]; }
    public boolean isSlotOccupied(int slot) { return slotOccupied[slot]; }

    public int getBallCount(){
        int ballCount = 0;
        for (boolean occupied : slotOccupied){
            if(occupied) ballCount++;
        }
        return ballCount;
    }

    public int getFirstEmptySlot() {
        for (int i = 0; i < slotOccupied.length; i++) {
            if (!slotOccupied[i]) return i;
        }
        return -1;
    }

    public int getCurrentFrontSlot(){
        double currentPosition = robot.spindexerEncoder.getCurrentPosition();
        double closestDistance = Double.MAX_VALUE;
        int closestSlot = 0;

        for(int i = 0; i < rConstants.SpindexerConstants.encoderShootingPositions.length; i++){
            double distance = Math.abs(currentPosition - rConstants.SpindexerConstants.encoderShootingPositions[i]);
            if(distance < closestDistance){
                closestDistance = distance;
                closestSlot = i;
            }
        }
        return closestSlot;
    }
}