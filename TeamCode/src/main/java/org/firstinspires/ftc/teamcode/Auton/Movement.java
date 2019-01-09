package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Movement extends DeclarationsAutonomous {

    public void craterSideSample(){
        /*//should come immediately after unlatching
        //for the most part this should be able to be copy/pasted to the depotSideSample, though a few changes
        //for the team marker may have to be made.
        gyroTurn(turningSpeed, 0);
        encoderDrive(.35, 2, 1, stayOnHeading, 2);
        while(goldPosition == 0 && getHeading() < 45 &&opModeIsActive()){
            pivot(.425, 1);
            getGoldPositionTwoMineral();
            unextendHangSlide();
        }
        if(goldPosition == 0){
            //failsafe, so that if this doesn't detect the right two minerals at least we'll still place
            // the team marker and park
            goldPosition = 1;
        }
        gyroTurn(turningSpeed, decideFirstSampleheading());
        if(goldPosition == 2){
            encoderDrive(.35, 12, forward, stayOnHeading, 5);
        }else{
            encoderDrive(.35, 24, forward, stayOnHeading, 5);
        }
*/
        ElapsedTime elapsedTime = new ElapsedTime();
//should come immediately after unlatching
        //for the most part this should be able to be copy/pasted to the depotSideSample, though a few changes
        //for the team marker may have to be made.
        gyroTurn(turningSpeed, 0);
        encoderDrive(.35, 2, 1, stayOnHeading, 2, true);
        gyroTurn(turningSpeed, 15);
        while(goldPosition == 0 && elapsedTime.seconds() < 3 && opModeIsActive()){
            getGoldPositionOneMineral();
            scoringSystems.unextendHangSlide(true);
        }
        if(goldPosition == 0){
            //failsafe, so that if this doesn't detect the right two minerals at least we'll still place
            // the team marker and park
            goldPosition = 1;
            //Position 1 is the leftmost mineral
        }
        gyroTurn(turningSpeed, decideFirstSampleheading());
        scoringSystems.putArmDown();
        scoringSystems.setIntakePower(.7);
        sleep(250);
        if(goldPosition == 2){
            encoderDrive(.25, 3, forward, stayOnHeading, 2, false);
        }else{
            encoderDrive(.25, 6, forward, stayOnHeading, 2, false);
        }
        if(goldPosition == 2){
            encoderDrive(.25, 4, forward, stayOnHeading, 2, true);
        }else{
            encoderDrive(.25, 5, forward, stayOnHeading, 2, true);
        }
        scoringSystems.setIntakePower(0);
    }
    public void depotSideSample(){
        ElapsedTime elapsedTime = new ElapsedTime();
//should come immediately after unlatching
        //for the most part this should be able to be copy/pasted to the depotSideSample, though a few changes
        //for the team marker may have to be made.
        gyroTurn(turningSpeed, 0);
        encoderDrive(.35, 2, 1, stayOnHeading, 2, true);
        gyroTurn(turningSpeed, 15);
        while(goldPosition == 0 && elapsedTime.seconds() < 3 && opModeIsActive()){
            getGoldPositionOneMineral();
            scoringSystems.unextendHangSlide(true);
        }
        if(goldPosition == 0){
            //failsafe, so that if this doesn't detect the right two minerals at least we'll still place
            // the team marker and park
            goldPosition = 1;
            //Position 1 is the leftmost mineral
        }
        gyroTurn(turningSpeed, decideFirstSampleheading());
        if(goldPosition == 2){
            encoderDrive(.5, 36, forward, stayOnHeading, 2.5, true);
        }else if(goldPosition == 3){
            encoderDrive(.5, 28, forward, stayOnHeading, 2.5, true);
        }else{
            encoderDrive(.5, 36, forward, stayOnHeading, 2.5, true);
        }
    }
    public void neutralSideSample(){
        double heading = getHeading();
        ElapsedTime elapsedTime = new ElapsedTime();
//should come immediately after unlatching
        //for the most part this should be able to be copy/pasted to the depotSideSample, though a few changes
        //for the team marker may have to be made.

        while(goldPosition == 0 && elapsedTime.seconds() < 3 && opModeIsActive()){
            getGoldPositionOneMineral();
            scoringSystems.unextendHangSlide(true);
        }
        if(goldPosition == 0){
            //failsafe, so that if this doesn't detect the right two minerals at least we'll still place
            // the team marker and park
            goldPosition = 1;
            //Position 1 is the leftmost mineral
        }
        gyroTurn(turningSpeed, decideSecondSampleheading());
        if(goldPosition == 2){
            encoderDrive(.5, 12, forward, stayOnHeading, 5, true);
        }else{
            encoderDrive(.5, 18, forward, stayOnHeading, 5, true);
        }
    }

    public void craterSideParkArmInCrater(){
        encoderDrive(.75, 18, reverse, stayOnHeading, 3, true);
        gyroTurn(turningSpeed, -115);
        encoderDrive(.75, 12, reverse, stayOnHeading, 3, true);
        gyroTurn(turningSpeed, -72);
        encoderDrive(.75, 20, reverse, stayOnHeading, 3, true);
        gyroTurn(turningSpeed, 0);
    }

    public void driveFromCraterAfterSampleToNearDepot(){
        encoderDrive(.5, 46, forward, stayOnHeading, 2.5, true);
        gyroTurn(turningSpeed, -130);//turn to the left, facing the depot
        encoderDrive(.5, 32, forward, stayOnHeading, 3, true);
    }


    public void depotSideDeployAndPark(){
        if(goldPosition == 1){
            gyroTurn(turningSpeed, 45);//turn towards the depot
            encoderDrive(.5, 18, forward, stayOnHeading, 3, true);
            deployTeamMarker();//At this point we'll be on the edge of the depot and about to place the marker
            sleep(250);
            encoderDrive(.75, 52, reverse, stayOnHeading, 5, true);
        }else if(goldPosition == 2){
            deployTeamMarker();
            sleep(200);
            encoderDrive(.5, 24, reverse, stayOnHeading, 2.5, true);
            gyroTurn(turningSpeed, -90);//At this point we'll be facing the other alliances crater-ish
            encoderDrive(.5, 18, forward, stayOnHeading, 2, true);
            gyroTurn(turningSpeed, -45);//face the near non-alliance wall
            encoderDrive(.35, 36, forward, stayOnHeading, 4, true);//just hit the wall
            encoderDrive(.2, 3, reverse, stayOnHeading, 2, true);//back away from the wall for turning clearance
            gyroTurn(turningSpeed, 45);//turn towards the depot
            encoderDrive(.5, 42, reverse, stayOnHeading, 3, true);
        }else{
            gyroTurn(turningSpeed, -45);//face the near non-alliance wall
            encoderDriveSmooth(.5, 8, forward, stayOnHeading, 2);//just hit the wall
            encoderDriveSmooth(.35, 8, forward, 90, 2);//just hit the wall
            encoderDrive(.5, 20, forward, 45, 2, true);//just hit the wall
            encoderDrive(.35, 2, reverse, stayOnHeading, 4, true);//just hit the wall
            gyroTurn(turningSpeed, 46);
            deployTeamMarker();//At this point we'll be on the edge of the depot and about to place the marker
            sleep(250);
            encoderDrive(.75, 52, reverse, stayOnHeading, 5, true);

        }
        gyroTurn(turningSpeed, 50);
        encoderDrive(.5, 12, reverse, stayOnHeading, 5, true);


    }
    public void deployTeamMarker(){
        TeamMarker.setPosition(teamMarkerDeploy);
    }
    public void depotTurnToFarCrater(){
        //This will drive to the other alliance's side's crater, to be out of the way of our partner's team marker.
        //We should also have a version that goes to our side, if our alliance partner also scores in the lander (so that
        // we get a little bit of extra time for cycles.
        gyroTurn(turningSpeed, -45);
        encoderDrive(.425, 18, forward, stayOnHeading, 2, true);
        double thisHeading = getHeading();
        encoderDrive(.35, 2.5, reverse, stayOnHeading, 1.5, true);
        TeamMarker.setPosition(teamMarkerResting);
        double turningHeading = -thisHeading + 91;
        gyroTurn(turningSpeed, turningHeading);
    }
    public void depotTurnToCloseCrater(){
        //This will drive to the other alliance's side's crater, to be out of the way of our partner's team marker.
        //We should also have a version that goes to our side, if our alliance partner also scores in the lander (so that
        // we get a little bit of extra time for cycles.
        gyroTurn(turningSpeed, 45);
        encoderDrive(.425, 18, forward, stayOnHeading, 2, true);
        double thisHeading = getHeading();
        encoderDrive(.35, 2, reverse, stayOnHeading, 1.5, true);
        TeamMarker.setPosition(teamMarkerResting);
        double turningHeading = -thisHeading - 89;
        gyroTurn(turningSpeed, turningHeading);
    }
    /* public void depotSideDoubleSample(){
         goToDistance(.35, 100, FrontDistance,5,3);
         gyroTurn(turningSpeed, -30);
         encoderDrive(.75, 18, reverse, stayOnHeading, 3);
         gyroTurn(turningSpeed, 18);
         encoderDrive(.75, 20, reverse, stayOnHeading, 3);
         gyroTurn(turningSpeed, 90);
         encoderDrive(.2, 4, reverse, stayOnHeading, 1.25);
         gyroTurn(turningSpeed, decideSecondSampleheading());
         if(goldPosition == 2){
             encoderDrive(.5, 16, forward, stayOnHeading, 2.5);
         }else{
             encoderDrive(.5, 24, forward, stayOnHeading, 2.5);
         }
     }*/
    public void driveFromDepot(){}

}
