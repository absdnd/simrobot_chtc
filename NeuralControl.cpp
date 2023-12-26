/**
 * @file NeuralControl.cpp
 *
 * This file implements an implementation for the NeuralControl skill.
 *
 * @author Arne Hasselbring (the actual behavior is older)
 */



#include <CompiledNN/CompiledNN.h>
#include <CompiledNN/Model.h>
#include <CompiledNN/SimpleNN.h>
#include <CompiledNN/Tensor.h>

#include "Tools/RLConfig.h"
#include "Tools/RL/RLAlg.h"
#include "Tools/RL/RLData.h"
#include "Tools/RL/RLEnv.h"
#include <map> 
#include "Tools/json.h"

#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/Libraries/LibWalk.h"
#include "Representations/BehaviorControl/PathPlanner.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/StrategyStatus.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/Sensing/ArmContactModel.h"
#include "Representations/Sensing/FootBumperState.h"
#include <cmath>
#include "Tools/BehaviorControl/Strategy/PositionRole.h"
#include "Tools/BehaviorControl/Framework/Skill/CabslSkill.h"

#include <stdio.h>
#include <fstream>
#include <iostream>
// #include <filesystem>


#define PI 3.14159265

int observation_size = 12;
int action_size = 3;


std::mutex cognitionLock;


#ifndef BUILD_NAO_FLAG
std::string policy_path = "../Policies/";
#endif
#ifdef BUILD_NAO_FLAG
std::string policy_path = "/home/nao/Config/Policies/";
#endif






DataTransfer data_transfer(true);

FieldPositions field_positions(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
Environment environment(field_positions, observation_size, action_size);


Algorithm attackerAlgorithm(policy_path, "AttackerPolicy");
Algorithm goalKeeperAlgorithm(policy_path, "GoalKeeperPolicy");
Algorithm * algorithm;


SKILL_IMPLEMENTATION(NeuralControlImpl,
{,
  IMPLEMENTS(NeuralControl),
  REQUIRES(ArmContactModel),
  REQUIRES(FootBumperState),
  REQUIRES(FrameInfo),
  REQUIRES(GameState),
  REQUIRES(LibWalk),
  REQUIRES(MotionInfo),
  REQUIRES(PathPlanner),
  REQUIRES(FieldBall),
  REQUIRES(RobotPose),
  REQUIRES(StrategyStatus),
  REQUIRES(WalkingEngineOutput),


  MODIFIES(BehaviorStatus),
  CALLS(LookForward),
  CALLS(Stand),
  CALLS(PublishMotion),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToPose),
  DEFINES_PARAMETERS(
  {,
    (float)(1000.f) switchToPathPlannerDistance, /**< If the target is further away than this distance, the path planner is used. */
    (float)(900.f) switchToLibWalkDistance, /**< If the target is closer than this distance, LibWalk is used. */
    (float)(175.f) targetForwardWalkingSpeed, /**< Reduce walking speed to reach this forward speed (in mm/s). */
  }),
});




int counter = 0;
int opened_file = false;
std::fstream f;
std::fstream action_file;
int time_steps = 0;
int action_count = 0;
// - Circle
// - Spiral
// - Different arcs combined (Possibly with random values for angle and x/y) - Possibly most promising?
// - Different size squares
// - Different shapes (aka triangle, hexagon, etc.)
// - Apply constant actions (in different directions)

// TODO (if needed): Add different size circles




// Grounding function params
// 0 = neural
// 1 = walk in arc
// 2 = walk in square with turning
// 3 = walk in circlegro
// 4 = walk in square no turning
// 5 = walk forward (1000 time steps)
// 6 = walk in circle (1000 time steps)
// 7 = walk in circle (opposite direction) (1000 time steps)
// 8 = x=1, y=0, sweep theta
// 9 = x=-1, y=0, sweep theta
// 10 = x=0, y=1, sweep theta
// 11 = x=0, y=-1, sweep theta
// 12 = sweep over all three params
// 13 = constant actions
// 14 = Read from file

// Control mode is chosen here. # 
float control = 3.0;
float repeat_cnt = 5;
float speed_scaling = 1.0;
float time_scaling = 2.0;
float turn_radius = 0.1f;
bool write_to_file = true;
std::map<int, char> trajectory_map = {
    {0, 'neural'}, 
    {1, 'walk_in_arc'}, 
    {2, 'walk_in_square_with_turning'}, 
    {3, 'walk_in_circle'}, 
    {4, 'walk_in_square_no_turning'}, 
    {5, 'walk_forward'},
    {13, 'constant_actions'}, 
    {14, 'read_from_file'}
};

// namespace fs = std::filesystem;
// fs::path currentPath = fs::current_path();
// fs::path rootPath = currentPath / ".."/ ".." / ".." / ".." / "data" / "trajectories";
// std::string grounding_path = rootPath.string() + "/";
// std::string action_path = "/home/abhi/Desktop/BHuman/BadgerRLSystem2022/data/load_actions.txt";
std::string grounding_path = "../../data/trajectories/";

class NeuralControlImpl : public NeuralControlImplBase
{ 
  option(NeuralControl)
  {

    cognitionLock.lock();

    if (opened_file == false){
      // Get current date/time, format is MM-DD.HH.mm
      time_t now = time(0);
      tm *ltm = localtime(&now);
      std::string date = std::to_string(1 + ltm->tm_mon) + "-" + std::to_string(ltm->tm_mday) + "." + std::to_string(ltm->tm_hour) + "." + std::to_string(ltm->tm_min);
      std::string path = grounding_path + "control_" + std::to_string(control) + "_scaling_" + std::to_string(speed_scaling) + "_" + date +  ".txt";
      f.open(path, std::ios::app);
      opened_file = true;

      // action_file.open(action_path, std::ios::in);
      
    }

    // Init speed array
    std::vector speed = {0.0f, 0.0f, 0.0f};
	
    if (control == 1){
      
      if (time_steps < 1000){
        // Walk in arc
        speed = {0.1f, 1.0f, 0.0f};
        
      }
      else{
        // Stop
        speed = {0.0f, 0.0f, 0.0f};
        printf("Done\n");
      }
        theWalkAtRelativeSpeedSkill({.speed = {speed[0], speed[1], speed[2]}});
      time_steps++;
    }
    // Walk in square with turning
    else if (control == 2){      
      if (time_steps < 500){
        // Walk forward
        speed = {0.0f, 1.0f, 0.0f};
      }
      else if (time_steps < 550){
        // Turn
        speed = {1.0f, 0.0f, 0.0f};
      }
      else if (time_steps < 1050){
        // Walk forward
        speed = {0.0f, 1.0f, 0.0f};
      }
      else if (time_steps < 1100){
        // Turn
        speed = {1.0f, 0.0f, 0.0f};
          
      }
      else if (time_steps < 1600){
        // Walk forward
        speed = {0.0f, 1.0f, 0.0f};
      }
      else if (time_steps < 1650){
        // Turn
        speed = {1.0f, 0.0f, 0.0f};
      }
      else if (time_steps < 2150){
        // Walk forward
        speed = {0.0f, 1.0f, 0.0f};
      }
      else{
        // Stop
        speed = {0.0f, 0.0f, 0.0f};
        printf("Done\n");
      }
        theWalkAtRelativeSpeedSkill({.speed = {speed[0], speed[1], speed[2]}});
      time_steps++;

    }
    // Walk in circle
    else if (control == 3){ 
      if (time_steps < 3000){
        speed = {turn_radius, 1.0f, 0.0f};
        // printf("Turning with radius %f\n", turn_radius);
      }
      else {
        speed = {0.0f, 0.0f, 0.0f};
        printf("Done\n");
      }
        theWalkAtRelativeSpeedSkill({.speed = {speed[0], speed[1], speed[2]}});
      time_steps++;
    }
    // Walk in square (No turning)
    else if (control == 4){
        if (time_steps < 1000){
            // Walk forward
            speed = {0.0f, 1.0f, 0.0f};
        }
        else if (time_steps < 2000){
            // Walk left
            speed = {0.0f, 0.0f, 1.0f};
        }
        else if (time_steps < 3000){
            // Walk backward
            speed = {0.0f, -1.0f, 0.0f};
        }
        else if (time_steps < 4000){
            // Walk right
            speed = {0.0f, 0.0f, -1.0f};
        }
        else{
            speed = {0.0f, 0.0f, 0.0f};
            printf("Done\n");
        }
        theWalkAtRelativeSpeedSkill({.speed = {speed[0], speed[1], speed[2]}});
        time_steps++;
    }
    // Walk forward 1000 time steps
    else if (control == 5) {
        if (time_steps < 1000) {
            speed = {0.0f, 1.0f, 0.0f};
        }
        else {
            speed = {0.0f, 0.0f, 0.0f};
            printf("Done\n");
        }
        theWalkAtRelativeSpeedSkill({.speed = {speed[0], speed[1], speed[2]}});
        time_steps++;
    }
    // Walk in circle
    else if (control == 6) {
        if (time_steps < 1000) {
            speed = {0.2f, 1.0f, 0.0f};
        }
        else {
            speed = {0.0f, 0.0f, 0.0f};
            printf("Done\n");
        }
        theWalkAtRelativeSpeedSkill({.speed = {speed[0], speed[1], speed[2]}});
        time_steps++;
    }
    // Walk in circle (opposite direction)
    else if (control == 7) {
        if (time_steps < 1000) {
            speed = {-0.2f, 1.0f, 0.0f};
        }
        else {
            speed = {0.0f, 0.0f, 0.0f};
            printf("Done\n");
        }
        theWalkAtRelativeSpeedSkill({.speed = {speed[0], speed[1], speed[2]}});
        time_steps++;
    }
    // Set x to 1, y to 0, sweep over theta
    else if (control == 8){
       if (time_steps < 5000){
            // Increment theta by 0.1 rads every 100 time steps starting at -pi
            float theta = -3.14159f + (time_steps / 100.0f) * 0.1f;
            speed = {theta, 1.0f, 0.0f};
        }
        else {
            speed = {0.0f, 0.0f, 0.0f};
            printf("Done\n");
        }
        theWalkAtRelativeSpeedSkill({.speed = {speed[0], speed[1], speed[2]}});
        time_steps++;
    }
    // Set x to -1, y to 0, sweep over theta
    else if (control == 9){
       if (time_steps < 5000){
            // Increment theta by 0.1 rads every 100 time steps starting at -pi
            float theta = -3.14159f + (time_steps / 100.0f) * 0.1f;
            speed = {theta, -1.0f, 0.0f};
        }
        else {
            speed = {0.0f, 0.0f, 0.0f};
            printf("Done\n");
        }
        theWalkAtRelativeSpeedSkill({.speed = {speed[0], speed[1], speed[2]}});
        time_steps++;
    }
    // Set x to 0, y to 1, sweep over theta
    else if (control == 10){
       if (time_steps < 5000){
            // Increment theta by 0.1 rads every 100 time steps starting at -pi
            float theta = -3.14159f + (time_steps / 100.0f) * 0.1f;
            speed = {theta, 0.0f, 1.0f};
        }
        else {
            speed = {0.0f, 0.0f, 0.0f};
            printf("Done\n");
        }
        theWalkAtRelativeSpeedSkill({.speed = {speed[0], speed[1], speed[2]}});
        time_steps++;
    }
    // Set x to 0, y to -1, sweep over theta
    else if (control == 11){
       if (time_steps < 5000){
            // Increment theta by 0.1 rads every 100 time steps starting at -pi
            float theta = -3.14159f + (time_steps / 100.0f) * 0.1f;
            speed = {theta, 0.0f, -1.0f};
        }
        else {
            speed = {0.0f, 0.0f, 0.0f};
            printf("Done\n");
        }
        theWalkAtRelativeSpeedSkill({.speed = {speed[0], speed[1], speed[2]}});
        time_steps++;
    }
    // Sweep over all three params
    else if (control == 12) {
      if (time_steps < 10000){
            // Sweep from -pi to pi over 100 time steps
            float theta = theta + 6.28318f / 100.0f;
            if (time_steps % 100 == 0) {
                theta = -3.14159f;
            }
            // Sweep from -1 to 1 over 1000 time steps
            float x = x + 2.0f / 1000.0f;
            if (time_steps % 1000 == 0) {
                x = -1.0f;
            }
            // Sweep from -1 to 1 over 10000 time steps
            float y = y + 2.0f / 10000.0f;
            if (time_steps % 10000 == 0) {
                y = -1.0f;
            }
            speed = {theta, x, y};
        }
        else {
            speed = {0.0f, 0.0f, 0.0f};
            printf("Done\n");
        }
        theWalkAtRelativeSpeedSkill({.speed = {speed[0], speed[1], speed[2]}});
        time_steps++;
    }
    // Constant actions
    else if (control == 13) {
      // forward 500 steps
      if (time_steps < 500) {
        speed = {0.0f, 1.0f, 0.0f};
      }
      // Opposite direction 500 steps
      else if (time_steps < 1000) {
        speed = {0.0f, -1.0f, 0.0f};
      }
      // Left 500 steps
      else if (time_steps < 1500) {
        speed = {0.0f, 0.0f, 1.0f};
      }
      // Right 500 steps
      else if (time_steps < 2000) {
        speed = {0.0f, 0.0f, -1.0f};
      }
      // Forward left 500 steps
      else if (time_steps < 2500) {
        speed = {0.0f, 1.0f, 1.0f};
      }
      // Backward right 500 steps
      else if (time_steps < 3000) {
        speed = {0.0f, -1.0f, -1.0f};
      }
      // Forward right 500 steps
      else if (time_steps < 3500) {
        speed = {0.0f, 1.0f, -1.0f};
      }
      // Backward left 500 steps
      else if (time_steps < 4000) {
        speed = {0.0f, -1.0f, 1.0f};
      }
      // Forward with rotation 500 steps
      else if (time_steps < 4500) {
        speed = {0.1f, 1.0f, 0.0f};
      }
      // Backward with rotation 500 steps
      else if (time_steps < 5000) {
        speed = {-0.1f, -1.0f, 0.0f};
      }
      // Forward with negative rotation 500 steps
      else if (time_steps < 5500) {
        speed = {-0.1f, 1.0f, 0.0f};
      }
      // Backward with negative rotation 500 steps
      else if (time_steps < 6000) {
        speed = {0.1f, -1.0f, 0.0f};
      }
      // Forward with rotation 500 steps
      else if (time_steps < 6500) {
        speed = {0.2f, 1.0f, 0.0f};
      }
      // Backward with rotation 500 steps
      else if (time_steps < 7000) {
        speed = {-0.2f, -1.0f, 0.0f};
      }
      // Forward with negative rotation 500 steps
      else if (time_steps < 7500) {
        speed = {-0.2f, 1.0f, 0.0f};
      }
      // Backward with negative rotation 500 steps
      else if (time_steps < 8000) {
        speed = {0.2f, -1.0f, 0.0f};
      }
      // Forward left with rotation 500 steps
      else if (time_steps < 8500) {
        speed = {0.1f, 1.0f, 1.0f};
      }
      // Backward right with rotation 500 steps
      else if (time_steps < 9000) {
        speed = {-0.1f, -1.0f, -1.0f};
      }
      // Forward right with rotation 500 steps
      else if (time_steps < 9500) {
        speed = {0.1f, 1.0f, -1.0f};
      }
      // Backward left with rotation 500 steps
      else if (time_steps < 10000) {
        speed = {-0.1f, -1.0f, 1.0f};
      }
      else {
        speed = {0.0f, 0.0f, 0.0f};
        printf("Done\n");
      }
      theWalkAtRelativeSpeedSkill({.speed = {speed[0], speed[1], speed[2]}});
      time_steps++;
    }
  else if (control == 14)
  {
    std::string line;
    std:: string str_x, str_y, str_z;
    float x = 0, y =0, z=0;
    if (time_steps >= 2500)
    {
      speed = {0.0f, 0.0f, 0.0f};
      printf("Done\n");
    }

    else
    {
      
      
      std::getline(action_file, line);
      std:: cout << "line is: " << line; 
      str_x = line.substr(0, line.find(" "));
      str_y = line.substr(line.find(" ") + 1, line.find(" ", line.find(" ") + 1));
      str_z = line.substr(line.find(" ", line.find(" ") + 1) + 1, line.find(" ", line.find(" ", line.find(" ") + 1)));
      const char* char_str_x=str_x.c_str();
      const char* char_str_y=str_y.c_str();
      const char* char_str_z=str_z.c_str();
      
      x = std::atof(char_str_x);
      y = std::atof(char_str_y);
      z = std::atof(char_str_z);
      
      std::cout << "x: " << str_x << " y: " << str_y << " z: " << str_z << std::endl;
      std::cout << "x: " << x << " y: " << y << " z: " << z << std::endl;
      speed = {x, y, z};
      
    } 
    speed[0] = speed[0]* speed_scaling;
    speed[1] = speed[1] * speed_scaling;
    speed[2] = speed[2] * speed_scaling;
    
    theWalkAtRelativeSpeedSkill({.speed = {speed[0], speed[1], speed[2]}});
    time_steps++;  
  }

  
      // else {
    // //   // Neural 

    // if (theStrategyStatus.role == PositionRole::toRole(PositionRole::goalkeeper))
    // {
    //     algorithm = & goalKeeperAlgorithm;
    // }
    // else
    // {
    //   algorithm =& attackerAlgorithm;
    // }

    // if (algorithm->getCollectNewPolicy()) {
    //         data_transfer.newTrajectoriesJSON();
    //         algorithm->waitForNewPolicy();

    //         algorithm->deleteModels();
    //         algorithm->updateModels();

    //         if (RLConfig::train_mode) {
    //             algorithm->deletePolicyFiles();
    //           }

    //         algorithm->setCollectNewPolicy(false);
    // }

    // const std::vector<NeuralNetwork::TensorLocation> &shared_input = algorithm->getSharedModel()->getInputs();
    // std::vector<NeuralNetwork::TensorXf> observation_input(algorithm->getSharedModel()->getInputs().size());
    // for (std::size_t i = 0; i < observation_input.size(); ++i) {
    //   observation_input[i].reshape(
    //                                 shared_input[i].layer->nodes[shared_input[i].nodeIndex].outputDimensions[shared_input[i].tensorIndex]);
    // }
    
    // std::vector<float> current_observation = environment.getObservation(theRobotPose, theFieldBall);
    // if (RLConfig::normalization) {
    //   current_observation = algorithm->normalizeObservation(current_observation);
    // }

    // for (int i = 0; i < observation_size; i++) {
    //   observation_input[0][i] = current_observation[i];
    // }
    


    // std::vector<NeuralNetwork::TensorXf> shared_output =
    //     algorithm->applyModel(algorithm->getSharedModel(), observation_input);
    
    // NeuralNetwork::TensorXf latent_action = shared_output[0];
    // NeuralNetwork::TensorXf latent_value = shared_output[1];

    // std::vector<NeuralNetwork::TensorXf> value_input(algorithm->getValueModel()->getInputs().size());
    // value_input[0] = latent_value;

    // std::vector<NeuralNetwork::TensorXf> value_output =
    //     algorithm->applyModel(algorithm->getValueModel(), value_input);

    // NeuralNetwork::TensorXf value_estimate = value_output[0];
    // algorithm->setCurrentValue(value_estimate(0));
    
    // std::vector<NeuralNetwork::TensorXf> action_input(algorithm->getActionModel()->getInputs().size());
    // action_input[0] = latent_action;
    

    // std::vector<NeuralNetwork::TensorXf> action_output =
    //     algorithm->applyModel(algorithm->getActionModel(), action_input);


    // std::vector<float> tempCurrentAction = std::vector<float>(algorithm->computeCurrentAction(action_output, environment.getActionLength()));
    
    // theLookForwardSkill();
    // if (theFieldBall.timeSinceBallWasSeen > 15000)
    // {
    //   theWalkAtRelativeSpeedSkill({.speed = {0.8f,
    //                                     0.0f,
    //                                     0.0f}});
    // }
    // else{
    //   theWalkAtRelativeSpeedSkill({.speed = {(float)(algorithm->getActionMeans()[0]) * 0.4f, (float)(algorithm->getActionMeans()[1]) > 1.0f ? 1.0f : (float)(algorithm->getActionMeans()[1]), (float)(algorithm->getActionMeans()[2])}});
    // }
    // }

    if (write_to_file) {
      std::string pose = std::to_string(theRobotPose.translation.x()) + " " + std::to_string(theRobotPose.translation.y()) + " " + std::to_string(theRobotPose.rotation) + " " + std::to_string(speed[0]) + " " + std::to_string(speed[1]) + " " + std::to_string(speed[2]);
      f << pose + "\n";
      // printf("Pose: %s\n", pose.c_str());
      counter++;
    }

    cognitionLock.unlock();

  }
};

MAKE_SKILL_IMPLEMENTATION(NeuralControlImpl);
