#pragma once

#include <crl-basic/gui/application.h>

#include "loco/controller/KinematicTrackingController.h"
#include "loco/planner/GaitPlanner.h"
#include "loco/planner/SimpleLocomotionTrajectoryPlanner.h"
#include "menu.h"

namespace locoApp {

class App : public crl::gui::ShadowApplication {
public:
    App() : crl::gui::ShadowApplication("Locomotion App") {
        this->showConsole = true;
        this->automanageConsole = true;
        this->showPlots = true;
        this->show_world_frame = false;
        this->currentJoint = "";
        this->upper = 0;

        // setup
        setupRobotAndController();
    }

    ~App() override = default;

    void process() override {
        // add gait plan
        controller_->planner->appendPeriodicGaitIfNeeded(gaitPlanner_->getPeriodicGait(robot_));

        double simTime = 0;
        while (simTime < 1.0 / targetFramerate) {
            simTime += dt;
            controller_->computeAndApplyControlSignals(dt);
            controller_->advanceInTime(dt);
        }

        // generate motion plan
        controller_->generateMotionTrajectories();

        // adjust light and camera
        const auto &center = robot_->getTrunk()->getWorldCoordinates(crl::P3D());
        if (followRobotWithCamera) {
            camera.target.x = (float)center.x;
            camera.target.z = (float)center.z;
        }
        light.target.x() = center.x;
        light.target.z() = center.z;
    }

    void restart() override {
        setupRobotAndController();
    }

    void drawObjectsWithShadows(const crl::gui::Shader &shader) override {
        ShadowApplication::drawObjectsWithShadows(shader);
    }

    void drawShadowCastingObjects(const crl::gui::Shader &shader) override {
        robot_->draw(shader);
    }

    void drawObjectsWithoutShadows(const crl::gui::Shader &shader) override {
        robot_->draw(shader);

        if (drawDebugInfo)
            controller_->drawDebugInfo(&basicShader);
    }

    std::string getJointName() {
        if(this->currentJoint == "1") {
            return "lowerback_x";
        } else if(this->currentJoint == "2") {
            return "lowerback_y";
        } else if(this->currentJoint == "3") {
            return "lowerback_z";
        } else if(this->currentJoint == "4") {
            return "upperback_x";
        } else if(this->currentJoint == "5") {
            return "upperback_y";
        } else if(this->currentJoint == "6") {
            return "upperback_z";
        } else if(this->currentJoint == "7") {
            return "lowerneck_x";
        } else if(this->currentJoint == "8") {
            return "lowerneck_y";
        } else if(this->currentJoint == "9") {
            return "lowerneck_z";
        } else if(this->currentJoint == "10") {
            return "upperneck_x";
        } else if(this->currentJoint == "11") {
            return "upperneck_y";
        } else if(this->currentJoint == "12") {
            return "upperneck_z";
        } else if(this->currentJoint == "13") {
            return "lScapula_y";
        } else if(this->currentJoint == "14") {
            return "lScapula_z";
        } else if(this->currentJoint == "15") {
            return "lShoulder_1";
        } else if(this->currentJoint == "16") {
            return "lShoulder_2";
        } else if(this->currentJoint == "17") {
            return "lShoulder_torsion";
        } else if(this->currentJoint == "18") {
            return "lElbow_flexion_extension";
        } else if(this->currentJoint == "19") {
            return "lElbow_torsion";
        }else if(this->currentJoint == "20") {
            return "lWrist_x";
        } else if(this->currentJoint == "21") {
            return "lWrist_z";
        } else if(this->currentJoint == "22") {
            return "rScapula_y";
        } else if(this->currentJoint == "23") {
            return "rScapula_z";
        } else if(this->currentJoint == "24") {
            return "rShoulder_1";
        } else if(this->currentJoint == "25") {
            return "rShoulder_2";
        } else if(this->currentJoint == "26") {
            return "rShoulder_torsion";
        } else if(this->currentJoint == "27") {
            return "rElbow_flexion_extension";
        } else if(this->currentJoint == "28") {
            return "rElbow_torsion";
        } else if(this->currentJoint == "29") {
            return "rWrist_x";
        } else if(this->currentJoint == "30") {
            return "rWrist_z";
        } else if(this->currentJoint == "31") {
            return "lHip_1";
        } else if(this->currentJoint == "32") {
            return "lHip_2";
        } else if(this->currentJoint == "33") {
            return "lHip_torsion";
        } else if(this->currentJoint == "34") {
            return "lKnee";
        } else if(this->currentJoint == "35") {
            return "lAnkle_1";
        } else if(this->currentJoint == "36") {
            return "lAnkle_2";
        } else if(this->currentJoint == "37") {
            return "lToeJoint";
        } else if(this->currentJoint == "38") {
            return "rHip_1";
        } else if(this->currentJoint == "39") {
            return "rHip_2";
        } else if(this->currentJoint == "40") {
            return "rHip_torsion";
        } else if(this->currentJoint == "41") {
            return "rKnee";
        } else if(this->currentJoint == "42") {
            return "rAnkle_1";
        } else if(this->currentJoint == "43") {
            return "rAnkle_2";
        } else if(this->currentJoint == "44") {
            return "rToeJoint";
        } else {
            return "";
        }
    }

    bool keyPressed(int key, int mods) override {
        if (key == GLFW_KEY_SPACE) {
            processIsRunning = !processIsRunning;
        }
        if (key == GLFW_KEY_ENTER) {
            if (!processIsRunning)
                process();
        }

        // joystick command
        bool dirty = false;
        if (key == GLFW_KEY_UP) {
            planner_->speedForward += 0.1;
            dirty = true;
        }
        if (key == GLFW_KEY_DOWN) {
            planner_->speedForward -= 0.1;
            dirty = true;
        }
        if (key == GLFW_KEY_LEFT) {
            planner_->turningSpeed += 0.1;
            dirty = true;
        }
        if (key == GLFW_KEY_RIGHT) {
            planner_->turningSpeed -= 0.1;
            dirty = true;
        }

        if (dirty) {
            planner_->appendPeriodicGaitIfNeeded(gaitPlanner_->getPeriodicGait(robot_));
            controller_->generateMotionTrajectories();
            return true;
        }

        //a
        if (key == 48 || key == 49 || key == 50 || key == 51 || key == 52 || key == 53 || key == 54 || key == 55 || key == 56 || key == 57) {
            this->currentJoint += key;
            std::string name = getJointName();
            std::cout << "Selected joint: " + name << std::endl;
        }

        if (key == 92) {
            this->currentJoint = "";
            std::cout << "Successfully reset joint key" << std::endl;
        }

        if (key == 73) {
            std::cout << this->currentJoint << std::endl;
            if (this->upper == 0) {
                std::cout << "You are currently adjustiing the upper angle limit of joint " + getJointName() << std::endl;
            } else if (this -> upper == 1) {
                std::cout << "You are currently adjustiing the lower angle limit of joint " + getJointName() << std::endl;
            } else {
                std::cout << "You are currently adjustiing both angle limits of joint " + getJointName() << std::endl;
            } 
        }

        if (key == 47) {
            const char *joint = getJointName().c_str();
            if (getJointName() != "") {
                if (this->upper == 1 or this->upper == 2) {
                    if(robot_ != nullptr) {
                        const char *joint = getJointName().c_str();
                        std::string name = getJointName();
                        if (name != "") {
                            robot_->getJointByName(joint)->minAngle -= 0.1;
                            std::cout << "Decremented min angle of " +  robot_->getJointByName(joint)->name + ". New limits: (" + std::to_string(robot_->getJointByName(joint)->minAngle) + "," + std::to_string(robot_->getJointByName(joint)->maxAngle) + ")" << std::endl;
                        }
                    }
                } 
                if (this->upper == 0 or this->upper == 2) {
                    if(robot_ != nullptr) {
                        const char *joint = getJointName().c_str();
                        std::string name = getJointName();
                        if (name!= "") {
                            robot_->getJointByName(joint)->maxAngle -= 0.1;
                            robot_->getJointByName(joint)->minAngle = std::min(robot_->getJointByName(joint)->minAngle,robot_->getJointByName(joint)->maxAngle);
                            std::cout << "Decremented max angle of " +  robot_->getJointByName(joint)->name + ". New limits: (" + std::to_string(robot_->getJointByName(joint)->minAngle) + "," + std::to_string(robot_->getJointByName(joint)->maxAngle) + ")" << std::endl;
                        }
                    }

                    /*std::cout << joint << std::endl;
                    std::cout << "Incremented max angle. New angles:" << std::endl;
                    std::cout << robot_->getJointByName(joint)->minAngle << std::endl;
                    std::cout << robot_->getJointByName(joint)->maxAngle << std::endl;*/
                }
            }
        }

        if (key == 93) {
            const char *joint = getJointName().c_str();
            if (getJointName() != "") {
                if (this->upper == 1 or this->upper == 2) {
                    if(robot_ != nullptr) {
                        const char *joint = getJointName().c_str();
                        std::string name = getJointName();
                        if (name != "") {
                            robot_->getJointByName(joint)->minAngle += 0.1;
                            robot_->getJointByName(joint)->maxAngle = std::max(robot_->getJointByName(joint)->maxAngle,robot_->getJointByName(joint)->minAngle);
                            std::cout << "Incremented min angle of " +  robot_->getJointByName(joint)->name + ". New limits: (" + std::to_string(robot_->getJointByName(joint)->minAngle) + "," + std::to_string(robot_->getJointByName(joint)->maxAngle) + ")" << std::endl;
                        }
                    }
                } 
                if (this->upper == 0 or this->upper == 2){
                    if(robot_ != nullptr) {
                        const char *joint = getJointName().c_str();
                        std::string name = getJointName();
                        if (name!= "") {
                            robot_->getJointByName(joint)->maxAngle += 0.1;
                            std::cout << "Incremented max angle of " +  robot_->getJointByName(joint)->name + ". New limits: (" + std::to_string(robot_->getJointByName(joint)->minAngle) + "," + std::to_string(robot_->getJointByName(joint)->maxAngle) + ")" << std::endl;
                        }
                    }

                    /*std::cout << joint << std::endl;
                    std::cout << "Incremented max angle. New angles:" << std::endl;
                    std::cout << robot_->getJointByName(joint)->minAngle << std::endl;
                    std::cout << robot_->getJointByName(joint)->maxAngle << std::endl;*/
                }
            }
        }

        if (key == 340) {
            this->upper = (this->upper + 1) % 3;
            if(this->upper == 0){
                std::cout << "Currrently only editing max angle constraint" << std::endl;
            } else if (this->upper == 1) {
                std::cout << "Currrently only editing min angle constraint" << std::endl;
            } else {
                std::cout << "Currrently editing both angle constraints" << std::endl;
            }
        }

        if (key == 69) {
            for(int i = 0; i < 45;i++) {
                this->currentJoint = std::to_string(i);
                const char *joint = getJointName().c_str();
                std::string name = getJointName();
                if (name!= "") {
                    std::shared_ptr<crl::loco::RBJoint> rbjoint = robot_->getJointByName(joint);
                    std::cout << name + ": (" + std::to_string(rbjoint->minAngle) + "," + std::to_string(rbjoint->maxAngle) + ")" << std::endl;
                }
            }
            this->currentJoint = "";
        }

        if(key == 80) {
            std::cout << "🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵" << std::endl;
            std::cout << "We are now adjusting the head trajectory" << std::endl;
            char answer;
            do {
                std::cout << "Do you want to default mode? (y/n)" << std::endl;
                std::cin >> answer;
                if(answer == 'y') {
                    std::cout << "Please enter the pelvisBop: " << std::endl;
                    std::cin >> robot_->getLimbByName("pelvis")->pelvisBop;
                    std::cout << "Please enter the pelvsShift: " << std::endl;
                    std::cin >> robot_->getLimbByName("head")->pelvisShift;
                    std::cout << "🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵" << std::endl;
                } else if (answer == 'n') {
                    std::cout << "More finegrained control is not yet implemented." << std::endl;
                }
            } while(answer != 'y' and answer != 'n');
        }

        if(key == 72) {
            std::cout << "🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵" << std::endl;
            std::cout << "We are now adjusting the hand trajectory" << std::endl;
            char answer;
            do {
                std::cout << "Do you want to default mode? (y/n)" << std::endl;
                std::cin >> answer;
                if(answer == 'y') {
                    std::cout << "Please enter the yMaxForBase: " << std::endl;
                    std::cin >> robot_->getLimbByName("lHand")->yMaxForBase;
                    std::cin >> robot_->getLimbByName("rHand")->yMaxForBase;
                    std::cout << "Please enter the yMaxForScaler: " << std::endl;
                    std::cin >> robot_->getLimbByName("lHand")->yMaxForScaler;
                    std::cin >> robot_->getLimbByName("rHand")->yMaxForScaler;
                    std::cout << "Please enter the zMaxForBase: " << std::endl;
                    std::cin >> robot_->getLimbByName("lHand")->zMaxForBase;
                    std::cin >> robot_->getLimbByName("rHand")->zMaxForBase;
                    std::cout << "Please enter the zMaxForScaler: " << std::endl;
                    std::cin >> robot_->getLimbByName("lHand")->zMaxForScaler;
                    std::cin >> robot_->getLimbByName("rHand")->zMaxForScaler;
                    std::cout << "Please enter the zMaxBackBase: " << std::endl;
                    std::cin >> robot_->getLimbByName("rHand")->zMaxBackBase;
                    std::cin >> robot_->getLimbByName("rHand")->zMaxBackBase;
                    std::cout << "Please enter the zMaxBackScaler: " << std::endl;
                    std::cin >> robot_->getLimbByName("lHand")->zMaxBackScaler;
                    std::cin >> robot_->getLimbByName("rHand")->zMaxBackScaler;
                    std::cout << "Please enter the yMaxBackBase: " << std::endl;
                    std::cin >> robot_->getLimbByName("lHand")->yMaxBackBase;
                    std::cin >> robot_->getLimbByName("rHand")->yMaxBackBase;
                    std::cout << "Please enter the yMaxBackScaler: " << std::endl;
                    std::cin >> robot_->getLimbByName("lHand")->yMaxBackScaler;
                    std::cin >> robot_->getLimbByName("rHand")->yMaxBackScaler;
                    std::cout << "Please enter the yMinMidBase: " << std::endl;
                    std::cin >> robot_->getLimbByName("rHand")->yMinMidBase;
                    std::cin >> robot_->getLimbByName("rHand")->yMinMidBase;
                    std::cout << "Please enter the yMinMidScaler: " << std::endl;
                    std::cin >> robot_->getLimbByName("lHand")->yMinMidScaler;
                    std::cin >> robot_->getLimbByName("rHand")->yMinMidScaler;
                    std::cout << "Please enter the xHandInBase: " << std::endl;
                    std::cin >> robot_->getLimbByName("lHand")->xHandInBase;
                    std::cin >> robot_->getLimbByName("rHand")->xHandInBase;
                    std::cout << "Please enter the xHandInScaler: " << std::endl;
                    std::cin >> robot_->getLimbByName("lHand")->xHandInScaler;
                    std::cin >> robot_->getLimbByName("rHand")->xHandInScaler;
                    std::cout << "🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵" << std::endl;
                } else if (answer == 'n') {
                    std::cout << "More finegrained control is not yet implemented." << std::endl;
                }
            } while(answer != 'y' and answer != 'n');
        }

        if(key == 75) {
            std::cout << "🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵" << std::endl;
            std::cout << "We are now adjusting the head trajectory" << std::endl;
            char answer;
            do {
                std::cout << "Do you want to default mode? (y/n)" << std::endl;
                std::cin >> answer;
                if(answer == 'y') {
                    std::cout << "Please enter the headBop: " << std::endl;
                    std::cin >> robot_->getLimbByName("head")->headBop;
                    std::cout << "Please enter the speedscaler: " << std::endl;
                    std::cin >> robot_->getLimbByName("head")->headSpeedScaler;
                    std::cout << "🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵" << std::endl;
                } else if (answer == 'n') {
                    std::cout << "More finegrained control is not yet implemented." << std::endl;
                }
            } while(answer != 'y' and answer != 'n');
        }

        if(key == 76) {
            std::cout << "🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵" << std::endl;
            std::cout << "We are now adjusting the leg trajectory" << std::endl;
            char answer;
            do {
                std::cout << "Do you want to default mode? (y/n)" << std::endl;
                std::cin >> answer;
                if(answer == 'y') {
                    std::cout << "Please enter height0: " << std::endl;
                    std::cin >> robot_->getLimbByName("lLowerLeg")->height0;
                    std::cin >> robot_->getLimbByName("rLowerLeg")->height0;
                    std::cin >> robot_->getLimbByName("lToes")->height0;
                    std::cin >> robot_->getLimbByName("rToes")->height0;
                    std::cout << "Please enter height1: " << std::endl;
                    std::cin >> robot_->getLimbByName("lLowerLeg")->height1;
                    std::cin >> robot_->getLimbByName("rLowerLeg")->height1;
                    std::cin >> robot_->getLimbByName("lToes")->height1;
                    std::cin >> robot_->getLimbByName("rToes")->height1;
                    std::cout << "Please enter height2: " << std::endl;
                    std::cin >> robot_->getLimbByName("lLowerLeg")->height2;
                    std::cin >> robot_->getLimbByName("rLowerLeg")->height2;
                    std::cin >> robot_->getLimbByName("lToes")->height2;
                    std::cin >> robot_->getLimbByName("rToes")->height2;
                    std::cout << "Please enter offset0: " << std::endl;
                    std::cin >> robot_->getLimbByName("lLowerLeg")->offset0;
                    std::cin >> robot_->getLimbByName("rLowerLeg")->offset0;
                    std::cin >> robot_->getLimbByName("lToes")->offset0;
                    std::cin >> robot_->getLimbByName("rToes")->offset0;
                    std::cout << "Please enter offset1: " << std::endl;
                    std::cin >> robot_->getLimbByName("lLowerLeg")->offset1;
                    std::cin >> robot_->getLimbByName("rLowerLeg")->offset1;
                    std::cin >> robot_->getLimbByName("lToes")->offset1;
                    std::cin >> robot_->getLimbByName("rToes")->offset1;
                    std::cout << "Please enter offset2: " << std::endl;
                    std::cin >> robot_->getLimbByName("lLowerLeg")->offset2;
                    std::cin >> robot_->getLimbByName("rLowerLeg")->offset2;
                    std::cin >> robot_->getLimbByName("lToes")->offset2;
                    std::cin >> robot_->getLimbByName("rToes")->offset2;
                    std::cout << "🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵🦵" << std::endl;
                } else if (answer == 'n') {
                    std::cout << "More finegrained control is not yet implemented." << std::endl;
                }
            } while(answer != 'y' and answer != 'n');
        }
        
        if (key == 88) {
            if(robot_ != nullptr) {
                const char *joint = getJointName().c_str();
                std::string name = getJointName();
                if (name != "") {
                    std::shared_ptr<crl::loco::RBJoint> rbjoint = robot_->getJointByName(joint);
                    robot_->getJointByName(joint)->minAngle = robot_->getJointByName(joint)->maxAngle;
                    std::cout << name + ": (" + std::to_string(rbjoint->minAngle) + "," + std::to_string(rbjoint->maxAngle) + ")" << std::endl;
                }
            }

        }

        return false;
    }

    template <typename T>
    void drawComboMenu(const std::string &menuName, const std::vector<T> &options, uint &selected) {
        if (ImGui::BeginCombo(menuName.c_str(), options[selected].name.c_str())) {
            for (uint n = 0; n < options.size(); n++) {
                bool is_selected = (selected == n);
                if (ImGui::Selectable(options[n].name.c_str(), is_selected)) {
                    selected = n;
                    setupRobotAndController();
                }
                if (is_selected) {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }
    }

    void drawImGui() override {
        crl::gui::ShadowApplication::drawImGui();

        ImGui::Begin("Main Menu");
        ImGui::Checkbox("Follow Robot with Camera", &followRobotWithCamera);
        ImGui::Checkbox("Terrain", &uneven_terrain);
        if (ImGui::CollapsingHeader("Character")) {
            drawComboMenu("Model##character", modelOptions, selectedModel);
        }
        if (ImGui::CollapsingHeader("Draw")) {
            if (ImGui::Checkbox("Show meshes", &robot_->showMeshes)) {
                robot_->showSkeleton = !robot_->showMeshes;
            }
            ImGui::Checkbox("Show end effectors", &robot_->showEndEffectors);
            ImGui::Checkbox("Draw debug info", &drawDebugInfo);
        }

        ImGui::End();

        planner_->visualizeContactSchedule();
        planner_->visualizeParameters();
    }

    void drawImPlot() override {
        crl::gui::ShadowApplication::drawImPlot();

        ImGui::Begin("Plots");
        // here, you can draw plots
        ImGui::End();
    }

    virtual bool drop(int count, const char **fileNames) override {
        return true;
    }

private:
    void setupRobotAndController() {
        std::cout << "Welcome to your Digital Bob Editor v  1.3 🦵" <<std::endl;
        std::cout << "********************************************************" <<std::endl;
        std::cout << ""  <<std::endl;
        std::cout << "To get started, familiarize yourself with the commands in readMe.md. \n Make sure you select the window when giving commands and \n only turn back to terminal for input prompts." <<std::endl;
        std::cout << ""  <<std::endl;
        std::cout << "********************************************************" <<std::endl;
        const auto &m = modelOptions[selectedModel];
        const char *rbsFile = m.filePath.c_str();
        robot_ = std::make_shared<crl::loco::LeggedRobot>(rbsFile);
        robot_->setRootState(crl::P3D(0, m.baseTargetHeight, 0));
        if (m.type == ModelOption::Type::DOG) {
            robot_->showMeshes = false;
            robot_->showSkeleton = true;
            gaitPlanner_ = std::make_shared<crl::loco::QuadrupedalGaitPlanner>();
        } else {
            gaitPlanner_ = std::make_shared<crl::loco::BipedalGaitPlanner>();
        }

        // add legs
        for (int i = 0; i < m.legs.size(); i++) {
            robot_->addLimb(m.legs[i].first, m.legs[i].second);
        }

        // setup planner and controller
        planner_ = std::make_shared<crl::loco::SimpleLocomotionTrajectoryPlanner>(robot_);
        planner_->trunkHeight = m.baseTargetHeight;
        planner_->targetStepHeight = m.swingFootHeight;
        controller_ = std::make_shared<crl::loco::KinematicTrackingController>(planner_);

        // generate plan
        planner_->appendPeriodicGaitIfNeeded(gaitPlanner_->getPeriodicGait(robot_));
        controller_->generateMotionTrajectories();
        
    }

public:
    // simulation
    std::shared_ptr<crl::loco::LeggedRobot> robot_ = nullptr;
    std::shared_ptr<crl::loco::GaitPlanner> gaitPlanner_ = nullptr;
    std::shared_ptr<crl::loco::LocomotionTrajectoryPlanner> planner_ = nullptr;
    std::shared_ptr<crl::loco::KinematicTrackingController> controller_ = nullptr;

    // parameters
    double dt = 1 / 60.0;

    // options
    uint selectedModel = 0;
    bool followRobotWithCamera = true;
    bool uneven_terrain = true;
    bool drawDebugInfo = true;
};

}  // namespace locoApp
