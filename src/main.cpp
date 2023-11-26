/******************************************************************************
 *                                                                            *
 * Copyright (C) 2022 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @authors: Ugo Pattacini <ugo.pattacini@iit.it>
 */

#include <cstdlib>
#include <cmath>
#include <iterator>
#include <string>
#include <vector>
#include <utility>
#include <fstream>
#include <iostream>

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Value.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>

#include <iCub/iKin/iKinFwd.h>

/************************************************************************/
auto readFile(const std::string& filename, const unsigned int nDof,
              const std::string& swap_torso_joints,
              std::vector<yarp::sig::Vector>& joints) {
    std::ifstream fin(filename);
    if (fin.is_open()) {
        while (!fin.eof()) {
            yarp::sig::Vector q;
            double read;
            for (unsigned int i = 0; i < nDof; i++) {
                fin >> read;
                q.push_back(read);
            }
            if (swap_torso_joints == "on") {
                std::swap(q[0], q[2]);
            }
            joints.push_back(q);
        }
        fin.close();
        yInfo() << "Data successfully read from" << filename;
        return true;
    } else {
        yError() << "Unable to read from file" << filename;
        return false;
    }
}

/************************************************************************/
int main(int argc, char* argv[]) {
    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);

    if (rf.check("help")) {
        std::cout << "Available options:\n"
                  << "--arm-type [left|right]_arm_v[x].[y]: specify the kinematic type of the arm\n"
                  << "--use-torso [on|off]: specify whether to use the torso; default on\n"
                  << "--swap-torso-joints [on|off]: swap the first and the third torso joints as read from the file; default on\n"
                  << "--joints-constraints [on|off]: specify to apply the internal joints bounds: default off\n"
                  << "--input-file filename: specify the file containing the joints values in degrees; default joints.txt\n"
                  << "--print-joints: print out the joints values read from the input file\n"
                  << "--help: this help\n"
                  << std::endl;
        return EXIT_SUCCESS;
    }

    const auto arm_type = rf.check("arm-type", yarp::os::Value("left_arm_v1")).asString();
    iCub::iKin::iCubArm arm(arm_type);
    yInfo() << "Arm type is" << arm.getType();

    const auto use_torso = rf.check("use-torso", yarp::os::Value("on")).asString();
    if (use_torso == "on") {
        arm.releaseLink(0);
        arm.releaseLink(1);
        arm.releaseLink(2);
    }
    auto nDof = arm.getDOF();
    yInfo() << "Arm DOF is" << nDof;

    const auto swap_torso_joints = rf.check("swap-torso-joints", yarp::os::Value("on")).asString();
    if ((use_torso == "on") && (swap_torso_joints == "on")) {
        yInfo() << "Torso joints will be read in reversed order";
    }

    const auto joints_constraints = rf.check("joints-constraints", yarp::os::Value("off")).asString();
    arm.setAllConstraints(joints_constraints == "on");
    yInfo() << "Joints constraints are" << joints_constraints;

    const auto input_filename = rf.check("input-file", yarp::os::Value("joints.txt")).asString();
    yInfo() << "Joints filename is" << input_filename;
    std::vector<yarp::sig::Vector> joints;
    if (!readFile(input_filename, nDof, swap_torso_joints, joints)) {
        return EXIT_FAILURE;
    }

    if (rf.check("print-joints")) {
        for (const auto& q:joints) {
            yDebug() << q.toString(3, 3);
        }
    }

    yInfo() << "Arm end-effector pose in x y z ax ay az theta:";
    for (const auto& q:joints) {
        std::cout << arm.EndEffPose((M_PI/180.)*q).toString(3, 3) << std::endl;
    }

    return EXIT_SUCCESS;
}