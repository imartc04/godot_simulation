#pragma once

#include <string>
#include "interfaces/joint_data.hpp"

#include <nlohmann/json.hpp>

/**
 * Function to parse from std::string to CJointData
 */
static void stringToJointData(std::string &f_string, CJointData &f_joint)
{

    // Parse the JSON string
    json jsonData = json::parse(f_string);

    if (jsonData.find("name") != jsonData.end())
    {
        f_joint.name = jsonData["name"];
    }

    // Check if an entry exists and get its value
    if (jsonData.find("lin_vel_x") != jsonData.end())
    {
        f_joint.lin_vel_x.valid = true;
        f_joint.lin_vel_x.value = jsonData["lin_vel_x"];
    }

    if (jsonData.find("lin_vel_y") != jsonData.end())
    {
        f_joint.lin_vel_x.valid = true;
        f_joint.lin_vel_x.value = jsonData["lin_vel_y"];
    }

    if (jsonData.find("lin_vel_z") != jsonData.end())
    {
        f_joint.lin_vel_x.valid = true;
        f_joint.lin_vel_x.value = jsonData["lin_vel_z"];
    }

    if (jsonData.find("angular_x") != jsonData.end())
    {
        f_joint.lin_vel_x.valid = true;
        f_joint.lin_vel_x.value = jsonData["angular_x"];
    }

    if (jsonData.find("angular_y") != jsonData.end())
    {
        f_joint.lin_vel_x.valid = true;
        f_joint.lin_vel_x.value = jsonData["angular_y"];
    }

    if (jsonData.find("angular_z") != jsonData.end())
    {
        f_joint.lin_vel_x.valid = true;
        f_joint.lin_vel_x.value = jsonData["angular_z"];
    }
};

/**
 * Function to parse from CJointData to std::string json
 */

static void jointDataToJsonString(CJointData &f_joint, std::string &f_str)
{
    json jsonData;
    jsonData["name"] = f_joint.name;


    jsonData["lin_vel_x"]["valid"] = jointData.lin_vel_x.valid;
    jsonData["lin_vel_x"]["value"] = jointData.lin_vel_x.value;

    jsonData["lin_vel_y"]["valid"] = jointData.lin_vel_y.valid;
    jsonData["lin_vel_y"]["value"] = jointData.lin_vel_y.value;


    jsonData["lin_vel_z"]["valid"] = jointData.lin_vel_z.valid;
    jsonData["lin_vel_z"]["value"] = jointData.lin_vel_z.value;

    jsonData["angular_x"]["valid"] = jointData.angular_x.valid;
    jsonData["angular_x"]["value"] = jointData.angular_x.value;

    jsonData["angular_y"]["valid"] = jointData.angular_y.valid;
    jsonData["angular_y"]["value"] = jointData.angular_y.value;

    jsonData["angular_x"]["valid"] = jointData.angular_z.valid;
    jsonData["angular_x"]["value"] = jointData.angular_z.value;


    f_str = jsonData.dump();
};
