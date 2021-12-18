//
// Created by luotinkai on 2021/12/5.
//

#include "rm_hw/transmission/multi_actuator_transmission_loader.h"
#include "rm_hw/transmission/multi_actuator_transmission.h"

#include <pluginlib/class_list_macros.hpp>
#include <hardware_interface/internal/demangle_symbol.h>

namespace transmission_interface
{
TransmissionSharedPtr MultiActuatorTransmissionLoader::load(const TransmissionInfo& transmission_info)
{

    if (!checkActuatorDimension(transmission_info, multi_transmission_.numActuators()))
    {
        return TransmissionSharedPtr();
    }
    if (!checkJointDimension(transmission_info, 1))
    {
        return TransmissionSharedPtr();
    }

    // Get actuator and joint configuration sorted by role: [actuator1, actuator2] and [joint1]
    std::vector<double> act_reduction;
    const bool act_config_ok = getActuatorConfig(transmission_info, act_reduction);
    if (!act_config_ok)
    {
        return TransmissionSharedPtr();
    }

    double jnt_reduction, jnt_offset;
    getJointConfig(transmission_info, jnt_reduction, jnt_offset);

    // Transmission instance
    try
    {
        TransmissionSharedPtr transmission(new MultiActuatorTransmission(act_reduction, jnt_reduction, jnt_offset));
        return transmission;
    }
    catch (const TransmissionInterfaceException& ex)
    {
        using hardware_interface::internal::demangledTypeName;
        ROS_ERROR_STREAM_NAMED("parser", "Failed to construct transmission '"
        << transmission_info.name_ << "' of type '"
        << demangledTypeName<MultiActuatorTransmissionLoader>() << "'. "
        << ex.what());
        return TransmissionSharedPtr();
    }
}

bool MultiActuatorTransmissionLoader::getActuatorConfig(const TransmissionInfo& transmission_info,
                                                        std::vector<double>& actuator_reduction)
{
    const std::string actuato_role = "main";

    std::vector<TiXmlElement> act_elements(multi_transmission_.numActuators(), "");
    std::vector<std::string> act_names(multi_transmission_.numActuators());
    std::vector<std::string> act_roles(multi_transmission_.numActuators());

    for (unsigned int i = 0; i < multi_transmission_.numActuators(); ++i)
    {
        // Actuator name
        act_names[i] = transmission_info.actuators_[i].name_;

        // Actuator xml element
        act_elements[i] = loadXmlElement(transmission_info.actuators_[i].xml_element_);

        // Populate role string
        std::string& act_role = act_roles[i];
        getActuatorRole(act_elements[i], act_names[i], transmission_info.name_, true,  // Required
                        act_role);
    }
    // Indices sorted according to role
    std::vector<unsigned int> id_map(multi_transmission_.numActuators());
    for (int i = 0; i < multi_transmission_.numActuators(); ++i)
    {
        id_map[i] = i;
    }
    if (!(actuato_role == act_roles[0]))
    {
        ROS_ERROR_STREAM_NAMED("parser", "Counld not find main actuator");
        return false;
    }
    // Parse required mechanical reductions
    actuator_reduction.resize(multi_transmission_.numActuators());
    for (unsigned int i = 0; i < multi_transmission_.numActuators(); ++i)
    {
        const unsigned int id = id_map[i];
        getActuatorReduction(act_elements[id], act_names[id], transmission_info.name_, true, actuator_reduction[i]);
    }

    return true;
}

bool MultiActuatorTransmissionLoader::getJointConfig(const TransmissionInfo& transmission_info,
                                                     double& joint_reduction, double& joint_offset)
{
    TiXmlElement jnt_elements = "";

    std::string jnt_names = transmission_info.joints_[0].name_;

    // Joint xml element
    jnt_elements = loadXmlElement(transmission_info.joints_[0].xml_element_);

    // Joint configuration
    // Parse optional mechanical reductions.
    getJointReduction(jnt_elements, jnt_names, transmission_info.name_, false, joint_reduction);
    // Parse optional joint offset.
    getJointOffset(jnt_elements, jnt_names, transmission_info.name_, false, joint_offset);
    return true;
                                                                                                                 }

}  // namespace transmission_interface

PLUGINLIB_EXPORT_CLASS(transmission_interface::MultiActuatorTransmissionLoader,
                       transmission_interface::TransmissionLoader)