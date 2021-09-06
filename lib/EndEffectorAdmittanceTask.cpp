#include "EndEffectorAdmittanceTask.h"

#include "fmt/format.h"

using namespace XBot::Cartesian;


EndEffectorAdmittanceImpl::EndEffectorAdmittanceImpl(YAML::Node task_node,
                                         Context::ConstPtr context):
    TaskDescriptionImpl(task_node,
                        context,
                        "EndEffectorAdmittance",
                        6),
    _force_ref(0,0,0),
    _torque_ref(0,0,0),
    _ref_timeout(-1)
{
    /* Here you can parse custom YAML fields from task_node */

}

Eigen::Vector3d EndEffectorAdmittanceImpl::getReferenceForce() const
{
    return _force_ref;
}

Eigen::Vector3d EndEffectorAdmittanceImpl::getReferenceTorque() const
{
    return _torque_ref;
}

void EndEffectorAdmittanceImpl::setReference(const Eigen::Vector3d& force_ref,const Eigen::Vector3d& torque_ref)
{
    _force_ref = force_ref;
    _torque_ref = torque_ref;
    _ref_timeout = getTime() + REF_TTL;
}

void EndEffectorAdmittanceImpl::setReferenceForce(const Eigen::Vector3d& force_ref)
{
    _force_ref = force_ref;
    _ref_timeout = getTime() + REF_TTL;
}

void EndEffectorAdmittanceImpl::setReferenceTorque(const Eigen::Vector3d& torque_ref)
{
    _torque_ref = torque_ref;
    _ref_timeout = getTime() + REF_TTL;
}

void XBot::Cartesian::EndEffectorAdmittanceImpl::update(double time, double period)
{
    // call base class
    TaskDescriptionImpl::update(time, period);

    // if the last reference has expired, the set it to zero
    if(time > _ref_timeout)
    { 
        _force_ref.setZero();
        _torque_ref.setZero();
    }
}

void XBot::Cartesian::EndEffectorAdmittanceImpl::reset()
{
    // call base class
    TaskDescriptionImpl::reset();

    _force_ref.setZero();
    _torque_ref.setZero();
}

void XBot::Cartesian::EndEffectorAdmittanceImpl::log(MatLogger2::Ptr logger,
                                               bool init_logger,
                                               int buf_size)
{
    // call base class
    TaskDescriptionImpl::log(logger, init_logger, buf_size);

    if(init_logger)
    {
        logger->create(getName() + "_force_ref", 3, 1, buf_size);
        logger->create(getName() + "_torque_ref", 3, 1, buf_size);
        return;
    }

    logger->add(getName() + "_force_ref", _force_ref);
    logger->add(getName() + "_torque_ref", _torque_ref);
}

CARTESIO_REGISTER_TASK_PLUGIN(EndEffectorAdmittanceImpl, EndEffectorAdmittance)
