#ifndef EEADMITTANCE_H
#define EEADMITTANCE_H

#include <cartesian_interface/sdk/problem/Task.h>

namespace XBot { namespace Cartesian {

/**
 * @brief The End Effector admittance class models the abstract
 * interface for the task.
 *
 * NOTE: it is mandatory to inherit **virtually** from TaskDescription
 */
class EndEffectorAdmittance : public virtual TaskDescription
{

public:

    CARTESIO_DECLARE_SMART_PTR(EndEffectorAdmittance)

    /* Define the task API (pure virtual methods) */
    virtual Eigen::Vector3d getReferenceForce() const = 0;
    virtual Eigen::Vector3d getReferenceTorque() const = 0;
    virtual void setReference(const Eigen::Vector3d& force_ref,const Eigen::Vector3d& torque_ref) = 0;
    virtual void setReferenceForce(const Eigen::Vector3d& force_ref) = 0;
    virtual void setReferenceTorque(const Eigen::Vector3d& torque_ref) = 0;
};

/**
 * @brief The EndEffectorAdmittanceImpl class implements the abstract
 * interface
 */
class EndEffectorAdmittanceImpl : public TaskDescriptionImpl,
                            public virtual EndEffectorAdmittance
{

public:

    CARTESIO_DECLARE_SMART_PTR(EndEffectorAdmittanceImpl)

    /* The task implementation constructor signature must be
     * as follows */
    EndEffectorAdmittanceImpl(YAML::Node task_node,
                        Context::ConstPtr context);

    /* Implement the task API */
    Eigen::Vector3d getReferenceForce() const override;
    Eigen::Vector3d getReferenceTorque() const override;
    void setReference(const Eigen::Vector3d& force_ref,const Eigen::Vector3d& torque_ref) override;
    void setReferenceForce(const Eigen::Vector3d& force_ref) override;
    void setReferenceTorque(const Eigen::Vector3d& torque_ref) override;

    /* Customize update, reset and log */
    void update(double time, double period) override;
    void reset() override;
    void log(MatLogger2::Ptr logger, bool init_logger, int buf_size) override;

private:

    static constexpr double REF_TTL = 1.0;

    Eigen::Vector3d _force_ref;
    Eigen::Vector3d _torque_ref;    
    double _ref_timeout;
};

} }

#endif // EEADMITTANCE_H