// Copyright (c) 2015, LAAS-CNRS
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of hpp-core.
// hpp-core is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-core is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-core. If not, see <http://www.gnu.org/licenses/>.

#include "hpp/core/locked-joint.hh"

#include <sstream>

#include <hpp/util/debug.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint.hh>

namespace hpp {
  namespace core {
    namespace {
        template <typename T>
	std::string numToStr (const T& v) {
 	  std::stringstream ss; ss<<v; return ss.str ();
	}
    }

    /// Copy object and return shared pointer to copy
    EquationPtr_t LockedJoint::copy () const
    {
      return createCopy (weak_.lock ());
    }

    LockedJointPtr_t LockedJoint::create (const JointPtr_t& joint,
					  const LiegroupElement& value)
    {
      LockedJoint* ptr = new LockedJoint (joint, value);
      LockedJointPtr_t shPtr (ptr);
      ptr->init (shPtr);
      return shPtr;
    }

    LockedJointPtr_t LockedJoint::create (const JointPtr_t& joint,
                                          const size_type index,
					  vectorIn_t value)
    {
      LockedJoint* ptr = new LockedJoint (joint, index, value);
      LockedJointPtr_t shPtr (ptr);
      ptr->init (shPtr);
      return shPtr;
    }

    LockedJointPtr_t LockedJoint::create (const DevicePtr_t& dev,
        const size_type index, vectorIn_t value)
    {
      LockedJoint* ptr = new LockedJoint (dev, index, value);
      LockedJointPtr_t shPtr (ptr);
      ptr->init (shPtr);
      return shPtr;
    }

    LockedJointPtr_t LockedJoint::createCopy (LockedJointConstPtr_t other)
    {
      LockedJoint* ptr = new LockedJoint (*other);
      LockedJointPtr_t shPtr (ptr);
      ptr->init (shPtr);
      return shPtr;
    }

    std::size_t LockedJoint::rankInConfiguration () const
    {
      return rankInConfiguration_;
    }

    std::size_t LockedJoint::rankInVelocity () const
    {
      return rankInVelocity_;
    }

    std::size_t LockedJoint::configSize () const
    {
      return configSpace_->nq ();
    }

    std::size_t LockedJoint::numberDof () const
    {
      return configSpace_->nv ();
    }

    const LiegroupSpacePtr_t& LockedJoint::configSpace () const
    {
      return configSpace_;
    }

    bool LockedJoint::isSatisfied (ConfigurationIn_t config)
    {
      vector_t error;
      return isSatisfied (config, error);
    }

    bool LockedJoint::isSatisfied (ConfigurationIn_t config, vector_t& error)
    {
      LiegroupElement q (config.segment (rankInConfiguration_, configSize ()),
                         configSpace_);
      error = q - (configSpace_->exp (rightHandSide ()));
      return error.isApprox (vector_t::Zero (joint_->numberDof ()));
    }

    void LockedJoint::rightHandSideFromConfig (ConfigurationIn_t config)
    {
      if (!constantRightHandSide ()) {
        LiegroupElement q (config.segment (rankInConfiguration_, configSize ()),
                           configSpace_);
        rightHandSide (q - configSpace_->neutral ());
      }
    }

    LockedJoint::LockedJoint (const JointPtr_t& joint,
                              const LiegroupElement& value) :
      Equation (ComparisonTypes_t(joint->numberDof(), constraints::Equality),
                vector_t::Zero (joint->numberDof ())),
      jointName_ (joint->name ()),
      rankInConfiguration_ (joint->rankInConfiguration ()),
      rankInVelocity_ (joint->rankInVelocity ()), joint_ (joint),
      configSpace_ (joint->configurationSpace ())
    {
      assert (rhsSize () == joint->numberDof ());
      assert (*(value.space ()) == *configSpace_);
      rightHandSide (value - configSpace_->neutral ());
      initFunction ();
    }

    LockedJoint::LockedJoint (const JointPtr_t& joint, const size_type index,
        vectorIn_t value) :
      Equation (ComparisonTypes_t(value.size(), constraints::Equality),
                vector_t::Zero (value.size())),
      jointName_ ("partial_" + joint->name ()),
      rankInConfiguration_ (joint->rankInConfiguration () + index),
      rankInVelocity_ (joint->rankInVelocity () + index),
      joint_ (joint), configSpace_
      (LiegroupSpace::Rn (joint->configSize () - index))
    {
      assert (joint->numberDof () == joint->configSize ());
      rightHandSide (value);
      assert (rhsSize () == value.size());
      initFunction ();
    }

    LockedJoint::LockedJoint (const DevicePtr_t& dev, const size_type index,
        vectorIn_t value) :
      Equation (ComparisonTypes_t(value.size(), constraints::Equality),
                vector_t::Zero (value.size())),
      jointName_ (dev->name() + "_extraDof" + numToStr (index)),
      rankInConfiguration_
      (dev->configSize () - dev->extraConfigSpace().dimension() + index),
      rankInVelocity_
      (dev->numberDof ()  - dev->extraConfigSpace().dimension() + index),
      joint_ (JointPtr_t ()), configSpace_ (LiegroupSpace::Rn (value.size ()))
    {
      assert (value.size() > 0);
      assert (rankInConfiguration_ + value.size() <= dev->configSize());
      rightHandSide (value);
      assert (rhsSize () == value.size());
      initFunction ();
    }

    void LockedJoint::init (const LockedJointPtr_t& self)
    {
      weak_ = self;
    }

    std::ostream& LockedJoint::print (std::ostream& os) const
    {
      os << "Locked joint " << jointName_
	 << ", value = " << pinocchio::displayConfig (rightHandSide ())
        << ": rank in configuration = " << rankInConfiguration_
        << ": rank in velocity = " << rankInVelocity_
        << std::endl;
      return os;
    }

    LockedJoint::LockedJoint (const LockedJoint& other) :
      Equation (other), jointName_ (other.jointName_),
      rankInConfiguration_ (other.rankInConfiguration_),
      rankInVelocity_ (other.rankInVelocity_), joint_ (other.joint_),
      configSpace_ (other.configSpace_), function_ (other.function_), weak_ ()
    {
    }

    bool LockedJoint::isEqual (const Equation& other, bool swapAndTest) const
    {
      try {
	const LockedJoint& lj =
	  dynamic_cast <const LockedJoint&> (other);
	if (!Equation::isEqual (other, false)) return false;
	if (jointName_ != lj.jointName_) return false;
	if (rankInConfiguration_ != lj.rankInConfiguration_) return false;
	if (rankInVelocity_ != lj.rankInVelocity_) return false;
	if (*configSpace_ != *(lj.configSpace_)) return false;
	if (swapAndTest) return lj.isEqual (*this, false);
	return true;
      } catch (const std::bad_cast& err) {
	return false;
      }
    }

    void LockedJoint::initFunction ()
    {
      function_ = constraints::ConstantFunctionPtr_t (
          new constraints::ConstantFunction (
            configSpace_->neutral(), 0, 0,
            "LockedJoint " + jointName()
            ));
    }
  } // namespace core
} // namespace hpp
