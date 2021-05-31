//
// Copyright (c) 2014, 2015, 2016, 2017, 2018 CNRS
// Authors: Florent Lamiraux, Joseph Mirabel, Diane Bury
//
// This file is part of hpp-core
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
// hpp-core  If not, see
// <http://www.gnu.org/licenses/>.

#define HPP_DEBUG 1

#include <hpp/core/continuous-validation/benchmarks/basic/continuous-validation.hh>
#include <hpp/core/continuous-validation/benchmarks/basic/solid-solid-collision.hh>

#include <limits>
#include <pinocchio/multibody/geometry.hpp>
#include <hpp/util/debug.hh>
#include <hpp/core/collision-path-validation-report.hh>
#include <hpp/core/straight-path.hh>
#include <hpp/core/path-vector.hh>

#include <iterator>
namespace hpp {
  namespace core {
    namespace basic {
      using continuousValidation::IntervalValidations_t;
      using continuousValidation::basic::SolidSolidCollision;
      using continuousValidation::basic::BodyPairCollisionPtr_t;
      using continuousValidation::basic::BodyPairCollision;
      
      ContinuousValidationBasic::Initialize::Initialize
          (ContinuousValidation& owner):
            hpp::core::ContinuousValidation::Initialize(owner)
          {
          }
    
      typedef std::pair<pinocchio::JointIndex, pinocchio::JointIndex>
      JointIndexPair_t;
      struct JointIndexPairCompare_t
      {
        bool operator()(const JointIndexPair_t &p0, const JointIndexPair_t &p1)
          const
        {
          if (p0.first < p1.first)
            return true;
          if (p0.first > p1.first)
            return false;
          return (p0.second < p1.second);
        }
      };

      typedef std::map <JointIndexPair_t, BodyPairCollisionPtr_t,
                            JointIndexPairCompare_t> BodyPairCollisionMap_t;

      void ContinuousValidationBasic::Initialize::doExecute() const
      {
        DevicePtr_t robot = owner().robot();
        const pinocchio::GeomModel &gmodel = robot->geomModel();
        const pinocchio::GeomData  &gdata  = robot->geomData();
        JointPtr_t joint1, joint2;
        BodyPairCollisionMap_t bodyPairMap;
        for (std::size_t i = 0; i < gmodel.collisionPairs.size(); ++i)
        {
          if (!gdata.activeCollisionPairs[i]) continue;

          const ::pinocchio::CollisionPair &cp = gmodel.collisionPairs[i];
          JointIndexPair_t jp(gmodel.geometryObjects[cp.first].parentJoint,
                              gmodel.geometryObjects[cp.second].parentJoint);

          // Ignore pairs of bodies that are in the same joint.
          if (jp.first == jp.second)
            continue;

          BodyPairCollisionMap_t::iterator _bp = bodyPairMap.find(jp);

          if (_bp == bodyPairMap.end())
          {
            joint1 = Joint::create (robot, jp.first );
            joint2 = Joint::create (robot, jp.second);
            if (!joint2) joint2.swap (joint1);
            assert(joint2);
            continuousValidation::basic::SolidSolidCollisionPtr_t ss
              (basic::SolidSolidCollision::create(joint2, joint1, owner().tolerance()));
            owner().addIntervalValidation(ss);
            bodyPairMap[jp] = ss;
          }
          CollisionObjectConstPtr_t co1
            (new pinocchio::CollisionObject(robot, cp.first));
          CollisionObjectConstPtr_t co2
            (new pinocchio::CollisionObject(robot, cp.second));
          bodyPairMap[jp]->addCollisionPair( co1,co2 );
        }
      }

      ContinuousValidationBasic::ContinuousValidationBasic(const DevicePtr_t &robot, const value_type &tolerance):
        ContinuousValidation(robot, tolerance)
      {
        add<ContinuousValidationBasic::Initialize>(ContinuousValidationBasic::Initialize(*this));
      }

    void ContinuousValidationBasic::initialize()
    {      
      for(std::vector<Initialize>::const_iterator it(initialize_.begin());
          it != initialize_.end(); ++it) {
        it->doExecute();
      }
    }

    template <>
    void ContinuousValidationBasic::add<ContinuousValidationBasic::Initialize>
    (const ContinuousValidationBasic::Initialize& delegate)
    {
      initialize_.push_back(delegate);
    }

    } // namespace basic
  } // namespace core
} // namespace hpp
