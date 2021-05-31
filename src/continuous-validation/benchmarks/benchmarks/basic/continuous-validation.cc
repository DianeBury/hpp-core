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

      ContinuousValidationBasic::ContinuousValidationBasic(const DevicePtr_t &robot, const value_type &tolerance):
        hpp::core::ContinuousValidation(robot, tolerance)
      {
      }

      /// Validate interval centered on a path parameter
      /// \param intervalValidations a reference to the pair with smallest interval.
      /// \param config Configuration at abscissa tmin on the path.
      /// \param t parameter value in the path interval of definition
      /// \retval interval interval validated for all validation elements
      /// \retval report reason why the interval is not valid,
      /// \return true if the configuration is collision free for this parameter
      ///         value, false otherwise.
      bool ContinuousValidationBasic::validateConfiguration
      (IntervalValidations_t& intervalValidations,
          const Configuration_t &config, const value_type &t,
          interval_t &interval, PathValidationReportPtr_t &report)
      {
        interval.first = -std::numeric_limits <value_type>::infinity ();
        interval.second = std::numeric_limits <value_type>::infinity ();
        hpp::pinocchio::DeviceSync robot (robot_);
        robot.currentConfiguration (config);
        robot.computeForwardKinematics();
        robot.updateGeometryPlacements();
        IntervalValidations_t::iterator smallestInterval
          (intervalValidations.begin());
        if (!validateIntervals
              (intervalValidations, t, interval, report,
              smallestInterval, robot.d()))
          return false;
        // Put the smallest interval first so that, at next iteration,
        // collision pairs with large interval are not computed.
        // if (intervalValidations.size() > 1 &&
        //     smallestInterval != intervalValidations.begin())
        //   std::iter_swap (intervalValidations.begin(), smallestInterval);
        return true;
      }

    } // namespace basic
  } // namespace core
} // namespace hpp
