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
// of MERCHANTABILITY or FITNESolidSolidSS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-core  If not, see
// <http://www.gnu.org/licenses/>.

#ifndef HPP_CORE_BASIC_CONTINUOUS_VALIDATION_HH
# define HPP_CORE_BASIC_CONTINUOUS_VALIDATION_HH

# include <hpp/pinocchio/pool.hh>

# include <hpp/core/fwd.hh>
# include <hpp/core/obstacle-user.hh>
# include <hpp/core/path.hh>
# include <hpp/core/path-validation.hh>
# include <hpp/core/path-validation-report.hh>
# include <hpp/core/continuous-validation/interval-validation.hh>
# include <hpp/core/continuous-validation.hh>

namespace hpp {
  namespace core {
    namespace basic {
//       using continuousValidation::IntervalValidation;
//       using continuousValidation::IntervalValidationPtr_t;

      class HPP_CORE_DLLAPI ContinuousValidationBasic :
        public hpp::core::ContinuousValidation
      {
        public:
          class Initialize : public hpp::core::ContinuousValidation::Initialize
          {
          public:
            Initialize(ContinuousValidation& owner);
            /// Initialize collision pairs between bodies of the robot
            /// Iterate over all collision pairs of the robot, and for each one,
            /// \li create an instance of continuousValidation::SolidSolidCollision,
            /// \li add it to class ContinuousValidation using method
            ///     ContinuousValidation::addIntervalValidation.
            virtual void doExecute() const;
          }; // class Initialize
          virtual void initialize();

          // virtual bool validate (const PathPtr_t& path, bool reverse,
          //     PathPtr_t& validPart,
          //     PathValidationReportPtr_t& report);
          template <class Delegate> void add(const Delegate& delegate);

        protected:
          typedef hpp::core::continuousValidation::IntervalValidations_t IntervalValidations_t;
          /// Constructor
          /// \param robot the robot for which validation is performed,
          /// \param tolerance maximal penetration allowed.
          ContinuousValidationBasic (const DevicePtr_t& robot,
              const value_type& tolerance);
        private:
          std::vector<ContinuousValidationBasic::Initialize> initialize_;
      }; // class ContinuousValidationBasic

      template <>
      void ContinuousValidationBasic::add<ContinuousValidationBasic::Initialize>
      (const ContinuousValidationBasic::Initialize& delegate);

    } // namespace basic
  } // namespace core
} // namespace hpp
#endif // HPP_CORE_CONTINUOUS_VALIDATION_HH
