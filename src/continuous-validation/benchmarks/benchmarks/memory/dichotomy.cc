//
// Copyright (c) 2014 CNRS
// Authors: Florent Lamiraux
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

#include <hpp/core/continuous-validation/benchmarks/memory/dichotomy.hh>

#include <iterator>

#include <hpp/util/debug.hh>
#include <hpp/core/collision-path-validation-report.hh>
#include <hpp/core/straight-path.hh>
#include <hpp/core/path-vector.hh>

#include "continuous-validation/intervals.hh"
#include "continuous-validation/helper.hh"

namespace hpp {
  namespace core {
    namespace continuousValidation {
      namespace memory {
        using hpp::core::basic::ContinuousValidationBasic;

        DichotomyPtr_t
        Dichotomy::create (const DevicePtr_t& robot, const value_type& tolerance)
        {
          Dichotomy* ptr = new Dichotomy (robot, tolerance);
          DichotomyPtr_t shPtr (ptr);
          ptr->init(shPtr);
          ptr->initialize();
          return shPtr;
        }

        Dichotomy::~Dichotomy ()
        {
        }

        bool Dichotomy::validateStraightPath
        (IntervalValidations_t& bpc, const PathPtr_t& path, bool reverse, PathPtr_t& validPart,
        PathValidationReportPtr_t& report)
        {
          if (reverse) return validateStraightPath<true> (bpc, path, validPart, report);
          else         return validateStraightPath<false>(bpc, path, validPart, report);
        }

        template <bool reverse>
        bool Dichotomy::validateStraightPath
        (IntervalValidations_t& bodyPairCollisions, const PathPtr_t& path, PathPtr_t& validPart,
        PathValidationReportPtr_t& report)
        {
          // start by validating end of path
          bool finished = false;
          bool valid = true;
          setPath(bodyPairCollisions, path, reverse);
          Intervals validSubset;
          const interval_t& tr (path->timeRange());
          value_type t = first(tr, reverse);
          validSubset.unionInterval (std::make_pair (t,t));
          Configuration_t q (path->outputSize ());
          value_type t0, t1, tmin, tmax;
          while (!finished) {
            bool success = (*path) (q, t);
            PathValidationReportPtr_t pathReport;
            interval_t interval;
            if (!success) {
              report = PathValidationReportPtr_t (new PathValidationReport (t,
                    ValidationReportPtr_t(new ProjectionError())));
              valid = false;
            } else if (!validateConfiguration (bodyPairCollisions, q, t, interval, pathReport)) {
              report = pathReport;
              valid = false;
            } else {
              validSubset.unionInterval (interval);
            }
            finished = (!valid || validSubset.contains (tr));
            // Compute next parameter to check as middle of first non tested
            // interval
            t0 = second (begin(validSubset.list(), reverse), reverse);
            t1 = second (tr                                , reverse);
            if (validSubset.list().size() > 1)
              t1 = first (Nth(validSubset.list(), 1, reverse), reverse);
            t = .5* (t0 + t1);
          }
          if (!valid) {
            assert ( begin(validSubset.list (), reverse).first <=
                    first(tr                 , reverse));
            assert ( begin(validSubset.list (), reverse).second >=
                    first(tr                 , reverse));
            if (reverse) {
              tmin = validSubset.list ().rbegin ()->first;
              tmax = tr.second;
            } else {
              tmin = tr.first;
              tmax = validSubset.list ().begin ()->second;
            }
            validPart = path->extract (tmin, tmax);
            return false;
          } else {
            validPart = path;
            return true;
          }
          return false;
        }

        void Dichotomy::init(const DichotomyWkPtr_t weak)
        {
          ContinuousValidation::init (weak);
          weak_ = weak;
        }

        Dichotomy::Dichotomy (const DevicePtr_t& robot, const value_type& tolerance):
          ContinuousValidationBasic (robot, tolerance), weak_()
        {
          // Tolerance should be equal to 0, otherwise end of valid
          // sub-path might be in collision.
          if (tolerance != 0) {
            throw std::runtime_error ("Dichotomy path validation method does not"
                    "support penetration.");
          }
        }
      } // namespace memory
    } // namespace continuousValidation
  } // namespace core
} // namespace hpp
