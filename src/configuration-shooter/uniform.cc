//
// Copyright (c) 2018 CNRS
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

# include <hpp/core/configuration-shooter/uniform.hh>

namespace hpp {
  namespace core {
    namespace configurationShooter {

      ConfigurationPtr_t Uniform::shoot () const
      {
        size_type extraDim = robot_->extraConfigSpace ().dimension ();
        size_type offset = robot_->configSize () - extraDim;

        Configuration_t config(robot_->configSize ());
        config.head (offset) = se3::randomConfiguration(robot_->model());

        // Shoot extra configuration variables
        for (size_type i=0; i<extraDim; ++i) {
          value_type lower = robot_->extraConfigSpace ().lower (i);
          value_type upper = robot_->extraConfigSpace ().upper (i);
          value_type range = upper - lower;
          if ((range < 0) ||
              (range == std::numeric_limits<double>::infinity())) {
            std::ostringstream oss
              ("Cannot uniformy sample extra config variable ");
            oss << i << ". min = " <<lower<< ", max = " << upper << std::endl;
            throw std::runtime_error (oss.str ());
          }
          config [offset + i] = lower + (upper - lower) * rand ()/RAND_MAX;
        }
        return boost::make_shared<Configuration_t> (config);
      }

    } // namespace configurationShooter
  } // namespace core
} // namespace hpp
