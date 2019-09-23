// Copyright (c) 2018, Joseph Mirabel
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

#define BOOST_TEST_MODULE configuration_shooters
#include <pinocchio/fwd.hpp>
#include <boost/test/included/unit_test.hpp>

#include <hpp/core/configuration-shooter/uniform.hh>
#include <hpp/core/configuration-shooter/gaussian.hh>

#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/pinocchio/simple-device.hh>

using hpp::pinocchio::DevicePtr_t;

using namespace hpp::core::configurationShooter;

namespace pin_test = hpp::pinocchio::unittest;

template <typename CS_t>
void basic_test (CS_t cs, DevicePtr_t robot)
{
  hpp::core::Configuration_t q;
  for (int i = 0; i < 10; ++i)
  {
    cs->shoot(q);
    hpp::pinocchio::ArrayXb unused(robot->numberDof());
    BOOST_CHECK(!hpp::pinocchio::saturate(robot, q, unused));
  }
}

BOOST_AUTO_TEST_CASE (uniform)
{
  DevicePtr_t robot = pin_test::makeDevice(pin_test::HumanoidSimple);
  UniformPtr_t cs = Uniform::create (robot);

  basic_test (cs, robot);
}

BOOST_AUTO_TEST_CASE (gaussian)
{
  DevicePtr_t robot = pin_test::makeDevice(pin_test::HumanoidSimple);

  GaussianPtr_t cs = Gaussian::create (robot);

  basic_test (cs, robot);

  cs->sigma (0);

  hpp::core::Configuration_t q;
  cs->shoot(q);
  BOOST_CHECK(q.isApprox(cs->center()));
}
