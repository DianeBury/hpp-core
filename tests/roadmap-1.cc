// Copyright (C) 2014 LAAS-CNRS
// Author: Mathieu Geisert
//
// This file is part of the hpp-core.
//
// hpp-core is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// test-hpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with hpp-core.  If not, see <http://www.gnu.org/licenses/>.

#include <boost/assign.hpp>

#include <hpp/util/debug.hh>
#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/configuration.hh>
#include <hpp/core/fwd.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/core/problem.hh>
#include <hpp/core/weighed-distance.hh>
#include <hpp/core/connected-component.hh>
#include <hpp/core/node.hh>
#include <hpp/core/nearest-neighbor.hh>

#include <hpp/core/steering-method/straight.hh>
#include <hpp/core/weighed-distance.hh>
#include <pinocchio/multibody/joint/joint-variant.hpp>
#include <pinocchio/multibody/geometry.hpp>





#define BOOST_TEST_MODULE roadmap-1
#include <boost/test/included/unit_test.hpp>

using hpp::pinocchio::Configuration_t;
using hpp::core::ConfigurationPtr_t;
using hpp::pinocchio::JointPtr_t;
using hpp::pinocchio::Device;
using hpp::pinocchio::DevicePtr_t;
using hpp::core::Problem;
using hpp::core::steeringMethod::Straight;
using hpp::core::steeringMethod::StraightPtr_t;
using hpp::core::RoadmapPtr_t;
using hpp::core::Roadmap;
using hpp::core::NodePtr_t;
using hpp::core::WeighedDistance;
using namespace hpp::core;
using namespace hpp::pinocchio;

using ::se3::JointModelPX;
using ::se3::JointModelPY;
using ::se3::JointModelPZ;
using ::se3::JointIndex;

BOOST_AUTO_TEST_SUITE( test_hpp_core )

void addEdge (const hpp::core::RoadmapPtr_t& r,
	      const hpp::core::SteeringMethod& sm,
	      const std::vector <NodePtr_t>& nodes,
	      std::size_t i, std::size_t j)
{
  r->addEdge (nodes [i], nodes [j], sm (*(nodes [i]->configuration ()),
					*(nodes [j]->configuration ())));
}

BOOST_AUTO_TEST_CASE (Roadmap1) {
  // Build robot
 /* DevicePtr_t robot = Device::create("robot");
  JointPtr_t xJoint = new JointTranslation <1> (fcl::Transform3f());
  xJoint->isBounded(0,1);
  xJoint->lowerBound(0,-3.);
  xJoint->upperBound(0,3.);
  JointPtr_t yJoint = new JointTranslation <1>
    (fcl::Transform3f(fcl::Quaternion3f (sqrt (2)/2, 0, 0, sqrt(2)/2)));
  yJoint->isBounded(0,1);
  yJoint->lowerBound(0,-3.);
  yJoint->upperBound(0,3.);

  robot->rootJoint (xJoint);
  xJoint->addChildJoint (yJoint);
*/
  DevicePtr_t robot = Device::create("robot");
  const std::string& name = robot->name ();
  ModelPtr_t m = ModelPtr_t(new ::se3::Model());
  GeomModelPtr_t gm = GeomModelPtr_t(new ::se3::GeometryModel());
  robot->model(m);
  robot->geomModel(gm);
  Transform3f mat; mat.setIdentity ();

  JointModelPX::TangentVector_t max_effort = JointModelPX::TangentVector_t::Constant(JointModelPX::NV,std::numeric_limits<double>::max());
  JointModelPX::TangentVector_t max_velocity = JointModelPX::TangentVector_t::Constant(JointModelPX::NV,std::numeric_limits<double>::max());
  JointModelPX::ConfigVector_t lower_position = JointModelPY::ConfigVector_t::Constant(-3);
  JointModelPX::ConfigVector_t upper_position = JointModelPY::ConfigVector_t::Constant(3);

  JointIndex idJoint = robot->model().addJoint(0,JointModelPX(), mat,name + "_x",max_effort,max_velocity,lower_position,upper_position);
  robot->model().addJoint(idJoint,JointModelPY(), mat,name + "_y",max_effort,max_velocity,lower_position,upper_position);

  robot->createData();
  robot->createGeomData();

  // Create steering method
  Problem p (robot);
  StraightPtr_t sm = Straight::create (p);
  // create roadmap
  hpp::core::DistancePtr_t distance (WeighedDistance::createWithWeight
				     (robot, boost::assign::list_of (1)(1)));
  RoadmapPtr_t r = Roadmap::create (distance, robot);

  std::vector <NodePtr_t> nodes;

  // nodes [0]
  ConfigurationPtr_t q (new Configuration_t (robot->configSize ()));
  (*q) [0] = 0; (*q) [1] = 0;
  // Set init node
  r->initNode (q);
  nodes.push_back (r->initNode ());

  // nodes [1]
  q = ConfigurationPtr_t (new Configuration_t (robot->configSize ()));
  (*q) [0] = 1; (*q) [1] = 0;
  nodes.push_back (r->addNode (q));

  // nodes [2]
  q = ConfigurationPtr_t (new Configuration_t (robot->configSize ()));
  (*q) [0] = 0.5; (*q) [1] = 0.9;
  nodes.push_back (r->addNode (q));

  // nodes [3]
  q = ConfigurationPtr_t (new Configuration_t (robot->configSize ()));
  (*q) [0] = -0.1; (*q) [1] = -0.9;
  nodes.push_back (r->addNode (q));

  // nodes [4]
  q = ConfigurationPtr_t (new Configuration_t (robot->configSize ()));
  (*q) [0] = 1.5; (*q) [1] = 2.9;
  nodes.push_back (r->addNode (q));

  // nodes [5]
  q = ConfigurationPtr_t (new Configuration_t (robot->configSize ()));
  (*q) [0] = 2.5; (*q) [1] = 2.9;
  nodes.push_back (r->addNode (q));
  r->addGoalNode (nodes [5]->configuration ());

  std::cout << *r << std::endl;
  BOOST_CHECK_EQUAL (r->connectedComponents ().size (), 6);
  for (std::size_t i=0; i < nodes.size (); ++i) {
    for (std::size_t j=i+1; j < nodes.size (); ++j) {
      BOOST_CHECK_MESSAGE (nodes [i]->connectedComponent () !=
			   nodes [j]->connectedComponent (),
			   "nodes " << i << " and " << j << "are in the same"
			   << " connected component");
    }
  }
  BOOST_CHECK (!r->pathExists ());
  // 0 -> 1
  addEdge (r, *sm, nodes, 0, 1);
  BOOST_CHECK_EQUAL (r->connectedComponents ().size (), 6);
  for (std::size_t i=0; i < nodes.size (); ++i) {
    for (std::size_t j=i+1; j < nodes.size (); ++j) {
      BOOST_CHECK_MESSAGE (nodes [i]->connectedComponent () !=
			   nodes [j]->connectedComponent (),
			   "nodes " << i << " and " << j << "are in the same"
			   << " connected component");
    }
  }
  BOOST_CHECK (!r->pathExists ());
  std::cout << *r << std::endl;

  // 1 -> 0
  addEdge (r, *sm, nodes, 1, 0);

  BOOST_CHECK_EQUAL (r->connectedComponents ().size (), 5);
  BOOST_CHECK (nodes [0]->connectedComponent () == 
	       nodes [1]->connectedComponent ());
  for (std::size_t i=1; i < nodes.size (); ++i) {
    for (std::size_t j=i+1; j < nodes.size (); ++j) {
      BOOST_CHECK_MESSAGE (nodes [i]->connectedComponent () !=
			   nodes [j]->connectedComponent (),
			   "nodes " << i << " and " << j << "are in the same"
			   << " connected component");
    }
  }
  BOOST_CHECK (!r->pathExists ());
  std::cout << *r << std::endl; 
  // 1 -> 2
  addEdge (r, *sm, nodes, 1, 2);
  BOOST_CHECK_EQUAL (r->connectedComponents ().size (), 5);
  BOOST_CHECK (nodes [0]->connectedComponent () == 
	       nodes [1]->connectedComponent ());
  for (std::size_t i=1; i < nodes.size (); ++i) {
    for (std::size_t j=i+1; j < nodes.size (); ++j) {
      BOOST_CHECK_MESSAGE (nodes [i]->connectedComponent () !=
			   nodes [j]->connectedComponent (),
			   "nodes " << i << " and " << j << "are in the same"
			   << " connected component");
    }
  }
  BOOST_CHECK (!r->pathExists ());
  std::cout << *r << std::endl;

  // 2 -> 0
  addEdge (r, *sm, nodes, 2, 0);
  BOOST_CHECK_EQUAL (r->connectedComponents ().size (), 4);
  BOOST_CHECK (nodes [0]->connectedComponent () == 
	       nodes [1]->connectedComponent ());
  BOOST_CHECK (nodes [0]->connectedComponent () == 
	       nodes [2]->connectedComponent ());
  for (std::size_t i=2; i < nodes.size (); ++i) {
    for (std::size_t j=i+1; j < nodes.size (); ++j) {
      BOOST_CHECK_MESSAGE (nodes [i]->connectedComponent () !=
			   nodes [j]->connectedComponent (),
			   "nodes " << i << " and " << j << "are in the same"
			   << " connected component");
    }
  }
  BOOST_CHECK (!r->pathExists ());
  std::cout << *r << std::endl;

  // 2 -> 3
  addEdge (r, *sm, nodes, 2, 3);
  BOOST_CHECK_EQUAL (r->connectedComponents ().size (), 4);
  BOOST_CHECK (nodes [0]->connectedComponent () == 
	       nodes [1]->connectedComponent ());
  BOOST_CHECK (nodes [0]->connectedComponent () == 
	       nodes [2]->connectedComponent ());
  for (std::size_t i=2; i < nodes.size (); ++i) {
    for (std::size_t j=i+1; j < nodes.size (); ++j) {
      BOOST_CHECK_MESSAGE (nodes [i]->connectedComponent () !=
			   nodes [j]->connectedComponent (),
			   "nodes " << i << " and " << j << "are in the same"
			   << " connected component");
    }
  }
  BOOST_CHECK (!r->pathExists ());
  std::cout << *r << std::endl;

  // 2 -> 4
  addEdge (r, *sm, nodes, 2, 4);
  BOOST_CHECK_EQUAL (r->connectedComponents ().size (), 4);
  BOOST_CHECK (nodes [0]->connectedComponent () == 
	       nodes [1]->connectedComponent ());
  BOOST_CHECK (nodes [0]->connectedComponent () == 
	       nodes [2]->connectedComponent ());
  for (std::size_t i=2; i < nodes.size (); ++i) {
    for (std::size_t j=i+1; j < nodes.size (); ++j) {
      BOOST_CHECK_MESSAGE (nodes [i]->connectedComponent () !=
			   nodes [j]->connectedComponent (),
			   "nodes " << i << " and " << j << "are in the same"
			   << " connected component");
    }
  }
  BOOST_CHECK (!r->pathExists ());
  std::cout << *r << std::endl;

  // 3 -> 5
  addEdge (r, *sm, nodes, 3, 5);
  BOOST_CHECK_EQUAL (r->connectedComponents ().size (), 4);
  BOOST_CHECK (nodes [0]->connectedComponent () == 
	       nodes [1]->connectedComponent ());
  BOOST_CHECK (nodes [0]->connectedComponent () == 
	       nodes [2]->connectedComponent ());
  for (std::size_t i=2; i < nodes.size (); ++i) {
    for (std::size_t j=i+1; j < nodes.size (); ++j) {
      BOOST_CHECK_MESSAGE (nodes [i]->connectedComponent () !=
			   nodes [j]->connectedComponent (),
			   "nodes " << i << " and " << j << "are in the same"
			   << " connected component");
    }
  }
  BOOST_CHECK (r->pathExists ());
  std::cout << *r << std::endl;

  // 4 -> 5
  addEdge (r, *sm, nodes, 4, 5);
  BOOST_CHECK_EQUAL (r->connectedComponents ().size (), 4);
  BOOST_CHECK (nodes [0]->connectedComponent () == 
	       nodes [1]->connectedComponent ());
  BOOST_CHECK (nodes [0]->connectedComponent () == 
	       nodes [2]->connectedComponent ());
  for (std::size_t i=2; i < nodes.size (); ++i) {
    for (std::size_t j=i+1; j < nodes.size (); ++j) {
      BOOST_CHECK_MESSAGE (nodes [i]->connectedComponent () !=
			   nodes [j]->connectedComponent (),
			   "nodes " << i << " and " << j << "are in the same"
			   << " connected component");
    }
  }
  BOOST_CHECK (r->pathExists ());
  std::cout << *r << std::endl;

  // 5 -> 0
  addEdge (r, *sm, nodes, 5, 0);
  BOOST_CHECK_EQUAL (r->connectedComponents ().size (), 1);
  BOOST_CHECK (nodes [0]->connectedComponent () == 
	       nodes [1]->connectedComponent ());
  BOOST_CHECK (nodes [0]->connectedComponent () == 
	       nodes [2]->connectedComponent ());
  BOOST_CHECK (nodes [0]->connectedComponent () == 
	       nodes [3]->connectedComponent ());
  BOOST_CHECK (nodes [0]->connectedComponent () == 
	       nodes [4]->connectedComponent ());
  BOOST_CHECK (nodes [0]->connectedComponent () == 
	       nodes [5]->connectedComponent ());
  BOOST_CHECK (r->pathExists ());
  std::cout << *r << std::endl;

  // Check that memory if well deallocated.
  std::set<ConnectedComponentWkPtr_t> ccs;
  for (ConnectedComponents_t::const_iterator _cc = r->connectedComponents().begin();
      _cc != r->connectedComponents().end(); ++_cc)
    ccs.insert(*_cc);

  r.reset();
  for (std::set<ConnectedComponentWkPtr_t>::const_iterator _cc = ccs.begin();
      _cc != ccs.begin(); ++_cc) {
    BOOST_CHECK (! _cc->lock());
  }
}

BOOST_AUTO_TEST_CASE (nearestNeighbor) {
  // Build robot
 /* DevicePtr_t robot = Device::create("robot");
  JointPtr_t xJoint = new JointTranslation <1> (fcl::Transform3f());
  xJoint->isBounded(0,1);
  xJoint->lowerBound(0,-3.);
  xJoint->upperBound(0,3.);
  JointPtr_t yJoint = new JointTranslation <1>
    (fcl::Transform3f(fcl::Quaternion3f (sqrt (2)/2, 0, 0, sqrt(2)/2)));
  yJoint->isBounded(0,1);
  yJoint->lowerBound(0,-3.);
  yJoint->upperBound(0,3.);

  robot->rootJoint (xJoint);
  xJoint->addChildJoint (yJoint);*/

  DevicePtr_t robot = Device::create("robot");
  const std::string& name = robot->name ();
  ModelPtr_t m = ModelPtr_t(new ::se3::Model());
  GeomModelPtr_t gm = GeomModelPtr_t(new ::se3::GeometryModel());
  robot->model(m);
  robot->geomModel(gm);
  Transform3f mat; mat.setIdentity ();

  JointModelPX::TangentVector_t max_effort = JointModelPX::TangentVector_t::Constant(JointModelPX::NV,std::numeric_limits<double>::max());
  JointModelPX::TangentVector_t max_velocity = JointModelPX::TangentVector_t::Constant(JointModelPX::NV,std::numeric_limits<double>::max());
  JointModelPX::ConfigVector_t lower_position = JointModelPY::ConfigVector_t::Constant(-3);
  JointModelPX::ConfigVector_t upper_position = JointModelPY::ConfigVector_t::Constant(3);


  JointIndex idJoint = robot->model().addJoint(0,::se3::JointModelPX(), mat,name + "_x",max_effort,max_velocity,lower_position,upper_position);
  robot->model().addJoint(idJoint,::se3::JointModelPY(), mat,name + "_y",max_effort,max_velocity,lower_position,upper_position);

  robot->createData();
  robot->createGeomData();


  // Create steering method
  Problem p (robot);
  StraightPtr_t sm = Straight::create (p);
  // create roadmap
  hpp::core::DistancePtr_t distance (WeighedDistance::createWithWeight
				     (robot, boost::assign::list_of (1)(1)));
  RoadmapPtr_t r = Roadmap::create (distance, robot);

  std::vector <NodePtr_t> nodes;

  // nodes [0]
  ConfigurationPtr_t q (new Configuration_t (robot->configSize ()));
  (*q) [0] = 0; (*q) [1] = 0;
  nodes.push_back (r->addNode (q));

  // nodes [1]
  q = ConfigurationPtr_t (new Configuration_t (robot->configSize ()));
  (*q) [0] = 1; (*q) [1] = 0;
  nodes.push_back (r->addNode (q));

  // nodes [2]
  q = ConfigurationPtr_t (new Configuration_t (robot->configSize ()));
  (*q) [0] = 0.5; (*q) [1] = 0.9;
  nodes.push_back (r->addNode (q));

  // nodes [3]
  q = ConfigurationPtr_t (new Configuration_t (robot->configSize ()));
  (*q) [0] = -0.1; (*q) [1] = -0.9;
  nodes.push_back (r->addNode (q));

  // nodes [4]
  q = ConfigurationPtr_t (new Configuration_t (robot->configSize ()));
  (*q) [0] = 1.5; (*q) [1] = 2.9;
  nodes.push_back (r->addNode (q));

  // nodes [5]
  q = ConfigurationPtr_t (new Configuration_t (robot->configSize ()));
  (*q) [0] = 2.5; (*q) [1] = 2.9;
  nodes.push_back (r->addNode (q));
  r->addGoalNode (nodes [5]->configuration ());

  // nodes [6]
  q = ConfigurationPtr_t (new Configuration_t (robot->configSize ()));
  (*q) [0] = 0; (*q) [1] = 0.2;
  nodes.push_back (r->addNode (q));
  r->addGoalNode (nodes [6]->configuration ());

  // 0 -> 1
  addEdge (r, *sm, nodes, 0, 1);
  // 1 -> 0
  addEdge (r, *sm, nodes, 1, 0);
  // 1 -> 2
  addEdge (r, *sm, nodes, 1, 2);
  // 2 -> 0
  addEdge (r, *sm, nodes, 2, 0);
  // 0 -> 3
  addEdge (r, *sm, nodes, 0, 3);
  // 3 -> 2
  addEdge (r, *sm, nodes, 3, 2);

  hpp::pinocchio::value_type dist;
  using hpp::core::Nodes_t;
  Nodes_t knearest = r->nearestNeighbor()->KnearestSearch
    (nodes[0]->configuration(), nodes[0]->connectedComponent (), 3, dist);
  for (Nodes_t::const_iterator it = knearest.begin (); it != knearest.end(); ++it) {
    BOOST_TEST_MESSAGE ("q = [" << (*it)->configuration()->transpose() << "] - dist : " << (*distance) (*nodes[0]->configuration(), *(*it)->configuration()));
  }
  for (std::vector<NodePtr_t>::const_iterator it = nodes.begin (); it != nodes.end(); ++it) {
    Configuration_t& q = *(*it)->configuration();
    BOOST_TEST_MESSAGE ("[" << q.transpose() << "] - dist : " << (*distance) (*nodes[0]->configuration(), q));
  }
}
BOOST_AUTO_TEST_SUITE_END()



