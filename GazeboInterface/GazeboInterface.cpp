#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include <iostream>
#include <boost/python.hpp>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>

using namespace boost::python;

//https://stackoverflow.com/questions/8009613/boost-python-not-supporting-parallelism
class releaseGIL{
  public:
    inline releaseGIL(){
        save_state = PyEval_SaveThread();
    }

    inline ~releaseGIL(){
        PyEval_RestoreThread(save_state);
    }
  private:
    PyThreadState *save_state;
};

struct ForceTorque {

  static float torque_x, torque_y, torque_z;
  static float force_x, force_y, force_z;
  //A flag to let the thread know it must stop
  bool stopped = false;

  // See /usr/include/gazebo-9/gazebo/msgs/wrench_stamped.pb.h
  static void wrenchCallback(ConstWrenchStampedPtr &wrenchMsg){
    torque_x  = wrenchMsg->wrench().torque().x();
    torque_y  = wrenchMsg->wrench().torque().y();
    torque_z  = wrenchMsg->wrench().torque().z();
    force_x   = wrenchMsg->wrench().force().x();
    force_y   = wrenchMsg->wrench().force().y();
    force_z   = wrenchMsg->wrench().force().z();
  }

  void stop(){
    this->stopped = true;
    gazebo::client::shutdown();
  }

  //Constructor
  ForceTorque(){
    torque_x = 0;
    torque_y = 0;
    torque_z = 0;
    force_x = 0;
    force_y = 0;
    force_z = 0;
  }

  void start(){
    // Load gazebo
    gazebo::client::setup();

    // Create our node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    gazebo::transport::SubscriberPtr sub = node->Subscribe("/gazebo/default/coro/wrist_3_joint/wrist_3_force_torque/wrench", &wrenchCallback);

    while (true){
      if(this->stopped) return;
      //Let other threads do their job. See releaseGIL definition.
      releaseGIL unlock = releaseGIL();
      //Do NOT use non-blocking gazebo::common::Time::MSleep, use this blocking call instead
      boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
    }
  }

  float getForceX(){ return force_x;}
  float getForceY(){ return force_y;}
  float getForceZ(){ return force_z;}
  float getTorqueX(){ return torque_x;}
  float getTorqueY(){ return torque_y;}
  float getTorqueZ(){ return torque_z;}
};

//Static variables needs to be declared so they can be found outside of their context
float ForceTorque::torque_x, ForceTorque::torque_y, ForceTorque::torque_z;
float ForceTorque::force_x, ForceTorque::force_y, ForceTorque::force_z;

BOOST_PYTHON_MODULE(ForceTorqueSensorInterface){
  PyEval_InitThreads();
  class_<ForceTorque>("ForceTorque", init<>() )
    .def("getForceX",   &ForceTorque::getForceX)
    .def("getForceY",   &ForceTorque::getForceY)
    .def("getForceZ",   &ForceTorque::getForceZ)
    .def("getTorqueX",  &ForceTorque::getTorqueX)
    .def("getTorqueY",  &ForceTorque::getTorqueY)
    .def("getTorqueZ",  &ForceTorque::getTorqueZ)
    .def("stop",        &ForceTorque::stop)
    .def("start",        &ForceTorque::start)
  ;
}
