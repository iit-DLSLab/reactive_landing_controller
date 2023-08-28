// Based on python_interface.cpp from unitree_legged_sdk in motion_imitation

#include "go1_hal/go1_hal.h"

#include <array>
#include <math.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>


using namespace UNITREE_LEGGED_SDK;
// using namespace go1;
// namespace ULS = UNITREE_LEGGED_SDK;


namespace py = pybind11;

// TODO: Expose all of comm.h and the RobotInterface Class.

PYBIND11_MODULE(pygo1_hal, m) {
  m.doc() = R"pbdoc(
        Go1 HAL Python Bindings
        -----------------------
        .. currentmodule:: go1_hal
        .. autosummary::
           :toctree: _generate
    )pbdoc";
  
  // Low Level Interface
  py::class_<go1hal::LowLevelInterface>(m, "LowLevelInterface")
    .def(py::init<>())
    .def("receive_observation", &go1hal::LowLevelInterface::ReceiveObservation)
    .def("send_command", &go1hal::LowLevelInterface::SendCommand)
    .def("send_low_command", &go1hal::LowLevelInterface::SendLowCmd);

  // Unitree's comm.h
  py::class_<BmsCmd>(m, "BmsCmd")
    .def(py::init<>())
    .def_readwrite("off", &BmsCmd::off)
    .def_readwrite("reserve", &BmsCmd::reserve);
    
  py::class_<BmsState>(m, "BmsState")
    .def(py::init<>())
    .def_readwrite("version_h", &BmsState::version_h)
    .def_readwrite("version_l", &BmsState::version_l)
    .def_readwrite("bms_status", &BmsState::bms_status)
    .def_readwrite("SOC", &BmsState::SOC)
    .def_readwrite("current", &BmsState::current)
    .def_readwrite("cycle", &BmsState::cycle)
    .def_readwrite("BQ_NTC", &BmsState::BQ_NTC)
    .def_readwrite("MCU_NTC", &BmsState::MCU_NTC)
    .def_readwrite("cell_vol", &BmsState::cell_vol);
    
  py::class_<Cartesian>(m, "Cartesian")
    .def(py::init<>())
    .def_readwrite("x", &Cartesian::x)
    .def_readwrite("y", &Cartesian::y)
    .def_readwrite("z", &Cartesian::z);

  py::class_<IMU>(m, "IMU")
    .def(py::init<>())
    .def_readwrite("quaternion", &IMU::quaternion)
    .def_readwrite("gyroscope", &IMU::gyroscope)
    .def_readwrite("accelerometer", &IMU::accelerometer)
    .def_readwrite("rpy", &IMU::rpy)
    .def_readwrite("temperature", &IMU::temperature);

  py::class_<LED>(m, "LED")
    .def(py::init<>())
    .def_readwrite("r", &LED::r)
    .def_readwrite("g", &LED::g)
    .def_readwrite("b", &LED::b);

  py::class_<MotorState>(m, "MotorState")
    .def(py::init<>())
    .def_readwrite("mode", &MotorState::mode)
    .def_readwrite("q", &MotorState::q)
    .def_readwrite("dq", &MotorState::dq)
    .def_readwrite("ddq", &MotorState::ddq)
    .def_readwrite("tauEst", &MotorState::tauEst)
    .def_readwrite("q_raw", &MotorState::q_raw)
    .def_readwrite("dq_raw", &MotorState::dq_raw)
    .def_readwrite("ddq_raw", &MotorState::ddq_raw)
    .def_readwrite("temperature", &MotorState::temperature)
    .def_readwrite("reserve", &MotorState::reserve);
  
  py::class_<MotorCmd>(m, "MotorCmd")
    .def(py::init<>())
    .def_readwrite("mode", &MotorCmd::mode)
    .def_readwrite("q", &MotorCmd::q)
    .def_readwrite("dq", &MotorCmd::dq)
    .def_readwrite("tau", &MotorCmd::tau)
    .def_readwrite("Kp", &MotorCmd::Kp)
    .def_readwrite("Kd", &MotorCmd::Kd)
    .def_readwrite("reserve", &MotorCmd::reserve);
  
  py::class_<LowState>(m, "LowState")
    .def(py::init<>())
    .def_readwrite("head", &LowState::head)
    .def_readwrite("levelFlag", &LowState::levelFlag)
    .def_readwrite("frameReserve", &LowState::frameReserve)
    .def_readwrite("SN", &LowState::SN)
    .def_readwrite("version", &LowState::version)
    .def_readwrite("bandWidth", &LowState::bandWidth)
    .def_readwrite("imu", &LowState::imu)
    .def_readwrite("motorState", &LowState::motorState)
    .def_readwrite("footForce", &LowState::footForce)
    .def_readwrite("footForceEst", &LowState::footForceEst)
    .def_readwrite("tick", &LowState::tick)
    .def_readwrite("wirelessRemote", &LowState::wirelessRemote)
    .def_readwrite("reserve", &LowState::reserve)
    .def_readwrite("crc", &LowState::crc);
  
  py::class_<LowCmd>(m, "LowCmd")
    .def(py::init<>())
    .def_readwrite("head", &LowCmd::head)
    .def_readwrite("levelFlag", &LowCmd::levelFlag)
    .def_readwrite("frameReserve", &LowCmd::frameReserve)
    .def_readwrite("SN", &LowCmd::SN)
    .def_readwrite("version", &LowCmd::version)
    .def_readwrite("bandWidth", &LowCmd::bandWidth)
    .def_readwrite("motorCmd", &LowCmd::motorCmd)
    .def_readwrite("bms", &LowCmd::bms)
    .def_readwrite("wirelessRemote", &LowCmd::wirelessRemote)
    .def_readwrite("reserve", &LowCmd::reserve)
    .def_readwrite("crc", &LowCmd::crc);
  
  py::class_<HighState>(m, "HighState")
    .def(py::init<>())
    .def_readwrite("head", &HighState::head)
    .def_readwrite("levelFlag", &HighState::levelFlag)
    .def_readwrite("frameReserve", &HighState::frameReserve)
    .def_readwrite("SN", &HighState::SN)
    .def_readwrite("version", &HighState::version)
    .def_readwrite("bandWidth", &HighState::bandWidth)
    .def_readwrite("imu", &HighState::imu)
    .def_readwrite("motorState", &HighState::motorState)
    .def_readwrite("bms", &HighState::bms)
    .def_readwrite("footForce", &HighState::footForce)
    .def_readwrite("footForceEst", &HighState::footForceEst)
    .def_readwrite("mode", &HighState::mode)
    .def_readwrite("progress", &HighState::progress)
    .def_readwrite("gaitType", &HighState::gaitType)
    .def_readwrite("footRaiseHeight", &HighState::footRaiseHeight)
    .def_readwrite("position", &HighState::position)
    .def_readwrite("bodyHeight", &HighState::bodyHeight)
    .def_readwrite("velocity", &HighState::velocity)
    .def_readwrite("yawSpeed", &HighState::yawSpeed)
    .def_readwrite("rangeObstacle", &HighState::rangeObstacle)
    .def_readwrite("footPosition2Body", &HighState::footPosition2Body)
    .def_readwrite("footSpeed2Body", &HighState::footSpeed2Body)
    .def_readwrite("wirelessRemote", &HighState::wirelessRemote)
    .def_readwrite("reserve", &HighState::reserve)
    .def_readwrite("crc", &HighState::crc);
  
  py::class_<HighCmd>(m, "HighCmd")
    .def(py::init<>())
    .def_readwrite("head", &HighCmd::head)
    .def_readwrite("levelFlag", &HighCmd::levelFlag)
    .def_readwrite("frameReserve", &HighCmd::frameReserve)
    .def_readwrite("SN", &HighCmd::SN)
    .def_readwrite("version", &HighCmd::version)
    .def_readwrite("bandWidth", &HighCmd::bandWidth)
    .def_readwrite("mode", &HighCmd::mode)   
    .def_readwrite("gaitType", &HighCmd::gaitType)
    .def_readwrite("speedLevel", &HighCmd::speedLevel)
    .def_readwrite("footRaiseHeight", &HighCmd::footRaiseHeight)
    .def_readwrite("bodyHeight", &HighCmd::bodyHeight)
    .def_readwrite("position", &HighCmd::position)
    .def_readwrite("euler", &HighCmd::euler)
    .def_readwrite("velocity", &HighCmd::velocity)
    .def_readwrite("yawSpeed", &HighCmd::yawSpeed)
    .def_readwrite("bms", &HighCmd::bms)
    .def_readwrite("led", &HighCmd::led)
    .def_readwrite("wirelessRemote", &HighCmd::wirelessRemote)
    .def_readwrite("reserve", &HighCmd::reserve)
    .def_readwrite("crc", &HighCmd::crc);
  
  py::class_<UDPState>(m, "UDPState")
    .def(py::init<>())
    .def_readwrite("TotalCount", &UDPState::TotalCount)
    .def_readwrite("SendCount", &UDPState::SendCount)
    .def_readwrite("RecvCount", &UDPState::RecvCount)
    .def_readwrite("SendError", &UDPState::SendError)
    .def_readwrite("FlagError", &UDPState::FlagError)
    .def_readwrite("RecvCRCError", &UDPState::RecvCRCError)
    .def_readwrite("RecvLoseError", &UDPState::RecvLoseError);
  
  // Unitree's constants
  m.attr("FR_0") = FR_0;
  m.attr("FR_1") = FR_1;      
  m.attr("FR_2") = FR_2;      
  m.attr("FL_0") = FL_0;
  m.attr("FL_1") = FL_1;
  m.attr("FL_2") = FL_2;
  m.attr("RR_0") = RR_0;
  m.attr("RR_1") = RR_1;
  m.attr("RR_2") = RR_2;
  m.attr("RL_0") = RL_0;
  m.attr("RL_1") = RL_1;
  m.attr("RL_2") = RL_2;
  
  #ifdef VERSION_INFO
    m.attr("__version__") = VERSION_INFO;
  #else
    m.attr("__version__") = "dev";
  #endif

  m.attr("TEST") = py::int_(int(42));
}
