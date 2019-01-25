#include "HardwareInterface.hpp"
#include <iostream>
#include <ros/ros.h>

HardwareInterface::HardwareInterface() : portName(""), baudrate(0), char_size(0)
{
}

HardwareInterface::HardwareInterface(const char* port, const uint32_t baudrate, const uint16_t char_size,
                                     const std::pair<uint16_t, uint16_t>& pwmRange)
{
  this->portName = port;
  this->pwmRange = pwmRange;
  this->baudrate = baudrate;
  this->char_size = char_size;
  initialiseSerial(port, baudrate, char_size);
}

HardwareInterface::~HardwareInterface()
{
  serial->close();
}

void HardwareInterface::initialiseSerial(const char* port, uint32_t baudrate, uint16_t char_size)
{
  serial = serial_port_ptr(new boost::asio::serial_port(ioservice, port));

  serial->set_option(boost::asio::serial_port_base::baud_rate(baudrate));
  serial->set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
  serial->set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
  serial->set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
  serial->set_option(boost::asio::serial_port::character_size(boost::asio::serial_port::character_size(char_size)));
}

void HardwareInterface::turnServo(const uint8_t servoNr, const uint32_t pwm, const uint32_t time)
{
  if (pwm < pwmRange.first || pwm > pwmRange.second)
  {
    ROS_WARN("%s", "Angle is out of servo angle range");
    return;
  }
  std::stringstream ss;
  ss << "#" << (int)servoNr << "P" << pwm << "T" << time << MSG_ENDING;
  write_some(ss.str());
}

void HardwareInterface::stop()
{
  for (size_t i = 0; i < 5; ++i)
  {
    std::string str("STOP" + std::to_string(i) + MSG_ENDING);
    write_some(str);
  }
}

HardwareInterface& HardwareInterface::operator=(const HardwareInterface& other)
{
  if (this != &other)
  {
    this->portName = other.portName;
    this->pwmRange = other.pwmRange;
    this->baudrate = other.baudrate;
    this->char_size = other.char_size;
    this->initialiseSerial(portName, baudrate, char_size);
  }
  return *this;
}

int HardwareInterface::write_some(const char* buf, const uint16_t size)
{
  boost::system::error_code ec;
  return (int)serial->write_some(boost::asio::buffer(buf, size), ec);
}

int HardwareInterface::write_some(const std::string& buf)
{
  return write_some(buf.c_str(), (uint16_t)buf.size());
}
