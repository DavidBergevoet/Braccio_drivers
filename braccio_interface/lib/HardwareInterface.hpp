#ifndef HARDWARE_INTERFACE_H_
#define HARDWARE_INTERFACE_H_

#define MSG_ENDING "\r"

#include <boost/asio.hpp>
#include <iostream>

typedef boost::shared_ptr<boost::asio::serial_port> serial_port_ptr;

class HardwareInterface
{
public:
  // Constructors and destructor
  HardwareInterface();
  HardwareInterface(const char* port, const uint32_t baudrate, const uint16_t char_size = 8,
                    const std::pair<uint16_t, uint16_t>& pwmRange = std::pair<uint16_t, uint16_t>(500, 2500));
  ~HardwareInterface();

  /**
   * Description: Turn the servo with servoNr to a pwm with a speed and time
   * Parameters
   * servoNr: The nr of the servo (0 to 5)
   * pwm: the signal in pwm that is send to the servo
   * speed: the maximum speed at which the robot arm should move
   * time: The duration of the action
   */
  void turnServo(const uint8_t servoNr, const uint32_t pwm, const uint32_t time);

  /**
   * Description: It stops all the servos of the robotic arm
   */
  void stop();

  HardwareInterface& operator=(const HardwareInterface& other);

private:
  /**
   * Description: It initialises the serial port that is given to open communication
   * Parameters
   * port: The portname of the serial port that should be opened
   * baudrate: The baudrate at which the serial port should operate
   * char_size: The size of a character in bits
   */
  void initialiseSerial(const char* port, uint32_t baudrate, uint16_t char_size);

  /**
   * Description: This writes the characters to the serial port
   * Parameters
   * buf: This is the message which is send to the port
   * size: this is the length of the message
   * Return: This returns the state of the serial port
   */
  int write_some(const char* buf, const uint16_t size);

  /**
   * Description: This is an overloaded function of the write_some(const char* buf,const uint16_t size) function
   * Parameters
   * buf: This is the message which is send over the serial port
   * Return: This returns the state of the serial port
   */

  int write_some(const std::string& buf);

  const char* portName;
  std::pair<uint16_t, uint16_t> pwmRange;
  uint32_t baudrate;
  uint16_t char_size;
  boost::asio::io_service ioservice;
  serial_port_ptr serial;
};
#endif  // HARDWARE_INTERFACE_H_
