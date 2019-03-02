#ifndef HARDWARE_INTERFACE_H_
#define HARDWARE_INTERFACE_H_

#define MSG_ENDING "\r"

#include <boost/asio.hpp>
#include <iostream>

typedef boost::shared_ptr<boost::asio::serial_port> serial_port_ptr;

/**
 * @class HardwareInterface
 * @brief The interface for the serial communication with the Braccio driver
 */
class HardwareInterface
{
public:
  /**
   * @brief Constructor
   */
  HardwareInterface();

  /**
   * @brief Constructor with parameters for setting the member variables
   * @param port The USB port which the Braccio driver is connected to
   * @param baudrate The baudrate of the USB port
   * @param char_size The bit size of a character
   * @param pwmRange The ranges of the PWM which should be allowed
   */
  HardwareInterface(const char* port, const uint32_t baudrate, const uint16_t char_size = 8,
                    const std::pair<uint16_t, uint16_t>& pwmRange = std::pair<uint16_t, uint16_t>(500, 2500));

  /**
   * @brief Destructor
   */
  ~HardwareInterface();

  /**
   * @brief Turn the servo with servoNr to a pwm with a speed and time
   * @param servoNr The number of the servo which needs to be turned
   * @param pwm The position where the servo should turn to
   * @param time The time in milliseconds what it needs to take for the servo to reach the position
   */
  void turnServo(const uint8_t servoNr, const uint32_t pwm, const uint32_t time);

  /**
   * @brief This stops all the servo's of the robotic arm
   */
  void stop();

  /**
   * @brief The assign operator
   * @param other The right hand side of the operator
   * @return The assigned hardware interface.
   */
  HardwareInterface& operator=(const HardwareInterface& other);

private:
  /**
   * @brief This will open the serial port to send and receive commands.
   * @param port The USB port where the USB is connected to
   * @param baudrate The baudrate at what the USB writes/reads
   * @param char_size The size of a single character in bits.
   */
  void initialiseSerial(const char* port, uint32_t baudrate, uint16_t char_size);

  /**
   * @brief This will write a given buffer onto the serial port
   * @param buf The buffer which needs to be written onto the serial port
   * @param size The size of the buffer which needs to be written
   * @return This returns the state of the serial port
   */
  int write_some(const char* buf, const uint16_t size);

  /**
   * @brief This is an overload of the other write_some function to also support strings
   * @param buf The string buffer which needs to be send
   * @return The state of the serial port
   */

  int write_some(const std::string& buf);

  const char* portName;                   /*!< The name of the USB serial port */
  std::pair<uint16_t, uint16_t> pwmRange; /*!< The ranges of valid PWM signals */
  uint32_t baudrate;                      /*!< The baudrate of the USB serial port */
  uint16_t char_size;                     /*!< The character size in bits of the serial port */
  boost::asio::io_service ioservice;      /*!< The ioservice which is necessary for the serial port */
  serial_port_ptr serial;                 /*!< The serial port which is used to send and receive commands */
};
#endif  // HARDWARE_INTERFACE_H_
