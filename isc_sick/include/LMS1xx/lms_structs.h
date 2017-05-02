/*
	lms_structs.h
	ISC SICK LMS1xx Node
	For use with LMS1xx series at 25Hz


*/

#ifndef LMS1XX_LMS_STRUCTS_H_
#define LMS1XX_LMS_STRUCTS_H_

#include <stdint.h>

/*!
* @class scanCfg
* @brief Structure containing scan configuration.
*
* @author Konrad Banachowicz
*/
struct scanCfg
{
  /*!
   * @brief Scanning frequency.
   * 1/100 Hz
   */
  int scaningFrequency;

  /*!
   * @brief Scanning resolution.
   * 1/10000 degree
   */
  int angleResolution;

  /*!
   * @brief Start angle.
   * 1/10000 degree
   */
  int startAngle;

  /*!
   * @brief Stop angle.
   * 1/10000 degree
   */
  int stopAngle;
};

/*!
* @class scanDataCfg
* @brief Structure containing scan data configuration.
*
* @author Konrad Banachowicz
*/
struct scanDataCfg
{

  /*!
   * @brief Output channels.
   * Defines which output channel is activated.
   */
  int outputChannel;

  /*!
   * @brief Remission.
   * Defines whether remission values are output.
   */
  bool remission;

  /*!
   * @brief Remission resolution.
   * Defines whether the remission values are output with 8-bit or 16bit resolution.
   */
  int resolution;

  /*!
   * @brief Encoders channels.
   * Defines which output channel is activated.
   */
  int encoder;

  /*!
   * @brief Position.
   * Defines whether position values are output.
   */
  bool position;

  /*!
   * @brief Device name.
   * Determines whether the device name is to be output.
   */
  bool deviceName;

  bool timestamp;

  /*!
   * @brief Output interval.
   * Defines which scan is output.
   *
   * 01 every scan\n
   * 02 every 2nd scan\n
   * ...\n
   * 50000 every 50000th scan
   */
  int outputInterval;
};

/*!
* @class outputRange
* @brief Structure containing scan output range configuration
*
* @author wpd
*/
struct scanOutputRange
{
  /*!
   * @brief Scanning resolution.
   * 1/10000 degree
   */
  int angleResolution;

  /*!
   * @brief Start angle.
   * 1/10000 degree
   */
  int startAngle;

  /*!
   * @brief Stop angle.
   * 1/10000 degree
   */
  int stopAngle;
};

/*!
* @class scanData
* @brief Structure containing single scan message.
*
* @author Konrad Banachowicz
*/
struct scanData
{

  /*!
   * @brief Number of samples in dist1.
   *
   */
  int dist_len1;

  /*!
   * @brief Radial distance for the first reflected pulse
   *
   */
  uint16_t dist1[1082];

  /*!
   * @brief Number of samples in dist2.
   *
   */
  int dist_len2;

  /*!
   * @brief Radial distance for the second reflected pulse
   *
   */
  uint16_t dist2[1082];

  /*!
   * @brief Number of samples in rssi1.
   *
   */
  int rssi_len1;

  /*!
   * @brief Remission values for the first reflected pulse
   *
   */
  uint16_t rssi1[1082];

  /*!
   * @brief Number of samples in rssi2.
   *
   */
  int rssi_len2;

  /*!
   * @brief Remission values for the second reflected pulse
   *
   */
  uint16_t rssi2[1082];
};

#endif  // LMS1XX_LMS_STRUCTS_H_
