#ifndef ROBOT_COMMUNICATION_HPP
#define ROBOT_COMMUNICATION_HPP

#include "BLEDevice.h"

/**
 * @brief Duration of the scan in seconds
 */
#define SCAN_DURATION 30

/**
 * @class RobotCommunication
 * @brief Create a BLE client to communicate with the robot
 */
class RobotCommunication : public BLEAdvertisedDeviceCallbacks
{
public:
    /**
     * @brief A constructor
     */
    RobotCommunication();

    /**
     * @brief A destructor
     */
    ~RobotCommunication();

    /**
     * @brief Setup the BLE client and connect to server
     */
    void setup();

    /**
     * @brief Update the start_char_ variable when the notification is triggered
     * @param p_BLE_remote_characteristic a model of the server characteristic
     * @param p_data the value of the characteristic
     * @param length the length of the value of the caracteristic
     * @param is_notify whether the server has published a notification
     */
    void startNotifyCallback(
        BLERemoteCharacteristic* p_BLE_remote_characteristic, uint8_t* p_data, size_t length,
        bool is_notify);

    /**
     * @brief Connect to the server on the robot
     * @param p_address the adress of the advertised server
     * @return if connected to the requested server, service and characteristic
     */
    bool connectToServer(BLEAddress& p_address);

    /**
     * @brief Check if the published value has changed
     * @return if the published value has changed
     */
    bool newValueReceived() const;

    /**
     * @brief Read the string published by the server
     * @return the string published by the server
     */
    std::string readServer();

    /**
     * @brief Called for each advertising BLE server check if its the searched server
     * @param advertised_device a representation of the server device
     */
    void onResult(BLEAdvertisedDevice advertised_device) override;

    /**
     * @brief The UUID of the service and characteristic we wish to connect to
     */
    BLEUUID service_UUID_;
    BLEUUID message_characteristic_UUID_;

    /**
     * @brief The adress of the server
     */
    BLEAddress * p_server_address_;

    /**
     * @brief Variable representing whether we have found the requested server and wether we are connected
     */
    bool do_connect_;
    bool connected_;

    /**
     * @brief Model of the start characteristic
     */
    BLERemoteCharacteristic* message_characteristic_;

    /**
     * @brief Variable to store the values published by the server
     */
    std::string message_characteristic_reponse_;

    /**
     * @brief Variable representing whether a new value has been published by the server
     */
    bool new_value_;

    /**
     * @brief Variables used to activate and desactivate notifications
     */
    const uint8_t notification_on_[2] = {0x1, 0x0};
    const uint8_t notification_off_[2] = {0x0, 0x0};

protected:

};
#endif // ROBOT_COMMUNICATION_HPP
