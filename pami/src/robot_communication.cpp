#include "robot_communication.hpp"
#include "Arduino.h"

RobotCommunication::RobotCommunication()
{
    service_UUID_ = BLEUUID("a9d058d7-8a5e-4241-981a-56b1b3c242fc");
    message_characteristic_UUID_ = BLEUUID("c803fc6c-d736-42cd-8547-c24d7b899f55");
    do_connect_ = false;
    connected_ = false;
    new_value_ = false;
}

RobotCommunication::~RobotCommunication()
{
    delete p_server_address_;
    p_server_address_ = nullptr;

    delete message_characteristic_;
    message_characteristic_ = nullptr;
}

void RobotCommunication::setup()
{

    BLEDevice::init("Pami");

    // Retrieve a Scanner and set the callback we want to use to be informed when we
    // have detected a new device.  Specify that we want active scanning and start the
    // scan to run for 30 seconds.
    BLEScan* pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(this);
    pBLEScan->setActiveScan(true);
    pBLEScan->start(SCAN_DURATION);

    // If the flag "doConnect" is true then we have scanned for and found the desired
    // BLE Server with which we wish to connect.  Now we connect to it.  Once we are
    // connected we set the connected flag to be true.
    if (do_connect_) {
        if (connectToServer(*p_server_address_)) {
            message_characteristic_->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue(
                (uint8_t*)notification_on_, 2, true);
            connected_ = true;
        }
        do_connect_ = false;
    }
    else {
        // TODO if server not found
    }

    delete pBLEScan;
}

void RobotCommunication::startNotifyCallback(
    BLERemoteCharacteristic* p_BLE_remote_characteristic,
    uint8_t* p_data, size_t length, bool is_notify)
{
    message_characteristic_reponse_ = std::string((char*)p_data);
    new_value_ = true;
}

std::string RobotCommunication::readServer()
{
    new_value_ = false;
    return message_characteristic_reponse_;
}

bool RobotCommunication::newValueReceived() const
{
    return new_value_;
}

bool RobotCommunication::connectToServer(BLEAddress& p_address)
{

    BLEClient* pClient = BLEDevice::createClient();

    // Connect to the BLE Server.
    pClient->connect(p_address);

    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pClient->getService(service_UUID_);
    if (pRemoteService == nullptr) {
        return false;
    }

    // Obtain a reference to the characteristic in the service of the remote BLE server.
    message_characteristic_ = pRemoteService->getCharacteristic(message_characteristic_UUID_);
    if (message_characteristic_ == nullptr) {
        return false;
    }

    // Read the value of the characteristic.
    std::string value = message_characteristic_->readValue();

    message_characteristic_->registerForNotify(
        [this](BLERemoteCharacteristic* p_BLE_remote_characteristic, uint8_t* p_data, size_t length, bool is_notify)
        {
            startNotifyCallback(p_BLE_remote_characteristic, p_data, length, is_notify);
        }
    );
    return true;

    delete pClient;
    delete pRemoteService;
}

/**
* Called for each advertising BLE server.
*/
void RobotCommunication::onResult(BLEAdvertisedDevice advertised_device)
{

    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertised_device.haveServiceUUID() && advertised_device.getServiceUUID().equals(service_UUID_)) {
        // Found our server
        advertised_device.getScan()->stop();

        p_server_address_ = new BLEAddress(advertised_device.getAddress());
        do_connect_ = true;

    }
}   // onResult
