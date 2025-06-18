#include "robot_communication.hpp"
#include "Arduino.h"

RobotCommunication::RobotCommunication()
{
    service_UUID_ = BLEUUID("91bad492-b950-4226-aa2b-4ede9fa42f59");
    start_characteristic_UUID_ = BLEUUID("cba1d466-344c-4be3-ab3f-189f80dd7518");
    do_connect_ = false;
    connected_ = false;
    new_start_ = false;
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
    pBLEScan->start(30);

    // If the flag "doConnect" is true then we have scanned for and found the desired
    // BLE Server with which we wish to connect.  Now we connect to it.  Once we are
    // connected we set the connected flag to be true.
    if (do_connect_) {
        if (connectToServer(*p_server_address_)) {
            start_characteristic_->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue(
                (uint8_t*)notification_on_, 2,
                true);
            connected_ = true;
        }
        do_connect_ = false;
    }
}

void RobotCommunication::startNotifyCallback(
    BLERemoteCharacteristic* p_BLE_remote_characteristic,
    uint8_t* p_data, size_t length, bool is_notify)
{
    start_char_ = std::string((char*)p_data);
    new_start_ = true;
}

std::string RobotCommunication::readServer()
{
    new_start_ = false;
    return start_char_;
}

bool RobotCommunication::shouldStart()
{
    return new_start_;
}

bool RobotCommunication::connectToServer(BLEAddress p_address)
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
    start_characteristic_ = pRemoteService->getCharacteristic(start_characteristic_UUID_);
    if (start_characteristic_ == nullptr) {
        return false;
    }

    // Read the value of the characteristic.
    std::string value = start_characteristic_->readValue();

    start_characteristic_->registerForNotify(
        [this](BLERemoteCharacteristic* p_BLE_remote_characteristic, uint8_t* p_data, size_t length, bool is_notify)
        {
            startNotifyCallback(p_BLE_remote_characteristic, p_data, length, is_notify);
        }
    );
    return true;
}

/**
* Called for each advertising BLE server.
*/
void RobotCommunication::onResult(BLEAdvertisedDevice advertised_device)
{

    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertised_device.haveServiceUUID() && advertised_device.getServiceUUID().equals(service_UUID_)) {

        advertised_device.getScan()->stop();

        p_server_address_ = new BLEAddress(advertised_device.getAddress());
        do_connect_ = true;

    } // Found our server
}   // onResult
