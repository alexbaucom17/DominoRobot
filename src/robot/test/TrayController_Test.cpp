#include <Catch/catch.hpp>

#include "TrayController.h"
#include "serial/MockSerialComms.h"
#include "serial/SerialCommsFactory.h"
#include "constants.h"

MockSerialComms* build_and_get_mock_serial_tray()
{
    SerialCommsBase* base_serial = SerialCommsFactory::getFactoryInstance()->get_serial_comms(CLEARCORE_USB);
    MockSerialComms* mock_serial = dynamic_cast<MockSerialComms*>(base_serial);
    mock_serial->purge_data();
    return mock_serial;
}

TEST_CASE("Send and waiting", "[TrayController]")
{
    MockSerialComms* mock_serial = build_and_get_mock_serial_tray();
    TrayController t;

    t.initialize();
    REQUIRE(mock_serial->mock_rcv_lift() == "");
    REQUIRE(t.isActionRunning() == true);

    // Expect to receive close command once
    t.update();
    REQUIRE(mock_serial->mock_rcv_lift() == "close");
    
    // Expect to receive empty command while lifter reports closing
    for (int i = 0; i < 5; i ++)
    {
        mock_serial->mock_send("lift:close");
        t.update();
        REQUIRE(mock_serial->mock_rcv_lift() == "");
    }

    // Expect to receive none response once lifter reports no action
    mock_serial->mock_send("lift:none");
    t.update();
    REQUIRE(mock_serial->mock_rcv_lift() == "home");
}

TEST_CASE("Initialize tray", "[TrayController]")
{
    MockSerialComms* mock_serial = build_and_get_mock_serial_tray();
    TrayController t;

    t.initialize();
    REQUIRE(mock_serial->mock_rcv_lift() == "");
    REQUIRE(t.isActionRunning() == true);

    t.update();
    REQUIRE(mock_serial->mock_rcv_lift() == "close");
    mock_serial->mock_send("lift:none");

    t.update();
    REQUIRE(mock_serial->mock_rcv_lift() == "home");
    mock_serial->mock_send("lift:none");

    t.update();
    int p = cfg.lookup("tray.default_pos_steps");
    REQUIRE(mock_serial->mock_rcv_lift() == std::to_string(p));
    mock_serial->mock_send("lift:none");

    t.update();
    REQUIRE(t.isActionRunning() == false);
}

TEST_CASE("Place tray", "[TrayController]")
{
    MockSerialComms* mock_serial = build_and_get_mock_serial_tray();
    TrayController t;

    t.place();
    REQUIRE(mock_serial->mock_rcv_lift() == "");
    REQUIRE(t.isActionRunning() == true);

    t.update();
    int p = cfg.lookup("tray.place_pos_steps");
    REQUIRE(mock_serial->mock_rcv_lift() == std::to_string(p));
    mock_serial->mock_send("lift:none");

    t.update();
    REQUIRE(mock_serial->mock_rcv_lift() == "open");
    mock_serial->mock_send("lift:none");

    t.update();
    p = cfg.lookup("tray.default_pos_steps");
    REQUIRE(mock_serial->mock_rcv_lift() == std::to_string(p));
    mock_serial->mock_send("lift:none");

    t.update();
    REQUIRE(mock_serial->mock_rcv_lift() == "close");
    mock_serial->mock_send("lift:none");

    t.update();
    REQUIRE(t.isActionRunning() == false);
}

TEST_CASE("Load tray", "[TrayController]")
{
    MockSerialComms* mock_serial = build_and_get_mock_serial_tray();
    TrayController t;

    t.load();
    REQUIRE(mock_serial->mock_rcv_lift() == "");
    REQUIRE(t.isActionRunning() == true);

    t.update();
    int p = cfg.lookup("tray.load_pos_steps");
    REQUIRE(mock_serial->mock_rcv_lift() == std::to_string(p));
    mock_serial->mock_send("lift:none");

    t.update();
    REQUIRE(mock_serial->mock_rcv_lift() == "");
    t.setLoadComplete();

    t.update();
    p = cfg.lookup("tray.default_pos_steps");
    REQUIRE(mock_serial->mock_rcv_lift() == std::to_string(p));
    mock_serial->mock_send("lift:none");

    t.update();
    REQUIRE(t.isActionRunning() == false);
}