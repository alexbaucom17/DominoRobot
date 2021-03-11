#include <Catch/catch.hpp>

#include "TrayController.h"
#include "test-utils.h"

TEST_CASE("Send and waiting", "[TrayController]")
{
    MockSerialComms* mock_serial = build_and_get_mock_serial();
    MockClockWrapper* mock_clock = get_mock_clock_and_reset();
    TrayController t;

    t.initialize();
    REQUIRE(mock_serial->mock_rcv_lift() == "");
    REQUIRE(t.isActionRunning() == true);

    // Expect to receive close command once and then start status requests
    mock_clock->advance_ms(1);
    t.update();
    REQUIRE(mock_serial->mock_rcv_lift() == "close");
    
    // Expect to receive status request command while lifter reports closing
    for (int i = 0; i < 5; i ++)
    {
        mock_clock->advance_ms(1);
        mock_serial->mock_send("lift:close");
        t.update();
        REQUIRE(mock_serial->mock_rcv_lift() == "status_req");
    }

    // Expect to receive none response once lifter reports no action
    mock_clock->advance_ms(1000);
    mock_serial->mock_send("lift:none");
    t.update();
    REQUIRE(mock_serial->mock_rcv_lift() == "status_req");
    REQUIRE(mock_serial->mock_rcv_lift() == "home");
}

TEST_CASE("Initialize tray", "[TrayController]")
{
    MockSerialComms* mock_serial = build_and_get_mock_serial();
    MockClockWrapper* mock_clock = get_mock_clock_and_reset();
    TrayController t;

    t.initialize();
    REQUIRE(mock_serial->mock_rcv_lift() == "");
    REQUIRE(t.isActionRunning() == true);

    mock_clock->advance_ms(1500);
    t.update();
    REQUIRE(mock_serial->mock_rcv_lift() == "close");
    mock_serial->mock_send("lift:none");

    mock_clock->advance_ms(1500);
    t.update();
    REQUIRE(mock_serial->mock_rcv_lift() == "status_req");
    REQUIRE(mock_serial->mock_rcv_lift() == "home");
    mock_serial->mock_send("lift:none");

    mock_clock->advance_ms(1500);
    t.update();
    int p = cfg.lookup("tray.default_pos_steps");
    REQUIRE(mock_serial->mock_rcv_lift() == "status_req");
    REQUIRE(mock_serial->mock_rcv_lift() == "pos:"+std::to_string(p));
    mock_serial->mock_send("lift:none");

    mock_clock->advance_ms(1500);
    t.update();
    REQUIRE(t.isActionRunning() == false);
}

TEST_CASE("Place tray", "[TrayController]")
{
    MockSerialComms* mock_serial = build_and_get_mock_serial();
    MockClockWrapper* mock_clock = get_mock_clock_and_reset();
    TrayController t;
    t.setTrayInitialized(true);

    bool status = t.place();    
    REQUIRE(status == true);
    REQUIRE(mock_serial->mock_rcv_lift() == "");
    REQUIRE(t.isActionRunning() == true);

    mock_clock->advance_ms(1500);
    t.update();
    int p = cfg.lookup("tray.place_pos_steps");
    REQUIRE(mock_serial->mock_rcv_lift() == "pos:"+std::to_string(p));
    mock_serial->mock_send("lift:none");

    mock_clock->advance_ms(1500);
    t.update();
    REQUIRE(mock_serial->mock_rcv_lift() == "status_req");
    REQUIRE(mock_serial->mock_rcv_lift() == "open");
    mock_serial->mock_send("lift:none");

    mock_clock->advance_ms(1500);
    t.update();
    p = cfg.lookup("tray.default_pos_steps");
    REQUIRE(mock_serial->mock_rcv_lift() == "status_req");
    REQUIRE(mock_serial->mock_rcv_lift() == "pos:"+std::to_string(p));
    mock_serial->mock_send("lift:none");

    mock_clock->advance_ms(1500);
    t.update();
    REQUIRE(mock_serial->mock_rcv_lift() == "status_req");
    REQUIRE(mock_serial->mock_rcv_lift() == "close");
    mock_serial->mock_send("lift:none");

    mock_clock->advance_ms(1500);
    t.update();
    REQUIRE(t.isActionRunning() == false);
}

TEST_CASE("Load tray", "[TrayController]")
{
    MockSerialComms* mock_serial = build_and_get_mock_serial();
    MockClockWrapper* mock_clock = get_mock_clock_and_reset();
    TrayController t;
    t.setTrayInitialized(true);

    bool status = t.load();
    REQUIRE(status == true);
    REQUIRE(mock_serial->mock_rcv_lift() == "");
    REQUIRE(t.isActionRunning() == true);

    mock_clock->advance_ms(1500);
    t.update();
    int p = cfg.lookup("tray.load_pos_steps");
    REQUIRE(mock_serial->mock_rcv_lift() == "pos:"+std::to_string(p));
    mock_serial->mock_send("lift:none");

    mock_clock->advance_ms(1500);
    t.update();
    REQUIRE(mock_serial->mock_rcv_lift() == "status_req");
    REQUIRE(mock_serial->mock_rcv_lift() == "");
    t.setLoadComplete();

    mock_clock->advance_ms(1500);
    t.update();
    p = cfg.lookup("tray.default_pos_steps");
    REQUIRE(mock_serial->mock_rcv_lift() == "pos:"+std::to_string(p));
    mock_serial->mock_send("lift:none");

    mock_clock->advance_ms(1500);
    t.update();
    REQUIRE(t.isActionRunning() == false);
}

TEST_CASE("Errors when tray not initialized", "[TrayController]")
{
    TrayController t;
    t.setTrayInitialized(false);

    REQUIRE(t.place() == false);
    REQUIRE(t.load() == false);
}