#include <Catch/catch.hpp>

#include "RobotServer.h"
#include "StatusUpdater.h"
#include "test-utils.h"
#include "sockets/MockSocketMultiThreadWrapper.h"


void testSimpleCommand(RobotServer& r, std::string msg, std::string expected_resp, COMMAND expected_command)
{
    MockSocketMultiThreadWrapper* mock_socket = build_and_get_mock_socket();
    mock_socket->sendMockData(msg);
    
    COMMAND c = r.oneLoop();

    std::string resp = mock_socket->getMockData();
    REQUIRE(resp == expected_resp);
    REQUIRE(c == expected_command);
}


TEST_CASE("Move", "[RobotServer]")
{
    std::string msg = "<{'type':'move','data':{'x':1,'y':2,'a':3}}>";
    std::string expected_response = "<{\"type\":\"ack\",\"data\":\"move\"}>";
    COMMAND expected_command = COMMAND::MOVE;

    StatusUpdater s;
    RobotServer r = RobotServer(s);
    testSimpleCommand(r, msg, expected_response, expected_command);

    RobotServer::PositionData data = r.getMoveData();
    REQUIRE(data.x == 1);
    REQUIRE(data.y == 2);
    REQUIRE(data.a == 3);
}

TEST_CASE("Move rel", "[RobotServer]")
{
    std::string msg = "<{'type':'move_rel','data':{'x':1,'y':2,'a':3}}>";
    std::string expected_response = "<{\"type\":\"ack\",\"data\":\"move_rel\"}>";
    COMMAND expected_command = COMMAND::MOVE_REL;

    StatusUpdater s;
    RobotServer r = RobotServer(s);
    testSimpleCommand(r, msg, expected_response, expected_command);

    RobotServer::PositionData data = r.getMoveData();
    REQUIRE(data.x == 1);
    REQUIRE(data.y == 2);
    REQUIRE(data.a == 3);
}

TEST_CASE("Move fine", "[RobotServer]")
{
    std::string msg = "<{'type':'move_fine','data':{'x':1,'y':2,'a':3}}>";
    std::string expected_response = "<{\"type\":\"ack\",\"data\":\"move_fine\"}>";
    COMMAND expected_command = COMMAND::MOVE_FINE;

    StatusUpdater s;
    RobotServer r = RobotServer(s);
    testSimpleCommand(r, msg, expected_response, expected_command);

    RobotServer::PositionData data = r.getMoveData();
    REQUIRE(data.x == 1);
    REQUIRE(data.y == 2);
    REQUIRE(data.a == 3);
}


TEST_CASE("Move const vel", "[RobotServer]")
{
    std::string msg = "<{'type':'move_const_vel','data':{'vx':1,'vy':2,'va':3,'t':4}}>";
    std::string expected_response = "<{\"type\":\"ack\",\"data\":\"move_const_vel\"}>";
    COMMAND expected_command = COMMAND::MOVE_CONST_VEL;

    StatusUpdater s;
    RobotServer r = RobotServer(s);
    testSimpleCommand(r, msg, expected_response, expected_command);

    RobotServer::VelocityData data = r.getVelocityData();
    REQUIRE(data.vx == 1);
    REQUIRE(data.vy == 2);
    REQUIRE(data.va == 3);
    REQUIRE(data.t == 4);
}

TEST_CASE("Place", "[RobotServer]")
{
    std::string msg = "<{'type':'place'}>";
    std::string expected_response = "<{\"type\":\"ack\",\"data\":\"place\"}>";
    COMMAND expected_command = COMMAND::PLACE_TRAY;

    StatusUpdater s;
    RobotServer r = RobotServer(s);
    testSimpleCommand(r, msg, expected_response, expected_command);
}

TEST_CASE("Load", "[RobotServer]")
{
    std::string msg = "<{'type':'load'}>";
    std::string expected_response = "<{\"type\":\"ack\",\"data\":\"load\"}>";
    COMMAND expected_command = COMMAND::LOAD_TRAY;

    StatusUpdater s;
    RobotServer r = RobotServer(s);
    testSimpleCommand(r, msg, expected_response, expected_command);
}


TEST_CASE("Init tray", "[RobotServer]")
{
    std::string msg = "<{'type':'init'}>";
    std::string expected_response = "<{\"type\":\"ack\",\"data\":\"init\"}>";
    COMMAND expected_command = COMMAND::INITIALIZE_TRAY;

    StatusUpdater s;
    RobotServer r = RobotServer(s);
    testSimpleCommand(r, msg, expected_response, expected_command);
}

TEST_CASE("Send position", "[RobotServer]")
{
    std::string msg = "<{'type':'p','data':{'x':1,'y':2,'a':3}}>";
    std::string expected_response = "<{\"type\":\"ack\",\"data\":\"p\"}>";
    COMMAND expected_command = COMMAND::POSITION;

    StatusUpdater s;
    RobotServer r = RobotServer(s);
    testSimpleCommand(r, msg, expected_response, expected_command);

    RobotServer::PositionData data = r.getPositionData();
    REQUIRE(data.x == 1);
    REQUIRE(data.y == 2);
    REQUIRE(data.a == 3);
}

TEST_CASE("Estop", "[RobotServer]")
{
    std::string msg = "<{'type':'estop'}>";
    std::string expected_response = "<{\"type\":\"ack\",\"data\":\"estop\"}>";
    COMMAND expected_command = COMMAND::ESTOP;

    StatusUpdater s;
    RobotServer r = RobotServer(s);
    testSimpleCommand(r, msg, expected_response, expected_command);
}


TEST_CASE("Load complete", "[RobotServer]")
{
    std::string msg = "<{'type':'lc'}>";
    std::string expected_response = "<{\"type\":\"ack\",\"data\":\"lc\"}>";
    COMMAND expected_command = COMMAND::LOAD_COMPLETE;

    StatusUpdater s;
    RobotServer r = RobotServer(s);
    testSimpleCommand(r, msg, expected_response, expected_command);
}