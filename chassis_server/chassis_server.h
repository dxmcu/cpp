/*
 * Server.h
 *
 *  Created on: Jun 7, 2018
 *      Author: fengwz
 */

#ifndef SEERSERVER_H_
#define SEERSERVER_H_

#include <memory>
#include <boost/asio.hpp>
#include "chassis_protocol.h"
#include "chassis_session.hpp"

#include <string.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>

using boost::asio::ip::tcp;
using boost::asio::ip::address;

typedef boost::shared_ptr<session> session_ptr;

class Server {
public:
    Server(boost::asio::io_service &io_service, tcp::endpoint &endpoint, tcp::endpoint &endpoint2,
           tcp::endpoint &endpoint3, tcp::endpoint &endpoint4);
	virtual ~Server();

public:
	void handle_accept(session_ptr new_session, const boost::system::error_code& error);
    void handle_accept2(session_ptr new_session, const boost::system::error_code& error);
    void handle_accept3(session_ptr new_session, const boost::system::error_code& error);
    void handle_accept4(session_ptr new_session, const boost::system::error_code& error);
    void run();


private:
        boost::asio::io_service &io_service_;
        tcp::acceptor acceptor_;
        tcp::acceptor acceptor2_;
        tcp::acceptor acceptor3_;
        tcp::acceptor acceptor4_;
};

#endif /* SEERSERVER_H_ */
