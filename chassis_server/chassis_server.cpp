/*
 * Server.cpp
 *
 *  Created on: Jun 7, 2018
 *      Author: liuzheng
 */

#include <boost/assert.hpp>
#include "chassis_server.h"

double session::xpos_ = 0;
double session::ypos_ = 0;
double session::angle_ = 0;

Server::Server(boost::asio::io_service &io_service, tcp::endpoint &endpoint, tcp::endpoint &endpoint2,
               tcp::endpoint &endpoint3, tcp::endpoint &endpoint4)
            : io_service_(io_service)
            , acceptor_(io_service, endpoint)
            , acceptor2_(io_service, endpoint2)
            , acceptor3_(io_service, endpoint3)
            , acceptor4_(io_service, endpoint4)
{
	session_ptr new_session(new session(io_service_));
	acceptor_.async_accept(new_session->socket(),
		boost::bind(&Server::handle_accept,
		this,
		new_session,
		boost::asio::placeholders::error));

	session_ptr new_session2(new session(io_service_));
	acceptor2_.async_accept(new_session2->socket(),
        boost::bind(&Server::handle_accept2,
		this,
		new_session2,
		boost::asio::placeholders::error));

    session_ptr new_session3(new session(io_service_));
    acceptor3_.async_accept(new_session3->socket(),
        boost::bind(&Server::handle_accept3,
        this,
        new_session3,
        boost::asio::placeholders::error));

    session_ptr new_session4(new session(io_service_));
    acceptor4_.async_accept(new_session4->socket(),
        boost::bind(&Server::handle_accept4,
        this,
        new_session4,
        boost::asio::placeholders::error));
}

Server::~Server() {
}

void Server::handle_accept(session_ptr new_session, const boost::system::error_code& error) 
{
	if (error) 
	{
		return;
	}

	new_session->start();
	new_session.reset(new session(io_service_));
	acceptor_.async_accept(new_session->socket(),
		boost::bind(&Server::handle_accept,
		this,
		new_session,
		boost::asio::placeholders::error));
}

void Server::handle_accept2(session_ptr new_session, const boost::system::error_code& error)
{
    if (error)
    {
        return;
    }

    new_session->start();
    new_session.reset(new session(io_service_));
    acceptor2_.async_accept(new_session->socket(),
        boost::bind(&Server::handle_accept2,
        this,
        new_session,
        boost::asio::placeholders::error));
}

void Server::handle_accept3(session_ptr new_session, const boost::system::error_code& error)
{
    if (error)
    {
        return;
    }

    new_session->start();
    new_session.reset(new session(io_service_));
    acceptor3_.async_accept(new_session->socket(),
        boost::bind(&Server::handle_accept3,
        this,
        new_session,
        boost::asio::placeholders::error));
}

void Server::handle_accept4(session_ptr new_session, const boost::system::error_code& error)
{
    if (error)
    {
        return;
    }

    new_session->start();
    new_session.reset(new session(io_service_));
    acceptor4_.async_accept(new_session->socket(),
        boost::bind(&Server::handle_accept4,
        this,
        new_session,
        boost::asio::placeholders::error));
}

void Server::run()
{
	io_service_.run();
}
