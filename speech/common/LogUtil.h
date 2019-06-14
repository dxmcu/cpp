/*
 * LogUtil.h
 *
 *  Created on: Jun 7, 2018
 *      Author: fengwz
 */

#ifndef LOGUTIL_H_
#define LOGUTIL_H_

#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/trivial.hpp>

void InitLog(const std::string &strPrefix)
{
    boost::log::add_common_attributes();
    boost::log::register_simple_formatter_factory<boost::log::trivial::severity_level, char>("Severity");

    std::string strHomeDir = getenv("HOME");
    if(strHomeDir.empty())
    {
        strHomeDir = "/var";
    }

    std::size_t index = strPrefix.find_last_of('/');
    std::string strPrefixSub = strPrefix.substr(index + 1);
    std::string strFileName = strHomeDir + "/.robot/logs/" + strPrefixSub + "_%Y-%m-%d_%H-%M-%S_%N.log";
    std::cout << "Log file name is " << strFileName << std::endl;

    boost::log::add_file_log(
                boost::log::keywords::file_name = strFileName,
                boost::log::keywords::format = "%LineID% [%TimeStamp%] [%ThreadID%] [%ProcessID%] [%Severity%] %Message%",
                boost::log::keywords::auto_flush = true,
                boost::log::keywords::time_based_rotation = boost::log::sinks::file::rotation_at_time_point(0, 0, 0),
                boost::log::keywords::rotation_size = 100 * 1024 * 1024,
                boost::log::keywords::min_free_space = 1 * 1024 * 1024);
}


#endif /* LOGUTIL_H_ */
