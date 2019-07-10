#include "loghelp.h"
#include "glog/logging.h"
#include <boost/filesystem.hpp>

void LogHelpInit()
{
    namespace fs = boost::filesystem;
    fs::path path = fs::current_path();
    path /= "PlaningOutput";
    if (!fs::exists(path)) {
      fs::create_directories(path);
    }
	google::InitGoogleLogging("");
	google::SetLogDestination(google::GLOG_INFO, "PlaningOutput/planout_");
	google::SetLogFilenameExtension("Plan");
	FLAGS_colorlogtostderr = false;
	FLAGS_alsologtostderr = false;
	FLAGS_stderrthreshold = 5;
	FLAGS_logbuflevel = 5;
	FLAGS_logbufsecs = 1;						// Set log output speed(s)
	FLAGS_max_log_size = 512;					// Set max log file size
	FLAGS_stop_logging_if_full_disk = true;		// If disk is full
}
