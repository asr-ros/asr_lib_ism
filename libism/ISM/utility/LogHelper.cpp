/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "LogHelper.hpp"
#include <iostream>

#include "boost/algorithm/string.hpp"
#include "boost/date_time/posix_time/posix_time.hpp"
#include "boost/date_time/posix_time/posix_time_io.hpp"
#include <boost/filesystem.hpp>

namespace ISM {

	LogHelperPtr LogHelper::mInstance = NULL;

	const char* LogHelper::LOG_COLOR_DEFAULT = "\033[0m";
	const char* LogHelper::LOG_COLOR_BLUE = "\033[34;1m";
	const char* LogHelper::LOG_COLOR_RED = "\033[31m";
	const char* LogHelper::LOG_COLOR_GREEN = "\033[32m";
	const char* LogHelper::LOG_COLOR_YELLOW = "\033[33m";
	const char* LogHelper::LOG_COLOR_MAGENTA = "\033[35m";

	unsigned int LogHelper::mBarWidth = 70;


	LogHelper::LogHelper(path logFilePath, LogLevel level)
	{
		mLogLevel = level;
		if (!boost::filesystem::exists(logFilePath.parent_path()))
		{
			boost::filesystem::create_directories(logFilePath.parent_path());
		}

		std::ios_base::iostate exceptionMask = mLogFile.exceptions() | std::ios::failbit | std::ios::badbit;
		mLogFile.exceptions(exceptionMask);
		try
		{
			mLogFile.open(logFilePath.string());
		} 
		catch (std::ios_base::failure& e)
		{
			std::cerr << e.what() << "\n";
		}
	}

	void LogHelper::log(const std::string & message, LogLevel logLevel, const char *logColor)
	{
		if (logLevel <= mLogLevel)
		{
			std::string coloredMessage = logColor + message + LOG_COLOR_DEFAULT;
			std::cout << coloredMessage << std::endl;

			std::vector<std::string> lines;
			boost::split(lines, message, boost::is_any_of("\n"));

			if (lines.size() != 0)
			{
				std::string infoString = "[ " + getLogLevelString(logLevel) + "][" + getTimeString() + "]: ";
				std::string indent(infoString.size(), ' ');

				bool logedInfo = false;
				for (unsigned int i = 0; i < lines.size(); ++i)
				{
					if (!lines[i].empty())
					{
						if (logedInfo)
						{
							mLogFile << indent << lines[i] << std::endl;
						}
						else
						{
							mLogFile << infoString << lines[i] << std::endl;
							logedInfo = true;
						}
					}
				}
				mLogFile.flush();
			}
		}
	}

	void LogHelper::logMessage(const std::string & message, LogLevel logLevel, const char *logColor)
	{
		LogHelper::getInstance()->log(message, logLevel, logColor);
	}

	void LogHelper::logLine(LogLevel logLevel)
	{
		LogHelper::getInstance()->log("\n==========================\n", logLevel);
	}

	void LogHelper::init(path logFilePath, LogLevel level)
	{
		if (mInstance)
		{
			LogHelper::close();
		}

		mInstance = LogHelperPtr(new LogHelper(logFilePath, level));
	}

	void LogHelper::close()
	{
		LogHelper::getInstance()->mLogFile.close();
		mInstance.reset();
	}

	LogHelperPtr LogHelper::getInstance()
	{
		if (mInstance) {
			return mInstance;
		}
		else
		{
			throw std::runtime_error("LogHelper was not initialiesed!");
		}
	}

	std::string LogHelper::getLogLevelString(LogLevel level)
	{
		std::string levels[] = {"FATAL", "ERROR", "WARNING", "INFO", "DEBUG"};
		return levels[level];
	}

	std::string LogHelper::getTimeString()
	{
		const boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();
		std::stringstream nowStream;
		nowStream << now;
		return nowStream.str();
	}

	void LogHelper::displayProgress(double progress)
	{
		unsigned int position = mBarWidth * progress;
		std::cout << "[";
		for (unsigned int i = 0; i < mBarWidth; ++i) {
			if (i <= position)
			{
				std::cout << "#";
			}
			else
			{
				std::cout << " ";
			}
		}
		std::cout << "] " << int(progress * 100) << " %\r";
		std::cout.flush();
	}

}
