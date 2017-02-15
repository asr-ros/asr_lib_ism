/**

Copyright (c) 2016, Hanselmann Fabian, Heller Florian, Heizmann Heinrich, Kübler Marcel, Mehlhaus Jonas, Meißner Pascal, Qattan Mohamad, Reckling Reno, Stroh Daniel
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#pragma once

#include <string>
#include <sstream>

#include <fstream>
#include <iostream>

#include <boost/shared_ptr.hpp>
#include <boost/filesystem/path.hpp>

#include "typedef.hpp"

namespace ISM {

using boost::filesystem::path;

enum LogLevel {LOG_FATAL, LOG_ERROR, LOG_WARNING, LOG_INFO, LOG_DEBUG};

    class LogHelper
    {

    public:
        LogHelper(path logFilePath, LogLevel level);

        const static char* LOG_COLOR_DEFAULT;
        const static char* LOG_COLOR_BLUE;
        const static char* LOG_COLOR_RED;
        const static char* LOG_COLOR_GREEN;
        const static char* LOG_COLOR_YELLOW;
        const static char* LOG_COLOR_MAGENTA;

        static void init(path logFilePath, LogLevel level);
        static void close();
        static LogHelperPtr getInstance();

        static void logMessage(const std::string & message, LogLevel logLevel = LOG_INFO, const char* logColor = LOG_COLOR_DEFAULT);
        static void logLine(LogLevel logLevel = LOG_INFO);
		static void displayProgress(double progress);

    private:
        static LogHelperPtr mInstance;

        std::ofstream mLogFile;
        LogLevel mLogLevel;

		static unsigned int mBarWidth;

        void log(const std::string & message, LogLevel logLevel = LOG_INFO, const char *logColor = LOG_COLOR_DEFAULT);
        LogHelper(const LogHelper&);
        LogHelper& operator =(const LogHelper&);

        std::string getLogLevelString(LogLevel level);
        std::string getTimeString();

    };


}
