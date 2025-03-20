/**
 * --------------------------------------------------
 *
 * @file    LogHandler.h
 * @brief   Logging utility
 * @author  Minkyu Kil
 * @date    2025-01-01
 * @version 1.0
 *
 * Copyright (c) 2025, Minkyu Kil
 * All rights reserved
 *
 * --------------------------------------------------
 */

#ifndef VETA_UTIL_LOGHANDLER_H_
#define VETA_UTIL_LOGHANDLER_H_

#include <string>

// Terminal log handler macro
#define VETA_DEBUG(msg, ...) veta::log::printLog(__FILE__, __LINE__, veta::log::LogLevel::DEBUG   , msg, ##__VA_ARGS__)
#define VETA_INFO(msg, ...)  veta::log::printLog(__FILE__, __LINE__, veta::log::LogLevel::INFO    , msg, ##__VA_ARGS__)
#define VETA_WARN(msg, ...)  veta::log::printLog(__FILE__, __LINE__, veta::log::LogLevel::WARNING , msg, ##__VA_ARGS__)
#define VETA_ERROR(msg, ...) veta::log::printLog(__FILE__, __LINE__, veta::log::LogLevel::ERROR   , msg, ##__VA_ARGS__)

namespace veta
{
    namespace log
    {
        // Output handler types
        enum class HandlerType
        {
            TERMINAL = 0,
            FILE     = 1

        }; // enum class HandlerType

        // Output handler log level
        enum LogLevel
        {
            DEBUG   = 0,
            INFO    = 1,
            WARNING = 2,
            ERROR   = 3

        }; // enum LogLevel

        /** @brief Base class of the log output handler */
        class LogHandler
        {
        // "LogHandler" member functions
        public:

            /** @brief Class constructor */
            LogHandler() = default;

            /** @brief Class destructor */
            virtual ~LogHandler() = default;

            /** @brief Log output member function */
            virtual void printLog(const char *fileName, int lineNumber, LogLevel logLevel, const std::string &msg) = 0;

        }; // class LogHandler

        /**
         * @brief Log output handler for terminal, default log output handler
         * @details Linux platform only
         */
        class LogHandlerTerminal : public LogHandler
        {
        // "LogHandlerTerminal" member functions
        public:

            /** @brief Class constructor */
            LogHandlerTerminal() = default;

            /** @brief Log output member function for terminal */
            void printLog(const char *fileName, int lineNumber, LogLevel logLevel, const std::string &msg) override;

        }; // class LogHandlerTerminal

        /** @brief Log output handler for log file */
        class LogHandlerFile : public LogHandler
        {
        // "LogHandlerFile" member variables
        private:

            // File pointer
            FILE *m_file;

        // "LogHandlerFile" member functions
        public:

            /** @brief Class constructor requires log file path */
            LogHandlerFile(const char *fileName);

            /** @brief Class destructor deallocates file pointer*/
            ~LogHandlerFile() override;

            /** @brief Log output member function */
            void printLog(const char *fileName, int lineNumber, LogLevel logLevel, const std::string &msg) override;

        }; // class LogHandlerFile

        // Log utility functions

        /** @brief Log level setter function */
        void setLogLevel(LogLevel logLevel);

        /** @brief Log level getter function */
        LogLevel getLogLevel();

        /** @brief Log file handler enabler function, log file path is required */
        void enableLogFileHandler(const std::string &fileName);

        /** @brief Log file handler disabler function, log file hadnler deallocator */
        void disableLogFileHandler();

        /** @brief Log output handler type setter function */
        void setLogHandlerType(HandlerType handlerType);

        /** @brief Log output handler type getter function */
        HandlerType getLogHandlerType();

        /** @brief Log output function */
        void printLog(const char *fileName, int lineNumber, LogLevel logLevel, const char *msg, ...);

    } // namespace log

} // namespace veta

#endif // VETA_UTIL_LOGHANDLER_H_