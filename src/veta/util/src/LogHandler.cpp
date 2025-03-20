/**
 * --------------------------------------------------
 *
 * @file    LogHandler.cpp
 * @brief   Logging utility source
 * @author  Minkyu Kil
 * @date    2025-01-01
 * @version 1.0
 *
 * Copyright (c) 2025, Minkyu Kil
 * All rights reserved
 *
 * --------------------------------------------------
 */

#include "veta/util/LogHandler.h"

#include <unistd.h>
#include <iostream>
#include <cstdio>
#include <cstdarg>
#include <mutex>
#include <memory>

// ANSI terminal output message color
#define ANSI_COLOR_RESET   "\x1b[0m"
#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"

// Maximum buffer size, DO NOT CHANGE
#define MAX_BUFFER_SIZE 1024

// Logging levels, DO NOT CHANGE
static const char *LogLevelString[4] = { "Debug: "
                                       , "Info: "
                                       , "Warning: "
                                       , "Error: " };

// Logging level colors are able to customize
static const char *LogLevelColor [4] = { ANSI_COLOR_CYAN
                                       , ANSI_COLOR_GREEN
                                       , ANSI_COLOR_YELLOW
                                       , ANSI_COLOR_RED    };

namespace // anonymous
{
    // Log output handler object, all handler types are included
    struct LogHandlerObject
    {
        // Thread safty mutex locker
        std::mutex m_syncLock;

        // Log level and log handler type
        veta::log::LogLevel m_logLevel;

        // Current log output handler
        veta::log::HandlerType m_handlerType;
        veta::log::LogHandler *m_currentHandler;

        // Log output handlers
        veta::log::LogHandlerTerminal *m_terminalHandler;
        veta::log::LogHandlerFile     *m_fileHandler;

        // Log output handler constructor
        LogHandlerObject()
        {
            // default log level is INFO
            m_logLevel = veta::log::LogLevel::INFO;

            // File log handler is not activated at default
            m_terminalHandler = new veta::log::LogHandlerTerminal();
            m_fileHandler     = nullptr;

            // Default log handler is terminal log handler
            m_handlerType    = veta::log::HandlerType::TERMINAL;
            m_currentHandler = m_terminalHandler;
        }

        // Log output handler destructor
        ~LogHandlerObject()
        {
            // Deallocate all log output handlers
            delete m_terminalHandler;
            delete m_fileHandler;

            // Set nullptr to each handlers
            m_currentHandler  = nullptr;
            m_terminalHandler = nullptr;
            m_fileHandler     = nullptr;
        }

    }; // struct LogHandlerObject

    // Log handler access function
    static LogHandlerObject *getLogHandler()
    {
        static LogHandlerObject LogHandler;
        return &LogHandler;
    }

    // Macro for calling static function "getLogHandler"
    #define GET_LOG_HANDLER                                              \
            LogHandlerObject *logHandler = getLogHandler();              \
            std::lock_guard<std::mutex> synclLock(logHandler->m_syncLock)

} // namespace annonymous

// "veta::log::LogHandlerTerminal" source

/** @brief Log output member function for terminal */
void veta::log::LogHandlerTerminal::printLog(const char *fileName, int lineNumber, LogLevel logLevel, const std::string &msg)
{
    // Linux system only
    bool isTTY(isatty(fileno(stderr)) != 0);

    if(veta::log::LogLevel::WARNING <= logLevel)
    {
        if(isTTY)
        {
            std::cerr << LogLevelColor[logLevel];
        }
        std::cerr << LogLevelString[logLevel] << msg 
                  << " at line " << lineNumber 
                  << " in " << fileName << std::endl;
        
        if(isTTY)
        {
            std::cerr << ANSI_COLOR_RESET;
        }
        std::cerr.flush();
    }
    else // logLevel <= veta::log::LogLevel::INFO
    {
        if(isTTY)
        {
            std::cout << LogLevelColor[logLevel];
        }
        std::cout << LogLevelString[logLevel] << msg << std::endl;

        if(isTTY)
        {
            std::cout << ANSI_COLOR_RESET;
        }
        std::cout.flush();
    }
}

// "veta::log::LogHandlerFile" source

/** @brief Class constructor requires log file path */
veta::log::LogHandlerFile::LogHandlerFile(const char *fileName)
{
    m_file = fopen(fileName, "a");
    if(m_file == nullptr)
    {
        std::cerr << "[LogHandlerFile]: Log file opening error" << std::endl;
    }
}

/** @brief Class destructor */
veta::log::LogHandlerFile::~LogHandlerFile()
{
    if(m_file != nullptr)
    {
        if(fclose(m_file) != 0)
        {
            std::cerr << "[LogHandlerFile]: Log file closing error" << std::endl;
        }
    }
}

/** @brief Log output member function */
void veta::log::LogHandlerFile::printLog(const char *fileName, int lineNumber, LogLevel logLevel, const std::string &msg)
{
    if(m_file != nullptr)
    {
        if(veta::log::LogLevel::WARNING <= logLevel)
        {
            fprintf(m_file, "%s%s", LogLevelString[logLevel], msg.c_str());
            fprintf(m_file, " at line %d in %s\n", lineNumber, fileName);
        }
        else
        {
            fprintf(m_file, "%s%s\n", LogLevelString[logLevel], msg.c_str());
        }
        fflush(m_file);
    }
}

// Log utility functions

/** @brief Log level setter function */
void veta::log::setLogLevel(LogLevel logLevel)
{
    GET_LOG_HANDLER;
    logHandler->m_logLevel = logLevel;
}

/** @brief Log level getter function */
veta::log::LogLevel veta::log::getLogLevel()
{
    GET_LOG_HANDLER;
    return logHandler->m_logLevel;
}

/** @brief Log file handler enabler function, log file path is required */
void veta::log::enableLogFileHandler(const std::string &fileName)
{
    GET_LOG_HANDLER;
    if(logHandler->m_fileHandler == nullptr)
    {
        logHandler->m_fileHandler = new LogHandlerFile(fileName.c_str());
    }
    else // (LogHandler->m_LogHandlerFile != nullptr)
    {
        std::cerr << "[LogHandlerFile]: File log handler is already enabled" << std::endl;
    }
}

/** @brief Log file handler disabler function, log file hadnler deallocator */
void veta::log::disableLogFileHandler()
{
    GET_LOG_HANDLER;
    if(logHandler->m_fileHandler == nullptr)
    {
        std::cerr << "[LogHandlerFile]: File log handler is already disabled" << std::endl;
    }
    else // (LogHandler->m_LogHandlerFile != nullptr)
    {
        delete logHandler->m_fileHandler;
        logHandler->m_fileHandler = nullptr;
    }
}

/** @brief Log output handler type setter function */
void veta::log::setLogHandlerType(HandlerType handlerType)
{
    GET_LOG_HANDLER;
    if(handlerType != logHandler->m_handlerType)
    {
        if(handlerType == HandlerType::FILE)
        {
            if(logHandler->m_fileHandler == nullptr)
            {
                std::cerr << "[LogHandlerFile]: File log handler is not allocated" << std::endl;
            }
            else
            {
                logHandler->m_handlerType = HandlerType::FILE;
                logHandler->m_currentHandler = logHandler->m_fileHandler;
            }
        }
        else // handlerType == veta::log::TERMINAL_HANDLER
        {
            logHandler->m_handlerType = HandlerType::TERMINAL;
            logHandler->m_currentHandler = logHandler->m_terminalHandler;
        }
    }
}

/** @brief Log output handler type getter function */
veta::log::HandlerType veta::log::getLogHandlerType()
{
    GET_LOG_HANDLER;
    return logHandler->m_handlerType;
}

/** @brief Log output function */
void veta::log::printLog(const char *fileName, int lineNumber, LogLevel logLevel, const char *msg, ...)
{
    GET_LOG_HANDLER;
    if(logHandler->m_currentHandler != nullptr && logHandler->m_logLevel <= logLevel)
    {
        va_list args;
        va_start(args, msg);

            char buffer[MAX_BUFFER_SIZE];
            vsnprintf(buffer, sizeof(buffer), msg, args);

        va_end(args);

        buffer[MAX_BUFFER_SIZE - 1] = '\0';
        logHandler->m_currentHandler->printLog(fileName, lineNumber, logLevel, buffer);
    }
}