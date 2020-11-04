// Copyright (C) 2022 Arm Technology (China) Co. Ltd. All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0

#ifndef __DbgLogger_H__
#define __DbgLogger_H__

#include <iostream>
#include <sstream>
#include <memory>
#include <thread>
#include <mutex>

extern std::mutex m_mtx;
class DbgLogger {
    enum
    {
        EM_ERR = 0,
        EM_CRIT,
        EM_INFO,
        EM_DBG,
        EM_MAX
    };

    public:
    inline DbgLogger(int loglevel = EM_DBG)
    {
        m_loglevel = loglevel;

        if (m_loglevel < EM_ERR || m_loglevel >= EM_MAX) {
            std::cout << "loglevel invalid: " << m_loglevel << std::endl;
            exit(-1);
        }

        m_oss << log_arr[m_loglevel] << std::hex << "<" << std::this_thread::get_id() << ">  ";
    }

    template <typename ...T>
    inline DbgLogger(int loglevel, std::string format, const T&... args)
    {
        m_loglevel = loglevel;

        if (m_loglevel < EM_CRIT || m_loglevel >= EM_MAX) {
            std::cout << "loglevel invalid: " << m_loglevel << std::endl;
            exit(-1);
        }

        m_oss << log_arr[m_loglevel];

        int len = snprintf(nullptr, 0, format.c_str(), args...);
        std::unique_ptr<char[]> buffer(new char[len]);
        if (buffer) {
            snprintf(buffer.get(), len, format.c_str(), args...);
            m_oss << buffer.get();
        }
    }

    ~DbgLogger()
    {
        std::lock_guard<std::mutex> lck(m_mtx);
        std::cout << m_oss.str() << std::endl;
    }

    template <typename T>
    DbgLogger &operator<< (const T &arg)
    {
        m_oss << arg;
        return *this;
    }

    template <typename ...T>
    DbgLogger &operator() (const std::string &format, const T&... args)
    {
        int len = snprintf(nullptr, 0, format.c_str(), args...);

        std::unique_ptr<char[]> buffer(new char[len]);
        if (buffer) {
            snprintf(buffer.get(), len, format.c_str(), args...);
            m_oss << buffer.get();
        }

        return *this;
    }

    private:
    int m_loglevel;
    std::ostringstream m_oss;
    std::string log_arr[EM_MAX] = {"[Err] ", "[Crit] ", "[Info] ", "[Dbg] "};
};

#define AIPU_ERR()  (DbgLogger(0))
#define AIPU_CRIT() (DbgLogger(1))
#define AIPU_INFO() (DbgLogger(2))
#define AIPU_DBG()  (DbgLogger(3))

#endif