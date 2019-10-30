#ifndef MUTEX_PROTECTED_BAG_WRITER_H
#define MUTEX_PROTECTED_BAG_WRITER_H

#error Depracated!

#include "rosbag/bag.h"
#include <mutex>

class MutexProtectedBagWriter
{
public:
    void open(std::string const &filename, uint32_t mode)
    {
        m_bag.open(filename,mode);
    }
    
    void close()
    {
        m_bag.close();
    }

    template <class T> void write(std::string const &topic, ros::Time const &time, T const &msg)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_bag.write(topic,time,msg);
    }

    
private:
    rosbag::Bag m_bag;
    std::mutex m_mutex;
};

#endif
