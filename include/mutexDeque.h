/*
 * @Author: chengwei zhao
 * @LastEditors: cc
 * @Data:
 */
#pragma once

#include <deque>
#include <mutex>
#include <assert.h>

template <typename T>
class MutexDeque
{
public:
    MutexDeque() = default;
    ~MutexDeque() {}

    void push_back(const T &t)
    {
        std::lock_guard<std::mutex> lck(mtx_);
        data_.push_back(t);
    }

    T pop_front()
    {
        T ret;
        std::lock_guard<std::mutex> lck(mtx_);

        assert(data_.size() > 0);

        ret = data_.front();
        data_.pop_front();
        return ret;
    }

    T front()
    {
        T ret;
        std::lock_guard<std::mutex> lck(mtx_);
        ret = data_.front();
        return ret;
    }

    T back()
    {
        T ret;
        std::lock_guard<std::mutex> lck(mtx_);
        ret = data_.back();
        return ret;
    }

    bool empty()
    {
        std::lock_guard<std::mutex> lck(mtx_);
        return data_.empty();
    }

    int size()
    {
        std::lock_guard<std::mutex> lck(mtx_);
        return data_.size();
    }

    void clear()
    {
        std::lock_guard<std::mutex> lck(mtx_);
        data_.clear();
    }

    typename std::deque<T>::iterator begin()
    {
        std::lock_guard<std::mutex> lck(mtx_);
        return data_.begin();
    }

    typename std::deque<T>::iterator end()
    {
        std::lock_guard<std::mutex> lck(mtx_);
        return data_.end();
    }

    void insert(typename std::deque<T>::iterator out_end, typename std::deque<T>::iterator in_front, typename std::deque<T>::iterator in_end)
    {
        std::lock_guard<std::mutex> lck(mtx_);
        data_.insert(out_end, in_front, in_end);
    }

    T operator[](int i)
    {
        std::lock_guard<std::mutex> lck(mtx_);
        assert(data_.size() > i);
        return data_[i];
    }

private:
    std::deque<T> data_;
    std::mutex mtx_;
};