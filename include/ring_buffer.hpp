#pragma once

#include <array>
#include <optional>

template <size_t CAP>
class RingBuffer
{
public:
    bool push(const uint8_t* src, size_t n) 
    {
        if (n > CAP) return false; // 太大直接丢弃
        // 如果空间不足，丢弃最旧数据（保证能 push 成功）
        if (size_ + n > CAP) 
        {
            size_t drop = (size_ + n) - CAP;
            pop(drop);
        }
        for (size_t i = 0; i < n; ++i) 
        {
            data_[(head_ + size_) % CAP] = src[i];
            ++size_;
        }
        return true;
    }

    uint8_t get(size_t idx) const 
    {
        return data_[(head_ + idx) % CAP];
    }

    void pop(size_t n) 
    {
        if (n >= size_) 
        {
            head_ = (head_ + size_) % CAP;
            size_ = 0;
            return;
        }
        head_ = (head_ + n) % CAP;
        size_ -= n;
    }

    std::optional<size_t> findFirst(uint8_t val) const {
        for (size_t i = 0; i < size_; ++i) {
            if (get(i) == val) return i;
        }
        return std::nullopt;
    }

    size_t size() const { return size_; }
    bool empty() const { return size_ == 0; }

private:
    std::array<uint8_t, CAP> data_{};
    size_t head_{0};
    size_t size_{0};
};