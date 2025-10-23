#pragma once

#include <array>
#include <optional>
#include <cstring> // memcpy

template <size_t CAP>
class RingBuffer
{
public:
    RingBuffer() = default;

    bool push(const uint8_t* src, size_t n) 
    {
        if (n > CAP) {
            dropped_count_ += n;
            return false;
        }
        if (size_ + n > CAP) {
            size_t drop = (size_ + n) - CAP;
            pop(drop);
            dropped_count_ += drop;
        }

        size_t tail = (head_ + size_) % CAP;
        size_t first_chunk = std::min(n, CAP - tail);
        memcpy(&data_[tail], src, first_chunk);
        if (n > first_chunk) {
            memcpy(&data_[0], src + first_chunk, n - first_chunk);
        }
        size_ += n;
        return true;
    }

    uint8_t get(size_t idx) const 
    {
        return data_[(head_ + idx) % CAP];
    }

    void pop(size_t n) 
    {
        if (n >= size_) {
            dropped_count_ += size_;
            head_ = 0;
            size_ = 0;
            return;
        }
        head_ = (head_ + n) % CAP;
        size_ -= n;
    }

    std::optional<size_t> findFirst(uint8_t val) const 
    {
        size_t max_search = std::min(size_, size_t(64));
        for (size_t i = 0; i < max_search; ++i) {
            if (get(i) == val) return i;
        }
        return std::nullopt;
    }

    // 新增：获取底层数组指针，需配合copyData使用
    const uint8_t* data() const { return data_.data(); }

    // 新增：拷贝连续数据，处理环形边界
    void copyData(size_t start, size_t len, uint8_t* dest) const 
    {
        if (start >= size_ || len > size_ - start) return;
        size_t pos = (head_ + start) % CAP;
        size_t first_chunk = std::min(len, CAP - pos);
        memcpy(dest, &data_[pos], first_chunk);
        if (len > first_chunk) {
            memcpy(dest + first_chunk, &data_[0], len - first_chunk);
        }
    }

    size_t size() const { return size_; }
    bool empty() const { return size_ == 0; }
    uint64_t getDroppedCount() const { return dropped_count_; }

private:
    std::array<uint8_t, CAP> data_{};
    size_t head_{0};
    size_t size_{0};
    uint64_t dropped_count_{0};
};