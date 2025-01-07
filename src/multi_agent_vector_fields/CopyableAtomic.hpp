#pragma once

#include <atomic>

namespace ghostplanner::cfplanner
{
    template <typename T>
    class CopyableAtomic
    {
      public:
        operator T() const
        {
            return data_.load();
        };
        T operator=(const T &other)
        {
            data_.store(other);
            return data_.load();
        };
        CopyableAtomic &operator=(const CopyableAtomic &other)
        {
            data_.store(other.data_.load());
            return *this;
        };
        CopyableAtomic(const CopyableAtomic &other)
        {
            data_.store(other.data_.load());
        };
        CopyableAtomic(const T &data)
        {
            data_.store(data);
        };
        CopyableAtomic() {};

      private:
        std::atomic<T> data_;
    };

}  // namespace ghostplanner::cfplanner