#pragma once
#include <memory> // For std::allocator
#include <string.h>

class bump_allocator {
private:
    char *begin, *data, *end;

public:
    bump_allocator(size_t count) {
        begin = data = (char *)malloc(count);
        memset(begin, 0, count);
        end = begin + count;
    }

    ~bump_allocator() {
        free(begin);
    }

    bump_allocator(const bump_allocator&) = delete;
    void operator=(const bump_allocator&) = delete;

    void* allocate(std::size_t n_bytes) {
        assert(data + n_bytes < end);
        char *ptr = data;
        data += n_bytes;
        return (void *)ptr;
    }

    // TODO: void deallocate(T* p, std::size_t n);
};
