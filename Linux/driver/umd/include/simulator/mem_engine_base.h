#ifndef __MEM_ENGINE_BASE_H__
#define __MEM_ENGINE_BASE_H__

#include <memory>
#include <stddef.h>
#include <stdint.h>

namespace sim_aipu {
class IMemEngine;

/** @brief Creates a memory object.
@param enable_calloc if true the result will clean memory data, otherwise no
@return memory object
@note The created memory only has a 16GiB size
*/
std::unique_ptr<IMemEngine> make_mem_engine(bool enable_calloc = false);

/** @brief Creates a memory object.
@param msize specified the memory total size in byte
@param psize specified the memory page size in byte
@param enable_calloc if true the result will clean memory data, otherwise no
@return memory object
*/
std::unique_ptr<IMemEngine> make_mem_engine(uint64_t msize, uint32_t psize,
                                            bool enable_calloc);

/** @brief IMemEngine is an abstraction class that defined a set of memory
   operator interfaces.

*/
class IMemEngine {
public:
  virtual ~IMemEngine() = default;
  /** @brief reads data from the memory
   */
  virtual int64_t read(uint64_t addr, void *dest, size_t size) const = 0;
  /** @brief writes data to the memory
   */
  virtual int64_t write(uint64_t addr, const void *src, size_t size) = 0;
  /** @brief clean the memory data
   */
  virtual int64_t zeroize(uint64_t addr, size_t size) = 0;
  /** @brief Gets the memory total size
   */
  virtual size_t size() const = 0;
  /** @brief Checks the input addr is invalid
   */
  virtual bool invalid(uint64_t addr) const = 0;
  /** @brief Gets the memory information
   */
  virtual bool get_info(uint64_t addr, uint64_t &base, size_t &size) const {
    (void)addr;
    base = 0;
    size = this->size();
    return true;
  }
};
} // namespace sim_aipu
#endif // !__MEM_ENGINE_BASE_H__
