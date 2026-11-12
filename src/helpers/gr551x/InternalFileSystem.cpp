#include "InternalFileSystem.h"
#include <gr551x_sdk.h>

#if !defined(GR551X_FS_ADDR) || !defined(GR551X_FS_SIZE)
  #error "GR551X_FS_ADDR / GR551X_FS_SIZE must be defined by the board variant"
#endif

#define LFS_BLOCK_SIZE 4096u     // flash sector
#define LFS_IO_SIZE    256u      // read/prog granularity (RAM cost: buffers of this size)

static bool inRange(uint32_t addr, uint32_t size) {
  return addr >= GR551X_FS_ADDR && addr + size <= GR551X_FS_ADDR + GR551X_FS_SIZE;
}

static int _flash_read(const struct lfs_config* c, lfs_block_t block, lfs_off_t off, void* buffer, lfs_size_t size) {
  uint32_t addr = GR551X_FS_ADDR + block * LFS_BLOCK_SIZE + off;
  if (!buffer || !inRange(addr, size)) return LFS_ERR_INVAL;
  return hal_flash_read(addr, (uint8_t*)buffer, size) == size ? LFS_ERR_OK : LFS_ERR_IO;
}

static int _flash_prog(const struct lfs_config* c, lfs_block_t block, lfs_off_t off, const void* buffer, lfs_size_t size) {
  uint32_t addr = GR551X_FS_ADDR + block * LFS_BLOCK_SIZE + off;
  if (!buffer || !inRange(addr, size)) return LFS_ERR_INVAL;
  gr551x_service();   // keep the watchdog fed during long write sequences
  return hal_flash_write(addr, (const uint8_t*)buffer, size) == size ? LFS_ERR_OK : LFS_ERR_IO;
}

static int _flash_erase(const struct lfs_config* c, lfs_block_t block) {
  uint32_t addr = GR551X_FS_ADDR + block * LFS_BLOCK_SIZE;
  if (!inRange(addr, LFS_BLOCK_SIZE)) return LFS_ERR_INVAL;
  gr551x_service();
  return hal_flash_erase(addr, LFS_BLOCK_SIZE) ? LFS_ERR_OK : LFS_ERR_IO;
}

static int _flash_sync(const struct lfs_config* c) {
  return LFS_ERR_OK;
}

static struct lfs_config _InternalFSConfig = {
  .context = NULL,
  .read = _flash_read,
  .prog = _flash_prog,
  .erase = _flash_erase,
  .sync = _flash_sync,
  .read_size = LFS_IO_SIZE,
  .prog_size = LFS_IO_SIZE,
  .block_size = LFS_BLOCK_SIZE,
  .block_count = GR551X_FS_SIZE / LFS_BLOCK_SIZE,
  .lookahead = 128,
  .read_buffer = NULL,
  .prog_buffer = NULL,
  .lookahead_buffer = NULL,
  .file_buffer = NULL
};

InternalFileSystem InternalFS;

InternalFileSystem::InternalFileSystem(void) : Adafruit_LittleFS(&_InternalFSConfig) { }

bool InternalFileSystem::begin(void) {
  if (!Adafruit_LittleFS::begin()) {
    this->format();
    if (!Adafruit_LittleFS::begin()) return false;
  }
  return true;
}
