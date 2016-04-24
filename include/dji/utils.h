#ifndef UTILS_H
#define UTILS_H

#include <pthread.h>
#include <semaphore.h>

#include <dji/guidance.h>

namespace dji {

class DJI_lock {
 public:
  DJI_lock();
  ~DJI_lock();
  void enter();
  void leave();

 private:
  pthread_mutex_t m_lock;
};

class DJI_event {
 public:
  DJI_event();
  ~DJI_event();
  int set_event();
  int wait_event();

 private:
  sem_t m_sem;
};

void sleep(int microsecond);

/**
 * @brief Convert a DJI guidance error code to a verbose string
 */
const char *err_code_str(e_sdk_err_code e);

}  // namespace dji

#endif  // UTILS_H
