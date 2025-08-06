#pragma once

#include <queue>
#include <mutex>

namespace rviz_mesh_tools_plugins
{


template <typename T>
class ThreadSafeQueue: protected std::queue<T>
{
public:
  T pop()
  {
    std::lock_guard lock(mutex_);
    T elem = this->std::queue<T>::front();
    this->std::queue<T>::pop();
    return elem;
  }

  std::vector<T> popAll()
  {
    std::lock_guard lock(mutex_);
    std::vector<T> data;
    data.reserve(this->size());

    while(!this->empty())
    {
      data.push_back(this->std::queue<T>::front());
      this->std::queue<T>::pop();
    }
    return data;
  }

  void push(const T& elem)
  {
    std::lock_guard lock(mutex_);
    this->std::queue<T>::push(elem);
  }

private:
  std::mutex mutex_;
};

} // namespace rviz_mesh_tools_plugins
