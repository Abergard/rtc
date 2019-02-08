#pragma once

#include <vector>
#include <map>
#include <set>

namespace std
{

template<typename T>
std::ostream& operator<<(std::ostream& ss, const std::vector<T>& v) noexcept
{
  ss << "{ ";

  if(not v.empty())
  {
    ss << v.front();
    for(auto e = std::next(v.begin()); e != v.end(); ++e) ss << ", " << *e;
  }

  return ss << " }";
}

template<typename T>
std::ostream& operator<<(std::ostream& ss, const std::set<T>& v) noexcept
{
  ss << "{ ";

  if(not v.empty())
  {
    ss << *v.begin();
    for(auto e = std::next(v.begin()); e != v.end(); ++e) ss << ", " << *e;
  }

  return ss << " }";
}

template<typename K, typename V>
std::ostream& operator<<(std::ostream& ss, const std::map<K, V>& v) noexcept
{
  ss << "{ ";

  if(not v.empty())
  {
    ss << "{ " << v.begin()->first << " = " << v.begin()->second << " }";
    for(auto e = std::next(v.begin()); e != v.end(); ++e) ss << ",\n{ " << e->first << " = " << e->second << " }";
  }

  return ss << " }";
}

}