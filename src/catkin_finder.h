/** \file catkin_finder.h
 * \brief Utility to resolve package to file URIs in catkin workspaces
 * \author Michael Koval <mkoval@cs.cmu.edu>
 * \author Chris Dellin <cdellin@gmail.com>
 * \date 2015
 */

/* (C) Copyright 2015 Carnegie Mellon University */

#ifndef OR_URDF_CATKIN_FINDER_H_
#define OR_URDF_CATKIN_FINDER_H_

#include <string>
#include <unordered_map>
#include <vector>

namespace or_urdf {

/// Utility to resolve "package://" paths against catkin workspaces.
/// Should replicate the behaviour of the tool
///    catkin_find package relative/path
///
/// Resolution code borrowed heavily from
/// aikido::util::CatkinResourceRetriever,
/// simplified to remove DART dependencies
/// and to do local path resolution only
class CatkinFinder
{
public:
  CatkinFinder();
  ~CatkinFinder();

  // input: package:// URI
  // output: native local filesystem path if found ("" otherwise)
  std::string find(const std::string & uri) const;

private:
  struct Workspace {
    std::string mPath;
    std::unordered_map<std::string, std::string> mSourceMap;
  };
  std::vector<Workspace> mWorkspaces;
  static std::vector<Workspace> getWorkspaces();
};

} // namespace or_urdf

#endif // ifndef OR_URDF_CATKIN_FINDER_H_
