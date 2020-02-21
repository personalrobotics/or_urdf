/** \file catkin_finder.h
 * \brief Utility to resolve package to file URIs in catkin workspaces
 * \author Michael Koval <mkoval@cs.cmu.edu>
 * \author Chris Dellin <cdellin@gmail.com>
 * \date 2015
 */

/* (C) Copyright 2015 Carnegie Mellon University */

#include <fstream>
#include <iostream>
#include <tinyxml2.h>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <openrave/openrave.h>
#include "catkin_finder.h"

static const std::string CATKIN_MARKER(".catkin");

namespace {

std::string getPackageNameFromXML(const std::string& _path)
{
  using tinyxml2::XMLHandle;
  using tinyxml2::XMLElement;

  tinyxml2::XMLDocument document;
  if (document.LoadFile(_path.c_str()))
  {
    RAVELOG_WARN("Failed loading package.xml file '%s': %s\n",
           _path.c_str(), document.ErrorStr());
    return "";
  }

  XMLHandle root_handle(document.RootElement());
  XMLHandle name_handle = root_handle.FirstChildElement("name");
  XMLElement* name_element = name_handle.ToElement();

  if (!name_element)
  {
    RAVELOG_WARN("Failed loading package.xml file '%s': "
      "File does not contain a <name> element.\n", _path.c_str());
    return "";
  }

  if (!name_element->GetText())
  {
    RAVELOG_WARN("Failed loading package.xml file '%s': "
      "<name> element is empty.\n", _path.c_str());
    return "";
  }

  std::string package_name = name_element->GetText();
  boost::algorithm::trim(package_name);

  if (package_name.empty())
  {
    RAVELOG_WARN("Failed loading package.xml file '%s': "
      "<name> element is empty.\n", _path.c_str());
    return "";
  }

  return package_name;
}

void searchForPackages(const boost::filesystem::path& _packagePath,
  std::unordered_map<std::string, std::string>& _packageMap)
{
  using boost::filesystem::directory_iterator;
  using boost::filesystem::path;
  using boost::filesystem::file_status;
  using boost::filesystem::exists;

  // Ignore this directory if it contains a CATKIN_IGNORE file.
  const path catkin_ignore_path = _packagePath / "CATKIN_IGNORE";
  if (exists(catkin_ignore_path))
    return;

  // Try loading the package.xml file.
  const path package_xml_path = _packagePath / "package.xml";
  if (exists(package_xml_path))
  {
    const std::string package_name = getPackageNameFromXML(
      package_xml_path.string());
    if (!package_name.empty())
    {
      const auto result = _packageMap.insert(
        std::make_pair(package_name, _packagePath.string()));
      if (!result.second)
      {
        RAVELOG_WARN("Found two package.xml files for package '%s': '%s' and '%s'.\n",
               package_name.c_str(),
               result.first->second.c_str(),
               _packagePath.c_str());
      }
      return; // Don't search for packages inside packages.
    }
  }

  // Recurse on subdirectories.
  directory_iterator it(_packagePath);
  directory_iterator end;

  while (it != end)
  {
    boost::system::error_code status_error;
    const file_status status = it->status(status_error);
    if (status_error)
    {
      RAVELOG_WARN("Failed recursing into directory '%s'.\n", it->path().c_str());
      continue;
    }

    if (status.type() == boost::filesystem::directory_file)
      searchForPackages(it->path().string(), _packageMap);

    ++it;
  }
}

} // anonymous namespace

or_urdf::CatkinFinder::CatkinFinder():
  mWorkspaces(getWorkspaces())
{
}

std::vector<or_urdf::CatkinFinder::Workspace> or_urdf::CatkinFinder::getWorkspaces()
{
  std::vector<or_urdf::CatkinFinder::Workspace> workspaces;

  // initialize workspaces
  // (from CatkinResourceRetriever::getWorkspaces())
  using boost::filesystem::path;

  const char *cmake_prefix_path = std::getenv("CMAKE_PREFIX_PATH");
  if (!cmake_prefix_path)
  {
    RAVELOG_WARN("The CMAKE_PREFIX_PATH"
              " environmental variable is not defined. Did you source"
              " 'setup.bash' in this shell?\n");
    return workspaces;
  }

  // Split CMAKE_PREFIX_PATH by the ':' path separator delimiter.
  std::vector<std::string> workspace_candidates;
  boost::split(workspace_candidates, cmake_prefix_path, boost::is_any_of(":"));

  for (const std::string& workspace_path : workspace_candidates)
  {
    // Skip empty or non-existant workspace directories
    if (workspace_path.empty())
      continue;
    boost::filesystem::path workspace_bpath(workspace_path);
    if (!boost::filesystem::is_directory(workspace_bpath))
      continue;

    // Construct workspace
    or_urdf::CatkinFinder::Workspace workspace;
    workspace.mPath = workspace_path;

    // Filter out directories that are missing the Catkin marker file. If the
    // marker file exists, then we also read its contents to build a list of all
    // source directories.
    boost::filesystem::path marker_bpath = workspace_bpath / CATKIN_MARKER;
    if (boost::filesystem::exists(marker_bpath))
    {
      std::ifstream ifs(marker_bpath.c_str());
      std::string contents( (std::istreambuf_iterator<char>(ifs) ),
                            (std::istreambuf_iterator<char>()    ) );

      // Split the string into a list of paths by the ';' delimiter. I'm not
      // sure why this doesn't use the standard path separator delimitor ':'.
      std::vector<std::string> source_paths;
      if (!contents.empty())
        boost::split(source_paths, contents, boost::is_any_of(";"));

      for (const std::string& source_path : source_paths)
        searchForPackages(source_path, workspace.mSourceMap);
    }
    else
    {
      RAVELOG_WARN("Failed reading package"
                " source directories from the marker file '%s"
                "'. Resources in the source"
                " space of this workspace will not resolve.\n",
                marker_bpath.c_str());
    }

    workspaces.push_back(workspace);
  }

  return workspaces;
}


or_urdf::CatkinFinder::~CatkinFinder()
{
}

std::string or_urdf::CatkinFinder::find(const std::string & uri) const
{
  using boost::filesystem::path;

  // ensure uri starts with package://
  if (uri.substr(0,10) != "package://")
  {
    RAVELOG_ERROR("CatkinFinder passed a non package:// path!\n");
    return "";
  }
  std::string nonprefix = uri.substr(10);

  // split into package name and relative path
  std::string packageName;
  std::string relativePath;
  size_t islash = nonprefix.find('/');
  if (islash == nonprefix.npos)
  {
    packageName = uri;
  }
  else
  {
    packageName = nonprefix.substr(0,islash);
    relativePath = nonprefix.substr(islash+1);
  }

  // Sequentially check each chained workspace.
  for (const Workspace& workspace : mWorkspaces)
  {
    // First check the 'devel' or 'install' space.
    const path develPath = path(workspace.mPath) / "share" / packageName / relativePath;
    if (boost::filesystem::exists(develPath))
      return develPath.native();

    // Next, check the source space.
    const auto it = workspace.mSourceMap.find(packageName);
    if (it != std::end(workspace.mSourceMap))
    {
      const path sourcePath = path(it->second) / relativePath;
      if (boost::filesystem::exists(sourcePath))
        return sourcePath.native();
    }
  }

  RAVELOG_WARN("Could not find file '%s' under catkin package '%s'!\n",
    packageName.c_str(), relativePath.c_str());
  return "";
}
