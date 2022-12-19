// Copyright 2022 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TOPNODE__VISIBILITY_HPP_
#define TOPNODE__VISIBILITY_HPP_

#ifdef __cplusplus
extern "C" {
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__

#ifdef __GNUC__
#define TOPNODE_EXPORT __attribute__((dllexport))
#define TOPNODE_IMPORT __attribute__((dllimport))
#else
#define TOPNODE_EXPORT __declspec(dllexport)
#define TOPNODE_IMPORT __declspec(dllimport)
#endif

#ifdef TOPNODE_DLL
#define TOPNODE_PUBLIC TOPNODE_EXPORT
#else
#define TOPNODE_PUBLIC TOPNODE_IMPORT
#endif

#define TOPNODE_PUBLIC_TYPE TOPNODE_PUBLIC

#define TOPNODE_LOCAL

#else

#define TOPNODE_EXPORT __attribute__((visibility("default")))
#define TOPNODE_IMPORT

#if __GNUC__ >= 4
#define TOPNODE_PUBLIC __attribute__((visibility("default")))
#define TOPNODE_LOCAL __attribute__((visibility("hidden")))
#else
#define TOPNODE_PUBLIC
#define TOPNODE_LOCAL
#endif

#define TOPNODE_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // TOPNODE__VISIBILITY_HPP_
