# Details

Date : 2026-10-08 12:11:33

Directory i:\\论文\\models\\nbvplanner-master

Total : 38 files,  3508 codes, 412 comments, 452 blanks, all 4372 lines

[Summary](results.md) / Details / [Diff Summary](diff.md) / [Diff Details](diff-details.md)

## Files
| filename | language | code | comment | blank | total |
| :--- | :--- | ---: | ---: | ---: | ---: |
| [README.md](/README.md) | Markdown | 42 | 0 | 17 | 59 |
| [interface\_nbvp\_rotors/CMakeLists.txt](/interface_nbvp_rotors/CMakeLists.txt) | CMake | 18 | 0 | 5 | 23 |
| [interface\_nbvp\_rotors/launch/area\_exploration.launch](/interface_nbvp_rotors/launch/area_exploration.launch) | XML | 29 | 0 | 4 | 33 |
| [interface\_nbvp\_rotors/launch/flat\_exploration.launch](/interface_nbvp_rotors/launch/flat_exploration.launch) | XML | 27 | 0 | 4 | 31 |
| [interface\_nbvp\_rotors/launch/mav\_inspector.launch](/interface_nbvp_rotors/launch/mav_inspector.launch) | XML | 66 | 1 | 3 | 70 |
| [interface\_nbvp\_rotors/launch/multiagent\_area\_exploration.launch](/interface_nbvp_rotors/launch/multiagent_area_exploration.launch) | XML | 89 | 0 | 8 | 97 |
| [interface\_nbvp\_rotors/launch/multiagent\_flat\_exploration.launch](/interface_nbvp_rotors/launch/multiagent_flat_exploration.launch) | XML | 77 | 0 | 8 | 85 |
| [interface\_nbvp\_rotors/package.xml](/interface_nbvp_rotors/package.xml) | XML | 23 | 0 | 7 | 30 |
| [interface\_nbvp\_rotors/resource/area\_exploration.yaml](/interface_nbvp_rotors/resource/area_exploration.yaml) | YAML | 33 | 2 | 5 | 40 |
| [interface\_nbvp\_rotors/resource/exploration.yaml](/interface_nbvp_rotors/resource/exploration.yaml) | YAML | 33 | 2 | 5 | 40 |
| [interface\_nbvp\_rotors/src/exploration.cpp](/interface_nbvp_rotors/src/exploration.cpp) | C++ | 107 | 26 | 14 | 147 |
| [kdtree/CMakeLists.txt](/kdtree/CMakeLists.txt) | CMake | 19 | 0 | 6 | 25 |
| [kdtree/include/kdtree/kdtree.h](/kdtree/include/kdtree/kdtree.h) | C++ | 37 | 57 | 21 | 115 |
| [kdtree/package.xml](/kdtree/package.xml) | XML | 15 | 0 | 5 | 20 |
| [kdtree/src/kdtree.c](/kdtree/src/kdtree.c) | C | 597 | 49 | 116 | 762 |
| [multiagent\_collision\_check/CMakeLists.txt](/multiagent_collision_check/CMakeLists.txt) | CMake | 30 | 0 | 8 | 38 |
| [multiagent\_collision\_check/include/multiagent\_collision\_check/multiagent\_collision\_checker.h](/multiagent_collision_check/include/multiagent_collision_check/multiagent_collision_checker.h) | C++ | 18 | 15 | 9 | 42 |
| [multiagent\_collision\_check/package.xml](/multiagent_collision_check/package.xml) | XML | 14 | 0 | 7 | 21 |
| [multiagent\_collision\_check/src/multiagent\_collision\_checker.cpp](/multiagent_collision_check/src/multiagent_collision_checker.cpp) | C++ | 97 | 22 | 8 | 127 |
| [nbvplanner/CMakeLists.txt](/nbvplanner/CMakeLists.txt) | CMake | 48 | 0 | 9 | 57 |
| [nbvplanner/include/nbvplanner/mesh\_structure.h](/nbvplanner/include/nbvplanner/mesh_structure.h) | C++ | 58 | 15 | 9 | 82 |
| [nbvplanner/include/nbvplanner/nbvp.h](/nbvplanner/include/nbvplanner/nbvp.h) | C++ | 53 | 15 | 13 | 81 |
| [nbvplanner/include/nbvplanner/nbvp.hpp](/nbvplanner/include/nbvplanner/nbvp.hpp) | C++ | 388 | 33 | 27 | 448 |
| [nbvplanner/include/nbvplanner/rrt.h](/nbvplanner/include/nbvplanner/rrt.h) | C++ | 48 | 15 | 8 | 71 |
| [nbvplanner/include/nbvplanner/tree.h](/nbvplanner/include/nbvplanner/tree.h) | C++ | 99 | 15 | 14 | 128 |
| [nbvplanner/include/nbvplanner/tree.hpp](/nbvplanner/include/nbvplanner/tree.hpp) | C++ | 99 | 15 | 17 | 131 |
| [nbvplanner/package.xml](/nbvplanner/package.xml) | XML | 26 | 0 | 7 | 33 |
| [nbvplanner/src/mesh\_structure.cpp](/nbvplanner/src/mesh_structure.cpp) | C++ | 461 | 38 | 22 | 521 |
| [nbvplanner/src/nbv\_planner\_node.cpp](/nbvplanner/src/nbv_planner_node.cpp) | C++ | 12 | 0 | 4 | 16 |
| [nbvplanner/src/nbvp.cpp](/nbvplanner/src/nbvp.cpp) | C++ | 2 | 0 | 1 | 3 |
| [nbvplanner/src/rrt.cpp](/nbvplanner/src/rrt.cpp) | C++ | 691 | 42 | 28 | 761 |
| [nbvplanner/src/tree.cpp](/nbvplanner/src/tree.cpp) | C++ | 1 | 15 | 2 | 18 |
| [prune\_pointcloud/CMakeLists.txt](/prune_pointcloud/CMakeLists.txt) | CMake | 22 | 0 | 6 | 28 |
| [prune\_pointcloud/include/prune\_pointcloud/prune.h](/prune_pointcloud/include/prune_pointcloud/prune.h) | C++ | 21 | 15 | 5 | 41 |
| [prune\_pointcloud/launch/prune\_pointcloud.launch](/prune_pointcloud/launch/prune_pointcloud.launch) | XML | 13 | 0 | 1 | 14 |
| [prune\_pointcloud/package.xml](/prune_pointcloud/package.xml) | XML | 16 | 0 | 7 | 23 |
| [prune\_pointcloud/src/prune.cc](/prune_pointcloud/src/prune.cc) | C++ | 69 | 20 | 7 | 96 |
| [prune\_pointcloud/src/prune\_node.cc](/prune_pointcloud/src/prune_node.cc) | C++ | 10 | 0 | 5 | 15 |

[Summary](results.md) / Details / [Diff Summary](diff.md) / [Diff Details](diff-details.md)