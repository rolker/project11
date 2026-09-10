cmake_minimum_required(VERSION 3.8)
project(marine_bathymetry_store)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(marine_autonomy REQUIRED)
find_package(marine_tiled_raster_store REQUIRED)
find_package(geographic_msgs REQUIRED)
# nlohmann_json backs the source registry (registry.json sidecar, #178). Declared
# explicitly here (and in package.xml) — it was only available transitively
# before, which is not a dependency this package may rely on.
find_package(nlohmann_json REQUIRED)
# GDAL is used directly by the GeoTIFF importer (geotiff_import.cpp): it opens
# the source raster and reads bands/geotransform/SRS. tile_io still delegates its
# GeoTIFF I/O to marine_tiled_raster_store; the importer needs GDAL directly, so
# declare it explicitly (also in package.xml).
find_package(GDAL REQUIRED)
# The S-102 importer (#278) additionally needs OpenSSL (SHA256 verification of
# catalog-listed tile digests) and marine_vertical_datum (#274, per-cell
# MLLW->ellipsoid at import). Both stay PRIVATE to the internal
# s102_import_core target below — they never enter this package's export.
find_package(OpenSSL REQUIRED)
find_package(marine_vertical_datum REQUIRED)

add_library(${PROJECT_NAME}
  src/bathymetry_store.cpp
  src/depth_adaptive_level.cpp
  src/geotiff_import.cpp
  src/overview_pyramid.cpp
  src/query.cpp
  src/registry.cpp
  src/tile_io.cpp
)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

# Build with position-independent code so this (static) library can be linked
# into SHARED consumers — notably the bathymetry_layer Nav2 costmap plugin
# (#164), which is a shared object loaded by pluginlib. Without PIC the link of
# the plugin fails (R_X86_64_PC32 relocation against source_layers_by_priority).
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
# marine_autonomy + geographic_msgs are in the public headers (GGGS / GeoPoint);
# marine_tiled_raster_store is public too — BathymetryTile wraps its
# TiledRasterTile<double>. GDAL arrives transitively through that package (which
# uses it privately in tile_io.cpp); this package no longer references GDAL.
target_link_libraries(${PROJECT_NAME} PUBLIC
  marine_autonomy::marine_autonomy
  marine_tiled_raster_store::marine_tiled_raster_store
  ${geographic_msgs_TARGETS}
)
# nlohmann_json is header-only and used only inside registry.cpp (the public
# registry.hpp keeps it out of its interface); GDAL is used only inside
# geotiff_import.cpp (the public geotiff_import.hpp keeps it out of its
# interface). Both are PRIVATE.
target_link_libraries(${PROJECT_NAME} PRIVATE
  nlohmann_json::nlohmann_json
  ${GDAL_LIBRARIES}
)
target_include_directories(${PROJECT_NAME} PRIVATE ${GDAL_INCLUDE_DIRS})

# Headers are laid out as include/${PROJECT_NAME}/*.hpp, so install the include/
# tree to `include` (not include/${PROJECT_NAME}) to avoid a doubled
# include/${PROJECT_NAME}/${PROJECT_NAME}/ path; the INSTALL_INTERFACE above is
# `include` to match.
install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# The `import_geotiff` CLI: import a GeoTIFF into a store layer's fused surface.
add_executable(import_geotiff src/import_geotiff_main.cpp)
# GDAL directly: the CLI reads the source raster's geotransform to window the
# staged-tile adoption (#339), so it needs the headers itself rather than only
# through ${PROJECT_NAME}.
target_link_libraries(import_geotiff ${PROJECT_NAME}
  marine_vertical_datum::marine_vertical_datum ${GDAL_LIBRARIES})
target_include_directories(import_geotiff PRIVATE ${GDAL_INCLUDE_DIRS})

# The `build_depth_overviews` CLI (ADR-0010 D9 / ADR-0011): offline batch builder
# of the shallowest-preserving depth overview pyramid for a draft/processed layer.
# Thin main() over the library's overview_pyramid production path; ${PROJECT_NAME}
# already carries the marine_tiled_raster_store (fold engine + GeoTIFF I/O) and
# marine_autonomy (GGGS) links it needs.
add_executable(build_depth_overviews src/build_depth_overviews.cpp)
target_link_libraries(build_depth_overviews ${PROJECT_NAME})

# S-102 importer core (#278): discover/fetch/convert/run stages as an internal
# static library so the pipeline is testable without spawning the CLI. Its
# headers live under src/s102/ and are NOT installed; nothing here is exported.
add_library(s102_import_core STATIC
  src/s102/catalog.cpp
  src/s102/convert.cpp
  src/s102/fetch.cpp
  src/s102/run.cpp
  src/s102/vdatum_provider.cpp
)
target_include_directories(s102_import_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>")
target_link_libraries(s102_import_core PUBLIC
  ${PROJECT_NAME}
  marine_vertical_datum::marine_vertical_datum
)
target_link_libraries(s102_import_core PRIVATE
  ${GDAL_LIBRARIES}
  OpenSSL::Crypto
  nlohmann_json::nlohmann_json
)
target_include_directories(s102_import_core PRIVATE ${GDAL_INCLUDE_DIRS})

# The `s102_import` CLI: fetch + convert + import NOAA S-102 tiles for an area.
add_executable(s102_import src/s102_import_main.cpp)
target_link_libraries(s102_import s102_import_core)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
# nlohmann_json and GDAL are PRIVATE link deps, but a static library propagates them
# into the export's INTERFACE_LINK_LIBRARIES as $<LINK_ONLY:...>, so a consumer that
# just find_package(marine_bathymetry_store) + links it must still resolve those
# targets at generate time. Export them so consumers don't each have to re-find them
# (#203; cube_bathymetry#57 hit this).
ament_export_dependencies(
  marine_autonomy marine_tiled_raster_store geographic_msgs nlohmann_json GDAL)

install(TARGETS import_geotiff s102_import build_depth_overviews
  RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_store test/test_store.cpp)
  target_link_libraries(test_store ${PROJECT_NAME})

  # The depth -> GGGS level policy (uma-ADR-0010 D9 as amended by #369): level
  # transition depths, both clamps, monotonicity and the fail-loud edge cases.
  # Pure arithmetic over the GGGS level table — no disk, no GDAL, so the ament
  # default timeout is plenty.
  ament_add_gtest(test_depth_adaptive_level test/test_depth_adaptive_level.cpp)
  target_link_libraries(test_depth_adaptive_level ${PROJECT_NAME})

  ament_add_gtest(test_query test/test_query.cpp)
  target_link_libraries(test_query ${PROJECT_NAME})

  ament_add_gtest(test_tile_io test/test_tile_io.cpp)
  # nlohmann_json is used directly in the test (registry tamper fixtures);
  # link it explicitly since it is PRIVATE in the library target.
  target_link_libraries(test_tile_io ${PROJECT_NAME} nlohmann_json::nlohmann_json)

  # The depth overview-pyramid builder tests (ADR-0010 D9 / ADR-0011): fold policy
  # (shallowest-preserving, {depth,σ} pair coherence, no-upsample invariant),
  # malformed-filename skip -> swap refusal, end-to-end sidecar swap, and the
  # tile_io loader's silent skip of overviews/. GeoTIFF I/O comes transitively from
  # marine_tiled_raster_store (linked via ${PROJECT_NAME}) — no direct GDAL/OpenSSL.
  # TIMEOUT above the ament default of 60s: these are integration tests over real
  # 960x960 two-band GeoTIFFs (the point — the fold, the swap and the manifest are
  # disk behaviour), and #331 added the mixed-level and golden-pin cases on top.
  # The fixtures are already kept to the minimum tile count each assertion needs.
  ament_add_gtest(test_depth_overview test/test_depth_overview.cpp TIMEOUT 300)
  target_link_libraries(test_depth_overview ${PROJECT_NAME})
  # The pinned single-level regression guard (#331) reads its fixture helpers
  # from test/ and its golden digest — captured from the PRE-#331 binary — from
  # test/data/, injected as a compile definition so the test needs no runtime cwd
  # assumption.
  target_include_directories(test_depth_overview PRIVATE test)
  target_compile_definitions(test_depth_overview PRIVATE
    DEPTH_OVERVIEW_GOLDEN_FILE="${CMAKE_CURRENT_SOURCE_DIR}/test/data/depth_overview_single_level_golden.txt")

  # The GeoTIFF importer test writes fixture rasters with GDAL directly, so it
  # links GDAL (PRIVATE in the library target).
  ament_add_gtest(test_geotiff_import test/test_geotiff_import.cpp)
  target_link_libraries(test_geotiff_import ${PROJECT_NAME} ${GDAL_LIBRARIES})
  target_include_directories(test_geotiff_import PRIVATE ${GDAL_INCLUDE_DIRS})

  # Subprocess test of the built import_geotiff binary (#289): the arg-parsing
  # paths in main() (--stage/--commit mode selection, positional counts,
  # chart-only stage guard, missing-operand handling) are otherwise untested.
  # The binary path is injected as a compile definition; the test writes a GDAL
  # fixture raster and invokes the binary, so it links GDAL and depends on the
  # binary being built first.
  ament_add_gtest(test_import_geotiff_cli test/test_import_geotiff_cli.cpp)
  target_link_libraries(test_import_geotiff_cli ${PROJECT_NAME} ${GDAL_LIBRARIES})
  target_include_directories(test_import_geotiff_cli PRIVATE ${GDAL_INCLUDE_DIRS})
  target_compile_definitions(test_import_geotiff_cli PRIVATE
    IMPORT_GEOTIFF_BINARY="$<TARGET_FILE:import_geotiff>")
  add_dependencies(test_import_geotiff_cli import_geotiff)

  # S-102 importer tests (#278). All offline: fixture GeoPackages, file://
  # fetches, synthetic projected rasters, ConstantOffsetProvider. They build
  # fixtures with GDAL directly, so they link it (PRIVATE in the core lib).
  ament_add_gtest(test_s102_catalog test/test_s102_catalog.cpp)
  target_link_libraries(test_s102_catalog s102_import_core ${GDAL_LIBRARIES})
  target_include_directories(test_s102_catalog PRIVATE ${GDAL_INCLUDE_DIRS})

  ament_add_gtest(test_s102_fetch test/test_s102_fetch.cpp)
  target_link_libraries(test_s102_fetch s102_import_core
    nlohmann_json::nlohmann_json)

  ament_add_gtest(test_s102_convert test/test_s102_convert.cpp)
  target_link_libraries(test_s102_convert s102_import_core ${GDAL_LIBRARIES})
  target_include_directories(test_s102_convert PRIVATE ${GDAL_INCLUDE_DIRS})

  ament_add_gtest(test_s102_run test/test_s102_run.cpp)
  target_link_libraries(test_s102_run s102_import_core ${GDAL_LIBRARIES})
  target_include_directories(test_s102_run PRIVATE ${GDAL_INCLUDE_DIRS})

  # Subprocess test of the built s102_import binary (#288): the --cache
  # resolution in main() (canonical default under $HOME when omitted, explicit
  # override, fail-loud on unset HOME, still-required args) is otherwise
  # untested. Offline-only, so no GDAL/network fixtures — the binary path is
  # injected as a compile definition and the binary is built first.
  ament_add_gtest(test_s102_import_cli test/test_s102_import_cli.cpp)
  target_compile_definitions(test_s102_import_cli PRIVATE
    S102_IMPORT_BINARY="$<TARGET_FILE:s102_import>")
  add_dependencies(test_s102_import_cli s102_import)
endif()

ament_package()
