// runtime_meshdata_test.cc
// Regression test for concurrent getMeshData()/getTextureData() access.
//
// Bug: GetMeshData() used lazy initialization of mesh_data_ (unique_ptr).
// When multiple threads (Metal render + video capture) called it concurrently
// on a newly loaded runtime, the unique_ptr reassignment freed the wrapper
// while another thread held a dangling pointer, crashing in
// vector<MJMeshInfo>::empty() via MJMeshDataGetMeshes().
//
// Fix: Eagerly create mesh_data_/texture_data_ in ExtractMeshData()/
// ExtractTextureData(), eliminating the concurrent lazy-init race.

#include "mjc_physics_runtime.h"

#include <gtest/gtest.h>

#include <atomic>
#include <string>
#include <thread>
#include <vector>

// Inline MuJoCo XML with a mesh (8 verts, 12 faces → box) and a texture.
// Uses inline vertex/face attributes so no external files are needed.
static const char* kMeshModel = R"(
<mujoco>
  <asset>
    <mesh name="box"
          vertex="0 0 0  1 0 0  1 1 0  0 1 0  0 0 1  1 0 1  1 1 1  0 1 1"
          face="0 1 2  0 2 3  4 6 5  4 7 6  0 4 5  0 5 1  2 6 7  2 7 3  0 3 7  0 7 4  1 5 6  1 6 2"/>
    <texture name="grid" type="2d" builtin="checker" width="64" height="64"
             rgb1="0.9 0.9 0.9" rgb2="0.1 0.1 0.1"/>
    <material name="grid_mat" texture="grid"/>
  </asset>
  <worldbody>
    <body>
      <geom type="mesh" mesh="box" material="grid_mat"/>
    </body>
  </worldbody>
</mujoco>
)";

// =============================================================================
// Test: After loading a model with meshes, concurrent getMeshData() calls from
// multiple threads should all return the same non-null pointer and allow safe
// access to the underlying data.
// =============================================================================

TEST(RuntimeMeshDataTest, ConcurrentGetMeshDataReturnsSamePointer) {
    MJRuntimeConfig config{};
    config.instanceIndex = 0;
    config.udpPort = 0;  // disable UDP for test
    auto* runtime = MJSimulationRuntime::create(config);
    ASSERT_NE(runtime, nullptr);

    std::string error;
    bool ok = runtime->loadModelXML(kMeshModel, &error);
    ASSERT_TRUE(ok) << "Failed to load model: " << error;

    // Verify mesh data is available after load
    auto* meshData = runtime->getMeshData();
    ASSERT_NE(meshData, nullptr) << "getMeshData() returned null after loading mesh model";
    EXPECT_GT(meshData->meshCount(), 0);

    auto* meshInfoPtr = MJMeshDataGetMeshes(meshData);
    ASSERT_NE(meshInfoPtr, nullptr);

    // Spawn multiple threads that all call getMeshData() concurrently,
    // simulating the Metal render thread + video capture threads.
    constexpr int kNumThreads = 8;
    constexpr int kIterations = 10000;
    std::vector<std::thread> threads;
    std::atomic<int> pointer_mismatches{0};
    std::atomic<int> null_info{0};
    MJMeshData* expectedPtr = meshData;

    for (int t = 0; t < kNumThreads; t++) {
        threads.emplace_back([&]() {
            for (int i = 0; i < kIterations; i++) {
                auto* md = runtime->getMeshData();
                if (md != expectedPtr) {
                    pointer_mismatches.fetch_add(1, std::memory_order_relaxed);
                    continue;
                }
                // Access the data — this is where the original bug crashed
                auto* meshes = MJMeshDataGetMeshes(md);
                if (!meshes) {
                    null_info.fetch_add(1, std::memory_order_relaxed);
                }
            }
        });
    }

    for (auto& t : threads) {
        t.join();
    }

    EXPECT_EQ(pointer_mismatches.load(), 0)
        << "getMeshData() returned different pointers across threads";
    EXPECT_EQ(null_info.load(), 0)
        << "MJMeshDataGetMeshes() returned null for valid mesh data";

    MJSimulationRuntime::destroy(runtime);
}

// =============================================================================
// Test: Same as above but for getTextureData().
// =============================================================================

TEST(RuntimeMeshDataTest, ConcurrentGetTextureDataReturnsSamePointer) {
    MJRuntimeConfig config{};
    config.instanceIndex = 1;
    config.udpPort = 0;
    auto* runtime = MJSimulationRuntime::create(config);
    ASSERT_NE(runtime, nullptr);

    std::string error;
    bool ok = runtime->loadModelXML(kMeshModel, &error);
    ASSERT_TRUE(ok) << "Failed to load model: " << error;

    auto* texData = runtime->getTextureData();
    ASSERT_NE(texData, nullptr) << "getTextureData() returned null after loading textured model";
    EXPECT_GT(texData->textureCount(), 0);

    constexpr int kNumThreads = 8;
    constexpr int kIterations = 10000;
    std::vector<std::thread> threads;
    std::atomic<int> pointer_mismatches{0};
    std::atomic<int> null_info{0};
    MJTextureData* expectedPtr = texData;

    for (int t = 0; t < kNumThreads; t++) {
        threads.emplace_back([&]() {
            for (int i = 0; i < kIterations; i++) {
                auto* td = runtime->getTextureData();
                if (td != expectedPtr) {
                    pointer_mismatches.fetch_add(1, std::memory_order_relaxed);
                    continue;
                }
                auto* textures = MJTextureDataGetTextures(td);
                if (!textures) {
                    null_info.fetch_add(1, std::memory_order_relaxed);
                }
            }
        });
    }

    for (auto& t : threads) {
        t.join();
    }

    EXPECT_EQ(pointer_mismatches.load(), 0);
    EXPECT_EQ(null_info.load(), 0);

    MJSimulationRuntime::destroy(runtime);
}

// =============================================================================
// Test: Unload then reload model, then concurrent getMeshData() — reproduces
// the exact gRPC "Unload → LoadModel" crash scenario.
// =============================================================================

TEST(RuntimeMeshDataTest, ReloadModelThenConcurrentAccess) {
    MJRuntimeConfig config{};
    config.instanceIndex = 2;
    config.udpPort = 0;
    auto* runtime = MJSimulationRuntime::create(config);
    ASSERT_NE(runtime, nullptr);

    std::string error;

    // Initial load
    ASSERT_TRUE(runtime->loadModelXML(kMeshModel, &error)) << error;
    auto* ptr1 = runtime->getMeshData();
    ASSERT_NE(ptr1, nullptr);

    // Unload (simulates gRPC Unload RPC)
    runtime->unload();
    EXPECT_EQ(runtime->getMeshData(), nullptr)
        << "getMeshData() should return null after unload";
    EXPECT_EQ(runtime->getTextureData(), nullptr)
        << "getTextureData() should return null after unload";

    // Reload (simulates gRPC LoadModel RPC)
    ASSERT_TRUE(runtime->loadModelXML(kMeshModel, &error)) << error;

    auto* ptr2 = runtime->getMeshData();
    ASSERT_NE(ptr2, nullptr);
    EXPECT_GT(ptr2->meshCount(), 0);

    // Concurrent access after reload — the exact crash scenario
    constexpr int kNumThreads = 8;
    constexpr int kIterations = 10000;
    std::vector<std::thread> threads;
    std::atomic<int> failures{0};

    for (int t = 0; t < kNumThreads; t++) {
        threads.emplace_back([&]() {
            for (int i = 0; i < kIterations; i++) {
                auto* md = runtime->getMeshData();
                if (md != ptr2) {
                    failures.fetch_add(1, std::memory_order_relaxed);
                    continue;
                }
                // Access mesh info — crashes here with the old lazy-init bug
                auto* meshes = MJMeshDataGetMeshes(md);
                if (!meshes) {
                    failures.fetch_add(1, std::memory_order_relaxed);
                    continue;
                }
                // Also read vertex/face pointers
                auto* verts = MJMeshDataGetVertices(md);
                auto* faces = MJMeshDataGetFaces(md);
                if (!verts || !faces) {
                    failures.fetch_add(1, std::memory_order_relaxed);
                }
            }
        });
    }

    for (auto& t : threads) {
        t.join();
    }

    EXPECT_EQ(failures.load(), 0);

    MJSimulationRuntime::destroy(runtime);
}
