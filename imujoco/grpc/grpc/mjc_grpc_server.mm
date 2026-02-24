// mjc_grpc_server.mm
// gRPC server implementation for SimulationControl service.

#import "mjc_grpc_server.h"

#include <algorithm>
#include <atomic>
#include <cstring>
#include <mutex>
#include <string>
#include <memory>

#include <grpcpp/grpcpp.h>
#include "schema/simulation_control.grpc.pb.h"
#include "schema/simulation_control.pb.h"

#include "imujoco/core/core/mjc_physics_runtime.h"
#include <mujoco/mujoco.h>

// MARK: - Proto State Mapping

static imujoco::SimulationState toProtoState(MJRuntimeState s) {
    switch (s) {
        case MJRuntimeState::Inactive: return imujoco::SIMULATION_STATE_INACTIVE;
        case MJRuntimeState::Loaded:   return imujoco::SIMULATION_STATE_LOADED;
        case MJRuntimeState::Running:  return imujoco::SIMULATION_STATE_RUNNING;
        case MJRuntimeState::Paused:   return imujoco::SIMULATION_STATE_PAUSED;
    }
    return imujoco::SIMULATION_STATE_UNSPECIFIED;
}

// MARK: - MJGrpcFillPhysicsState

void MJGrpcFillPhysicsState(MJGrpcPhysicsState* outState, MJSimulationRuntime* runtime) {
    if (!outState || !runtime) return;

    const mjModel* m = runtime->getModel();
    const mjData* d = runtime->getData();
    if (!m || !d) return;

    outState->valid = true;
    outState->time = d->time;

    outState->nq = std::min(m->nq, kMJGrpcMaxDof);
    outState->nv = std::min(m->nv, kMJGrpcMaxDof);
    outState->nu = std::min(m->nu, kMJGrpcMaxDof);
    outState->nsensordata = std::min(m->nsensordata, kMJGrpcMaxSensor);

    std::memcpy(outState->qpos, d->qpos, outState->nq * sizeof(double));
    std::memcpy(outState->qvel, d->qvel, outState->nv * sizeof(double));
    std::memcpy(outState->ctrl, d->ctrl, outState->nu * sizeof(double));
    std::memcpy(outState->sensordata, d->sensordata, outState->nsensordata * sizeof(double));

    outState->energyPotential = d->energy[0];
    outState->energyKinetic = d->energy[1];
}

// MARK: - SimulationControlServiceImpl

class SimulationControlServiceImpl final : public imujoco::SimulationControl::Service {
public:
    explicit SimulationControlServiceImpl(MJGrpcServerImpl* impl) : impl_(impl) {}

    grpc::Status ListInstances(grpc::ServerContext* ctx,
                               const imujoco::ListInstancesRequest* req,
                               imujoco::ListInstancesResponse* resp) override;

    grpc::Status GetInstanceInfo(grpc::ServerContext* ctx,
                                 const imujoco::GetInstanceInfoRequest* req,
                                 imujoco::InstanceInfo* resp) override;

    grpc::Status LoadModel(grpc::ServerContext* ctx,
                           const imujoco::LoadModelRequest* req,
                           imujoco::ControlResponse* resp) override;

    grpc::Status Unload(grpc::ServerContext* ctx,
                        const imujoco::InstanceRequest* req,
                        imujoco::ControlResponse* resp) override;

    grpc::Status Start(grpc::ServerContext* ctx,
                       const imujoco::InstanceRequest* req,
                       imujoco::ControlResponse* resp) override;

    grpc::Status Pause(grpc::ServerContext* ctx,
                       const imujoco::InstanceRequest* req,
                       imujoco::ControlResponse* resp) override;

    grpc::Status Reset(grpc::ServerContext* ctx,
                       const imujoco::InstanceRequest* req,
                       imujoco::ControlResponse* resp) override;

    grpc::Status ResetToKeyframe(grpc::ServerContext* ctx,
                                 const imujoco::ResetToKeyframeRequest* req,
                                 imujoco::ControlResponse* resp) override;

    grpc::Status Step(grpc::ServerContext* ctx,
                      const imujoco::InstanceRequest* req,
                      imujoco::ControlResponse* resp) override;

    grpc::Status GetState(grpc::ServerContext* ctx,
                          const imujoco::InstanceRequest* req,
                          imujoco::PhysicsState* resp) override;

private:
    MJGrpcServerImpl* impl_;
};

// MARK: - MJGrpcServerImpl (pimpl)

static constexpr int kMaxInstances = 4;

struct MJGrpcServerImpl {
    MJGrpcServerConfig config;

    std::atomic<uint64_t> rpcCount{0};
    std::mutex mutex;
    MJSimulationRuntime* runtimes[kMaxInstances] = {};
    std::string modelNames[kMaxInstances];

    MJGrpcLoadModelCallback loadCallback = nullptr;
    void* loadContext = nullptr;
    MJGrpcUnloadCallback unloadCallback = nullptr;
    void* unloadContext = nullptr;
    MJGrpcOperationCallback operationCallback = nullptr;
    void* operationContext = nullptr;
    MJGrpcGetStateCallback getStateCallback = nullptr;
    void* getStateContext = nullptr;
    MJGrpcGetInstanceInfoCallback getInstanceInfoCallback = nullptr;
    void* getInstanceInfoContext = nullptr;

    std::unique_ptr<SimulationControlServiceImpl> service;
    std::unique_ptr<grpc::Server> server;
};

// MARK: - Instance ID Validation

/// Returns true if instanceId is valid (1-based, within numInstances).
static bool validateInstanceId(int32_t instanceId, int32_t numInstances,
                               imujoco::ControlResponse* resp) {
    if (instanceId < 1 || instanceId > numInstances) {
        if (resp) {
            resp->set_success(false);
            resp->set_error("Invalid instance_id: " + std::to_string(instanceId));
        }
        return false;
    }
    return true;
}

// MARK: - SimulationControlServiceImpl — RPC Implementations

grpc::Status SimulationControlServiceImpl::ListInstances(
        grpc::ServerContext* /*ctx*/,
        const imujoco::ListInstancesRequest* /*req*/,
        imujoco::ListInstancesResponse* resp) {
    impl_->rpcCount.fetch_add(1, std::memory_order_relaxed);
    MJGrpcGetInstanceInfoCallback cb;
    void* cbCtx;
    int numInstances;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        cb = impl_->getInstanceInfoCallback;
        cbCtx = impl_->getInstanceInfoContext;
        numInstances = impl_->config.numInstances;
    }

    for (int i = 0; i < numInstances; i++) {
        auto* info = resp->add_instances();
        int instanceId = i + 1;
        info->set_instance_id(instanceId);

        MJGrpcInstanceInfo snapshot{};
        if (cb && cb(instanceId, &snapshot, cbCtx) && snapshot.valid) {
            info->set_state(toProtoState(static_cast<MJRuntimeState>(snapshot.state)));
            // Model name from gRPC-side storage (set by register/unregister under mutex)
            {
                std::lock_guard<std::mutex> lock(impl_->mutex);
                info->set_model_name(impl_->modelNames[i]);
            }
            info->set_udp_port(snapshot.udpPort);
            auto* s = info->mutable_stats();
            s->set_steps_per_second(snapshot.stepsPerSecond);
            s->set_tx_rate(snapshot.txRate);
            s->set_rx_rate(snapshot.rxRate);
            s->set_packets_sent(snapshot.packetsSent);
            s->set_packets_received(snapshot.packetsReceived);
            s->set_has_client(snapshot.hasClient);
        } else {
            info->set_state(imujoco::SIMULATION_STATE_INACTIVE);
        }
    }
    return grpc::Status::OK;
}

grpc::Status SimulationControlServiceImpl::GetInstanceInfo(
        grpc::ServerContext* /*ctx*/,
        const imujoco::GetInstanceInfoRequest* req,
        imujoco::InstanceInfo* resp) {
    impl_->rpcCount.fetch_add(1, std::memory_order_relaxed);
    int id = req->instance_id();
    int idx = id - 1;
    if (idx < 0 || idx >= impl_->config.numInstances) {
        return grpc::Status(grpc::INVALID_ARGUMENT, "Invalid instance ID");
    }

    MJGrpcGetInstanceInfoCallback cb;
    void* cbCtx;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        cb = impl_->getInstanceInfoCallback;
        cbCtx = impl_->getInstanceInfoContext;
    }

    resp->set_instance_id(id);

    MJGrpcInstanceInfo snapshot{};
    if (cb && cb(id, &snapshot, cbCtx) && snapshot.valid) {
        resp->set_state(toProtoState(static_cast<MJRuntimeState>(snapshot.state)));
        {
            std::lock_guard<std::mutex> lock(impl_->mutex);
            resp->set_model_name(impl_->modelNames[idx]);
        }
        resp->set_udp_port(snapshot.udpPort);
        auto* s = resp->mutable_stats();
        s->set_steps_per_second(snapshot.stepsPerSecond);
        s->set_tx_rate(snapshot.txRate);
        s->set_rx_rate(snapshot.rxRate);
        s->set_packets_sent(snapshot.packetsSent);
        s->set_packets_received(snapshot.packetsReceived);
        s->set_has_client(snapshot.hasClient);
    } else {
        resp->set_state(imujoco::SIMULATION_STATE_INACTIVE);
    }
    return grpc::Status::OK;
}

grpc::Status SimulationControlServiceImpl::LoadModel(
        grpc::ServerContext* /*ctx*/,
        const imujoco::LoadModelRequest* req,
        imujoco::ControlResponse* resp) {
    impl_->rpcCount.fetch_add(1, std::memory_order_relaxed);
    if (!validateInstanceId(req->instance_id(), impl_->config.numInstances, resp))
        return grpc::Status::OK;
    MJGrpcLoadModelCallback cb;
    void* cbCtx;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        cb = impl_->loadCallback;
        cbCtx = impl_->loadContext;
    }

    if (!cb) {
        resp->set_success(false);
        resp->set_error("No load callback registered");
        return grpc::Status::OK;
    }

    bool ok = cb(req->instance_id(), req->model_name().c_str(), cbCtx);
    resp->set_success(ok);
    if (!ok) {
        resp->set_error("Load failed");
    }
    return grpc::Status::OK;
}

grpc::Status SimulationControlServiceImpl::Unload(
        grpc::ServerContext* /*ctx*/,
        const imujoco::InstanceRequest* req,
        imujoco::ControlResponse* resp) {
    impl_->rpcCount.fetch_add(1, std::memory_order_relaxed);
    if (!validateInstanceId(req->instance_id(), impl_->config.numInstances, resp))
        return grpc::Status::OK;
    MJGrpcUnloadCallback cb;
    void* cbCtx;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        cb = impl_->unloadCallback;
        cbCtx = impl_->unloadContext;
    }

    if (!cb) {
        resp->set_success(false);
        resp->set_error("No unload callback registered");
        return grpc::Status::OK;
    }

    bool ok = cb(req->instance_id(), cbCtx);
    resp->set_success(ok);
    if (!ok) {
        resp->set_error("Unload failed");
    }
    return grpc::Status::OK;
}

grpc::Status SimulationControlServiceImpl::Start(
        grpc::ServerContext* /*ctx*/,
        const imujoco::InstanceRequest* req,
        imujoco::ControlResponse* resp) {
    impl_->rpcCount.fetch_add(1, std::memory_order_relaxed);
    if (!validateInstanceId(req->instance_id(), impl_->config.numInstances, resp))
        return grpc::Status::OK;
    MJGrpcOperationCallback cb;
    void* cbCtx;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        cb = impl_->operationCallback;
        cbCtx = impl_->operationContext;
    }
    if (!cb) {
        resp->set_success(false);
        resp->set_error("No operation callback registered");
        return grpc::Status::OK;
    }
    bool ok = cb(req->instance_id(), MJ_GRPC_OP_START, 0, cbCtx);
    resp->set_success(ok);
    if (!ok) resp->set_error("Start failed");
    return grpc::Status::OK;
}

grpc::Status SimulationControlServiceImpl::Pause(
        grpc::ServerContext* /*ctx*/,
        const imujoco::InstanceRequest* req,
        imujoco::ControlResponse* resp) {
    impl_->rpcCount.fetch_add(1, std::memory_order_relaxed);
    if (!validateInstanceId(req->instance_id(), impl_->config.numInstances, resp))
        return grpc::Status::OK;
    MJGrpcOperationCallback cb;
    void* cbCtx;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        cb = impl_->operationCallback;
        cbCtx = impl_->operationContext;
    }
    if (!cb) {
        resp->set_success(false);
        resp->set_error("No operation callback registered");
        return grpc::Status::OK;
    }
    bool ok = cb(req->instance_id(), MJ_GRPC_OP_PAUSE, 0, cbCtx);
    resp->set_success(ok);
    if (!ok) resp->set_error("Pause failed");
    return grpc::Status::OK;
}

grpc::Status SimulationControlServiceImpl::Reset(
        grpc::ServerContext* /*ctx*/,
        const imujoco::InstanceRequest* req,
        imujoco::ControlResponse* resp) {
    impl_->rpcCount.fetch_add(1, std::memory_order_relaxed);
    if (!validateInstanceId(req->instance_id(), impl_->config.numInstances, resp))
        return grpc::Status::OK;
    MJGrpcOperationCallback cb;
    void* cbCtx;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        cb = impl_->operationCallback;
        cbCtx = impl_->operationContext;
    }
    if (!cb) {
        resp->set_success(false);
        resp->set_error("No operation callback registered");
        return grpc::Status::OK;
    }
    bool ok = cb(req->instance_id(), MJ_GRPC_OP_RESET, 0, cbCtx);
    resp->set_success(ok);
    if (!ok) resp->set_error("Reset failed");
    return grpc::Status::OK;
}

grpc::Status SimulationControlServiceImpl::ResetToKeyframe(
        grpc::ServerContext* /*ctx*/,
        const imujoco::ResetToKeyframeRequest* req,
        imujoco::ControlResponse* resp) {
    impl_->rpcCount.fetch_add(1, std::memory_order_relaxed);
    if (!validateInstanceId(req->instance_id(), impl_->config.numInstances, resp))
        return grpc::Status::OK;
    MJGrpcOperationCallback cb;
    void* cbCtx;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        cb = impl_->operationCallback;
        cbCtx = impl_->operationContext;
    }
    if (!cb) {
        resp->set_success(false);
        resp->set_error("No operation callback registered");
        return grpc::Status::OK;
    }
    bool ok = cb(req->instance_id(), MJ_GRPC_OP_RESET_TO_KEYFRAME, req->keyframe_index(), cbCtx);
    resp->set_success(ok);
    if (!ok) resp->set_error("ResetToKeyframe failed");
    return grpc::Status::OK;
}

grpc::Status SimulationControlServiceImpl::Step(
        grpc::ServerContext* /*ctx*/,
        const imujoco::InstanceRequest* req,
        imujoco::ControlResponse* resp) {
    impl_->rpcCount.fetch_add(1, std::memory_order_relaxed);
    if (!validateInstanceId(req->instance_id(), impl_->config.numInstances, resp))
        return grpc::Status::OK;
    MJGrpcOperationCallback cb;
    void* cbCtx;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        cb = impl_->operationCallback;
        cbCtx = impl_->operationContext;
    }
    if (!cb) {
        resp->set_success(false);
        resp->set_error("No operation callback registered");
        return grpc::Status::OK;
    }
    bool ok = cb(req->instance_id(), MJ_GRPC_OP_STEP, 0, cbCtx);
    resp->set_success(ok);
    if (!ok) resp->set_error("Step failed");
    return grpc::Status::OK;
}

grpc::Status SimulationControlServiceImpl::GetState(
        grpc::ServerContext* /*ctx*/,
        const imujoco::InstanceRequest* req,
        imujoco::PhysicsState* resp) {
    impl_->rpcCount.fetch_add(1, std::memory_order_relaxed);
    int32_t id = req->instance_id();
    if (id < 1 || id > impl_->config.numInstances) {
        return grpc::Status(grpc::INVALID_ARGUMENT,
                            "Invalid instance_id: " + std::to_string(id));
    }
    MJGrpcGetStateCallback cb;
    void* cbCtx;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        cb = impl_->getStateCallback;
        cbCtx = impl_->getStateContext;
    }
    if (!cb) {
        return grpc::Status(grpc::FAILED_PRECONDITION, "No getState callback registered");
    }

    MJGrpcPhysicsState state{};
    bool ok = cb(req->instance_id(), &state, cbCtx);
    if (!ok || !state.valid) {
        return grpc::Status(grpc::FAILED_PRECONDITION, "No runtime or model for instance");
    }

    resp->set_time(state.time);
    for (int i = 0; i < state.nq; i++) resp->add_qpos(state.qpos[i]);
    for (int i = 0; i < state.nv; i++) resp->add_qvel(state.qvel[i]);
    for (int i = 0; i < state.nu; i++) resp->add_ctrl(state.ctrl[i]);
    for (int i = 0; i < state.nsensordata; i++) resp->add_sensordata(state.sensordata[i]);
    resp->set_energy_potential(state.energyPotential);
    resp->set_energy_kinetic(state.energyKinetic);

    return grpc::Status::OK;
}

// MARK: - MJGrpcServer

MJGrpcServer::MJGrpcServer(std::unique_ptr<MJGrpcServerImpl> impl)
    : impl_(std::move(impl)) {}

MJGrpcServer::~MJGrpcServer() {
    stop();
}

MJGrpcServer* MJGrpcServer::create(const MJGrpcServerConfig& config) {
    auto impl = std::make_unique<MJGrpcServerImpl>();
    impl->config = config;
    impl->service = std::make_unique<SimulationControlServiceImpl>(impl.get());
    return new MJGrpcServer(std::move(impl));
}

void MJGrpcServer::destroy(MJGrpcServer* server) {
    if (server) {
        server->stop();
        delete server;
    }
}

void MJGrpcServer::start() {
    if (impl_->server) return;  // Already started

    std::string address = "0.0.0.0:" + std::to_string(impl_->config.port);
    grpc::ServerBuilder builder;
    builder.AddListeningPort(address, grpc::InsecureServerCredentials());
    builder.RegisterService(impl_->service.get());
    impl_->server = builder.BuildAndStart();
}

void MJGrpcServer::stop() {
    if (impl_ && impl_->server) {
        impl_->server->Shutdown();
        impl_->server.reset();
    }
}

void MJGrpcServer::registerRuntime(int32_t instanceId,
                                   MJSimulationRuntime* runtime,
                                   const char* modelName) {
    int idx = instanceId - 1;
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (idx >= 0 && idx < impl_->config.numInstances) {
        impl_->runtimes[idx] = runtime;
        impl_->modelNames[idx] = modelName ? modelName : "";
    }
}

void MJGrpcServer::unregisterRuntime(int32_t instanceId) {
    int idx = instanceId - 1;
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (idx >= 0 && idx < impl_->config.numInstances) {
        impl_->runtimes[idx] = nullptr;
        impl_->modelNames[idx].clear();
    }
}

uint64_t MJGrpcServer::rpcCount() const {
    return impl_->rpcCount.load(std::memory_order_relaxed);
}

void MJGrpcServer::setLoadCallback(MJGrpcLoadModelCallback callback, void* context) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->loadCallback = callback;
    impl_->loadContext = context;
}

void MJGrpcServer::setUnloadCallback(MJGrpcUnloadCallback callback, void* context) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->unloadCallback = callback;
    impl_->unloadContext = context;
}

void MJGrpcServer::setOperationCallback(MJGrpcOperationCallback callback, void* context) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->operationCallback = callback;
    impl_->operationContext = context;
}

void MJGrpcServer::setGetStateCallback(MJGrpcGetStateCallback callback, void* context) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->getStateCallback = callback;
    impl_->getStateContext = context;
}

void MJGrpcServer::setGetInstanceInfoCallback(MJGrpcGetInstanceInfoCallback callback, void* context) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->getInstanceInfoCallback = callback;
    impl_->getInstanceInfoContext = context;
}
