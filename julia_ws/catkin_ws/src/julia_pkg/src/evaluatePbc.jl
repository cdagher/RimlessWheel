#!/usr/bin/env julia
using Pkg
cd("/home/bsurobotics/repos/RimlessWheel/julia_ws/catkin_ws/src/julia_pkg/src")
Pkg.activate(".")

using DiffEqFlux, MLBasedESC
using LinearAlgebra, BSON
using RobotOS

@rosimport sensor_msgs.msg: JointState
@rosimport std_msgs.msg: Float32
rostypegen()
using .sensor_msgs.msg, .std_msgs.msg

const l1  = 0.3f0                  #wheel
const l2  = 0.06f0                  #torso
const k   = 10
const α   = Float32(360.0/k/2.0 * pi/180.0)
const DEG_TO_RAD = pi/180.0

inputLayer(x) = [cos(x[1]), sin(x[1]), cos(x[2]), sin(x[2]), x[3], x[4]]        #x[1] = torso, x[2] = spoke

Hd = FastChain(
        FastDense(6, 8, elu, bias=true),
        FastDense(8, 7, elu, bias=true),
        FastDense(7, 1, bias=true)
    )

npbc = MLBasedESC.NeuralPBC(6, Hd)
ps = BSON.load("./saved_weights/deterministic_hardware_6-8-8-7-7-1_elu.bson")[:param]

function initialState(ϕ0, θ0, ϕ0dot, θ0dot)
    @assert pi-α <= θ0 <= pi+α "Give an initial spoke angle for the spoke in contact. This will help set the rimless wheel in contact with the surface"
    
    x = l1*sin(pi-θ0)
    y = l1*cos(pi-θ0)
    ϕ = ϕ0 
    θ = θ0 
    xdot = -l1*cos(pi-θ0) * θ0dot
    ydot = l1*sin(pi-θ0) * θ0dot
    ϕdot = ϕ0dot 
    θdot = θ0dot

    return [x, y, ϕ, θ, xdot, ydot, ϕdot, θdot]
end

function update_state!(msg::sensor_msgs.msg.JointState, state::Vector)
    state[1] = msg.position[1]       #torso
    state[2] = pi + msg.position[2]       #spokes
    state[3] = msg.velocity[1]
    state[4] = msg.velocity[2]
end

function main()
    init_node("nn_controller")
    state = zeros(Float32,4)
    sensorData = Vector{Vector{Float32}}()
    pub = Publisher{JointState}("/torso_command", queue_size=1)
    sub = Subscriber{JointState}("/sensors", update_state!, (state,), queue_size=1)
    @info "ROS node initialized. Loading models..."

    x0 = initialState(0.0f0, pi, 0.0f0, 0.0f0)
    torque = MLBasedESC.controller(npbc, inputLayer(x0), ps)

    @info "Model loaded. Spinning ROS..."
    loop_rate = Rate(100.0)
    torque_msg = JointState();
    while !is_shutdown()
        torque_msg.header = std_msgs.msg.Header()
        torque_msg.header.stamp = RobotOS.now()
        push!(sensorData, state)
        torque = MLBasedESC.controller(npbc, inputLayer(state), ps)
        torque_msg.effort = zeros(1)
        torque_msg.effort[1] = torque
        publish(pub, torque_msg)
        rossleep(loop_rate)
    end
    BSON.@save "/home/bsurobotics/repos/RimlessWheel/julia_ws/catkin_ws/src/julia_pkg/src/hardware_data/sensor_data.bson" sensorData
    safe_shutdown_hack()
end

function safe_shutdown_hack()
    @info "Publishing zero torque."
    run(`rostopic pub -1 /torso_command sensor_msgs/JointState "effort: [0.] "`, wait=false);
end

Base.atexit(safe_shutdown_hack)

if !isinteractive()
    @info "Julia packages loaded. Starting main()"
    main()
end
