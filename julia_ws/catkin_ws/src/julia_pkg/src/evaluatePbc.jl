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
ps = BSON.load("/home/bsurobotics/repos/RimlessWheel/julia_ws/catkin_ws/src/julia_pkg/src/saved_weights/hardware_even_deter_1mpers_6-8-8-7-7-1_elu.bson")[:param]

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
    # state[1] = msg.position[1]       #torso
    # state[2] = msg.position[2]     #spoke0
    # state[3] = msg.velocity[1]	 #torso
    # state[4] = msg.velocity[2]	#spoke0
    # state[5] = msg.position[3] 	#spoke1
    # state[6] = msg.velocity[3]	#spoke1
    # state[7] = msg.position[4] 	#yaw

    state[1] = msg.position[1]       #torso
    state[2] = pi - abs(msg.position[2] )    #spoke0
    state[3] = msg.velocity[1]	 #torso
    state[4] = -abs(msg.velocity[2])	#spoke0
    state[5] = pi - abs(msg.position[3]) 	#spoke1
    state[6] = -abs(msg.velocity[3])	#spoke1
    state[7] = msg.position[4] 	#yaw
end

function isolateSpokeStates(state)
    # wrappedStates = deepcopy(state)
    # wrappedStates[1] = state[1]       #torso
    # wrappedStates[2] = pi - abs(state[2])     #spoke0
    # wrappedStates[3] = state[3]	 #torso
    # wrappedStates[4] = -abs(state[4])  #spoke0
    # wrappedStates[5] = pi - abs(state[5]) 	#spoke1
    # wrappedStates[6] = -abs(state[6])	#spoke1
    # wrappedStates[7] = state[7] 	#yaw
    
    # spoke0 = wrappedStates[1:4]
    # spoke1 = [wrappedStates[1], wrappedStates[5], wrappedStates[3], wrappedStates[6]]

    spoke0 = state[1:4]
    spoke1 = [state[1], state[5], state[3], state[6]]

    return spoke0, spoke1 
end

function computeTorque(spoke0)
    tau = 1.0f0*MLBasedESC.controller(npbc, inputLayer(spoke0), ps)
    return tau 
end

function main()
    init_node("nn_controller")
    state = zeros(Float32,7)
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
        spoke0, spoke1 = isolateSpokeStates(state)
        torque = computeTorque(spoke0)
        torque_msg.effort = zeros(1)
        torque_msg.effort[1] = torque
        publish(pub, torque_msg)
        push!(sensorData, deepcopy(state))
        rossleep(loop_rate)
    end
    BSON.@save "/home/bsurobotics/repos/RimlessWheel/julia_ws/catkin_ws/src/julia_pkg/src/hardware_data/deterministic_sensor_data.bson" sensorData
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
