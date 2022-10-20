using MLBasedESC, MLBasedESCZoo, DiffEqFlux
using LinearAlgebra, BSON
using RobotOS

@rosimport sensor_msgs.msg: JointState
@rosimport std_msgs.msg: Float64
rostypegen()
using .sensor_msgs.msg, .std_msgs.msg

const l1  = 0.26f0                  #wheel
const l2  = 0.05f0                  #torso
const k   = 10
const α   = Float32(360.0/k/2.0 * pi/180.0)

unn  = FastChain(FastDense(6, 8, elu), 
                FastDense(8, 5, elu),
                FastDense(5, 1))

inputLayer(x) = [cos(x[3]), sin(x[3]), cos(x[4]), sin(x[4]), x[7], x[8]]

ps = BSON.@load "nnController_viaLCP_6-8-8-5-5-1elu.bson"

function initialState(θ0, θ0dot, ϕ0, ϕ0dot)
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

function main()
    @info "Julia started with weights located at $(weightdir)."
    init_node("nn_controller")
    state = zeros(Float64,4)
    pub = Publisher{Float64Msg}("torso/command", queue_size=1)
    sub = Subscriber{JointState}("/joint_states", update_state!, (state,), queue_size=1)
    @info "ROS node initialized. Loading models..."

    x0 = initialState(pi, -1.0f0, 0.0f0, 0.0f0)
    effort = unn(inputLayer(x0), ps)[1]

    @info "Model loaded. Spinning ROS..."
    loop_rate = Rate(800.0)
    while !is_shutdown()
        header = std_msgs.msg.Header()
        header.stamp = RobotOS.now()
        effort = compute_control(state, policy)
        gear_ratio = 1.0
        eta = 0.98
        k_tau = 0.230    # N-m/a
        current = effort / gear_ratio / k_tau / eta
        cmd = Float64Msg(current)
        publish(pub, cmd)
        rossleep(loop_rate)
    end
    safe_shutdown_hack()
end

function safe_shutdown_hack()
    @info "Publishing zero torque."
    run(`rostopic pub -1 /torso/command std_msgs/Float64 "data: 0. "`, wait=false);
end

Base.atexit(safe_shutdown_hack)

# if !isinteractive()
#     @info "Julia packages loaded. Starting main()"
#     main()
# end