using POMDPs
using POMDPModels
using SARSOP
using Statistics
using POMDPXFiles
using POMDPSimulators
using POMDPPolicies
using SARSOP
using Plots
using BasicPOMCP
using POMDPs, POMDPModelTools, POMDPSimulators, POMDPToolbox, BeliefUpdaters

# Use this link to see the description of the states, actions and observations: https://docs.google.com/spreadsheets/d/1kqjr2rYbx_u0uSQhD0zdd1rpFyc1PwxZUp-Vue6Kq3o/edit#gid=2135435755

mutable struct StatefulBelief
    belief::Any
end

function initial_belief()
    # Transition Matrix
    T = zeros(2,3,2) # |S|x|A|x|S|, T[s', a, s] = p(s'|a,s)
    # Alert action
    T[1,1,1]=1
    T[2,1,2]=1
    # Gather info action
    T[1,2,1]=0.5
    T[2,2,1]=0.5
    T[1,2,2]=0.2
    T[2,2,2]=0.8
    # Continue
    T[1,3,1]=0.6
    T[1,3,2]=0.4
    T[2,3,1]=0.4
    T[2,3,2]=0.6

    # Observation Matrix - "Gather info: 2"
    O = zeros(2,3,2) # |O|x|A|x|S|, O[o, a, s] = p(o|a,s)
    confidence_score = 0.9
        # Confidence high
        O[1,2,1] = 1-confidence_score
        O[1,2,2] = confidence_score
        # Confidence low
        O[2,2,1] = confidence_score
        O[2,2,2] = 1-confidence_score

    for a in [1,3]
        for s in 1:2
            O[1, a, s] = 0.5 # some default value
            O[2, a, s] = 0.5 # ensure probabilities sum to 1
        end
    end

    # Reward Matrix
    R = zeros(2,3) # |S|x|A|, R[s, a]
    R[1,1]=-100
    R[1,2]=0
    R[1,3]=80
    R[2,1]=100
    R[2,2]=-75
    R[2,3]=-75

    # Model
    discount = 0.95
    pomdp = TabularPOMDP(T, R, O, discount);
    updater = DiscreteUpdater(pomdp)
    belief = initialize_belief(updater, POMDPModels.DiscreteDistribution{Vector{Float64}}([0.5, 0.5]))
    return belief
end

function reset_belief()
    global statefulbelief = StatefulBelief(initial_belief()) 
end

function generate_action(cs)
    # Transition Matrix
    T = zeros(2,3,2) # |S|x|A|x|S|, T[s', a, s] = p(s'|a,s)
    # Alert action
    T[1,1,1]=1
    T[2,1,2]=1
    # Gather info action
    T[1,2,1]=0.6
    T[2,2,1]=0.4
    T[1,2,2]=0.2
    T[2,2,2]=0.8
    # Continue
    T[1,3,1]=0.6
    T[1,3,2]=0.4
    T[2,3,1]=0.4
    T[2,3,2]=0.6

    # Observation Matrix - "Gather info: 2"
    O = zeros(2,3,2) # |O|x|A|x|S|, O[o, a, s] = p(o|a,s)
    confidence_score = 0.9
        # Confidence high
        O[1,2,1] = 1-confidence_score
        O[1,2,2] = confidence_score
        # Confidence low
        O[2,2,1] = confidence_score
        O[2,2,2] = 1-confidence_score

    for a in [1,3]
        for s in 1:2
            O[1, a, s] = 0.5 # some default value
            O[2, a, s] = 0.5 # ensure probabilities sum to 1
        end
    end


    # Reward Matrix
    R = zeros(2,3) # |S|x|A|, R[s, a]
    R[1,1]=-100
    R[1,2]=0
    R[1,3]=80
    R[2,1]=100
    R[2,2]=-75
    R[2,3]=-75

    # Model
    discount = 0.95
    pomdp = TabularPOMDP(T, R, O, discount);

    if cs > 0.7
        observation = 1
    else 
        observation = 2
    end

    global statefulbelief
    if statefulbelief == nothing 
        belief = initial_belief()  # initialize the belief
        statefulbelief = StatefulBelief(belief)
    else
        belief = statefulbelief.belief
    end

    sarsop_policy = SARSOP.load_policy(pomdp, "policy.out") 
    updater = DiscreteUpdater(pomdp)
    action, _ = action_info(sarsop_policy, belief) 
    #println(belief)
    #println(action)
    if cs == -1 
        belief = initial_belief()  # initialize the belief
        statefulbelief = StatefulBelief(belief)
    else
    belief = update(updater, belief, action, observation) 
    end
    Main.statefulbelief.belief = belief
    action, _ = action_info(sarsop_policy, belief) 
    print(belief)
    println(action)
    return action, belief
end

#=
cs_block = LinRange(0.1, 0.99, 10) 
for cs in cs_block
    action, belief = generate_action(cs)
    println(action)
    println(belief)
    println("---------------------------------------------------")
end
=#