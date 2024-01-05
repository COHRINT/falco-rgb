using Pkg

dependencies = [
    "POMDPs",
    "POMDPModels",
    "SARSOP",
    "Statistics",
    "POMDPXFiles",
    "POMDPSimulators",
    "POMDPPolicies",
    "Plots",
    "BasicPOMCP",
    "POMDPModelTools",
    "POMDPToolbox",
    "BeliefUpdaters"
]

Pkg.add(dependencies)
