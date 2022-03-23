
# Defines the risk profile for the agent.
# Agent should attempt to ride the ideal risk band.
# If the agent becomes trapped in a scenario where its risk threshold is too high, it must escape the risk. 
RISK_THRESHOLD_HIGH = 10
RISK_THRESHOLD_LOW = 5
IDEAL_RISK = (RISK_THRESHOLD_HIGH - RISK_THRESHOLD_LOW) / 2

current_risk = 0
received_power = 0
strongest_signal_angle = 0

current_location = (0, 0)
jammer_est_location = (0, 0)

scan_interval = 20

signal_profile = list()

