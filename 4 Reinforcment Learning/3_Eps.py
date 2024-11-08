import gymnasium as gym
import numpy as np

env = gym.make("MountainCar-v0", render_mode="human")

# Q-Learning settings
alpha = 0.1     # learning rate
gamma = 0.95    # discount factor
eps = 25000     # episodes

DISCRETE_OS_SIZE = [20] * len(env.observation_space.high)
discrete_os_win_size = (env.observation_space.high - env.observation_space.low)/DISCRETE_OS_SIZE

q_table = np.random.uniform(low=-2, high=0, size=(DISCRETE_OS_SIZE + [env.action_space.n]))

# function to get discrete state from continuous state
def get_discrete_state(state):
    discrete_state = (state[0] - env.observation_space.low) / DISCRETE_OS_SIZE
    return tuple(discrete_state.astype(np.int32))

# our simulation starts here
for espisode in range(eps):
    discrete_state = get_discrete_state(env.reset())
    done = False
    while not done:
        action = np.argmax(q_table[discrete_state]) # instead of 2
        new_state, reward, done, _, _ = env.step(action)
        new_discrete_state = get_discrete_state(new_state)
        if not done:
            # Maximum possible Q value in next step (for new state)
            max_future_q = np.max(q_table[new_discrete_state])

            # Current Q value (for current state and performed action)
            current_q = q_table[discrete_state + (action,)]

            # And here's our equation for a new Q value for current state and action
            new_q = (1 - alpha) * current_q + alpha * (reward + gamma * max_future_q)

            # Update Q table with new Q value
            q_table[discrete_state + (action,)] = new_q

        # if sim ends for any reason,
        elif new_state[0] >= env.goal_position:
            # q_table[discrete_state + (action,)] = reward
            q_table[discrete_state + (action,)] = 0

        discrete_state = new_discrete_state