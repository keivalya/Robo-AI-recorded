import gymnasium as gym
import numpy as np

env = gym.make("MountainCar-v0", render_mode="human")

alpha = 0.1  # Learning rate
gamma = 0.95  # Discount factor
eps = 25000  # Number of episodes
render_at = 3000

# Exploration settings
epsilon = 1.0  # Start with high exploration
min_epsilon = 0.01  # Minimum exploration rate
epsilon_decay_rate = 0.995  # Decay rate per episode

DISCRETE_OS_SIZE = [20] * len(env.observation_space.high)
discrete_os_win_size = (env.observation_space.high - env.observation_space.low) / DISCRETE_OS_SIZE

q_table = np.random.uniform(low=-2, high=0, size=(DISCRETE_OS_SIZE + [env.action_space.n]))

# Function to get discrete state from continuous state
def get_discrete_state(state):
    discrete_state = (state - env.observation_space.low) / discrete_os_win_size
    return tuple(discrete_state.astype(np.int32))

# Main loop for running episodes
for episode in range(eps):
    state = env.reset()[0]
    discrete_state = get_discrete_state(state)
    done = False
    truncated = False

    if episode % render_at == 0:
        render_flag = True
        print(f"Episode: {episode}")
    else:
        render_flag = False

    while not (done or truncated):
        # Epsilon-greedy action selection
        if np.random.random() > epsilon:
            action = np.argmax(q_table[discrete_state])
        else:
            action = np.random.randint(0, env.action_space.n)

        new_state, reward, done, truncated, _ = env.step(action)
        new_discrete_state = get_discrete_state(new_state)

        if render_flag:
            env.unwrapped.render_mode = "human"
        else:
            env.unwrapped.render_mode = None

        # Update Q-value if not done
        if not (done or truncated):
            max_future_q = np.max(q_table[new_discrete_state])
            current_q = q_table[discrete_state + (action,)]
            new_q = (1 - alpha) * current_q + alpha * (reward + gamma * max_future_q)
            q_table[discrete_state + (action,)] = new_q
        elif new_state[0] >= env.unwrapped.goal_position:
            q_table[discrete_state + (action,)] = 0

        discrete_state = new_discrete_state

    if epsilon > min_epsilon:
        epsilon *= epsilon_decay_rate

env.close()