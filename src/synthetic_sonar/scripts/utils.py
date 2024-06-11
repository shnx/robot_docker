import numpy as np



def prob_to_log_odds(x):
    return np.round(np.log(x) - np.log(1 - x),6)


def log_odds_to_prob(x):
    x=1 - (1 / (1 + np.exp(x)))
    return np.clip(x,0.0,1.0)


def linear_mapping_of_values(x, old_min=0, old_max=1, new_min=0, new_max=100):
    """
    https://stackoverflow.com/a/929107/1253729
    """
    old_range = old_max - old_min
    new_range = new_max - new_min
    return ((((x - old_min) * new_range) / old_range) + new_min).astype(int)