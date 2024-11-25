import numpy as np


def divide_by_cos_or_sin(value_cos: float, value_sin: float, theta: float) -> float:
    c = np.cos(theta)
    if np.fabs(c) < 1E-6:
        return -value_sin / np.sin(theta)
    return -value_cos / c


def force_zero(m: np.ndarray, min_number=1E-8) -> np.ndarray:
    mask = np.bitwise_and((-min_number < m), (m < min_number))
    m[mask] = 0
    return m


def normalize_angle(a: float) -> float:
    return np.mod(a + np.pi, 2 * np.pi) - np.pi


def isXVector(ax: np.ndarray) -> bool:
    return close_to(np.linalg.norm(ax - np.array([1, 0, 0])), 0)


def gram_schmidt(ax: np.ndarray) -> np.ndarray:
    M = np.zeros((3, 3))

    Z = ax / np.linalg.norm(ax)
    M[:, 2] = Z
    V = np.array([0, 1, 0], dtype=float) if isXVector(ax) else np.array([1, 0, 0], dtype=float)

    # Compute X
    X = np.cross(Z, V)
    M[:, 0] = X / np.linalg.norm(X)

    # Compute Y
    Y = np.cross(Z, X)
    M[:, 1] = Y / np.linalg.norm(Y)

    return M


def close_to(v1, v2, tol: float = 1E-5) -> bool:
    return np.fabs(v1 - v2) < tol


def powers_array(v: float, to: int) -> np.ndarray:
    P = np.zeros(to + 1)
    P[0] = 1
    for i in range(1, to + 1):
        P[i] = P[i - 1] * v
    return P
