import numpy as np


class Path:
    """Represents a path defined by a series of (x, y) waypoints.

    Interpolates the waypoints at a specified resolution (in meters).
    """

    def __init__(self, waypoints: list[tuple[float, float]], resolution: float = 0.01):
        if len(waypoints) < 2:
            raise ValueError("A path must have at least 2 waypoints.")

        self.raw_waypoints = np.array(waypoints)
        self.resolution = resolution
        self.total_length = 0.0
        self.s = np.array([])
        self.x = np.array([])
        self.y = np.array([])
        self.theta = np.array([])
        self.interpolate_path()

    def interpolate_path(self):
        """Interpolates waypoints and calculates tangent angles along the path."""
        dx = np.diff(self.raw_waypoints[:, 0])
        dy = np.diff(self.raw_waypoints[:, 1])
        segment_lengths = np.sqrt(dx**2 + dy**2)

        cumulative_dist = np.insert(np.cumsum(segment_lengths), 0, 0.0)
        self.total_length = cumulative_dist[-1]

        # Determine number of interpolation points based on resolution
        num_points = max(2, int(np.ceil(self.total_length / self.resolution)) + 1)
        self.s = np.linspace(0, self.total_length, num_points)

        # Interpolate coordinates along path length s
        self.x = np.interp(self.s, cumulative_dist, self.raw_waypoints[:, 0])
        self.y = np.interp(self.s, cumulative_dist, self.raw_waypoints[:, 1])

        # Compute heading theta (tangent vector) at each point along the path
        dx_interp = np.gradient(self.x, self.s)
        dy_interp = np.gradient(self.y, self.s)
        self.theta = np.arctan2(dy_interp, dx_interp)

    def get_closest_point(
        self, pos_x: float, pos_y: float
    ) -> tuple[float, float, float, float, float]:
        """Find the closest point on the path to a given position.

        Returns:
            x (float): Closest x coordinate on the path (m)
            y (float): Closest y coordinate on the path (m)
            theta (float): Tangent heading angle at that point (rad)
            s (float): Path progress/distance from start (m)
            distance (float): Absolute distance from the position to the path (m)
        """
        dx = self.x - pos_x
        dy = self.y - pos_y
        sq_distances = dx**2 + dy**2
        min_idx = np.argmin(sq_distances)

        return (
            self.x[min_idx],
            self.y[min_idx],
            self.theta[min_idx],
            self.s[min_idx],
            np.sqrt(sq_distances[min_idx]),
        )

    def get_point_at_distance(self, target_s: float) -> tuple[float, float, float]:
        """Get the (x, y, theta) coordinates at a specific path distance s."""
        target_s = np.clip(target_s, 0.0, self.total_length)
        x = np.interp(target_s, self.s, self.x)
        y = np.interp(target_s, self.s, self.y)

        # Handle angle interpolation safely using sine and cosine components
        cos_theta = np.interp(target_s, self.s, np.cos(self.theta))
        sin_theta = np.interp(target_s, self.s, np.sin(self.theta))
        theta = np.arctan2(sin_theta, cos_theta)

        return x, y, theta
