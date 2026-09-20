"""Pure algorithm tests for the dynamic-obstacle preprocessing stage."""

from types import SimpleNamespace

from usv_navigation.dynamic_obstacle_tracker import (
    _centroid,
    _grid_clusters,
    _transform_xy,
)


def test_grid_clusters_rejects_singleton_and_keeps_two_objects():
    first = [(4.0 + 0.1 * i, 1.0 + 0.1 * (i % 2)) for i in range(5)]
    second = [(8.0 + 0.1 * i, -2.0) for i in range(4)]
    clusters = _grid_clusters(first + second + [(15.0, 15.0)],
                               cell_size=0.8, min_points=3,
                               max_extent=8.0, max_points=250)
    assert len(clusters) == 2
    centroids = sorted(_centroid(cluster) for cluster in clusters)
    assert centroids[0][0] == 4.2
    assert centroids[1][0] == 8.15


def test_grid_clusters_rejects_oversized_shore_component():
    shoreline = [(float(i) * 0.4, 10.0) for i in range(30)]
    assert _grid_clusters(shoreline, cell_size=0.8, min_points=3,
                          max_extent=8.0, max_points=250) == []


def test_transform_xy_applies_translation_and_yaw():
    transform = SimpleNamespace(
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=2.0, y=-1.0),
            rotation=SimpleNamespace(x=0.0, y=0.0,
                                     z=0.7071067812, w=0.7071067812)))
    x, y = _transform_xy((1.0, 0.0), transform)
    assert abs(x - 2.0) < 1e-6
    assert abs(y - 0.0) < 1e-6
