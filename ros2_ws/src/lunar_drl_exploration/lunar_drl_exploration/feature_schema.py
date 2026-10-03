"""Measured task_graph_v4 node and action column definitions."""
NODE_FEATURE_DIM = 27
ACTION_FEATURE_DIM = 7
IN_GAIN_SLICE = slice(3, 11)
OUT_GAIN_SLICE = slice(11, 19)
HISTORY_SLICE = slice(19, 27)
IN_GAIN_SCALE = 100.0
OUT_GAIN_SCALE = 100.0
