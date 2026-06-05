"""Matplotlib chart widgets embedded in PyQt6."""

import numpy as np
import matplotlib
matplotlib.use("QtAgg")

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from matplotlib.figure import Figure


CATPPUCCIN = {
    "bg": "#eff1f5",
    "surface": "#ffffff",
    "overlay": "#ccd0da",
    "text": "#4c4f69",
    "subtext": "#6c6f85",
    "blue": "#1e66f5",
    "green": "#40a02b",
    "red": "#d20f39",
    "peach": "#fe640b",
    "yellow": "#df8e1d",
    "mauve": "#8839ef",
    "teal": "#179299",
    "pink": "#ea76cb",
}

MODE_COLORS = {
    "rmf": CATPPUCCIN["blue"],
    "dkr": CATPPUCCIN["peach"],
    "idkr": CATPPUCCIN["green"],
}

ROBOT_COLORS = [CATPPUCCIN["blue"], CATPPUCCIN["peach"], CATPPUCCIN["green"], CATPPUCCIN["mauve"]]
TIME_COLORS = [CATPPUCCIN["blue"], CATPPUCCIN["red"], CATPPUCCIN["yellow"], CATPPUCCIN["teal"]]


def _style_ax(ax):
    ax.set_facecolor(CATPPUCCIN["surface"])
    ax.tick_params(colors=CATPPUCCIN["subtext"], labelsize=9)
    ax.xaxis.label.set_color(CATPPUCCIN["subtext"])
    ax.yaxis.label.set_color(CATPPUCCIN["subtext"])
    ax.title.set_color(CATPPUCCIN["text"])
    ax.title.set_fontsize(12)
    ax.title.set_fontweight("bold")
    for spine in ax.spines.values():
        spine.set_color(CATPPUCCIN["overlay"])
    ax.grid(axis="y", color=CATPPUCCIN["overlay"], alpha=0.4, linewidth=0.5)


def _make_figure(rows=1, cols=1, figsize=(7, 4)):
    fig = Figure(figsize=figsize, facecolor=CATPPUCCIN["bg"], tight_layout=True)
    axes = fig.subplots(rows, cols)
    return fig, axes


class GroupedBarChart(FigureCanvasQTAgg):
    """Grouped bar chart: one group per mode, bars per scenario."""

    def __init__(self, parent=None, figsize=(8, 4)):
        fig = Figure(figsize=figsize, facecolor=CATPPUCCIN["bg"], tight_layout=True)
        super().__init__(fig)
        self.fig = fig

    def plot(self, data: dict, title: str = "", ylabel: str = "", higher_is_better: bool = True):
        """
        data: {mode: {scenario: value}}
        """
        self.fig.clear()
        ax = self.fig.add_subplot(111)
        _style_ax(ax)

        modes = list(data.keys())
        if not modes:
            ax.set_title("No data")
            self.draw()
            return

        scenarios = list(next(iter(data.values())).keys())
        n_modes = len(modes)
        n_scenarios = len(scenarios)
        x = np.arange(n_scenarios)
        width = 0.7 / n_modes

        for i, mode in enumerate(modes):
            values = [data[mode].get(s, 0) for s in scenarios]
            bars = ax.bar(
                x + i * width - (n_modes - 1) * width / 2,
                values, width,
                label=mode.upper(),
                color=MODE_COLORS.get(mode, CATPPUCCIN["blue"]),
                edgecolor="none",
                alpha=0.9,
                zorder=3,
            )
            for bar, val in zip(bars, values):
                ax.text(
                    bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.02 * max(max(v for v in data[m].values()) for m in modes if data[m]),
                    f"{val:.1f}" if isinstance(val, float) else str(val),
                    ha="center", va="bottom",
                    color=CATPPUCCIN["subtext"], fontsize=8,
                )

        from data.results_loader import SCENARIO_LABELS
        ax.set_xticks(x)
        ax.set_xticklabels([SCENARIO_LABELS.get(s, s) for s in scenarios])
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.legend(facecolor=CATPPUCCIN["surface"], edgecolor=CATPPUCCIN["overlay"],
                  labelcolor=CATPPUCCIN["text"], fontsize=9)
        self.draw()


class RadarChart(FigureCanvasQTAgg):
    """Spider/radar chart for multi-metric comparison."""

    def __init__(self, parent=None, figsize=(5, 5)):
        fig = Figure(figsize=figsize, facecolor=CATPPUCCIN["bg"], tight_layout=True)
        super().__init__(fig)
        self.fig = fig

    def plot(self, data: dict, metric_labels: list[str], title: str = ""):
        """
        data: {mode: [normalized_value_0_to_1, ...]}
        metric_labels: list of metric names
        """
        self.fig.clear()
        n = len(metric_labels)
        if n < 3:
            ax = self.fig.add_subplot(111)
            ax.set_facecolor(CATPPUCCIN["surface"])
            ax.set_title("Need >= 3 metrics for radar")
            ax.title.set_color(CATPPUCCIN["text"])
            self.draw()
            return

        angles = np.linspace(0, 2 * np.pi, n, endpoint=False).tolist()
        angles += angles[:1]

        ax = self.fig.add_subplot(111, polar=True)
        ax.set_facecolor(CATPPUCCIN["surface"])
        ax.set_theta_offset(np.pi / 2)
        ax.set_theta_direction(-1)

        ax.set_xticks(angles[:-1])
        ax.set_xticklabels(metric_labels, color=CATPPUCCIN["subtext"], fontsize=9)

        ax.set_ylim(0, 1.1)
        ax.set_yticks([0.25, 0.5, 0.75, 1.0])
        ax.set_yticklabels(["0.25", "0.50", "0.75", "1.00"],
                           color=CATPPUCCIN["overlay"], fontsize=7)
        ax.spines["polar"].set_color(CATPPUCCIN["overlay"])
        ax.grid(color=CATPPUCCIN["overlay"], alpha=0.3)

        for mode, values in data.items():
            v = values + values[:1]
            color = MODE_COLORS.get(mode, CATPPUCCIN["blue"])
            ax.plot(angles, v, "o-", linewidth=2, label=mode.upper(), color=color, markersize=4)
            ax.fill(angles, v, alpha=0.15, color=color)

        ax.legend(loc="upper right", bbox_to_anchor=(1.3, 1.1),
                  facecolor=CATPPUCCIN["surface"], edgecolor=CATPPUCCIN["overlay"],
                  labelcolor=CATPPUCCIN["text"], fontsize=9)
        ax.set_title(title, color=CATPPUCCIN["text"], fontsize=12, fontweight="bold", pad=20)
        self.draw()


class StackedBarChart(FigureCanvasQTAgg):
    """Stacked horizontal bar chart for per-robot time breakdown."""

    def __init__(self, parent=None, figsize=(8, 4)):
        fig = Figure(figsize=figsize, facecolor=CATPPUCCIN["bg"], tight_layout=True)
        super().__init__(fig)
        self.fig = fig

    def plot(self, robot_data: dict, title: str = ""):
        """
        robot_data: {robot_name: {"moving": v, "waiting": v, "idle": v, "charging": v}}
        """
        self.fig.clear()
        ax = self.fig.add_subplot(111)
        _style_ax(ax)

        if not robot_data:
            ax.set_title("No robot data")
            self.draw()
            return

        robots = list(robot_data.keys())
        short_names = [r.replace("warehouseRobot", "R") for r in robots]
        categories = ["moving", "waiting", "idle", "charging"]
        cat_labels = ["Moving", "Waiting", "Idle", "Charging"]
        colors = TIME_COLORS[:len(categories)]

        y = np.arange(len(robots))
        lefts = np.zeros(len(robots))

        for cat, label, color in zip(categories, cat_labels, colors):
            values = [robot_data[r].get(cat, 0) for r in robots]
            ax.barh(y, values, left=lefts, height=0.6,
                    label=label, color=color, edgecolor="none", alpha=0.9, zorder=3)
            lefts += np.array(values)

        ax.set_yticks(y)
        ax.set_yticklabels(short_names)
        ax.set_xlabel("Time (s)")
        ax.set_title(title)
        ax.legend(facecolor=CATPPUCCIN["surface"], edgecolor=CATPPUCCIN["overlay"],
                  labelcolor=CATPPUCCIN["text"], fontsize=9, loc="lower right")
        ax.invert_yaxis()
        self.draw()


class HeatmapTable(FigureCanvasQTAgg):
    """Heatmap-style table: scenarios x modes."""

    def __init__(self, parent=None, figsize=(7, 3)):
        fig = Figure(figsize=figsize, facecolor=CATPPUCCIN["bg"], tight_layout=True)
        super().__init__(fig)
        self.fig = fig

    def plot(self, data: dict, metric_name: str, higher_is_better: bool = True):
        """
        data: {scenario: {mode: value}}
        """
        self.fig.clear()
        ax = self.fig.add_subplot(111)
        ax.set_facecolor(CATPPUCCIN["bg"])

        from data.results_loader import MODE_LABELS, SCENARIO_LABELS

        scenarios = list(data.keys())
        modes = list(next(iter(data.values())).keys()) if scenarios else []

        if not scenarios or not modes:
            ax.set_title("No data")
            ax.title.set_color(CATPPUCCIN["text"])
            self.draw()
            return

        matrix = []
        for s in scenarios:
            row = [data[s].get(m, 0) for m in modes]
            matrix.append(row)
        matrix = np.array(matrix, dtype=float)

        cmap = "RdYlGn" if higher_is_better else "RdYlGn_r"
        im = ax.imshow(matrix, cmap=cmap, aspect="auto", alpha=0.8)

        ax.set_xticks(range(len(modes)))
        ax.set_xticklabels([MODE_LABELS.get(m, m) for m in modes],
                           color=CATPPUCCIN["text"], fontsize=11)
        ax.set_yticks(range(len(scenarios)))
        ax.set_yticklabels([SCENARIO_LABELS.get(s, s) for s in scenarios],
                           color=CATPPUCCIN["text"], fontsize=11)

        for i in range(len(scenarios)):
            for j in range(len(modes)):
                val = matrix[i, j]
                fmt = f"{val:.2f}" if isinstance(val, float) and val != int(val) else f"{val:.0f}"
                ax.text(j, i, fmt, ha="center", va="center",
                        color="#1e1e2e", fontsize=12, fontweight="bold")

        ax.set_title(metric_name, color=CATPPUCCIN["text"], fontsize=13, fontweight="bold", pad=10)
        for spine in ax.spines.values():
            spine.set_visible(False)
        ax.tick_params(length=0)
        self.draw()
