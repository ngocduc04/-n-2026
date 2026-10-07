"""
Ve bieu do 2D (3 truc con) cho "Che do 2: Phuc hoi tu the lat" tu file recovery_log.csv
do pid_flight.cpp ghi ra. Mo o mot cua so rieng, giong het cach Che do 1 dung
plot_trajectory.py — tu dong bat len khi dong cua so mo phong.

Cai dat truoc khi chay:
    pip install matplotlib pandas

Chay:
    python plot_recovery.py
    python plot_recovery.py duong/dan/khac/recovery_log.csv
"""

import sys
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.widgets import RangeSlider


def main():
    csv_path = sys.argv[1] if len(sys.argv) > 1 else "recovery_log.csv"
    df = pd.read_csv(csv_path)
    t_min, t_max = df["time"].min(), df["time"].max()

    # 4 bieu do chinh (luc day, Ox, Oy, Oz) + 1 truc nho cho thanh truot zoom
    fig, (ax1, ax_x, ax_y, ax_z, ax_slider) = plt.subplots(
        5, 1, figsize=(9, 11),
        gridspec_kw={"height_ratios": [4, 3, 3, 3, 0.4]},
    )
    fig.canvas.manager.set_window_title("Che do 2 - Phuc hoi tu the lat")

    for col, label, color in [
        ("motor1_N", "Dong co 1", "tab:red"),
        ("motor2_N", "Dong co 2", "tab:orange"),
        ("motor3_N", "Dong co 3", "tab:green"),
        ("motor4_N", "Dong co 4", "tab:blue"),
    ]:
        if col in df.columns:
            ax1.plot(df["time"], df[col], label=label, color=color, linewidth=1.3)
    ax1.set_ylabel("Luc day (N)")
    ax1.set_title("Luc day tung dong co theo thoi gian")
    ax1.legend(loc="upper right", fontsize=8)
    ax1.grid(True, which="both", alpha=0.3)
    ax1.minorticks_on()

    plot_axes = [ax1]

    def add_deviation_plot(ax, col, title, ylabel, color):
        if col not in df.columns:
            ax.set_visible(False)
            return
        ax.plot(df["time"], df[col], color=color, linewidth=1.6)
        ax.axhline(0, color="gray", linestyle="--", linewidth=0.8)
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.grid(True, which="both", alpha=0.3)
        ax.minorticks_on()
        plot_axes.append(ax)

    add_deviation_plot(ax_x, "x_error_m", "Do lech truc Ox so voi diem can bang", "Do lech Ox (m)", "tab:purple")
    add_deviation_plot(ax_y, "y_error_m", "Do lech truc Oy so voi diem can bang", "Do lech Oy (m)", "tab:brown")
    add_deviation_plot(ax_z, "z_error_m", "Do lech truc Oz so voi diem can bang", "Do lech Oz (m)", "tab:blue")
    plot_axes[-1].set_xlabel("Thoi gian (s)")

    for ax in plot_axes:
        ax.set_xlim(t_min, t_max)

    # ----- Thanh truot chon khoang thoi gian de "zoom" truc X, khong dong den truc Y -----
    range_slider = RangeSlider(ax_slider, "Khoang thoi gian (s)", t_min, t_max,
                                valinit=(t_min, t_max))

    def on_slider_change(val):
        lo, hi = val
        if hi <= lo:   # tranh khoang rong khi keo 2 nam trung nhau
            return
        for ax in plot_axes:
            ax.set_xlim(lo, hi)
        fig.canvas.draw_idle()

    range_slider.on_changed(on_slider_change)

    plt.tight_layout()
    plt.savefig("recovery_plot.png", dpi=150)
    print("Da luu bieu do vao recovery_plot.png")
    print("Keo 2 dau thanh truot o duoi cung de thu phong theo truc thoi gian.")
    plt.show()


if __name__ == "__main__":
    main()