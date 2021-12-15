#       ____  ____
#      /   /\/   /
#     /___/  \  /   Copyright (c) 2021, Xilinx®.
#     \   \   \/    Author: Víctor Mayoral Vilches <victorma@xilinx.com>
#      \   \
#      /   /
#     /___/   /\
#     \   \  /  \
#      \___\/\___\
#
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
import bt2
import sys
import datetime
import os

from bokeh.plotting import figure, show
from bokeh.sampledata.iris import flowers
from bokeh.io import export_png
from bokeh.models import ColumnDataSource
from bokeh.models import Legend, LegendItem

# Create a trace collection message iterator from the first command-line
# argument.
msg_it = bt2.TraceCollectionMessageIterator(
    # str(os.environ["HOME"]) + "/.ros/tracing/power_capture_acceleration2"
    # str(os.environ["HOME"]) + "/.ros/tracing/power_capture"               # CPU only
    # str(os.environ["HOME"]) + "/.ros/tracing/power_capture_nofpga"        # CPU only, FPGA disa
    str(os.environ["HOME"]) + "/.ros/tracing/power_capture_acceleration"  # CPU + FPGA
)

init = 10
end = 100

# p = figure(title="vadd power measurements (iterations 10-20) @ KV260 (CPU only)")
p = figure(
    title="ROS 2 publisher power consumption (iterations "
    + str(init)
    + "-"
    + str(end)
    + ") @ KV260"
)
p.xaxis.axis_label = "Time offset (second)"
p.yaxis.axis_label = "Power (Watts)"

debug = False  # debug flag, set to True if desired


def yellow(text):
    print("\033[33m", text, "\033[0m", sep="")


def yellowinline(text):
    print("\033[33m", text, "\033[0m", end="")


def magenta(text):
    print("\033[35m", text, "\033[0m", sep="")


def red(text):
    print("\033[31m", text, "\033[0m", sep="")


def redinline(text):
    print("\033[31m", text, "\033[0m", end="")


def get_change(first, second):
    """
    Get change in percentage between two values
    """
    if first == second:
        return 0
    try:
        return (abs(first - second) / second) * 100.0
    except ZeroDivisionError:
        return float("inf")


def power_in_list(list, initial_timing=None):
    """
    Calculates and reports the power consumption given a list of msgs

    :param list of msgs containing power information (see Power ROS 2 Node)
    :param initial_timing, initial time event (in ns) from where to start
        counting for power calculations.
    :return (pwr_msgs, pwr_traces), a tuple containing the total power
        consumption derived from 1) the msgs information (using differential time,
        dts), and 2) the analysis of the traces, extracting time directly from
        there.
    """
    # Use the ns_from_origin_pre as last event's time
    last_event_ns_from_origin = initial_timing

    # lists to store power measurements and time differences considering two
    # scenarios:
    #   - using_msg: using the time difference obtained from the trace (calculated in the process)
    #   - using_trace: using the time difference calculated from the trace (actual real time)
    using_msg_power = []  # watts
    using_msg_dt = []  # seconds
    using_trace_power = []  # watts
    using_trace_dt = []  # seconds

    for msg_block in list:
        # Get event message's default clock snapshot's ns from origin value.
        ns_from_origin = msg_block.default_clock_snapshot.ns_from_origin

        # Compute the time difference since the last event message.
        dt = 0

        if last_event_ns_from_origin is not None:
            dt = (ns_from_origin - last_event_ns_from_origin) / 1e9

            using_msg_power.append(msg_block.event["power"] * msg_block.event["dt"])
            using_msg_dt.append(msg_block.event["dt"])
            using_trace_power.append(msg_block.event["power"] * dt)
            using_trace_dt.append(dt)

            if debug:
                #
                # debug
                #
                # Create a `datetime.datetime` objects from
                # `ns_from_origin|last_event_ns_from_origin` for
                # presentation. Note that such an object is less accurate than
                # `ns_from_origin` as it holds microseconds, not nanoseconds.
                date_past = datetime.datetime.fromtimestamp(
                    last_event_ns_from_origin / 1e9
                )
                date_now = datetime.datetime.fromtimestamp(ns_from_origin / 1e9)
                #
                fmt = "\t{} → {} (+{:.6f} s): using-dt ({}), using-tracing ({})"
                print(
                    fmt.format(
                        date_past,
                        date_now,
                        dt,
                        using_msg_power[-1],
                        using_trace_power[-1],
                    )
                )

        # Update last event's time.
        last_event_ns_from_origin = ns_from_origin

    if debug:
        fmt = "using_msg ({:.6f} s): {} W |||| using_trace ({:.6f} s): {} W"
        print("Subtotal:")
        print(
            fmt.format(
                sum(using_msg_dt),
                # sum([pwr * dt for pwr, dt in zip(using_msg_power, using_msg_dt)]),
                sum(using_msg_power),
                sum(using_trace_dt),
                # sum([pwr * dt for pwr, dt in zip(using_trace_power, using_trace_dt)]),
                sum(using_trace_power),
            )
        )
        print(
            "Change: "
            + str(
                get_change(
                    # sum([pwr * dt for pwr, dt in zip(using_msg_power, using_msg_dt)]),
                    # sum([pwr * dt for pwr, dt in zip(using_trace_power, using_trace_dt)]),
                    sum(using_msg_power),
                    sum(using_trace_power),
                )
            )
            + " %"
        )
    return using_msg_power, using_trace_power


def plot_power_in_list(
    figure,
    list_msgs,
    initial_timing=None,
    line_color="tomato",
    legend_label="cum. power",
    cumulative=True,
    line_dash="dashed",
):
    """
    Line plots power given a list of msgs

    :param figure
    :param list of msgs containing power information (see Power ROS 2 Node)
    :param initial_timing, initial time event (in ns) from where to start
        counting for power calculations.

    """
    import numpy as np

    if initial_timing:
        initial_ns = initial_timing
    else:
        initial_ns = list_msgs[0].default_clock_snapshot.ns_from_origin

    # get the timestamp from the msg trace, in seconds
    x_axis = [
        (x.default_clock_snapshot.ns_from_origin - initial_ns) / 1e9 for x in list_msgs
    ]

    y_axis = []
    pwr_cum = 0
    for msg in list_msgs:
        if cumulative:
            pwr_cum += float(msg.event["power"])
            y_axis.append(pwr_cum)
        else:
            y_axis.append(float(msg.event["power"]))

    figure.line(
        x_axis,
        y_axis,
        legend_label=legend_label,
        line_color=line_color,
        line_dash=line_dash,
    )

    return (x_axis, y_axis)


def areaplot_power_in_list(
    figure,
    list_msgs,
    initial_timing=None,
    cumulative=True,
):
    """
    Area plots power given a list of msgs

    :param figure
    :param list of msgs containing power information (see Power ROS 2 Node)
    :param initial_timing, initial time event (in ns) from where to start
        counting for power calculations.
    :returns
    """
    import numpy as np

    if initial_timing:
        initial_ns = initial_timing
    else:
        initial_ns = list_msgs[0].default_clock_snapshot.ns_from_origin

    # get the timestamp from the msg trace, in seconds
    x_axis = [
        (x.default_clock_snapshot.ns_from_origin - initial_ns) / 1e9 for x in list_msgs
    ]

    y_axis = []
    y_axis_cum = []
    pwr_cum = 0
    for msg in list_msgs:
        if cumulative:
            pwr_cum += msg.event["power"] * msg.event["dt"]
            y_axis_cum.append(pwr_cum)

        y_axis.append(msg.event["power"] * msg.event["dt"])

    # area plot the interval power measurements and cumulative power consumption
    if cumulative:
        source = ColumnDataSource(
            data=dict(
                x=x_axis,
                # y1=y_axis,
                # y2=y_axis_cum,
                y1=y_axis_cum,
            )
        )
        # figure.varea_stack(['y1', 'y2'], x='x', color=("grey", "lightgrey"), source=source, legend=["power", "cumulative power"])
        # return (x_axis, y_axis, y_axis_cum)
        figure.varea_stack(
            ["y1"],
            x="x",
            color=("lightgrey"),
            source=source,
            legend=["Energy (Watt-second)"],
        )
        # hatch_pattern=":"
        return (x_axis, y_axis_cum)

    else:
        source = ColumnDataSource(
            data=dict(
                x=x_axis,
                y1=y_axis,
            )
        )
        figure.varea_stack(
            ["y1"], x="x", color=("grey"), source=source, legend=["power"]
        )
        return (x_axis, y_axis)


def timestamp_in(new, initial_boundary, end_boundary) -> bool:
    """
    Check if a new message timestamp is within the initial-end boundary
    and return True or False.

    :param new: new message to consider
    :param initial_boundary: trace msg that defines the initial boundary
    :param end_boundary: trace msg that defines the end boundary
    return: bool
    """
    init_ns = initial_boundary.default_clock_snapshot.ns_from_origin
    end_ns = end_boundary.default_clock_snapshot.ns_from_origin
    new_ns = new.default_clock_snapshot.ns_from_origin

    if (init_ns <= new_ns) and (new_ns <= end_ns):
        return True
    else:
        return False


def power_within(init_msg, end_msg, pwr_msgs):
    """
    Returns the power value calculated from all the Power traces found
    between init_msg and end_msg in the trace

    :param init_msgs
    :param end_msg
    :param pwr_msgs, all power messages recorded in the trace
    :return (pwr_msgs, pwr_traces), a tuple containing the total power
        consumption derived from 1) the msgs information (using differential time,
        dts), and 2) the analysis of the traces, extracting time directly from
        there.

    """
    msgs_within = []
    for msg in pwr_msgs:
        if timestamp_in(msg, init_msg, end_msg):
            msgs_within.append(msg)

    pwr_msgs, pwr_traces = power_in_list(msgs_within)
    return pwr_msgs, pwr_traces


def power_msgs_within(init_msg, end_msg, pwr_msgs):
    """
    Returns the power msgs found between init_msg and end_msg in the trace

    :param init_msgs
    :param end_msg
    :param pwr_msgs, all power messages recorded in the trace
    :return list of power messages within

    """
    msgs_within = []
    for msg in pwr_msgs:
        if timestamp_in(msg, init_msg, end_msg):
            ## debug
            # print(
            #     datetime.datetime.fromtimestamp(
            #         msg.default_clock_snapshot.ns_from_origin / 1e9
            #     )
            # )
            msgs_within.append(msg)
    return msgs_within


def custom_plot(figure, vadd_msgs, power_msgs, init, end):
    """
    Custom plot

    :param figure object
    :param vadd_msgs containing both pre and post traces
    :param power_msgs collecting power information in parallel to vadd_msgs
    :init iteration initial number
    :end iteration final (end) number
    """
    import numpy as np

    # find the initial and end msgs based on arguments
    init_msg = [
        y
        for y in vadd_msgs
        if (
            int([x.strip() for x in str(y.event["iteration"]).split(":")][1]) == init
            and (y.event.name == "ros2_acceleration:vadd_pre")
        )
    ][0]

    end_msg = [
        y
        for y in vadd_msgs
        if (
            int([x.strip() for x in str(y.event["iteration"]).split(":")][1]) == end
            and (y.event.name == "ros2_acceleration:vadd_post")
        )
    ][0]

    if debug:
        print(
            str(init_msg.event.name)
            + "("
            + str(init_msg.event["iteration"])
            + ")"
            + ": "
            + str(
                datetime.datetime.fromtimestamp(
                    init_msg.default_clock_snapshot.ns_from_origin / 1e9
                )
            )
        )

        print(
            str(end_msg.event.name)
            + "("
            + str(end_msg.event["iteration"])
            + ")"
            + ": "
            + str(
                datetime.datetime.fromtimestamp(
                    end_msg.default_clock_snapshot.ns_from_origin / 1e9
                )
            )
        )

    # vadd messages in the boundary
    vadd_msgs_to_plot = []
    for msg in vadd_msgs:
        if timestamp_in(msg, init_msg, end_msg):
            vadd_msgs_to_plot.append(msg)

    # power_msgs in the boundary
    power_msgs_to_plot = []
    for msg in power_msgs:
        if timestamp_in(msg, init_msg, end_msg):
            power_msgs_to_plot.append(msg)

    # get the timestamp from the msg trace, in seconds
    initial_ns = init_msg.default_clock_snapshot.ns_from_origin
    x_axis = [
        (x.default_clock_snapshot.ns_from_origin - initial_ns) / 1e9
        for x in vadd_msgs_to_plot
    ]

    y_axis = []
    y_axis_cum = [0]
    previous_message = vadd_msgs_to_plot[0]
    for x_axis_msg in vadd_msgs_to_plot:
        pwr_msgs, pwr_traces = power_within(previous_message, x_axis_msg, power_msgs)
        y_axis.append(sum(pwr_traces))
        y_axis_cum.append(sum(pwr_traces) + y_axis_cum[-1])
        previous_message = x_axis_msg

    y_axis_cum = y_axis_cum[1:]

    # colors
    colormap = {
        "other": "red",
        "ros2_acceleration:vadd_pre": "yellow",
        "ros2_acceleration:vadd_post": "orange",
    }
    colors = [colormap[x.event.name] for x in vadd_msgs_to_plot]

    # x_axis_power = [
    #     (x.default_clock_snapshot.ns_from_origin - initial_ns) / 1e9
    #     for x in power_msgs_to_plot
    # ]
    # y_axis_power = [
    #     (x.event.[]] - initial_ns) / 1e9
    #     for x in power_msgs_to_plot
    # ]

    ############################################################################
    # the actual plots
    ############################################################################

    # # area plot the interval power measurements and cumulative power consumption
    # source = ColumnDataSource(data=dict(
    #     x=x_axis,
    #     y1=y_axis,
    #     y2=y_axis_cum,
    # ))
    # r0 = figure.varea_stack(['y1', 'y2'], x='x', color=("grey", "lightgrey"), source=source, legend=["power", "cumulative power"])

    x_axis_result, y_axis_cum_result = areaplot_power_in_list(
        figure, power_msgs_to_plot, initial_timing=None, cumulative=True
    )

    # # plot cumulative power consumption from power_msgs
    # power_msgs_custom = power_msgs_within(
    #     vadd_msgs_to_plot[0], vadd_msgs_to_plot[-1], power_msgs
    # )
    # plot_power_in_list(figure, power_msgs_custom)

    # # scatter plot vadd datapoints
    # r1 = figure.scatter(
    #     x_axis,
    #     y_axis,
    #     color=colors,
    #     fill_alpha=0.2,
    #     size=7,
    # )

    # legend = Legend(items=[
    #     LegendItem(label="vadd_pre", renderers=[r1], index=0),
    #     LegendItem(label="vadd_post", renderers=[r1], index=1),
    # ])
    # figure.add_layout(legend)

    # plot power messages in a raw manner
    # negligible, almost all zero
    raw_power_msgs = power_msgs_within(init_msg, end_msg, power_msgs)
    # print(raw_power_msgs)
    powerline_x_axis, powerline_y_axis = plot_power_in_list(
        figure,
        raw_power_msgs,
        line_color="black",
        legend_label="Power (Watts)",
        cumulative=False,
        line_dash="solid",
    )

    # put a coloured line on each vadd measurement that stops at energy
    x_axis_pre = [
        (x.default_clock_snapshot.ns_from_origin - initial_ns) / 1e9
        for x in vadd_msgs_to_plot
        if x.event.name == "ros2_acceleration:vadd_pre"
    ]

    x_axis_post = [
        (x.default_clock_snapshot.ns_from_origin - initial_ns) / 1e9
        for x in vadd_msgs_to_plot
        if x.event.name == "ros2_acceleration:vadd_post"
    ]

    for i in x_axis_pre:
        index = min(range(len(x_axis_result)), key=lambda j: abs(x_axis_result[j] - i))
        figure.line(
            i,
            # y=[0, y_axis_cum_result[index]],  # line up to energy
            y=[0, powerline_y_axis[index]],  # line up to power
            color="orange",
            legend_label="vadd_pre (tracepoint)",
            line_dash="dotted",
        )

    for i in x_axis_post:
        index = min(range(len(x_axis_result)), key=lambda j: abs(x_axis_result[j] - i))
        figure.line(
            i,
            # y=[0, y_axis_cum_result[index]],  # line up to energy
            y=[0, powerline_y_axis[index]],  # line up to power
            color="red",
            legend_label="vadd_post (tracepoint)",
            line_dash="dashdot",
        )

    # # plot power
    # figure.line(
    #     x_axis, y_axis, color="black", legend_label="power readings (intervals)"
    # )
    #
    #
    # # plot cum. power consumption from intervals
    # figure.line(
    #     x_axis,
    #     y_axis_cum,
    #     legend_label="cum. power (intervals)",
    #     line_color="grey",
    #     line_dash="dashdot",
    # )


######################################################

# Iterate the trace messages and pick those ones of interest
power_msgs = []
vadd_msgs = []
for msg in msg_it:
    # `bt2._EventMessageConst` is the Python type of an event message.
    if type(msg) is bt2._EventMessageConst:

        # An event message holds a trace event.
        event = msg.event

        # Only check `sched_switch` events.
        if event.name == "ros2_acceleration:kria_power_dt":
            power_msgs.append(msg)

        # Only check `sched_switch` events.
        if (
            event.name == "ros2_acceleration:vadd_pre"
            or event.name == "ros2_acceleration:vadd_post"
        ):
            vadd_msgs.append(msg)


for msg in vadd_msgs:
    if msg.event.name == "ros2_acceleration:vadd_pre":
        # debug
        # print("vadd " + str(msg.event["iteration"]))

        # Get event message's default clock snapshot's ns from origin
        # value.
        ns_from_origin_pre = msg.default_clock_snapshot.ns_from_origin
        post_msgs = [
            msg_post
            for msg_post in vadd_msgs
            if msg_post.event["iteration"] == msg.event["iteration"] and msg_post != msg
        ]
        if len(post_msgs) > 1:
            print("various post messages found for " + str(msg.event["iteration"]))
            print([aux_msg.event.name for aux_msg in post_msgs])
            sys.exit(1)
        elif len(post_msgs) == 0:
            continue
        else:
            post_msg = post_msgs[0]

        ns_from_origin_post = post_msg.default_clock_snapshot.ns_from_origin
        diff_ns = ns_from_origin_post - ns_from_origin_pre
        diff_ms = (ns_from_origin_post - ns_from_origin_pre) / 1e6
        diff_s = (ns_from_origin_post - ns_from_origin_pre) / 1e9

        date_pre = datetime.datetime.fromtimestamp(ns_from_origin_pre / 1e9)
        date_post = datetime.datetime.fromtimestamp(ns_from_origin_post / 1e9)

        if debug:
            fmt = (
                "{} → {} (+{:.6f} s): \033[31m vadd "
                + str(msg.event["iteration"])
                + "\033[0m"
            )
            print(
                fmt.format(
                    date_pre,
                    date_post,
                    diff_s,
                )
            )

        # fetch all the power measurements between 'ns_from_origin_pre' and
        # 'ns_from_origin_post'
        block_power_msgs = []
        for msg_pwr in power_msgs:
            ns_from_origin_pwr = msg_pwr.default_clock_snapshot.ns_from_origin

            if (ns_from_origin_pwr > ns_from_origin_pre) and (
                ns_from_origin_pwr < ns_from_origin_post
            ):
                if debug:
                    # debug
                    date_pwr = datetime.datetime.fromtimestamp(ns_from_origin_pwr / 1e9)
                    fmt = "\t{}: power ({}), dt ({})"
                    print(
                        fmt.format(
                            date_pwr,
                            msg_pwr.event["power"],
                            msg_pwr.event["dt"],
                        )
                    )
                block_power_msgs.append(msg_pwr)

        if debug:
            # determine power consumption within each vadd block
            pwr_msgs, pwr_traces = power_in_list(block_power_msgs)
            fmt = "Power consumed: \033[33m{:.4f} W\033[0m ({:.4f} W, from msgs)"
            print(
                fmt.format(
                    sum(pwr_traces),
                    sum(pwr_msgs),
                )
            )


# Calculate only for the interval
init_msg = [
    y
    for y in vadd_msgs
    if (
        int([x.strip() for x in str(y.event["iteration"]).split(":")][1]) == init
        and (y.event.name == "ros2_acceleration:vadd_pre")
    )
][0]

end_msg = [
    y
    for y in vadd_msgs
    if (
        int([x.strip() for x in str(y.event["iteration"]).split(":")][1]) == end
        and (y.event.name == "ros2_acceleration:vadd_post")
    )
][0]

# calculate the time difference between init_msg and end_msg
ns_init_msg = init_msg.default_clock_snapshot.ns_from_origin
date_init_msg = datetime.datetime.fromtimestamp(ns_init_msg / 1e9)
ns_end_msg = end_msg.default_clock_snapshot.ns_from_origin
date_end_msg = datetime.datetime.fromtimestamp(ns_init_msg / 1e9)

tf_ns = ns_end_msg - ns_init_msg
tf_s = tf_ns / 1e9

# power_msgs in the boundary
power_msgs_to_consider = []
for msg in power_msgs:
    if timestamp_in(msg, init_msg, end_msg):
        power_msgs_to_consider.append(msg)
total_pwr_msgs, total_pwr_traces = power_in_list(power_msgs_to_consider)
# total_pwr_msgs, total_pwr_traces = power_in_list(power_msgs)
# fmt = "\033[33mTotal power consumed: {:.4f} W\033[0m ({:.4f} W, from msgs)"
fmt = (
    "Total energy consumed in the {}-{} interval (\033[35m{:.6f} s\033[0m - \033[0;35m{} Hz\033[0m):"
    + " \033[33m{:.4f} W-s\033[0m ({:.4f} W-s, from msgs) → Avg. power: {} W → Performance per Watt:"
    + " \033[31m{}\033[0m Hz/W"
)

# calc Performance-per-watt
time_per_iteration = tf_s / (end - init)  # seconds/iteration
frequency = 1 / time_per_iteration  # Hz
average_power = sum(total_pwr_traces) / tf_s  # Watt
energy = sum(total_pwr_traces)  # Watt-second
energy_msgs = sum(total_pwr_msgs)  # Watt-second
performance_watt = frequency / average_power  # Hz/Watt-second

print(
    fmt.format(
        init,
        end,
        tf_s,
        frequency,
        energy,
        energy_msgs,
        average_power,
        performance_watt,
    )
)


# # plot_power_in_list(p, power_msgs, power_msgs[0].default_clock_snapshot.ns_from_origin)
# custom_plot(p, vadd_msgs, power_msgs, init=init, end=end)
# # export_png(p, filename="plot.png")
# p.legend.location = "bottom_right"
# show(p)


def generate_launch_description():
    return LaunchDescription()
