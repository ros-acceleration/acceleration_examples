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

from tabnanny import verbose
from turtle import width
from launch import LaunchDescription
import bt2
import sys
import datetime
import os
from wasabi import color
from typing import List, Optional, Tuple, Union
import pandas as pd
import numpy as np
import pprint

from bokeh.plotting.figure import figure, Figure
from bokeh.plotting import output_notebook
from bokeh.io import show
from bokeh.layouts import row
from bokeh.models import ColumnDataSource, DatetimeTickFormatter, PrintfTickFormatter, Legend, Segment
from bokeh.models.annotations import Label

# color("{:02x}".format(x), fg=16, bg="green")
# debug = True  # debug flag, set to True if desired

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

def add_durations_to_figure(
    figure: Figure,
    segment_type: str,
    durations: List[Union[Tuple[datetime.datetime, datetime.datetime]]],
    color: str,
    line_width: int = 60,
    legend_label: Optional[str] = None,
) -> None:
    for duration in durations:
        duration_begin, duration_end, _ = duration
        base_kwargs = dict()
        if legend_label:
            base_kwargs['legend_label'] = legend_label
        figure.line(
            x=[duration_begin, duration_end],
            y=[segment_type, segment_type],
            color=color,
            line_width=line_width,
            **base_kwargs,
        )

def add_markers_to_figure(
    figure: Figure,
    segment_type: str,
    times: List[datetime.datetime],
    color: str,
    line_width: int = 60,
    legend_label: Optional[str] = None,
    size: int = 30,
    marker_type: str = 'diamond',
) -> None:
    for time in times:
        base_kwargs = dict()
        if legend_label:
            base_kwargs['legend_label'] = legend_label
        if marker_type == 'diamond':
            figure.diamond(
                x=[time],
                y=[segment_type],
                fill_color=color,
                line_color=color,
                size=size,
                **base_kwargs,
            )
        elif marker_type == 'plus':
            figure.plus(
                x=[time],
                y=[segment_type],
                fill_color=color,
                line_color=color,
                size=size,
                **base_kwargs,
            )
        else:
            assert False, 'invalid marker_type value'

def msgsets_from_trace(tracename):
    global target_chain

    # Create a trace collection message iterator from the first command-line
    # argument.
    msg_it = bt2.TraceCollectionMessageIterator(tracename)

    # Iterate the trace messages and pick ros2 ones
    image_pipeline_msgs = []
    for msg in msg_it:
        # `bt2._EventMessageConst` is the Python type of an event message.
        if type(msg) is bt2._EventMessageConst:
            # An event message holds a trace event.
            event = msg.event
            # Only check `sched_switch` events.
            if ("ros2" in event.name):
                image_pipeline_msgs.append(msg)

    # Form sets with each pipeline
    image_pipeline_msg_sets = []
    new_set = []  # used to track new complete sets
    chain_index = 0  # track where in the chain we are so far
    vpid_chain = -1  # used to track a set and differentiate from other callbacks

    # NOTE: NOT CODED FOR MULTIPLE NODES RUNNING CONCURRENTLY
    # this classification is going to miss the initial matches because
    # "ros2:callback_start" will not be associated with the target chain and it won't stop
    # being considered until a "ros2:callback_end" of that particular process is seen
    for index in range(len(image_pipeline_msgs)):
        # first one
        if chain_index == 0 and image_pipeline_msgs[index].event.name == target_chain[chain_index]:
            new_set.append(image_pipeline_msgs[index])
            vpid_chain = image_pipeline_msgs[index].event.common_context_field.get("vpid")
            chain_index += 1
            # print(color("Found: " + str(image_pipeline_msgs[index].event.name) + " - " + str([x.event.name for x in new_set]), fg="blue"))
        # last one
        elif image_pipeline_msgs[index].event.name == target_chain[chain_index] and target_chain[chain_index] == target_chain[-1] and \
                new_set[-1].event.name == target_chain[-2] and \
                image_pipeline_msgs[index].event.common_context_field.get("vpid") == vpid_chain:
            new_set.append(image_pipeline_msgs[index])
            image_pipeline_msg_sets.append(new_set)
            # print(color("Found: " + str(image_pipeline_msgs[index].event.name) + " - " + str([x.event.name for x in new_set]), fg="blue"))
            chain_index = 0  # restart
            new_set = []  # restart
        # match
        elif image_pipeline_msgs[index].event.name == target_chain[chain_index] and \
                image_pipeline_msgs[index].event.common_context_field.get("vpid") == vpid_chain:
            new_set.append(image_pipeline_msgs[index])
            chain_index += 1
            # print(color("Found: " + str(image_pipeline_msgs[index].event.name), fg="green"))
        # altered order
        elif image_pipeline_msgs[index].event.name in target_chain and \
                image_pipeline_msgs[index].event.common_context_field.get("vpid") == vpid_chain:
            new_set.append(image_pipeline_msgs[index])
            # print(color("Altered order: " + str([x.event.name for x in new_set]) + ", restarting", fg="red"))
            chain_index = 0  # restart
            new_set = []  # restart
    return image_pipeline_msg_sets


# def msgsets_from_trace_start_end_streamlined(tracename):
#     """ 
#     Fetches start-end messages for the streamlining case
#     """
#     global target_chain

#     # Create a trace collection message iterator from the first command-line
#     # argument.
#     msg_it = bt2.TraceCollectionMessageIterator(tracename)

#     # Iterate the trace messages and pick ros2 ones
#     image_pipeline_msgs = []
#     for msg in msg_it:
#         # `bt2._EventMessageConst` is the Python type of an event message.
#         if type(msg) is bt2._EventMessageConst:
#             # An event message holds a trace event.
#             event = msg.event
#             # Only check `sched_switch` events.
#             if ("ros2" in event.name):
#                 image_pipeline_msgs.append(msg)

#     # Form sets with each pipeline
#     image_pipeline_msg_sets = []
#     new_set = []  # used to track new complete sets
#     chain_index = 0  # track where in the chain we are so far
#     vpid_chain = -1  # used to track a set and differentiate from other callbacks

#     for index in range(len(image_pipeline_msgs)):
#         # first one
#         if image_pipeline_msgs[index].event.name == "ros2:callback_start" and \
#             image_pipeline_msgs[index].event.common_context_field.get("vpid") == image_pipeline_msgs[index].event.common_context_field.get("vtid"):            
#             new_set.append(image_pipeline_msgs[index])

#         # last one
#         elif image_pipeline_msgs[index].event.name == "ros2:callback_end" and \
#             image_pipeline_msgs[index].event.common_context_field.get("vpid") != image_pipeline_msgs[index].event.common_context_field.get("vtid"):
#             new_set.append(image_pipeline_msgs[index])
#             image_pipeline_msg_sets.append(new_set)
#             new_set = []  # restart

#         elif image_pipeline_msgs[index].event.name == "ros2:callback_end" and \
#             image_pipeline_msgs[index].event.common_context_field.get("vpid") != image_pipeline_msgs[index].event.common_context_field.get("vtid"):

#     return image_pipeline_msg_sets


def msgsets_from_trace_concurrent(tracename):
    global target_chain

    # NOTE: considered chains of "ros2:rclcpp_publish" roughly

    # Create a trace collection message iterator from the first command-line
    # argument.
    msg_it = bt2.TraceCollectionMessageIterator(tracename)
    # Iterate the trace messages and pick ros2 ones
    image_pipeline_msgs = []
    for msg in msg_it:
        # `bt2._EventMessageConst` is the Python type of an event message.
        if type(msg) is bt2._EventMessageConst:
            # An event message holds a trace event.
            event = msg.event
            # Only check `sched_switch` events.
            if ("ros2" in event.name):
                image_pipeline_msgs.append(msg)

    # Form sets with each pipeline
    image_pipeline_msg_sets = []
    candidates = {}  # dict of sets (vpid as key) being considered as candicates to be complete
    # NOTE:
    # - vpid remains the same for all Components in an executor, even if multithreaded
    # - vtid changes for each component in a multithreaded executor

    for trace in image_pipeline_msgs:
        vtid = trace.event.common_context_field.get("vtid")
        if trace.event.name == target_chain[0]:
            if (vtid in candidates) and (candidates[vtid][-1].event.name ==  target_chain[-1]):  # account for chained traces, use "ros2:callback_end"
                # print(color("Continuing: " + str(trace.event.name), fg="green"))
                candidates[vtid].append(trace)
            elif vtid in candidates:
                # print(color("Already a set, re-starting: " + str(trace.event.name) + " - " \
                #     + str([x.event.name for x in candidates[vtid]]) , fg="yellow"))
                candidates[vtid] = [trace]  # already a set existing (pop and) re-start
            else:
                candidates[vtid] = [trace]  # new set
                # print(color("New: " + str(trace.event.name) + " - " + \
                #     str([x.event.name for x in candidates[vtid]]), fg="blue"))
        elif (trace.event.name in target_chain) and (vtid in candidates):
            if len(candidates[vtid]) >= 9 and (trace.event.name in target_chain[9:]):
                trace_index = target_chain[9:].index(trace.event.name) + 9
                expected_index = target_chain[9:].index(candidates[vtid][-1].event.name) + 1 + 9
            elif len(candidates[vtid]) >= 9:
                # print(color("Skipping: " + str(trace.event.name), fg="yellow"))
                continue  # skip
            else:
                trace_index = target_chain.index(trace.event.name)
                expected_index = target_chain.index(candidates[vtid][-1].event.name) + 1
            # Account for chains of callbacks
            if trace.event.name == target_chain[-1] and candidates[vtid][-1].event.name == target_chain[0]:
                if len(candidates[vtid]) > 1:
                    candidates[vtid] = candidates[vtid][:-1]  # pop last start and continue looking
                    # print(color("Chain of callbacks, popping: " + str(trace.event.name) , fg="yellow"))
                else:
                    candidates.pop(vtid)
                    # print(color("Chain of callbacks while starting, popping: " + str(trace.event.name) , fg="yellow"))
            elif trace_index == expected_index:
                candidates[vtid].append(trace)
                # print(color("Found: " + str(trace.event.name), fg="green"))
                if trace.event.name == target_chain[-1] and candidates[vtid][-2].event.name == target_chain[-2] \
                        and len(candidates[vtid]) == len(target_chain):  # last one
                    image_pipeline_msg_sets.append(candidates[vtid])
                    # print(color("complete set!", fg="pink"))
                    candidates.pop(vtid)
            else:
                if trace.event.name == "ros2:rclcpp_publish" or \
                        trace.event.name == "ros2:rcl_publish" or \
                        trace.event.name == "ros2:rmw_publish":
                    # print(color("Potential chain of publish: " + str(trace.event.name) + ", skipping" , fg="yellow"))
                    pass
                else:
                    candidates[vtid].append(trace)
                    # print(color("Altered order: " + str([x.event.name for x in candidates[vtid]]) + ", discarding", fg="red"))
                    candidates.pop(vtid)
        else:
            # print(color("Skipped: " + str(trace.event.name), fg="grey"))
            pass
    return image_pipeline_msg_sets

def barplot_all(image_pipeline_msg_sets, title="Barplot"):
    global target_chain
    global target_chain_dissambiguous

    image_pipeline_msg_sets_ns = []
    for set_index in range(len(image_pipeline_msg_sets)):
        aux_set = []
        target_chain_ns = []
        for msg_index in range(len(image_pipeline_msg_sets[set_index])):
            target_chain_ns.append(image_pipeline_msg_sets[set_index][msg_index].default_clock_snapshot.ns_from_origin)
        init_ns = target_chain_ns[0]
        for msg_index in range(len(image_pipeline_msg_sets[set_index])):
            aux_set.append((target_chain_ns[msg_index] - init_ns)/1e6)
        image_pipeline_msg_sets_ns.append(aux_set)

    df = pd.DataFrame(image_pipeline_msg_sets_ns)
    df.columns = target_chain_dissambiguous
    import plotly.express as px

    # pd.set_option("display.max_rows", None, "display.max_columns", None)
    # print(df)

    fig = px.box(
        df,
        points="all",
        template="plotly_white",
        title=title,
    )
    fig.update_xaxes(title_text = "Trace event")
    fig.update_yaxes(title_text = "Milliseconds")
    fig.show()

def traces(msg_set):
    global target_chain_colors_fg_bokeh
    global segment_types
    global target_chain_marker
    global target_chain
    global target_chain_layer

    fig = figure(
        title='Image pipeline tracing',
        x_axis_label=f'Milliseconds',
        y_range=segment_types,
        plot_width=2000,
        plot_height=600,
    )
    fig.title.align = 'center'
    fig.title.text_font_size = '20px'
    # fig.xaxis[0].formatter = DatetimeTickFormatter(milliseconds = ['%3Nms'])
    fig.xaxis[0].formatter = PrintfTickFormatter(format="%f ms")
    fig.xaxis[0].ticker.desired_num_ticks = 20
    fig.xaxis[0].axis_label_text_font_size = '30px'
    fig.yaxis[0].major_label_text_font_size = '25px'

    target_chain_ns = []
    for msg_index in range(len(msg_set)):
        target_chain_ns.append(msg_set[msg_index].default_clock_snapshot.ns_from_origin)
    init_ns = target_chain_ns[0]

    print("1")

    # draw durations
    ## rclcpp callbacks - rectify
    callback_start = (target_chain_ns[0] - init_ns)/1e6
    callback_end = (target_chain_ns[8] - init_ns)/1e6
    duration = callback_end - callback_start
    add_durations_to_figure(
        fig,
        target_chain_layer[0],
        [(callback_start, callback_start + duration, duration)],
        'lightgray'
    )

    ## rclcpp callbacks - resize
    callback_start = (target_chain_ns[9] - init_ns)/1e6
    callback_end = (target_chain_ns[17] - init_ns)/1e6
    duration = callback_end - callback_start
    add_durations_to_figure(
        fig,
        target_chain_layer[0],
        [(callback_start, callback_start + duration, duration)],
        'lightgray'
    )

    ## rectify callback
    callback_start = (target_chain_ns[1] - init_ns)/1e6
    callback_end = (target_chain_ns[7] - init_ns)/1e6
    duration = callback_end - callback_start
    add_durations_to_figure(
        fig,
        target_chain_layer[1],
        [(callback_start, callback_start + duration, duration)],
        'whitesmoke'
    )

    ## rectify op
    callback_start = (target_chain_ns[2] - init_ns)/1e6
    callback_end = (target_chain_ns[3] - init_ns)/1e6
    duration = callback_end - callback_start
    add_durations_to_figure(
        fig,
        target_chain_layer[1],
        [(callback_start, callback_start + duration, duration)],
        'seashell'
    )

    ## resize callback
    callback_start = (target_chain_ns[10] - init_ns)/1e6
    callback_end = (target_chain_ns[16] - init_ns)/1e6
    duration = callback_end - callback_start
    add_durations_to_figure(
        fig,
        target_chain_layer[1],
        [(callback_start, callback_start + duration, duration)],
        'whitesmoke'
    )
    ## resize op
    callback_start = (target_chain_ns[11] - init_ns)/1e6
    callback_end = (target_chain_ns[12] - init_ns)/1e6
    duration = callback_end - callback_start
    add_durations_to_figure(
        fig,
        target_chain_layer[1],
        [(callback_start, callback_start + duration, duration)],
        'seashell'
    )

    print("2")

    for msg_index in range(len(msg_set)):
    #     add_markers_to_figure(fig, msg_set[msg_index].event.name, [(target_chain_ns[msg_index] - init_ns)/1e6], 'blue', marker_type='plus', legend_label='timing')
        print("marker ms: " + str((target_chain_ns[msg_index] - init_ns)/1e6))
        add_markers_to_figure(
            fig,
            target_chain_layer[msg_index],
            [(target_chain_ns[msg_index] - init_ns)/1e6],
            target_chain_colors_fg_bokeh[msg_index],
            marker_type=target_chain_marker[msg_index],
            # legend_label=msg_set[msg_index].event.name,
            legend_label=target_chain_dissambiguous[msg_index],

            size=10,
        )
        if "image_proc_resize_init" in msg_set[msg_index].event.name:
            label = Label(
                x= (target_chain_ns[msg_index] - init_ns)/1e6,
                y=target_chain_label_layer[msg_index],
                x_offset=0,
                y_offset=-90,
                text=target_chain_dissambiguous[msg_index].split(":")[-1]
            )
        elif "image_proc_rectify_init" in msg_set[msg_index].event.name:
            label = Label(
                x= (target_chain_ns[msg_index] - init_ns)/1e6,
                y=target_chain_label_layer[msg_index],
                x_offset=0,
                y_offset=-100,
                text=target_chain_dissambiguous[msg_index].split(":")[-1]
            )
        elif "image_proc_rectify_fini" in msg_set[msg_index].event.name:
            label = Label(
                x= (target_chain_ns[msg_index] - init_ns)/1e6,
                y=target_chain_label_layer[msg_index],
                x_offset=-60,
                y_offset=-50,
                text=target_chain_dissambiguous[msg_index].split(":")[-1]
            )
        elif "image_proc_rectify_cb_fini" in msg_set[msg_index].event.name:
            label = Label(
                x= (target_chain_ns[msg_index] - init_ns)/1e6,
                y=target_chain_label_layer[msg_index],
                x_offset=-30,
                y_offset=-50,
                text=target_chain_dissambiguous[msg_index].split(":")[-1]
            )
        elif "callback_start" in msg_set[msg_index].event.name:
            label = Label(
                x= (target_chain_ns[msg_index] - init_ns)/1e6,
                y=target_chain_label_layer[msg_index],
                x_offset=-30,
                y_offset=-90,
                text=target_chain_dissambiguous[msg_index].split(":")[-1]
            )
        elif "image_proc_resize_fini" in msg_set[msg_index].event.name:
            label = Label(
                x= (target_chain_ns[msg_index] - init_ns)/1e6,
                y=target_chain_label_layer[msg_index],
                x_offset=20,
                y_offset=-50,
                text=target_chain_dissambiguous[msg_index].split(":")[-1]
            )
        else:
            label = Label(
                x= (target_chain_ns[msg_index] - init_ns)/1e6,
                y=target_chain_label_layer[msg_index],
                x_offset=-30,
                y_offset=-30,
                text=target_chain_dissambiguous[msg_index].split(":")[-1]
            )
        fig.add_layout(label)


    # hack legend to the right
    fig.legend.location = "right"
    new_legend = fig.legend[0]
    fig.legend[0] = None
    fig.add_layout(new_legend, 'right')
    show(fig)

def barchart_data(image_pipeline_msg_sets):
    """Converts a tracing message list into its corresponding
    relative (to the previous tracepoint) latency list in
    millisecond units.

    Args:
        image_pipeline_msg_sets ([type]): [description]

    Returns:
        list: list of relative latencies, in ms
    """
    image_pipeline_msg_sets_ns = []
    for set_index in range(len(image_pipeline_msg_sets)):
        aux_set = []
        target_chain_ns = []
        for msg_index in range(len(image_pipeline_msg_sets[set_index])):
            target_chain_ns.append(image_pipeline_msg_sets[set_index][msg_index].default_clock_snapshot.ns_from_origin)
        for msg_index in range(len(image_pipeline_msg_sets[set_index])):
            if msg_index == 0:
                previous = target_chain_ns[0]
            else:
                previous = target_chain_ns[msg_index - 1]
            aux_set.append((target_chain_ns[msg_index] - previous)/1e6)
        image_pipeline_msg_sets_ns.append(aux_set)
    return image_pipeline_msg_sets_ns

def print_timeline(image_pipeline_msg_sets):
    global target_chain
    global target_chain_colors_fg

    for msg_set in image_pipeline_msg_sets:
        if len(msg_set) != len(target_chain):
            print(color("Not a complete set: " + str([x.event.name for x in msg_set]), fg="red"))
            pass
        else:
            target_chain_ns = []
            for msg_index in range(len(msg_set)):
                target_chain_ns.append(msg_set[msg_index].default_clock_snapshot.ns_from_origin)

            init_ns = target_chain_ns[0]
            fixed_target_chain_ns = [init_ns] + target_chain_ns
            # stringout = color("raw image → " + msg_set[0].event.name + " → ")
            stringout = color("raw image ")
            for msg_index in range(len(msg_set)):
                stringout +=" → " + color(msg_set[msg_index].event.name + \
                    " ({} ms) ".format((fixed_target_chain_ns[msg_index + 1] - fixed_target_chain_ns[msg_index])/1e6),
                    fg=target_chain_colors_fg[msg_index], bg="black")
                # stringout += " → " + msg_set[msg_index].event.name + \
                #     " ({} ms) ".format((fixed_target_chain_ns[msg_index + 1] - fixed_target_chain_ns[msg_index])/1e6)

            stringout += color("total " + \
                " ({} ms) ".format((target_chain_ns[-1] - target_chain_ns[0])/1e6), fg="black", bg="white")
            print(stringout)

def rms(list):
    return np.sqrt(np.mean(np.array(list)**2))

def mean(list):
    return np.mean(np.array(list))

def max(list):
    return np.max(np.array(list))

def min(list):
    return np.min(np.array(list))

def rms_sets(image_pipeline_msg_sets, indices=None):
    """
    Root-Mean-Square (RMS) (in the units provided) for a
    given number of time trace sets.

    NOTE: last value of the lists should not include the total

    :param: image_pipeline_msg_sets, list of lists, each containing the time traces
    :param: indices, list of indices to consider on each set which will be summed
    for rms. By default, sum of all values on each set.
    """

    if indices:
        with_indices_sets = []
        for set in image_pipeline_msg_sets:
            indices_sum = 0
            for i in indices:
                indices_sum += set[i]
            with_indices_sets.append(indices_sum)
        return rms(with_indices_sets)
    else:
        total_in_sets = [sum(set) for set in image_pipeline_msg_sets]
        return rms(total_in_sets)

def mean_sets(image_pipeline_msg_sets, indices=None):
    if indices:
        with_indices_sets = []
        for set in image_pipeline_msg_sets:
            indices_sum = 0
            for i in indices:
                indices_sum += set[i]
            with_indices_sets.append(indices_sum)
        return mean(with_indices_sets)
    else:
        total_in_sets = [sum(set) for set in image_pipeline_msg_sets]
        return mean(total_in_sets)

def max_sets(image_pipeline_msg_sets, indices=None):
    if indices:
        with_indices_sets = []
        for set in image_pipeline_msg_sets:
            indices_sum = 0
            for i in indices:
                indices_sum += set[i]
            with_indices_sets.append(indices_sum)
        return max(with_indices_sets)
    else:
        total_in_sets = [sum(set) for set in image_pipeline_msg_sets]
        return max(total_in_sets)

def min_sets(image_pipeline_msg_sets, indices=None):
    if indices:
        with_indices_sets = []
        for set in image_pipeline_msg_sets:
            indices_sum = 0
            for i in indices:
                indices_sum += set[i]
            with_indices_sets.append(indices_sum)
        return min(with_indices_sets)
    else:
        total_in_sets = [sum(set) for set in image_pipeline_msg_sets]
        return min(total_in_sets)

def print_timeline_average(image_pipeline_msg_sets):
    """
    Doing averages may lead to negative numbers while substracting the previous average.
    This is only useful to get an intuition of the totals.
    """
    global target_chain
    global target_chain_colors_fg

    image_pipeline_msg_sets_ns = []
    for msg_set in image_pipeline_msg_sets:
        if len(msg_set) != len(target_chain):
            print(color("Not a complete set: " + str([x.event.name for x in msg_set]), fg="red"))
            pass
        else:
            target_chain_ns = []
            final_target_chain_ns = []
            for msg_index in range(len(msg_set)):
                target_chain_ns.append(msg_set[msg_index].default_clock_snapshot.ns_from_origin)
            init_ns = target_chain_ns[0]
            fixed_target_chain_ns = [init_ns] + target_chain_ns

            for msg_index in range(len(msg_set)):
                final_target_chain_ns.append((fixed_target_chain_ns[msg_index + 1] - fixed_target_chain_ns[msg_index]))
            final_target_chain_ns.append((fixed_target_chain_ns[-1] - fixed_target_chain_ns[0]))  # total
            image_pipeline_msg_sets_ns.append(final_target_chain_ns)

    image_pipeline_msg_ns_average = [sum(x) / len(x) for x in zip(*image_pipeline_msg_sets_ns)]
    # print(image_pipeline_msg_ns_average)
    stringout = color("raw image ")
    for msg_index in range(len(image_pipeline_msg_ns_average[:-1])):
        stringout +=" → " + color(image_pipeline_msg_sets[0][msg_index].event.name + \
            " ({} ms) ".format((image_pipeline_msg_ns_average[msg_index + 1] - image_pipeline_msg_ns_average[msg_index])/1e6),
            fg=target_chain_colors_fg[msg_index], bg="black")

    stringout += color("total " + \
        " ({} ms) ".format((image_pipeline_msg_ns_average[-1] - image_pipeline_msg_ns_average[0])/1e6), fg="black", bg="white")
    print(stringout)

def statistics(image_pipeline_msg_sets_ms, verbose=False):
    global target_chain_dissambiguous

    mean_ = mean_sets(image_pipeline_msg_sets_ms)
    rms_ = rms_sets(image_pipeline_msg_sets_ms)
    min_ = min_sets(image_pipeline_msg_sets_ms)
    max_ = max_sets(image_pipeline_msg_sets_ms)

    mean_accelerators = mean_sets(image_pipeline_msg_sets_ms,
        [
            target_chain_dissambiguous.index("ros2_image_pipeline:image_proc_rectify_fini"),
            target_chain_dissambiguous.index("ros2_image_pipeline:image_proc_resize_fini"),
        ]
    )
    rms_accelerators = rms_sets(image_pipeline_msg_sets_ms,
        [
            target_chain_dissambiguous.index("ros2_image_pipeline:image_proc_rectify_fini"),
            target_chain_dissambiguous.index("ros2_image_pipeline:image_proc_resize_fini"),
        ]
    )
    max_accelerators = max_sets(image_pipeline_msg_sets_ms,
        [
            target_chain_dissambiguous.index("ros2_image_pipeline:image_proc_rectify_fini"),
            target_chain_dissambiguous.index("ros2_image_pipeline:image_proc_resize_fini"),
        ]
    )
    min_accelerators = min_sets(image_pipeline_msg_sets_ms,
        [
            target_chain_dissambiguous.index("ros2_image_pipeline:image_proc_rectify_fini"),
            target_chain_dissambiguous.index("ros2_image_pipeline:image_proc_resize_fini"),
        ]
    )

    if verbose:
        print(color("mean: " + str(mean_), fg="yellow"))
        print("rms: " + str(rms_))
        print("min: " + str(min_))
        print(color("max: " + str(max_), fg="red"))

        print(color("mean accelerators: " + str(mean_accelerators), fg="yellow"))
        print("rms accelerators: " + str(rms_accelerators))
        print("min accelerators: " + str(min_accelerators))
        print(color("max accelerators: " + str(max_accelerators), fg="red"))

    return [mean_accelerators, rms_accelerators, max_accelerators, min_accelerators, mean_, rms_, max_, min_]

def table(list_sets, list_sets_names):
    """
    Creates a markdown table from a list of sets

    NOTE: assumes base is always the first set in list_sets, which
    is then used to calculate % of change.
    """

    list_statistics = []
    # generate statistics
    for sets in list_sets:
        list_statistics.append(statistics(sets))

    # Add name to each statistics list
    for stat_list_index in range(len(list_statistics)):
        list_statistics[stat_list_index].insert(0, list_sets_names[stat_list_index])

    # add headers
    list_statistics.insert(0, ["---", "---", "---", "---", "---", "---", "---", "---", "---",])
    list_statistics.insert(0, [
            " ", "Accel. Mean", "Accel. RMS",
            "Accel. Max ", "Accel. Min", "Mean",
            "RMS", "Max", "Min"])

    baseline = list_statistics[2]  # baseline for %

    length_list = [len(row) for row in list_statistics]
    column_width = max(length_list)
    count = 0
    for row in list_statistics:
        row_str = " | "
        if count == 2:
            for element_index in range(len(row)):
                if type(row[element_index]) != str:
                    if row[element_index] > baseline[element_index]:
                        row_str += "**{:.2f}** ms".format(row[element_index]) + " (:small_red_triangle_down: `" \
                                + "{:.2f}".format(get_change(row[element_index], baseline[element_index])) + "`%) | "
                    else:
                        row_str += "**{:.2f}** ms".format(row[element_index]) + " (`" \
                                + "{:.2f}".format(get_change(row[element_index], baseline[element_index])) + "`%) | "
                else:
                    row_str += row[element_index] + " | "

        else:
            for element_index in range(len(row)):
                if type(row[element_index]) != str:
                    if row[element_index] > baseline[element_index]:
                        row_str += "{:.2f} ms".format(row[element_index]) + " (:small_red_triangle_down: `" \
                                + "{:.2f}".format(get_change(row[element_index], baseline[element_index])) + "`%) | "
                    else:
                        row_str += "{:.2f} ms".format(row[element_index]) + " (`" \
                                + "{:.2f}".format(get_change(row[element_index], baseline[element_index])) + "`%) | "
                else:
                    row_str += row[element_index] + " | "
        count += 1
        print(row_str)


        # if count == 2:
        #     row = "|" + "|".join("**{:.2f}** ms".format(row[element_index]) + " (`"
        #             + "{:.2f}".format(get_change(row[element_index], baseline[element_index])) + "`%)"
        #         if type(row[element_index]) != str
        #         else row[element_index]
        #             for element_index in range(len(row))) + "|"
        # else:
        #     row = "|" + "|".join("{:.2f} ms".format(row[element_index]) + " (`"
        #             + "{:.2f}".format(get_change(row[element_index], baseline[element_index])) + "`%)"
        #         if type(row[element_index]) != str else row[element_index]
        #             for element_index in range(len(row))) + "|"
        # count += 1
        # print(row)


def generate_launch_description():
    return LaunchDescription()

##############################
##############################

# targeted chain of messages for tracing
target_chain = [
    "ros2:callback_start",
    "ros2_image_pipeline:image_proc_rectify_cb_init",
    "ros2_image_pipeline:image_proc_rectify_init",
    "ros2_image_pipeline:image_proc_rectify_fini",
    "ros2:rclcpp_publish",
    "ros2:rcl_publish",
    "ros2:rmw_publish",
    "ros2_image_pipeline:image_proc_rectify_cb_fini",
    "ros2:callback_end",
    "ros2:callback_start",
    "ros2_image_pipeline:image_proc_resize_cb_init",
    "ros2_image_pipeline:image_proc_resize_init",
    "ros2_image_pipeline:image_proc_resize_fini",
    "ros2:rclcpp_publish",
    "ros2:rcl_publish",
    "ros2:rmw_publish",
    "ros2_image_pipeline:image_proc_resize_cb_fini",
    "ros2:callback_end",
]
target_chain_dissambiguous = [
    "ros2:callback_start",
    "ros2_image_pipeline:image_proc_rectify_cb_init",
    "ros2_image_pipeline:image_proc_rectify_init",
    "ros2_image_pipeline:image_proc_rectify_fini",
    "ros2:rclcpp_publish",
    "ros2:rcl_publish",
    "ros2:rmw_publish",
    "ros2_image_pipeline:image_proc_rectify_cb_fini",
    "ros2:callback_end",
    "ros2:callback_start (2)",
    "ros2_image_pipeline:image_proc_resize_cb_init",
    "ros2_image_pipeline:image_proc_resize_init",
    "ros2_image_pipeline:image_proc_resize_fini",
    "ros2:rclcpp_publish (2)",
    "ros2:rcl_publish (2)",
    "ros2:rmw_publish (2)",
    "ros2_image_pipeline:image_proc_resize_cb_fini",
    "ros2:callback_end (2)",
]
target_chain_colors_fg = [
    "blue",
    "yellow",
    "red",
    "red",
    "blue",
    "blue",
    "blue",
    "yellow",
    "blue",
    "blue",
    "yellow",
    "red",
    "red",
    "blue",
    "blue",
    "blue",
    "yellow",
    "blue",
]
# target_chain_colors_fg_bokeh = [
#     "lightgray",
#     "silver",
#     "darkgray",
#     "gray",
#     "dimgray",
#     "lightslategray",
#     "slategray",
#     "darkslategray",
#     "black",
#     "burlywood",
#     "tan",
#     "rosybrown",
#     "sandybrown",
#     "goldenrod",
#     "darkgoldenrod",
#     "peru",
#     "chocolate",
#     "saddlebrown",
#     # "blue",
#     # "blueviolet",
#     # "brown",
#     # "burlywood",
#     # "cadetblue",
#     # "chartreuse",
#     # "chocolate",
#     # "coral",
#     # "cornflowerblue",
# ]
target_chain_colors_fg_bokeh = [
    "lightsalmon",
    "salmon",
    "darksalmon",
    "lightcoral",
    "indianred",
    "crimson",
    "firebrick",
    "darkred",
    "red",
    "lavender",
    "thistle",
    "plum",
    "fuchsia",
    "mediumorchid",
    "mediumpurple",
    "darkmagenta",
    "indigo",
    "mediumslateblue",
]
target_chain_layer = [
    "rclcpp",
    "userland",
    "userland",
    "userland",
    "rclcpp",
    "rcl",
    "rmw",
    "userland",
    "rclcpp",
    "rclcpp",
    "userland",
    "userland",
    "userland",
    "rclcpp",
    "rcl",
    "rmw",
    "userland",
    "rclcpp",
]
target_chain_label_layer = [  # associated with the layer
    3,
    4,
    4,
    4,
    3,
    2,
    1,
    4,
    3,
    3,
    4,
    4,
    4,
    3,
    2,
    1,
    4,
    3,
]
target_chain_marker = [
    "diamond",
    "plus",
    "plus",
    "plus",
    "plus",
    "plus",
    "plus",
    "plus",
    "diamond",
    "diamond",
    "plus",
    "plus",
    "plus",
    "plus",
    "plus",
    "plus",
    "plus",
    "diamond",
]
# For some reason it seems to be displayed in the reverse order on the Y axis
segment_types = [
    "rmw",
    "rcl",
    "rclcpp",
    "userland"
]

# # ####################
# # print timing pipeline
# # ####################
# # image_pipeline_msg_sets = msgsets_from_trace_concurrent(str(os.environ["HOME"]) + "/.ros/tracing/trace_rectify_resize_fpga")
# # # print(len(image_pipeline_msg_sets))
# # # print_timeline(image_pipeline_msg_sets)  # all timelines
# # print_timeline([image_pipeline_msg_sets[-1]])  # timeline of last message
# # # print_timeline_average(image_pipeline_msg_sets)  # timeline of averages, NOTE only totals are of interest

# target_chain = [
#     "ros2:callback_start",
#     "ros2_image_pipeline:image_proc_rectify_cb_init",
#     "ros2_image_pipeline:image_proc_rectify_init",
#     "ros2_image_pipeline:image_proc_rectify_fini",
#     "ros2:rclcpp_publish",
#     "ros2:rcl_publish",
#     "ros2:rmw_publish",
#     "ros2_image_pipeline:image_proc_rectify_cb_fini",
#     "ros2:callback_end",
# ]
# target_chain_colors_fg = [
#     "blue",
#     "yellow",
#     "red",
#     "red",
#     "blue",
#     "blue",
#     "blue",
#     "yellow",
#     "blue",
# ]
# # image_pipeline_msg_sets = msgsets_from_trace_concurrent(str(os.environ["HOME"]) + "/.ros/tracing/trace_rectify_resize_fpga_integrated")
# image_pipeline_msg_sets = msgsets_from_trace_concurrent(str(os.environ["HOME"]) + "/.ros/tracing/trace_rectify_resize_fpga_integrated_250_node")
# # print(len(image_pipeline_msg_sets))
# # print_timeline(image_pipeline_msg_sets)  # all timelines
# # print_timeline([image_pipeline_msg_sets[-1]])  # timeline of last message
# print_timeline(image_pipeline_msg_sets[-10:])  # timeline of last 10 messages
# # print_timeline_average(image_pipeline_msg_sets)  # timeline of averages, NOTE only totals are of interest


######################
# draw tracepoints
######################
# image_pipeline_msg_sets = msgsets_from_trace_concurrent(str(os.environ["HOME"]) + "/.ros/tracing/trace_rectify_resize")
# msg_set = image_pipeline_msg_sets[-1]
# traces(msg_set)

# ######################
# # draw barplot all data
# ######################
# # # NOTE: Discard first few
# image_pipeline_msg_sets = msgsets_from_trace_concurrent(str(os.environ["HOME"]) + "/.ros/tracing/trace_rectify_resize")
# barplot_all(image_pipeline_msg_sets[10:], title="image_pipeline in CPU")
# image_pipeline_msg_sets = msgsets_from_trace_concurrent(str(os.environ["HOME"]) + "/.ros/tracing/trace_rectify_resize_fpga")
# barplot_all(image_pipeline_msg_sets[10:], title="image_pipeline in FPGA")

# # image_pipeline_msg_sets = msgsets_from_trace_concurrent(str(os.environ["HOME"]) + "/.ros/tracing/trace_rectify_resize_stress")
# # barplot_all(image_pipeline_msg_sets[10:], title="image_pipeline in CPU and with stress")
# # image_pipeline_msg_sets = msgsets_from_trace_concurrent(str(os.environ["HOME"]) + "/.ros/tracing/trace_rectify_resize_fpga_stress")
# # barplot_all(image_pipeline_msg_sets[10:], title="image_pipeline in FPGA and with stress")


# target_chain = [
#     "ros2:callback_start",
#     "ros2_image_pipeline:image_proc_rectify_cb_init",
#     "ros2_image_pipeline:image_proc_rectify_init",
#     "ros2_image_pipeline:image_proc_rectify_fini",
#     "ros2:rclcpp_publish",
#     "ros2:rcl_publish",
#     "ros2:rmw_publish",
#     "ros2_image_pipeline:image_proc_rectify_cb_fini",
#     "ros2:callback_end",
#     # "ros2:callback_start",
#     # "ros2_image_pipeline:image_proc_resize_cb_init",
#     # "ros2_image_pipeline:image_proc_resize_init",
#     # "ros2_image_pipeline:image_proc_resize_fini",
#     # "ros2:rclcpp_publish",
#     # "ros2:rcl_publish",
#     # "ros2:rmw_publish",
#     # "ros2_image_pipeline:image_proc_resize_cb_fini",
#     # "ros2:callback_end",
# ]
# target_chain_dissambiguous = target_chain
# target_chain_colors_fg = [
#     "blue",
#     "yellow",
#     "red",
#     "red",
#     "blue",
#     "blue",
#     "blue",
#     "yellow",
#     "blue",
#     # "blue",
#     # "yellow",
#     # "red",
#     # "red",
#     # "blue",
#     # "blue",
#     # "blue",
#     # "yellow",
#     # "blue",
# ]

# image_pipeline_msg_sets = msgsets_from_trace_concurrent(str(os.environ["HOME"]) + "/.ros/tracing/trace_rectify_resize_fpga_integrated")
# barplot_all(image_pipeline_msg_sets[10:], title="image_pipeline integrated @ 250 MHz in FPGA")

# target_chain = [
#     "ros2:callback_start", "ros2_image_pipeline:image_proc_resize_cb_init",
#     "ros2_image_pipeline:image_proc_resize_init", "ros2_image_pipeline:image_proc_resize_fini",
#     "ros2:rclcpp_publish", "ros2:rcl_publish", "ros2:rmw_publish",
#     "ros2_image_pipeline:image_proc_resize_cb_fini", "ros2:callback_end",
# ]
# target_chain_dissambiguous = target_chain
# image_pipeline_msg_sets = msgsets_from_trace_concurrent(str(os.environ["HOME"]) + "/.ros/tracing/trace_test2")
# barplot_all(image_pipeline_msg_sets[10:], title="image_pipeline, streams @ 250 MHz in FPGA")




# ######################
# # draw bar charts
# ######################

#///////////////////
# Data sources
#///////////////////

# # NOTE: Discard first few
discard_count = 10

image_pipeline_msg_sets_ms_cpu = barchart_data(msgsets_from_trace_concurrent(str(os.environ["HOME"]) \
    + "/.ros/tracing/trace_rectify_resize")[discard_count:])
image_pipeline_msg_sets_ms_fpga = barchart_data(msgsets_from_trace_concurrent(str(os.environ["HOME"]) \
    + "/.ros/tracing/trace_rectify_resize_fpga")[discard_count:])
# image_pipeline_msg_sets_ms_fpga_streamlined = barchart_data(msgsets_from_trace_concurrent(str(os.environ["HOME"]) \
#     + "/.ros/tracing/trace_rectify_resize_fpga_streamlined")[discard_count:])
# image_pipeline_msg_sets_ms_fpga_streamlined_xrt = barchart_data(msgsets_from_trace_concurrent(str(os.environ["HOME"]) \
#     + "/.ros/tracing/trace_rectify_resize_fpga_streamlined_xrt")[discard_count:])
image_pipeline_msg_sets_ms_cpu_stress = barchart_data(msgsets_from_trace_concurrent(str(os.environ["HOME"]) \
    + "/.ros/tracing/trace_rectify_resize_stress")[discard_count:])
image_pipeline_msg_sets_ms_fpga_stress = barchart_data(msgsets_from_trace_concurrent(str(os.environ["HOME"]) \
    + "/.ros/tracing/trace_rectify_resize_fpga_stress")[discard_count:])

target_chain = [
    "ros2:callback_start", "ros2_image_pipeline:image_proc_rectify_cb_init",
    "ros2_image_pipeline:image_proc_rectify_init", "ros2_image_pipeline:image_proc_rectify_fini",
    "ros2:rclcpp_publish", "ros2:rcl_publish", "ros2:rmw_publish",
    "ros2_image_pipeline:image_proc_rectify_cb_fini", "ros2:callback_end",
]
image_pipeline_msg_sets_ms_fpga_integrated = barchart_data(msgsets_from_trace_concurrent(str(os.environ["HOME"]) \
    + "/.ros/tracing/trace_rectify_resize_fpga_integrated")[discard_count:])
# fix data of "*_integrated" to align with dimensions of the rest
for i_set in range(len(image_pipeline_msg_sets_ms_fpga_integrated)):
    image_pipeline_msg_sets_ms_fpga_integrated[i_set] += [0, 0, 0, 0, 0, 0, 0, 0, 0]

image_pipeline_msg_sets_ms_fpga_integrated_xrt = barchart_data(msgsets_from_trace_concurrent(str(os.environ["HOME"]) \
    + "/.ros/tracing/trace_rectify_resize_fpga_integrated_xrt")[discard_count:])
# fix data of "*_integrated" to align with dimensions of the rest
for i_set in range(len(image_pipeline_msg_sets_ms_fpga_integrated_xrt)):
    image_pipeline_msg_sets_ms_fpga_integrated_xrt[i_set] += [0, 0, 0, 0, 0, 0, 0, 0, 0]

image_pipeline_msg_sets_ms_fpga_integrated_streamlined = barchart_data(msgsets_from_trace_concurrent(str(os.environ["HOME"]) \
    + "/.ros/tracing/trace_rectify_resize_fpga_integrated_streamlined")[discard_count:])
# fix data of "*_integrated" to align with dimensions of the rest
for i_set in range(len(image_pipeline_msg_sets_ms_fpga_integrated_streamlined)):
    image_pipeline_msg_sets_ms_fpga_integrated_streamlined[i_set] += [0, 0, 0, 0, 0, 0, 0, 0, 0]

image_pipeline_msg_sets_ms_fpga_integrated_streamlined_xrt = barchart_data(msgsets_from_trace_concurrent(str(os.environ["HOME"]) \
    + "/.ros/tracing/trace_rectify_resize_fpga_integrated_streamlined_xrt")[discard_count:])
# fix data of "*_integrated" to align with dimensions of the rest
for i_set in range(len(image_pipeline_msg_sets_ms_fpga_integrated_streamlined_xrt)):
    image_pipeline_msg_sets_ms_fpga_integrated_streamlined_xrt[i_set] += [0, 0, 0, 0, 0, 0, 0, 0, 0]

# stress
image_pipeline_msg_sets_ms_fpga_integrated_stress = barchart_data(msgsets_from_trace_concurrent(str(os.environ["HOME"]) \
    + "/.ros/tracing/trace_rectify_resize_fpga_integrated_stress")[discard_count:])
# fix data of "*_integrated" to align with dimensions of the rest
for i_set in range(len(image_pipeline_msg_sets_ms_fpga_integrated_stress)):
    image_pipeline_msg_sets_ms_fpga_integrated_stress[i_set] += [0, 0, 0, 0, 0, 0, 0, 0, 0]

image_pipeline_msg_sets_ms_fpga_integrated_xrt_stress = barchart_data(msgsets_from_trace_concurrent(str(os.environ["HOME"]) \
    + "/.ros/tracing/trace_rectify_resize_fpga_integrated_xrt_stress")[discard_count:])
# fix data of "*_integrated" to align with dimensions of the rest
for i_set in range(len(image_pipeline_msg_sets_ms_fpga_integrated_xrt_stress)):
    image_pipeline_msg_sets_ms_fpga_integrated_xrt_stress[i_set] += [0, 0, 0, 0, 0, 0, 0, 0, 0]

# image_pipeline_msg_sets_ms_fpga_integrated_streamlined_stress = barchart_data(msgsets_from_trace_concurrent(str(os.environ["HOME"]) \
#     + "/.ros/tracing/trace_rectify_resize_fpga_integrated_streamlined_stress")[discard_count:])
# # fix data of "*_integrated" to align with dimensions of the rest
# for i_set in range(len(image_pipeline_msg_sets_ms_fpga_integrated_streamlined_stress)):
#     image_pipeline_msg_sets_ms_fpga_integrated_streamlined_stress[i_set] += [0, 0, 0, 0, 0, 0, 0, 0, 0]

image_pipeline_msg_sets_ms_fpga_integrated_streamlined_xrt_stress = barchart_data(msgsets_from_trace_concurrent(str(os.environ["HOME"]) \
    + "/.ros/tracing/trace_rectify_resize_fpga_integrated_streamlined_xrt_stress")[discard_count:])
# fix data of "*_integrated" to align with dimensions of the rest
for i_set in range(len(image_pipeline_msg_sets_ms_fpga_integrated_streamlined_xrt_stress)):
    image_pipeline_msg_sets_ms_fpga_integrated_streamlined_xrt_stress[i_set] += [0, 0, 0, 0, 0, 0, 0, 0, 0]

target_chain = [
    "ros2:callback_start", "ros2_image_pipeline:image_proc_resize_cb_init",
    "ros2_image_pipeline:image_proc_resize_init", "ros2_image_pipeline:image_proc_resize_fini",
    "ros2:rclcpp_publish", "ros2:rcl_publish", "ros2:rmw_publish",
    "ros2_image_pipeline:image_proc_resize_cb_fini", "ros2:callback_end",
]
image_pipeline_msg_sets_ms_fpga_streamlined = barchart_data(msgsets_from_trace_concurrent(str(os.environ["HOME"]) \
    + "/.ros/tracing/trace_rectify_resize_fpga_streamlined")[discard_count:])
# fix data of "*_integrated" to align with dimensions of the rest
for i_set in range(len(image_pipeline_msg_sets_ms_fpga_streamlined)):
    image_pipeline_msg_sets_ms_fpga_streamlined[i_set] = [0, 0, 0, 0, 0, 0, 0, 0, 0] + image_pipeline_msg_sets_ms_fpga_streamlined[i_set]

image_pipeline_msg_sets_ms_fpga_streamlined_xrt = barchart_data(msgsets_from_trace_concurrent(str(os.environ["HOME"]) \
    + "/.ros/tracing/trace_rectify_resize_fpga_streamlined_xrt")[discard_count:])
# fix data of "*_integrated" to align with dimensions of the rest
for i_set in range(len(image_pipeline_msg_sets_ms_fpga_streamlined_xrt)):
    image_pipeline_msg_sets_ms_fpga_streamlined_xrt[i_set] = [0, 0, 0, 0, 0, 0, 0, 0, 0] + image_pipeline_msg_sets_ms_fpga_streamlined_xrt[i_set]


target_chain = [
    "ros2:callback_start", "ros2_image_pipeline:image_proc_rectify_cb_init",
    "ros2_image_pipeline:image_proc_rectify_init", "ros2_image_pipeline:image_proc_rectify_fini",    
    "ros2_image_pipeline:image_proc_rectify_cb_fini", "ros2:callback_end",
]

image_pipeline_msg_sets_ms_fpga_streamlined_rectify = barchart_data(msgsets_from_trace_concurrent(str(os.environ["HOME"]) \
    + "/.ros/tracing/trace_rectify_resize_fpga_streamlined")[discard_count:])
# fix data of "*_integrated" to align with dimensions of the rest
for i_set in range(len(image_pipeline_msg_sets_ms_fpga_streamlined_rectify)):
    image_pipeline_msg_sets_ms_fpga_streamlined_rectify[i_set] = image_pipeline_msg_sets_ms_fpga_streamlined_rectify[i_set] + [0, 0, 0, 0, 0, 0, 0, 0, 0]


# image_pipeline_msg_sets_ms_fpga_integrated = barchart_data(msgsets_from_trace_concurrent(str(os.environ["HOME"]) \
#     + "/.ros/tracing/trace_rectify_resize_fpga_integrated")[discard_count:])
# # fix data of "*_integrated" to align with dimensions of the rest
# for i_set in range(len(image_pipeline_msg_sets_ms_fpga_integrated)):
#     image_pipeline_msg_sets_ms_fpga_integrated[i_set] += [0, 0, 0, 0, 0, 0, 0, 0, 0]

# image_pipeline_msg_sets_ms_fpga_integrated_200 = barchart_data(msgsets_from_trace_concurrent(str(os.environ["HOME"]) \
#     + "/.ros/tracing/trace_rectify_resize_fpga_integrated_200")[discard_count:])
# # fix data of "*_integrated" to align with dimensions of the rest
# for i_set in range(len(image_pipeline_msg_sets_ms_fpga_integrated_200)):
#     image_pipeline_msg_sets_ms_fpga_integrated_200[i_set] += [0, 0, 0, 0, 0, 0, 0, 0, 0]

# image_pipeline_msg_sets_ms_fpga_integrated_250 = barchart_data(msgsets_from_trace_concurrent(str(os.environ["HOME"]) \
#     + "/.ros/tracing/trace_rectify_resize_fpga_integrated_250")[discard_count:])
# # fix data of "*_integrated" to align with dimensions of the rest
# for i_set in range(len(image_pipeline_msg_sets_ms_fpga_integrated_250)):
#     image_pipeline_msg_sets_ms_fpga_integrated_250[i_set] += [0, 0, 0, 0, 0, 0, 0, 0, 0]

# image_pipeline_msg_sets_ms_fpga_integrated_250_stress = barchart_data(msgsets_from_trace_concurrent(str(os.environ["HOME"]) \
#     + "/.ros/tracing/trace_rectify_resize_fpga_integrated_250_stress")[discard_count:])
# # fix data of "*_integrated" to align with dimensions of the rest
# for i_set in range(len(image_pipeline_msg_sets_ms_fpga_integrated_250_stress)):
#     image_pipeline_msg_sets_ms_fpga_integrated_250_stress[i_set] += [0, 0, 0, 0, 0, 0, 0, 0, 0]

# image_pipeline_msg_sets_ms_fpga_integrated_250_xrt = barchart_data(msgsets_from_trace_concurrent(str(os.environ["HOME"]) \
#     + "/.ros/tracing/trace_rectify_resize_fpga_integrated_250_xrt")[discard_count:])
# # fix data of "*_integrated" to align with dimensions of the rest
# for i_set in range(len(image_pipeline_msg_sets_ms_fpga_integrated_250_xrt)):
#     image_pipeline_msg_sets_ms_fpga_integrated_250_xrt[i_set] += [0, 0, 0, 0, 0, 0, 0, 0, 0]

# image_pipeline_msg_sets_ms_fpga_streamlined = barchart_data(msgsets_from_trace_concurrent(str(os.environ["HOME"]) \
#     + "/.ros/tracing/trace_rectify_resize_fpga_integrated_streamlined")[discard_count:])
# # fix data of "*_integrated" to align with dimensions of the rest
# for i_set in range(len(image_pipeline_msg_sets_ms_fpga_streamlined)):
#     image_pipeline_msg_sets_ms_fpga_streamlined[i_set] += [0, 0, 0, 0, 0, 0, 0, 0, 0]

# image_pipeline_msg_sets_ms_fpga_streamlined_xrt = barchart_data(msgsets_from_trace_concurrent(str(os.environ["HOME"]) \
#     + "/.ros/tracing/trace_rectify_resize_fpga_integrated_streamlined_xrt")[discard_count:])
# # fix data of "*_integrated" to align with dimensions of the rest
# for i_set in range(len(image_pipeline_msg_sets_ms_fpga_streamlined_xrt)):
#     image_pipeline_msg_sets_ms_fpga_streamlined_xrt[i_set] += [0, 0, 0, 0, 0, 0, 0, 0, 0]


# #///////////////////
# # Markdown Table results
# #///////////////////

# table(
#     [
#         # full pipeline
#         image_pipeline_msg_sets_ms_cpu,
#         image_pipeline_msg_sets_ms_fpga,
#         # # integrated
#         image_pipeline_msg_sets_ms_fpga_integrated,
#         # image_pipeline_msg_sets_ms_fpga_integrated_xrt,
#         # streamlined
#         image_pipeline_msg_sets_ms_fpga_streamlined,
#         image_pipeline_msg_sets_ms_fpga_streamlined_xrt,
#         # # integrated, streamlined
#         # image_pipeline_msg_sets_ms_fpga_integrated_streamlined,
#         # image_pipeline_msg_sets_ms_fpga_integrated_streamlined_xrt,
#         #
#         # # full pipeline stress
#         # image_pipeline_msg_sets_ms_cpu,
#         # image_pipeline_msg_sets_ms_fpga,
#         # # image_pipeline_msg_sets_ms_fpga_streamlined,
#         # # image_pipeline_msg_sets_ms_fpga_streamlined_xrt,
#         # # integrated stress
#         # image_pipeline_msg_sets_ms_fpga_integrated,
#         # image_pipeline_msg_sets_ms_fpga_integrated_xrt,
#         # # integrated, streamlined stress
#         # # image_pipeline_msg_sets_ms_fpga_integrated_streamlined,
#         # image_pipeline_msg_sets_ms_fpga_integrated_streamlined_xrt,
#     ],
#     [
#         # full pipeline
#         "CPU **baseline**",
#         "FPGA @ 250 MHz",
#         # # integrated
#         "FPGA, integrated @ 250 MHz",
#         # "FPGA, integrated, XRT @ 250 MHz",
#         # streamlined
#         "FPGA, streams (resize) @ 250 MHz",
#         "FPGA, streams (resize), XRT @ 250 MHz",
#         # # integrated, streamlined
#         # "FPGA, integrated, streams @ 250 MHz",
#         # "FPGA, integrated, streams, XRT @ 250 MHz",
#         #
#         # # full pipeline stress
#         # "CPU **baseline**",
#         # "FPGA @ 250 MHz",
#         # # "FPGA, streams @ 250 MHz",
#         # # "FPGA, streams, XRT @ 250 MHz",
#         # # integrated stress
#         # "FPGA, integrated @ 250 MHz",
#         # "FPGA, integrated, XRT @ 250 MHz",
#         # # integrated, streamlined stress
#         # # "FPGA, integrated, streams @ 250 MHz",
#         # "FPGA, integrated, streams, XRT @ 250 MHz",
#     ]
# )

#///////////////////
# Plot, either averages or latest, etc
#///////////////////

# # plot latest values
# df_cpu = pd.DataFrame(image_pipeline_msg_sets_ms_cpu[-1:])  # pick the latest one
# df_fpga = pd.DataFrame(image_pipeline_msg_sets_ms_fpga[-1:])  # pick the latest one
# df = pd.concat([df_cpu, df_fpga], ignore_index=True)
# df.columns = target_chain_dissambiguous
# substrates = pd.DataFrame({'substrate': ["CPU","FPGA"]})
# df = df.join(substrates)

# plot averages
df_cpu_mean = pd.DataFrame(image_pipeline_msg_sets_ms_cpu).mean()
df_fpga_mean = pd.DataFrame(image_pipeline_msg_sets_ms_fpga).mean()
df_fpga_mean_streamlined = pd.DataFrame(image_pipeline_msg_sets_ms_fpga_streamlined).mean()
df_fpga_mean_streamlined_rectify = pd.DataFrame(image_pipeline_msg_sets_ms_fpga_streamlined_rectify).mean()

df_fpga_mean_streamlined_xrt = pd.DataFrame(image_pipeline_msg_sets_ms_fpga_streamlined_xrt).mean()
df_fpga_integrated_mean = pd.DataFrame(image_pipeline_msg_sets_ms_fpga_integrated).mean()
df_fpga_integrated_mean_xrt = pd.DataFrame(image_pipeline_msg_sets_ms_fpga_integrated_xrt).mean()
df_fpga_integrated_streamlined_mean = pd.DataFrame(image_pipeline_msg_sets_ms_fpga_integrated_streamlined).mean()
df_fpga_integrated_streamlined_mean_xrt = pd.DataFrame(image_pipeline_msg_sets_ms_fpga_integrated_streamlined_xrt).mean()



df_cpu_mean_stress = pd.DataFrame(image_pipeline_msg_sets_ms_cpu_stress).mean()
df_fpga_mean_stress = pd.DataFrame(image_pipeline_msg_sets_ms_fpga_stress).mean()
# df_fpga_mean_stress_streamlined = pd.DataFrame(image_pipeline_msg_sets_ms_fpga_streamlined_stress).mean()
# df_fpga_mean_stress_streamlined_xrt = pd.DataFrame(image_pipeline_msg_sets_ms_fpga_streamlined_xrt_stress).mean()
df_fpga_integrated_mean_stress = pd.DataFrame(image_pipeline_msg_sets_ms_fpga_integrated_stress).mean()
df_fpga_integrated_mean_xrt_stress = pd.DataFrame(image_pipeline_msg_sets_ms_fpga_integrated_xrt_stress).mean()
# df_fpga_integrated_streamlined_mean_stress = pd.DataFrame(image_pipeline_msg_sets_ms_fpga_integrated_streamlined_stress).mean()
df_fpga_integrated_streamlined_mean_xrt_stress = pd.DataFrame(image_pipeline_msg_sets_ms_fpga_integrated_streamlined_xrt_stress).mean()

# df_cpu_stress_mean = pd.DataFrame(image_pipeline_msg_sets_ms_cpu_stress).mean()
# df_fpga_stress_mean = pd.DataFrame(image_pipeline_msg_sets_ms_fpga_stress).mean()
# df_fpga_integrated_mean = pd.DataFrame(image_pipeline_msg_sets_ms_fpga_integrated).mean()
# df_fpga_integrated_mean_200 = pd.DataFrame(image_pipeline_msg_sets_ms_fpga_integrated_200).mean()
# df_fpga_integrated_mean_250 = pd.DataFrame(image_pipeline_msg_sets_ms_fpga_integrated_250).mean()
# df_fpga_integrated_mean_250_stress = pd.DataFrame(image_pipeline_msg_sets_ms_fpga_integrated_250_stress).mean()
# df_fpga_integrated_mean_250_xrt = pd.DataFrame(image_pipeline_msg_sets_ms_fpga_integrated_250_xrt).mean()
# df_fpga_mean_streamlined = pd.DataFrame(image_pipeline_msg_sets_ms_fpga_streamlined).mean()
# df_fpga_mean_streamlined_xrt = pd.DataFrame(image_pipeline_msg_sets_ms_fpga_streamlined_xrt).mean()
# df_fpga_integrated_mean_250_new = pd.DataFrame(image_pipeline_msg_sets_ms_fpga_integrated_250_new).mean()
# df_fpga_integrated_mean_250_node = pd.DataFrame(image_pipeline_msg_sets_ms_fpga_integrated_250_node).mean()

print(target_chain_dissambiguous)
print("CPU: " + str(list(df_cpu_mean)))
print("FPGA @ 250 MHz: " + str(list(df_fpga_mean)))
print("FPGA, integrated @ 250 MHz: " + str(list(df_fpga_integrated_mean)))
print("FPGA, streams (resize) @ 250 MHz: " + str(list(df_fpga_mean_streamlined)))



# df_mean = pd.concat(
#     [
#         # full pipeline
#         df_cpu_mean,
#         # df_fpga_mean,
#         # # integrated
#         # df_fpga_integrated_mean,
#         # df_fpga_integrated_mean_xrt,
#         # streamlined
#         df_fpga_mean_streamlined,
#         df_fpga_mean_streamlined_rectify,
#         # df_fpga_mean_streamlined_xrt,
#         # # integrated, streamlined
#         # df_fpga_integrated_streamlined_mean,
#         # df_fpga_integrated_streamlined_mean_xrt,
#     ], axis=1).transpose()
# df_mean.columns = target_chain_dissambiguous
# substrates = pd.DataFrame({'substrate':
#     [
#         # full pipeline
#         "CPU",
#         # "FPGA @ 250 MHz",
#         # # integrated
#         # "FPGA, integrated @ 250 MHz",
#         # "FPGA, integrated, XRT @ 250 MHz",
#         # streamlined
#         "FPGA, streams (resize) @ 250 MHz",
#         "FPGA, streams (rectify) @ 250 MHz",
#         # "FPGA, streams (resize), XRT @ 250 MHz",
#         # # integrated, streamlined
#         # "FPGA, integrated, streams @ 250 MHz",
#         # "FPGA, integrated, streams, XRT @ 250 MHz",
#     ]})
# df_mean = df_mean.join(substrates)

# import plotly.express as px
# fig = px.bar(
#     df_mean,
#     template="plotly_white",
#     x="substrate",
#     y=target_chain_dissambiguous,
#     color_discrete_sequence=px.colors.sequential.Inferno + px.colors.diverging.BrBG,
#     # colors at https://plotly.com/python/discrete-color/
# )
# fig.update_xaxes(title_text = "")
# fig.update_yaxes(title_text = "Milliseconds")
# fig.show()
# # fig.write_image("/tmp/benchmarkintegrated_only.png", width=1400, height=1000)