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
from wasabi import color
from typing import List, Optional, Tuple, Union
import pandas as pd

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

    for trace in image_pipeline_msgs:        
        vpid = trace.event.common_context_field.get("vpid")
        if trace.event.name == target_chain[0]:
            if (vpid in candidates) and (candidates[vpid][-1].event.name ==  target_chain[-1]):  # account for chained traces, use "ros2:callback_end"
                # print(color("Continuing: " + str(trace.event.name), fg="green"))
                candidates[vpid].append(trace)
            elif vpid in candidates:
                # print(color("Already a set, re-starting: " + str(trace.event.name) + " - " \
                #     + str([x.event.name for x in candidates[vpid]]) , fg="yellow"))
                candidates[vpid] = [trace]  # already a set existing (pop and) re-start
            else:
                candidates[vpid] = [trace]  # new set
                # print(color("New: " + str(trace.event.name) + " - " + \
                #     str([x.event.name for x in candidates[vpid]]), fg="blue"))
        elif (trace.event.name in target_chain) and (vpid in candidates):            
            if len(candidates[vpid]) >= 9 and (trace.event.name in target_chain[9:]):
                trace_index = target_chain[9:].index(trace.event.name) + 9
                expected_index = target_chain[9:].index(candidates[vpid][-1].event.name) + 1 + 9
            elif len(candidates[vpid]) >= 9:
                # print(color("Skipping: " + str(trace.event.name), fg="yellow"))
                continue  # skip
            else:
                trace_index = target_chain.index(trace.event.name)
                expected_index = target_chain.index(candidates[vpid][-1].event.name) + 1
            # Account for chains of callbacks
            if trace.event.name == target_chain[-1] and candidates[vpid][-1].event.name == target_chain[0]:
                if len(candidates[vpid]) > 1:
                    candidates[vpid] = candidates[vpid][:-1]  # pop last start and continue looking
                    # print(color("Chain of callbacks, popping: " + str(trace.event.name) , fg="yellow"))
                else:
                    candidates.pop(vpid)
                    # print(color("Chain of callbacks while starting, popping: " + str(trace.event.name) , fg="yellow"))
            elif trace_index == expected_index:
                candidates[vpid].append(trace)
                # print(color("Found: " + str(trace.event.name), fg="green"))
                if trace.event.name == target_chain[-1] and candidates[vpid][-2].event.name == target_chain[-2] \
                        and len(candidates[vpid]) == len(target_chain):  # last one
                    image_pipeline_msg_sets.append(candidates[vpid])
                    # print(color("complete set!", fg="pink"))
                    candidates.pop(vpid)
            else:
                if trace.event.name == "ros2:rclcpp_publish" or \
                        trace.event.name == "ros2:rcl_publish" or \
                        trace.event.name == "ros2:rmw_publish":
                    # print(color("Potential chain of publish: " + str(trace.event.name) + ", skipping" , fg="yellow"))
                    pass
                else:
                    candidates[vpid].append(trace)
                    # print(color("Altered order: " + str([x.event.name for x in candidates[vpid]]) + ", discarding", fg="red"))
                    candidates.pop(vpid)
        else:
            # print(color("Skipped: " + str(trace.event.name), fg="grey"))
            pass
    return image_pipeline_msg_sets

def barplot_all(image_pipeline_msg_sets, title="Barplot"):
    global target_chain

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

# #####################
# # print timing pipeline
# #####################
# image_pipeline_msg_sets = msgsets_from_trace_concurrent(str(os.environ["HOME"]) + "/.ros/tracing/trace_rectify_resize")
# print(len(image_pipeline_msg_sets))
# for msg_set in image_pipeline_msg_sets:
#     if len(msg_set) != len(target_chain):
#         print(color("Not a complete set: " + str([x.event.name for x in msg_set]), fg="red"))
#         pass
#     else:
#         target_chain_ns = []
#         for msg_index in range(len(msg_set)):
#             target_chain_ns.append(msg_set[msg_index].default_clock_snapshot.ns_from_origin)

#         init_ns = target_chain_ns[0]
#         fixed_target_chain_ns = [init_ns] + target_chain_ns
#         # stringout = color("raw image → " + msg_set[0].event.name + " → ")
#         stringout = color("raw image ")
#         for msg_index in range(len(msg_set)):
#             stringout +=" → " + color(msg_set[msg_index].event.name + \
#                 " ({} ms) ".format((fixed_target_chain_ns[msg_index + 1] - fixed_target_chain_ns[msg_index])/1e6), 
#                 fg=target_chain_colors_fg[msg_index], bg="black")
#             # stringout += " → " + msg_set[msg_index].event.name + \
#             #     " ({} ms) ".format((fixed_target_chain_ns[msg_index + 1] - fixed_target_chain_ns[msg_index])/1e6)

#         stringout += color("total " + \
#             " ({} ms) ".format((target_chain_ns[-1] - target_chain_ns[0])/1e6), fg="black", bg="white")
#         print(stringout)

######################
# draw last data entry
######################
image_pipeline_msg_sets = msgsets_from_trace_concurrent(str(os.environ["HOME"]) + "/.ros/tracing/trace_rectify_resize")
msg_set = image_pipeline_msg_sets[-1]
traces(msg_set)


# ######################
# # draw barplot all data
# ######################
# # NOTE: Discard first few
# image_pipeline_msg_sets = msgsets_from_trace_concurrent(str(os.environ["HOME"]) + "/.ros/tracing/trace_rectify_resize")
# barplot_all(image_pipeline_msg_sets[10:], title="image_pipeline in CPU")
# image_pipeline_msg_sets = msgsets_from_trace_concurrent(str(os.environ["HOME"]) + "/.ros/tracing/trace_rectify_resize_fpga")
# barplot_all(image_pipeline_msg_sets[10:], title="image_pipeline in FPGA")

# image_pipeline_msg_sets = msgsets_from_trace_concurrent(str(os.environ["HOME"]) + "/.ros/tracing/trace_rectify_resize_stress")
# barplot_all(image_pipeline_msg_sets[10:], title="image_pipeline in CPU and with stress")
# image_pipeline_msg_sets = msgsets_from_trace_concurrent(str(os.environ["HOME"]) + "/.ros/tracing/trace_rectify_resize_fpga_stress")
# barplot_all(image_pipeline_msg_sets[10:], title="image_pipeline in FPGA and with stress")


# # ######################
# # # draw bar charts
# # ######################
# # # NOTE: Discard first few
# image_pipeline_msg_sets_ns_cpu = barchart_data(msgsets_from_trace_concurrent(str(os.environ["HOME"]) \
#     + "/.ros/tracing/trace_rectify_resize")[10:])
# image_pipeline_msg_sets_ns_fpga = barchart_data(msgsets_from_trace_concurrent(str(os.environ["HOME"]) \
#     + "/.ros/tracing/trace_rectify_resize_fpga")[10:])
# image_pipeline_msg_sets_ns_cpu_stress = barchart_data(msgsets_from_trace_concurrent(str(os.environ["HOME"]) \
#     + "/.ros/tracing/trace_rectify_resize_stress")[10:])
# image_pipeline_msg_sets_ns_fpga_stress = barchart_data(msgsets_from_trace_concurrent(str(os.environ["HOME"]) \
#     + "/.ros/tracing/trace_rectify_resize_fpga_stress")[10:])

# # # plot latest values
# # df_cpu = pd.DataFrame(image_pipeline_msg_sets_ns_cpu[-1:])  # pick the latest one
# # df_fpga = pd.DataFrame(image_pipeline_msg_sets_ns_fpga[-1:])  # pick the latest one
# # df = pd.concat([df_cpu, df_fpga], ignore_index=True)
# # df.columns = target_chain_dissambiguous
# # substrates = pd.DataFrame({'substrate': ["CPU","FPGA"]})
# # df = df.join(substrates)

# # plot averages
# df_cpu_mean = pd.DataFrame(image_pipeline_msg_sets_ns_cpu).mean()
# df_fpga_mean = pd.DataFrame(image_pipeline_msg_sets_ns_fpga).mean()
# df_cpu_stress_mean = pd.DataFrame(image_pipeline_msg_sets_ns_cpu_stress).mean()
# df_fpga_stress_mean = pd.DataFrame(image_pipeline_msg_sets_ns_fpga_stress).mean()

# df_mean = pd.concat([df_cpu_mean, df_fpga_mean, df_cpu_stress_mean, df_fpga_stress_mean], axis=1).transpose()
# df_mean.columns = target_chain_dissambiguous
# substrates = pd.DataFrame({'substrate': ["CPU","FPGA","CPU stress", "FPGA stress"]})
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

