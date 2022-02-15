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
from typing import List, Optional
import pandas as pd

from bokeh.plotting.figure import figure
from bokeh.plotting.figure import Figure
from bokeh.plotting import output_notebook
from bokeh.io import show
from bokeh.layouts import row
from bokeh.models import ColumnDataSource
from bokeh.models import DatetimeTickFormatter, PrintfTickFormatter, Legend
from bokeh.models import Segment
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

def generate_launch_description():
    return LaunchDescription()

######################################################################
######################################################################

# Create a trace collection message iterator from the first command-line
# argument.
msg_it = bt2.TraceCollectionMessageIterator(
    # str(os.environ["HOME"]) + "/.ros/tracing/trace_rectify_fpga"
    str(os.environ["HOME"]) + "/.ros/tracing/trace_rectify"
)

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
]
target_chain_colors_fg_bokeh = [
    "lightgray",
    "silver",
    "darkgray",
    "gray",
    "dimgray",
    "lightslategray",
    "slategray",
    "darkslategray",
    "black",
    # "blue",
    # "blueviolet",
    # "brown",
    # "burlywood",
    # "cadetblue",
    # "chartreuse",
    # "chocolate",
    # "coral",
    # "cornflowerblue",
]
target_chain_layer = [
    "userland",
    "userland",
    "userland",
    "userland",
    "rclcpp",
    "rcl",
    "rmw",
    "userland",
    "userland",
]
target_chain_label_layer = [  # associated with the layer
    4,
    4,
    4,
    4,
    3,
    2,
    1,
    4,
    4,
]
target_chain_marker = [
    "dot",
    "circle",
    "cross",
    "diamond",
    "hex",
    "plus",
    "star",
    "triangle",
    "asterisk",
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
]

# For some reason it seems to be displayed in the reverse order on the Y axis
segment_types = [
    "rmw",
    "rcl",
    "rclcpp",
    "userland"
]

######################
# # print timing pipeline
######################
# image_pipeline_msg_sets = msgsets_from_trace(str(os.environ["HOME"]) + "/.ros/tracing/trace_rectify")
# for msg_set in image_pipeline_msg_sets:
#     if len(msg_set) != 9:
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

# target_chain_dates = [datetime.datetime.fromtimestamp(ns/1e9) for ns in target_chain_ns]
# start_time = target_chain_dates[0].strftime('%Y-%m-%d %H:%M')



######################
# # draw all data points
######################
# image_pipeline_msg_sets = msgsets_from_trace(str(os.environ["HOME"]) + "/.ros/tracing/trace_rectify")
# fig = figure(
#     title='Image pipeline tracing',
#     x_axis_label=f'Milliseconds',
#     y_range=segment_types,
#     plot_width=2000,
#     plot_height=600,
# )
# fig.title.align = 'center'
# fig.title.text_font_size = '40px'
# # fig.xaxis[0].formatter = DatetimeTickFormatter(milliseconds = ['%3Nms'])
# fig.xaxis[0].formatter = PrintfTickFormatter(format="%f ms")
# fig.xaxis[0].ticker.desired_num_ticks = 20
# fig.xaxis[0].axis_label_text_font_size = '30px'
# fig.yaxis[0].major_label_text_font_size = '25px'
# target_chain_colors_fg_bokeh_background = [
#     "aliceblue",
#     "ghostwhite",
#     "whitesmoke",
#     "seashell",
#     "beige",
#     "oldlace",
#     "floralwhite",
#     "ivory",
#     "antiquewhite",
# ]
# for msg_set in image_pipeline_msg_sets[:20]:
#     target_chain_ns = []
#     for msg_index in range(len(msg_set)):
#         target_chain_ns.append(msg_set[msg_index].default_clock_snapshot.ns_from_origin)
#     init_ns = target_chain_ns[0]

#     for msg_index in range(len(msg_set)):
#     #     add_markers_to_figure(fig, msg_set[msg_index].event.name, [(target_chain_ns[msg_index] - init_ns)/1e6], 'blue', marker_type='plus', legend_label='timing')
#         add_markers_to_figure(
#             fig, 
#             target_chain_layer[msg_index], 
#             [(target_chain_ns[msg_index] - init_ns)/1e6], 
#             target_chain_colors_fg_bokeh_background[msg_index], 
#             marker_type=target_chain_marker[msg_index], 
#             legend_label="(all) " + msg_set[msg_index].event.name
#         )
# # hack legend to the right
# fig.legend.location = "right"
# new_legend = fig.legend[0]
# fig.legend[0] = None
# fig.add_layout(new_legend, 'right')
# show(fig)


# ######################
# # draw last data entry
# ######################
# image_pipeline_msg_sets = msgsets_from_trace(str(os.environ["HOME"]) + "/.ros/tracing/trace_rectify")
# fig = figure(
#     title='Image pipeline tracing',
#     x_axis_label=f'Milliseconds',
#     y_range=segment_types,
#     plot_width=2000,
#     plot_height=600,
# )
# fig.title.align = 'center'
# fig.title.text_font_size = '40px'
# # fig.xaxis[0].formatter = DatetimeTickFormatter(milliseconds = ['%3Nms'])
# fig.xaxis[0].formatter = PrintfTickFormatter(format="%f ms")
# fig.xaxis[0].ticker.desired_num_ticks = 20
# fig.xaxis[0].axis_label_text_font_size = '30px'
# fig.yaxis[0].major_label_text_font_size = '25px'
# for msg_index in range(len(msg_set)):
# #     add_markers_to_figure(fig, msg_set[msg_index].event.name, [(target_chain_ns[msg_index] - init_ns)/1e6], 'blue', marker_type='plus', legend_label='timing')
#     add_markers_to_figure(
#         fig, 
#         target_chain_layer[msg_index], 
#         [(target_chain_ns[msg_index] - init_ns)/1e6], 
#         target_chain_colors_fg_bokeh[msg_index], 
#         marker_type=target_chain_marker[msg_index], 
#         legend_label=msg_set[msg_index].event.name
#     )
#     if "image_proc_rectify_init" in msg_set[msg_index].event.name:
#         label = Label(
#             x= (target_chain_ns[msg_index] - init_ns)/1e6, 
#             y=target_chain_label_layer[msg_index],
#             x_offset=0, 
#             y_offset=-100,
#             text=msg_set[msg_index].event.name.split(":")[-1]
#         )
#     else:
#         label = Label(
#             x= (target_chain_ns[msg_index] - init_ns)/1e6, 
#             y=target_chain_label_layer[msg_index],
#             x_offset=-30, 
#             y_offset=-30,
#             text=msg_set[msg_index].event.name.split(":")[-1]
#         )        
#     fig.add_layout(label)
# # hack legend to the right
# fig.legend.location = "right"
# new_legend = fig.legend[0]
# fig.legend[0] = None
# fig.add_layout(new_legend, 'right')
# show(fig)


######################
# draw barplot all data
######################
image_pipeline_msg_sets = msgsets_from_trace(str(os.environ["HOME"]) + "/.ros/tracing/trace_rectify_fpga")
## with matplotlib
# # image_pipeline_msg_sets_ns = [[0]*len(image_pipeline_msg_sets[:5])]*len(image_pipeline_msg_sets[0])
# #  see https://stackoverflow.com/questions/240178/list-of-lists-changes-reflected-across-sublists-unexpectedly/37804636
# image_pipeline_msg_sets_ns =  [[0]*len(image_pipeline_msg_sets[:5]) for _ in range(len(image_pipeline_msg_sets[0]))]
# for set_index in range(len(image_pipeline_msg_sets[:5])):
#     target_chain_ns = []
#     for msg_index in range(len(image_pipeline_msg_sets[set_index])):
#         target_chain_ns.append(image_pipeline_msg_sets[set_index][msg_index].default_clock_snapshot.ns_from_origin)
#     init_ns = target_chain_ns[0]
#     for msg_index in range(len(image_pipeline_msg_sets[set_index])):
#         image_pipeline_msg_sets_ns[msg_index][set_index] = (target_chain_ns[msg_index] - init_ns)/1e6
#
# import matplotlib.pyplot as plt
# fig = plt.figure(figsize =(10, 7))
# plt.boxplot(image_pipeline_msg_sets_ns)
# plt.boxplot(
#     image_pipeline_msg_sets_ns,
#     labels=target_chain
# )
# plt.xticks(rotation=90)
# plt.show()

## with plotly
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
df.columns = target_chain
import plotly.express as px

fig = px.box(
    df,    
    points="all",
    template="plotly_white",
)
fig.update_xaxes(title_text = "Trace event")
fig.update_yaxes(title_text = "Milliseconds")
fig.show()


# # ######################
# # # draw bar charts
# # ######################
# image_pipeline_msg_sets_cpu = msgsets_from_trace(str(os.environ["HOME"]) + "/.ros/tracing/trace_rectify")
# image_pipeline_msg_sets_ns_cpu = []
# for set_index in range(len(image_pipeline_msg_sets_cpu)):
#     aux_set = []
#     target_chain_ns = []
#     for msg_index in range(len(image_pipeline_msg_sets_cpu[set_index])):
#         target_chain_ns.append(image_pipeline_msg_sets_cpu[set_index][msg_index].default_clock_snapshot.ns_from_origin)
#     for msg_index in range(len(image_pipeline_msg_sets_cpu[set_index])):
#         if msg_index == 0:
#             previous = target_chain_ns[0]
#         else:
#             previous = target_chain_ns[msg_index - 1]
#         aux_set.append((target_chain_ns[msg_index] - previous)/1e6)
#     image_pipeline_msg_sets_ns_cpu.append(aux_set)

# image_pipeline_msg_sets_fpga = msgsets_from_trace(str(os.environ["HOME"]) + "/.ros/tracing/trace_rectify_fpga")
# image_pipeline_msg_sets_ns_fpga = []
# for set_index in range(len(image_pipeline_msg_sets_fpga)):
#     aux_set = []
#     target_chain_ns = []
#     for msg_index in range(len(image_pipeline_msg_sets_fpga[set_index])):
#         target_chain_ns.append(image_pipeline_msg_sets_fpga[set_index][msg_index].default_clock_snapshot.ns_from_origin)
#     for msg_index in range(len(image_pipeline_msg_sets_fpga[set_index])):
#         if msg_index == 0:
#             previous = target_chain_ns[0]
#         else:
#             previous = target_chain_ns[msg_index - 1]
#         aux_set.append((target_chain_ns[msg_index] - previous)/1e6)
#     image_pipeline_msg_sets_ns_fpga.append(aux_set)

# # plot latest values
# df_cpu = pd.DataFrame(image_pipeline_msg_sets_ns_cpu[-1:])  # pick the latest one
# df_fpga = pd.DataFrame(image_pipeline_msg_sets_ns_fpga[-1:])  # pick the latest one
# df = pd.concat([df_cpu, df_fpga], ignore_index=True)
# df.columns = target_chain
# substrates = pd.DataFrame({'substrate': ["CPU","FPGA"]})
# df = df.join(substrates)

# # plot averages
# df_cpu_mean = pd.DataFrame(image_pipeline_msg_sets_ns_cpu).mean()
# df_fpga_mean = pd.DataFrame(image_pipeline_msg_sets_ns_fpga).mean()
# df_mean = pd.concat([df_cpu_mean, df_fpga_mean], axis=1).transpose()
# df_mean.columns = target_chain
# substrates = pd.DataFrame({'substrate': ["CPU","FPGA"]})
# df_mean = df_mean.join(substrates)

# import plotly.express as px
# fig = px.bar(
#     df_mean, 
#     template="plotly_white",    
#     x="substrate", y=target_chain,
# )
# fig.update_xaxes(title_text = "")
# fig.update_yaxes(title_text = "Milliseconds")
# fig.show()

