#! /usr/bin/python3
import rclpy
import rclpy.node as Node
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import FuncFormatter
import seaborn as sns

class PlannerDataVisualizer(Node):
    # private attribs
    _topic = "/cell_certainty_grid/data"
    _rate = 5.0
    _bins = 30
    _range = None

    # public attribs
    def __init__(self):
        super().__init__("planner_data_visualizer")
        self._data = np.array([])

        self.create_subscription(Float32MultiArray, self._topic, self._callback, 10)
        # # capped at 10 Hz
        self.create_timer(1.0 / min(self._rate, 10.0), self._plot)


        sns.set_theme(style="darkgrid")
        plt.ion()
        self._fig, self._ax = plt.subplots()
        plt.grid(True)
        plt.show(block=False)

    def _callback(self, msg):
        # extracting the CCV_grid data
        self._data = np.asarray(msg.data, dtype=np.float32)


    def _plot(self):
        # if self._data.size:
        #     self._ax.cla()
        #     self._ax.hist(self._data, bins=self._bins)
        #     self._ax.yaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{int(v)}"))
        #     self._ax.set_xlabel("x")
        #     self._ax.set_ylabel("count")
        # # keeps the window responsive and redraws
        # plt.pause(0.001)

        if self._data.size:
            self._ax.cla()
            # kde needs spread in the data, stat can be "count", "density", "probability" or "percent"
            sns.histplot(x=self._data, bins=self._bins, binrange=self._range,
                         kde=np.ptp(self._data) > 0, stat="count", ax=self._ax)
            self._ax.yaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{int(v)}"))
            self._ax.set(xlabel="certainty", ylabel="count")
        # keeps the window responsive and redraws
        plt.pause(0.001)



def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        # run node
        node = PlannerDataVisualizer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        # add
        pass
    finally:
        # add
        if node is not None:
            node.destroy_node()
        plt.close("all")
        rclpy.shutdown()

    exit(0)


if __name__ == "__main__":
    main()