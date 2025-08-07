from bokeh.io import curdoc
from bokeh.layouts import column, row
from bokeh.models import ColumnDataSource, Slider, CustomJS
from bokeh.plotting import figure, show

def visualize_with_JS():
    # Define frame and time-tick based polygon data.
    # Each frame is a dictionary of time ticks; each time tick contains a dict
    # with keys "xs" and "ys" for polygon coordinates.
    frames_data = {
        "0": {  # frame 0 with 2 time ticks: 0 and 1
            "0": {"xs": [[1, 2, 3]], "ys": [[1, 4, 1]]},
            "1": {"xs": [[2, 3, 4]], "ys": [[2, 5, 2]]},
        },
        "1": {  # frame 1 with 3 time ticks: 0, 1 and 2
            "0": {"xs": [[3, 4, 5]], "ys": [[3, 6, 3]]},
            "1": {"xs": [[4, 5, 6]], "ys": [[4, 7, 4]]},
            "2": {"xs": [[5, 6, 7]], "ys": [[5, 8, 5]]},
        }
    }

    print(frames_data["0"]["0"])
    print(frames_data["1"]["2"])

    with open('frame_data.txt', 'a') as f:
        print("debug file in frame_data.txt")
        print(frames_data, file=f)


    # Set initial frame and time tick.
    init_frame = "0"
    init_tick = "0"  # showing tick "0" initially

    # Create a ColumnDataSource with the initial polygon data.
    source = ColumnDataSource(data=frames_data[init_frame][init_tick])

    # Create the plot and add patches (polygons) from the source.
    p = figure(title="Time Ticked Polygons", x_range=(0, 10), y_range=(0, 10),
            width=600, height=400)
    p.patches('xs', 'ys', source=source, fill_alpha=0.6, line_color="black")

    # Create the frame slider.
    frame_slider = Slider(start=0, end=len(frames_data)-1, value=0, step=1, title="Frame")

    # For the time tick slider:
    # Use -1 to represent "all" and 0..max_tick for each frame.
    # For frame "0", ticks are "0" and "1", so set the range from -1 to 1.
    time_slider = Slider(start=-1, end=1, value=0, step=1, title="Time Tick (-1 means all)")

    # Create a CustomJS callback that updates theobj_slider polygon data.
    # When the frame slider changes, it recomputes the available time ticks (and sets the slider end).
    # When time_slider.value == -1, all ticks in that frame are combined.
    callback = CustomJS(args=dict(source=source,
                                frame_slider=frame_slider,
                                time_slider=time_slider,
                                frames_data=frames_data),
    code="""
        // Convert the frame value to a string to index our data.
        var frame = frame_slider.value.toString();
        var tickKeys = Object.keys(frames_data[frame]).map(Number);
        tickKeys.sort(function(a, b){ return a - b; });
        var maxTick = tickKeys[tickKeys.length - 1];

        // Update the time_slider range. Always start at -1 for the "all" option.
        time_slider.start = -1;
        time_slider.end = maxTick;

        // If the current time tick slider value is out of bounds, reset it to -1.
        if (time_slider.value < -1 || time_slider.value > maxTick) {
            time_slider.value = -1;
        }

        var new_data;
        if (time_slider.value == -1) {
            // Combine polygon data from all time ticks for this frame.
            new_data = {xs: [], ys: []};
            for (var i = 0; i < tickKeys.length; i++) {
                var tick = tickKeys[i].toString();
                var tick_data = frames_data[frame][tick];
                new_data.xs = new_data.xs.concat(tick_data.xs);
                new_data.ys = new_data.ys.concat(tick_data.ys);
            }
        } else {
            // Show only the polygon data for the selected time tick.
            new_data = frames_data[frame][time_slider.value.toString()];
        }

        // Update the data source and trigger a change.
        source.data = new_data;
        source.change.emit();
    """)

    # Attach the callback to both sliders.
    frame_slider.js_on_change('value', callback)
    time_slider.js_on_change('value', callback)

    # Arrange the layout and add it to the current document.
    layout = column(p, row(frame_slider, time_slider))
    show(layout)
    return 


if __name__ == "__main__":
    visualize_with_JS()

