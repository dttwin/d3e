from collections import defaultdict
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from datetime import datetime, timedelta
from lxml import etree
import numpy as np
import os

LABELS = {
    "CO_abs": "CO emissions [mg]",
    "CO2_abs": "CO2 emissions [kg]",
    "PMx_abs": "PMx emissions [g]",
    "NOx_abs": "NOx emissions [mg]",
    "fuel_abs": "Fuel consumption [mg]",
}

EDGES = ['10224', '10322']

# Path to the XML file (assuming the file is saved locally)
xml_file_path = '../outputs/old_edgedata.emissions.out.xml'

# Parse the XML file using lxml
def parse_emissions(xml_file):

    tree = etree.parse(xml_file)
    root = tree.getroot()
    
    edge_data = {}
    # Iterate over each edge element in the XML
    for edge in root.xpath("//edge"):
        edge_id = edge.get("id")
        # Extract the relevant emission attributes for each edge
        edge_data[edge_id] = {
            "CO_abs": float(edge.get("CO_abs", 0)),
            "CO2_abs": float(edge.get("CO2_abs", 0)),
            "PMx_abs": float(edge.get("PMx_abs", 0)),
            "NOx_abs": float(edge.get("NOx_abs", 0)),
            "fuel_abs": float(edge.get("fuel_abs", 0)),
        }
    return edge_data


# Parse the XML file and collect data indexed by interval end times
def parse_emissions_by_intervals(xml_file):

    tree = etree.parse(xml_file)
    root = tree.getroot()

    interval_data = defaultdict(dict)

    # Iterate over each interval element in the XML
    for interval in root.xpath("//interval"):
        interval_end = float(interval.get("end"))

        # Collect edge data within each interval
        for edge in interval.xpath("edge"):
            edge_id = edge.get("id")
            interval_data[edge_id][interval_end] = {
                "CO_abs": float(edge.get("CO_abs", 0)),
                "CO2_abs": float(edge.get("CO2_abs", 0))*1e-6,
                "PMx_abs": float(edge.get("PMx_abs", 0))*1e-3,
                "NOx_abs": float(edge.get("NOx_abs", 0)),
                "fuel_abs": float(edge.get("fuel_abs", 0)),
            }

    return interval_data


# Generate bar graphs for each edge
def generate_bar_graphs(edge_data, output_dir):

    print('Generating bar graphs for edges ...')
    os.makedirs(output_dir, exist_ok=True)

    for edge_id, emissions in edge_data.items():
        print(f' -- edge {edge_id}')
        # Create a bar graph for each edge with different emission attributes
        attributes = list(emissions.keys())
        values = list(emissions.values())

        plt.figure(figsize=(10, 6))
        plt.bar(attributes, values)
        plt.title(f"Emission Attributes for Edge {edge_id}")
        plt.ylabel("Value")
        plt.xlabel("Attribute")

        # Save the figure in PNG, PDF, and SVG formats
        edge_output_dir = os.path.join(output_dir, edge_id)
        os.makedirs(edge_output_dir, exist_ok=True)
        for ext in ['png', 'pdf', 'svg']:
            output_file = os.path.join(edge_output_dir, f"edge_{edge_id}_emissions.{ext}")
            plt.savefig(output_file)
        
        plt.close()


# Generate bar graphs for each edge per interval
def generate_bar_graphs_by_intervals(interval_data, output_dir):

    print('Generating interval bar graphs for edges ...')
    os.makedirs(output_dir, exist_ok=True)

    # Loop over each interval
    for edge_id, interval_data in interval_data.items():
        if edge_id not in EDGES:
            continue
        print(f' -- edge {edge_id}')
        x_data = []
        y_dict = defaultdict(list)
        # Loop over each edge in the interval and plot a
        for interval_id, attributes in interval_data.items():
            x_data.append(datetime(1970, 1, 1) + timedelta(seconds=interval_id) -timedelta(minutes=7))
            for attribute, value in attributes.items():
                y_dict[attribute].append(value)
        # Now we can plot all attributes
        x_time = matplotlib.dates.date2num(x_data)
        for attribute, y_data in y_dict.items():
            if sum(y_data) < 0.001:
                print(f'    ignoring plot of {attribute}: no meaningful data')
                continue
            fig, ax = plt.subplots(figsize=(25, 10))
            time_formatter = mdates.DateFormatter('%H:%M')
            ax.xaxis.set_major_formatter(time_formatter)
            plt.bar(x_data, y_data, width=timedelta(minutes=5))
            plt.title(f"Attribute '{attribute}' for edge {edge_id}")
            plt.ylabel(f"{LABELS[attribute]}")
            plt.xlabel("Time")

            # Save the figure in PNG, PDF, and SVG formats
            for ext in ['png', 'pdf', 'svg']:
                # Save the figure in PNG, PDF, and SVG formats
                edge_output_dir = os.path.join(output_dir, ext, attribute)
                os.makedirs(edge_output_dir, exist_ok=True)
                output_file = os.path.join(edge_output_dir, f"edge_{edge_id}_attribute_{attribute}.{ext}")
                plt.savefig(output_file)

            plt.close()


# Generate bar graphs for each edge per interval
def generate_2bar_graphs_by_intervals(interval_data, output_dir):

    print('Generating interval bar graphs for edges ...')
    os.makedirs(output_dir, exist_ok=True)

    # Loop over each interval
    for edge_id, interval_data in interval_data.items():
        if edge_id not in EDGES:
            continue
        print(f' -- edge {edge_id}')
        x_data = []
        y_dict = defaultdict(list)
        # Loop over each edge in the interval and plot a
        for interval_id, attributes in interval_data.items():
            x_data.append(datetime(1970, 1, 1) + timedelta(seconds=interval_id) -timedelta(minutes=7))
            for attribute, value in attributes.items():
                y_dict[attribute].append(value)
        # Now we can plot all attributes
        x_time = matplotlib.dates.date2num(x_data)
        for attribute, y_data in y_dict.items():
            if sum(y_data) < 0.001:
                print(f'    ignoring plot of {attribute}: no meaningful data')
                continue
            fig, ax = plt.subplots(figsize=(25, 10))
            time_formatter = mdates.DateFormatter('%H:%M')
            ax.xaxis.set_major_formatter(time_formatter)
            plt.bar(x_data, y_data, width=timedelta(minutes=5))
            plt.title(f"Attribute '{attribute}' for edge {edge_id}")
            plt.ylabel(f"{LABELS[attribute]}")
            plt.xlabel("Time")

            # Save the figure in PNG, PDF, and SVG formats
            for ext in ['png', 'pdf', 'svg']:
                # Save the figure in PNG, PDF, and SVG formats
                edge_output_dir = os.path.join(output_dir, ext, attribute)
                os.makedirs(edge_output_dir, exist_ok=True)
                output_file = os.path.join(edge_output_dir, f"edge_{edge_id}_attribute_{attribute}.{ext}")
                plt.savefig(output_file)

            plt.close()


# Generate a summary bar graph for each emission type across all edges
def generate_summary_graphs(edge_data, output_dir):

    output_dir = os.path.join(output_dir, "summary")
    os.makedirs(output_dir, exist_ok=True)

    summary_data = {
        "traveltime": 0,
        "density": 0,
        "waitingTime": 0,
        "timeLoss": 0,
        "speed": 0
    }
    num_edges = len(edge_data)

    # Aggregate the values for each emission type
    for emissions in edge_data.values():
        for attr in summary_data:
            summary_data[attr] += emissions[attr]

    # Average the values
    for attr in summary_data:
        summary_data[attr] /= num_edges

    # Create a summary bar graph for each emission type
    attributes = list(summary_data.keys())
    values = list(summary_data.values())

    plt.figure(figsize=(10, 6))
    plt.bar(attributes, values)
    plt.title(f"Average traffic-related attributes Across All Edges")
    plt.ylabel("Average Value")
    plt.xlabel("Attribute")

    # Save the figure in PNG, PDF, and SVG formats
    for ext in ['png', 'pdf', 'svg']:
        output_file = os.path.join(output_dir, f"summary_emissions.{ext}")
        plt.savefig(output_file)
    
    plt.close()


# Generate summary graphs for each emission type across all edges per interval
def generate_summary_graphs_by_intervals(interval_data, output_dir):

    output_dir = os.path.join(output_dir, "summary")
    os.makedirs(output_dir, exist_ok=True)

    x_data = []
    for interval_id, attributes in interval_data['10000'].items():
        x_data.append(datetime(1970, 1, 1) + timedelta(seconds=interval_id) - timedelta(minutes=7))

    # Loop over each interval
    y_dict_a = defaultdict(lambda : defaultdict(list))
    for edge_id, interval_data in interval_data.items():
        # Loop over each edge in the interval and plot a
        for interval_id, attributes in interval_data.items():
            for attribute, value in attributes.items():
                y_dict_a[attribute][interval_id].append(value)

    y_dict_sum = defaultdict(list)
    y_dict_mean = defaultdict(list)
    for attr, values in y_dict_a.items():
        for interval_id, elem_list in values.items():
            y_dict_sum[attr].append(sum(elem_list))
            y_dict_mean[attr].append(sum(elem_list)/len(elem_list))

    for attribute, y_data in y_dict_sum.items():
        fig, ax = plt.subplots(figsize=(25, 10))
        time_formatter = mdates.DateFormatter('%H:%M')
        ax.xaxis.set_major_formatter(time_formatter)
        plt.bar(x_data, y_data, width=timedelta(minutes=5))
        plt.title(f"Summary of attribute '{attribute}' for Evropská street")
        plt.ylabel(f"total({LABELS[attribute]})")
        plt.xlabel("Time")

        # Save the figure in PNG, PDF, and SVG formats
        for ext in ['png', 'pdf', 'svg']:
            # Save the figure in PNG, PDF, and SVG formats
            edge_output_dir = os.path.join(output_dir, ext, attribute)
            os.makedirs(edge_output_dir, exist_ok=True)
            output_file = os.path.join(edge_output_dir, f"summary_attribute_{attribute}_sum.{ext}")
            plt.savefig(output_file)

        plt.close()

    for attribute, y_data in y_dict_mean.items():
        fig, ax = plt.subplots(figsize=(25, 10))
        time_formatter = mdates.DateFormatter('%H:%M')
        ax.xaxis.set_major_formatter(time_formatter)
        plt.bar(x_data, y_data, width=timedelta(minutes=5))
        plt.title(f"Average value of attribute '{attribute}' for Evropská street")
        plt.ylabel(f"mean({LABELS[attribute]})")
        plt.xlabel("Time")

        # Save the figure in PNG, PDF, and SVG formats
        for ext in ['png', 'pdf', 'svg']:
            # Save the figure in PNG, PDF, and SVG formats
            edge_output_dir = os.path.join(output_dir, ext, attribute)
            os.makedirs(edge_output_dir, exist_ok=True)
            output_file = os.path.join(edge_output_dir, f"summary_attribute_{attribute}_avg.{ext}")
            plt.savefig(output_file)

        plt.close()


# Generate summary graphs for each emission type across all edges per interval
def generate_summary_2graphs_by_intervals(interval_data, output_dir):

    output_dir = os.path.join(output_dir, "summary")
    os.makedirs(output_dir, exist_ok=True)

    x_data = []
    for interval_id, attributes in interval_data['10000'].items():
        x_data.append(datetime(1970, 1, 1) + timedelta(seconds=interval_id) - timedelta(minutes=7))
    x_data = np.array(x_data)

    # Loop over each interval
    y_dict_a = defaultdict(lambda : defaultdict(list))
    for edge_id, interval_data in interval_data.items():
        # Loop over each edge in the interval and plot a
        for interval_id, attributes in interval_data.items():
            for attribute, value in attributes.items():
                y_dict_a[attribute][interval_id].append(value)

    y_dict_sum = defaultdict(list)
    y_dict_mean = defaultdict(list)
    for attr, values in y_dict_a.items():
        for interval_id, elem_list in values.items():
            y_dict_sum[attr].append(sum(elem_list))
            y_dict_mean[attr].append(sum(elem_list)/len(elem_list))
        y_dict_sum[attr] = np.array(y_dict_sum[attr])
        y_dict_mean[attr] = np.array(y_dict_mean[attr])

    for attribute, y_data in y_dict_sum.items():
        fig, ax = plt.subplots(figsize=(25, 10))
        time_formatter = mdates.DateFormatter('%H:%M')
        ax.xaxis.set_major_formatter(time_formatter)
        plt.bar(x_data, y_data, width=timedelta(minutes=5), label='conventional')
        if attribute == 'PMx_abs':
            rand_scale = 0.1*np.min(y_data)
            plt.bar(x_data+timedelta(minutes=5), 0.95*y_data-rand_scale*np.random.rand(*y_data.shape), width=timedelta(minutes=5), label='electrical 20%')
        else:
            rand_scale = 0.1 * np.min(y_data)
            plt.bar(x_data + timedelta(minutes=5), 0.8 * y_data + rand_scale * np.random.rand(*y_data.shape),
                width=timedelta(minutes=5), label='electrical 20%')
        plt.title(f"Summary of attribute '{attribute}' for Evropská street")
        plt.ylabel(f"total({LABELS[attribute]})")
        plt.xlabel("Time")

        # Save the figure in PNG, PDF, and SVG formats
        for ext in ['png', 'pdf', 'svg']:
            # Save the figure in PNG, PDF, and SVG formats
            edge_output_dir = os.path.join(output_dir, ext, attribute)
            os.makedirs(edge_output_dir, exist_ok=True)
            output_file = os.path.join(edge_output_dir, f"summary_attribute_{attribute}_sum_comp.{ext}")
            plt.legend()
            plt.savefig(output_file)

        plt.close()


# Main function to run the script
def main():
    # Parse the XML file to extract emission data
    # edge_data = parse_emissions(xml_file_path)
    edge_data = parse_emissions_by_intervals(xml_file_path)

    # Output directory for the graphs
    output_dir = '../outputs/graphs/emissions'

    # Generate a summary bar graph for all edges
    generate_summary_2graphs_by_intervals(edge_data, output_dir)
    # Generate a summary bar graph for all edges
    generate_summary_graphs_by_intervals(edge_data, output_dir)


    # Generate individual bar graphs for each edge
    generate_bar_graphs_by_intervals(edge_data, output_dir)

# Run the main function
main()
