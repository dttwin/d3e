import lzma
import os
import csv
from lxml import etree as et
from collections import defaultdict

class FCDExtractor(object):

    def __init__(self, my_vehicle_list, my_float_attributes=(), my_mean_attributes=()):
        self.vehicle_ids = my_vehicle_list
        self.float_attribs = my_float_attributes
        self.mean_attribs = my_mean_attributes

    def extract_from_file(self, my_fcd_filename):
        # with lzma.open(my_fcd_filename) as cf:
        waiting_times = defaultdict(float)
        with open(my_fcd_filename, 'rb') as cf:
            last_percentage_int = 0
            file_length = float(os.path.getsize(my_fcd_filename))
            print(f'FCDExtractor: file has {file_length:.0f} bytes')
            limited_data = list()
            total_data = list()
            for event, element in et.iterparse(cf, tag="timestep"):
                time = float(element.get('time'))
                first_child = True
                total_row = {'timestamp': time}
                num_records = 0
                for child in element:
                    num_records += 1
                    row = dict(child.attrib)
                    row['timestamp'] = time
                    vid = child.get('id')
                    # Patch waiting time
                    old_waiting = waiting_times[vid]
                    new_waiting = float(row['waiting'])
                    waiting = max(0, new_waiting-old_waiting)
                    row['waiting'] = waiting
                    waiting_times[vid] = new_waiting
                    # Output selected vehicles
                    if vid in VEHICLE_IDS:
                        limited_data.append(row)
                    # Prepare total output
                    if first_child:
                        for elem in row:
                            if elem in self.float_attribs:
                                total_row[elem] = float(row[elem])
                        first_child = False
                    else:
                        for elem in row:
                            if elem in self.float_attribs:
                                total_row[elem] += float(row[elem])
                for attrib in self.mean_attribs:
                    total_row[attrib] /= num_records
                total_row['number of vehicles'] = num_records
                total_data.append(total_row)
                element.clear()
                file_percentage = 100 * cf.tell()/file_length
                file_percentage_int = int(10*file_percentage)
                if file_percentage_int > last_percentage_int:
                    print(f'FCDExtractor: processed {file_percentage:.1f}%')
                    last_percentage_int = file_percentage_int

        return limited_data, total_data


if __name__ == "__main__":

    # VEHICLE_IDS =  ['1.7', '2.114', '1.415', '1.818', '2.846', '2.1023', '2.1340', '1.2010']
    # VEHICLE_IDS = ['1.139', '1.621', '1.864', 'probe3.mid.2', '1.1285', '1.1500', '1.1915']
    VEHICLE_IDS = ['7.31', '1.166', '0.248', '6.348', '2.1535']

    BASE_FILE = r'notts\notts_emission.out.xml'
    SAMPLES_FILE = 'fcd_notts_samples.csv'
    TOTALS_FILE = 'fcd_notts_total.csv'

    # BASE_FILE = r'llr\llr_emission_20190529T113150.out.xml'
    # SAMPLES_FILE = 'fcd_llr_samples.csv'
    # TOTALS_FILE = 'fcd_llr_total.csv'

    FLOAT_ATTRIBS = ['CO2', 'CO', 'HC', 'NOx', 'PMx', 'fuel', 'noise', 'waiting', 'speed']
    MEAN_ATTRIBS = ['noise', 'speed']

    extractor = FCDExtractor(VEHICLE_IDS, FLOAT_ATTRIBS, MEAN_ATTRIBS)
    vehicle_data, total_data = extractor.extract_from_file(BASE_FILE)
    # print(vehicle_data)

    fieldnames = [
        'timestamp', 'id',
        'eclass', 'CO2', 'CO', 'HC', 'NOx', 'PMx', 'fuel', 'electricity', 'noise',
        'route', 'type', 'waiting', 'lane', 'pos', 'speed', 'angle', 'x', 'y'
    ]
    with open(SAMPLES_FILE, 'w', newline='') as cf:
        dw = csv.DictWriter(cf, fieldnames=fieldnames)
        dw.writeheader()
        for data_row in vehicle_data:
            dw.writerow(data_row)

    fieldnames = ['timestamp', 'number of vehicles'] + FLOAT_ATTRIBS
    with open(TOTALS_FILE, 'w', newline='') as cf:
        dw = csv.DictWriter(cf, fieldnames=fieldnames)
        dw.writeheader()
        for data_row in total_data:
            dw.writerow(data_row)
