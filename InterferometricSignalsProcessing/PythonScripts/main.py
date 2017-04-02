import xlsxwriter
import random
import csv

#reading data
file = open('data.txt', 'r')

N = 1
k = []
background = []
amplitude = []
frequency = []
phase = []

for line in file:
    data = line.split('\t')
    k.append(N)
    background.append(float(data[0]))
    amplitude.append(float(data[1]))
    frequency.append(float(data[2]))
    phase.append(float(data[3]))
    N = N + 1

N = k

file.close()

# Create a excel sheet
workbook = xlsxwriter.Workbook('file.xlsx')

# Writing data
# k_column_begin = [0, 0]
# k_column_end = [k_start_loc[0] + N-1, 0]

worksheet = workbook.add_worksheet()
worksheet.write_column('A1', k)
worksheet.write_column('B1', background)
worksheet.write_column('C1', amplitude)
worksheet.write_column('D1', frequency)
worksheet.write_column('E1', phase)

# Creating charts
chart = workbook.add_chart({'type': 'scatter', 'subtype': 'smooth'})
chart.set_y_axis({
    'name': 'Background value',
    'name_font': {'name': 'Times New Roman', 'size': 10},
    'num_font': {'name': 'Times New Roman', 'size': 10},
})

chart.set_x_axis({
    'name': 'Discrete signal sample',
    'name_font': {'name': 'Times New Roman', 'size': 10},
    'num_font': {'name': 'Times New Roman', 'size': 10},
    'max': N,
    'minor_unit': 200,
    'crossing': 0,
    'major_gridlines': {'visible': 'true'},
})

chart.add_series({
    'categories': [[worksheet.name]] ,
    'values': [worksheet.name] + data_start_loc + data_end_loc,
    'name': "Random data",
})

worksheet.insert_chart('F1', chart)

workbook.close()  # Write to file