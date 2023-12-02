import xlsxwriter

workbook = xlsxwriter.Workbook('dataOut.xlsx')

worksheet = workbook.add_worksheet("Accelerometer")

with open("data.txt") as file:
  data = file.readlines()

row = 0
col = 0

for i in data:
    #print(col)
    #print(row)
    if row == 0:
        for x in i.split():
            worksheet.write(row, col, x)
            col += 1
    else:
        for x in i.split():
            worksheet.write(row, col, float(x))
            col += 1
    row += 1
    col = 0

workbook.close()

#used to convert the text file output to an excel file.
